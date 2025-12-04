#![no_std]
#![no_main]
#![feature(variant_count)]
#![feature(type_alias_impl_trait)]
#![feature(iter_collect_into)]
#![feature(iterator_try_collect)]
#![feature(generic_const_exprs)]

mod adc;
mod control_loop;
#[allow(dead_code)]
mod pwr_src;

use adc::AdcCtrl;
use control_loop::ControlLoop;
use heapless::Vec;
use pwr_src::{
    aux_pwr::AuxPwr,
    battery::{Battery, tmp100_drv::*},
    d_flip_flop::DFlipFlop,
    sink_ctrl::SinkCtrl,
};

use defmt::*;

use embassy_executor::Spawner;
use embassy_stm32::{
    Config,
    adc::{Adc, AdcChannel},
    bind_interrupts,
    can::{self, CanConfigurator, RxBuf, TxBuf},
    gpio::{Level, Output, Speed},
    i2c::{self, I2c, Master},
    mode::Async,
    peripherals::{self, DMA1_CH1, FDCAN1, IWDG},
    rcc::{self, mux::Fdcansel},
    time::khz,
    wdg::IndependentWatchdog,
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::{Channel, Receiver, Sender}, mutex::Mutex, watch::Watch};
use embassy_time::Timer;
use static_cell::StaticCell;

use crate::{adc::AdcCtrlChannel, control_loop::telecommands::Telecommand};

use rodos_can_interface::{RodosCanInterface, receiver::RodosCanReceiver, sender::RodosCanSender};

use {defmt_rtt as _, panic_probe as _};

// bind interrupts
bind_interrupts!(struct Irqs {
    I2C2_3 => i2c::EventInterruptHandler<peripherals::I2C2>, i2c::ErrorInterruptHandler<peripherals::I2C2>;
    TIM16_FDCAN_IT0 => can::IT0InterruptHandler<FDCAN1>;
    TIM17_FDCAN_IT1 => can::IT1InterruptHandler<FDCAN1>;
});


/// config rcc for higher sysclock and fdcan periph clock to make sure
/// all messages can be received without package drop
fn get_rcc_config() -> rcc::Config {
    let mut rcc_config = rcc::Config::default();
    rcc_config.hsi = Some(rcc::Hsi {
        sys_div: rcc::HsiSysDiv::DIV1,
    });
    rcc_config.sys = rcc::Sysclk::PLL1_R;
    rcc_config.pll = Some(rcc::Pll {
        source: rcc::PllSource::HSI,
        prediv: rcc::PllPreDiv::DIV1,
        mul: rcc::PllMul::MUL8,
        divp: None,
        divq: Some(rcc::PllQDiv::DIV2),
        divr: Some(rcc::PllRDiv::DIV2),
    });
    rcc_config.mux.fdcansel = Fdcansel::PLL1_Q;
    rcc_config
}

// static concurrency sync management types
static ITW: StaticCell<Watch<ThreadModeRawMutex, i16, 1>> = StaticCell::new();
static B1W: StaticCell<Watch<ThreadModeRawMutex, i16, 1>> = StaticCell::new();
static B2W: StaticCell<Watch<ThreadModeRawMutex, i16, 1>> = StaticCell::new();
static APW: StaticCell<Watch<ThreadModeRawMutex, i16, 1>> = StaticCell::new();

const TM_CHANNEL_BUF_SIZE: usize = 5;
const CMD_CHANNEL_BUF_SIZE: usize = 5;
static TMC: StaticCell<Channel<ThreadModeRawMutex, EpsTelem, TM_CHANNEL_BUF_SIZE>> = StaticCell::new();
static CMDC: StaticCell<Channel<ThreadModeRawMutex, Telecommand, CMD_CHANNEL_BUF_SIZE>> = StaticCell::new();

// static peripherals
static I2C: StaticCell<Mutex<ThreadModeRawMutex, I2c<'static, Async, Master>>> = StaticCell::new();

// can configuration
const RODOS_DEVICE_ID: u8 = 0x02;

const RX_BUF_SIZE: usize = 500;
const TX_BUF_SIZE: usize = 30;

const RODOS_MAX_RAW_MSG_LEN: usize = 32;
const NUMBER_OF_SENDING_DEVICES: usize = 4;

static RX_BUF: StaticCell<embassy_stm32::can::RxBuf<RX_BUF_SIZE>> = StaticCell::new();
static TX_BUF: StaticCell<embassy_stm32::can::TxBuf<TX_BUF_SIZE>> = StaticCell::new();

/// Watchdog petting task
#[embassy_executor::task]
async fn petter(mut watchdog: IndependentWatchdog<'static, IWDG>) {
    loop {
        watchdog.pet();
        Timer::after_millis(200).await;
    }
}

// Adc reading task
#[embassy_executor::task]
pub async fn adc_thread(mut adc: AdcCtrl<'static, 'static, DMA1_CH1, 4>) {
    const ADC_LOOP_LEN_MS: u64 = 50;
    loop {
        adc.run().await;
        Timer::after_millis(ADC_LOOP_LEN_MS).await;
    }
}

// battery task
#[embassy_executor::task(pool_size = 2)]
pub async fn battery_thread(mut battery: Battery<'static, 'static>) {
    const BTRY_LOOP_LEN_MS: u64 = 10;
    loop {
        battery.run().await;
        Timer::after_millis(BTRY_LOOP_LEN_MS).await;
    }
}

// Aux pwr task
#[embassy_executor::task]
pub async fn aux_pwr_thread(mut aux_pwr: AuxPwr<'static>) {
    const AUX_LOOP_LEN_MS: u64 = 100;
    loop {
        aux_pwr.run().await;
        Timer::after_millis(AUX_LOOP_LEN_MS).await;
    }
}

// control loop task
#[embassy_executor::task]
pub async fn ctrl_thread(mut control_loop: ControlLoop<'static>) {
    loop { control_loop.run().await; }
}

type EpsTelem = (u32, Vec<u8, 2>);
// tm sending task
#[embassy_executor::task]
pub async fn tm_thread(mut can_sender: RodosCanSender, tm_channel: Receiver<'static, ThreadModeRawMutex, EpsTelem, TM_CHANNEL_BUF_SIZE>) {
    loop {
        let (id, value) = tm_channel.receive().await;
        if let Err(e) = can_sender.send(id as u16, &value).await {
            error!("error sending can message: {}", e);
        }
    }
}

// tc receiving task
#[embassy_executor::task]
pub async fn tc_thread(
    mut can_receiver: RodosCanReceiver<NUMBER_OF_SENDING_DEVICES, RODOS_MAX_RAW_MSG_LEN>,
    tc_channel: Sender<'static, ThreadModeRawMutex, Telecommand, TM_CHANNEL_BUF_SIZE>
    ) {
    loop {
        match can_receiver.receive().await {
            Ok(frame) => {
                match Telecommand::parse(frame.data()) {
                    Ok(cmd) => tc_channel.send(cmd).await,
                    Err(e) => error!("error parsing tc {}", e),
                }
            }
            Err(e) => error!("error in frame! {}", e),
        }
    }
}


/// program entry
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    config.rcc = get_rcc_config();
    let p = embassy_stm32::init(config);
    info!("Launching");

    // independent watchdog with timeout 300 MS
    let mut watchdog = IndependentWatchdog::new(p.IWDG, 300_000);
    watchdog.unleash();

    // i2c for temperature sensors
    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = khz(400);
    let temp_sensor_i2c = I2C.init(Mutex::new(I2c::new(
        p.I2C2, p.PA7, p.PA6, Irqs, p.DMA1_CH2, p.DMA1_CH3, i2c_config,
    )));

    // flip flop
    let source_flip_flop = DFlipFlop::new(
        p.PB3, p.PC14, // bat 1
        p.PB4, p.PC6, // bat 2
        p.PB5, p.PC15, // aux pwr
        p.PB6,  // clk
    );

    // sink ctrl
    let sink_ctrl = SinkCtrl::new(p.PA9, p.PA5, p.PA0, p.PA15);

    // ADC setup
    let adc_periph = Adc::new(p.ADC1);

    let internal_temperature_watch = ITW.init(Watch::new());
    let bat_1_watch = B1W.init(Watch::new());
    let bat_2_watch = B2W.init(Watch::new());
    let aux_pwr_watch = APW.init(Watch::new());

    let bat_1_channel = AdcCtrlChannel::new(
        p.PA4.degrade_adc(),
        bat_1_watch.sender().as_dyn(),
        adc::conversion::calculate_voltage_10mv,
    );
    let bat_2_channel = AdcCtrlChannel::new(
        p.PA3.degrade_adc(),
        bat_2_watch.sender().as_dyn(),
        adc::conversion::calculate_voltage_10mv,
    );
    let aux_pwr_channel = AdcCtrlChannel::new(
        p.PA2.degrade_adc(),
        aux_pwr_watch.sender().as_dyn(),
        adc::conversion::calculate_voltage_10mv,
    );

    let adc = AdcCtrl::new(
        adc_periph,
        p.DMA1_CH1,
        internal_temperature_watch.sender().as_dyn(),
        [bat_1_channel, bat_2_channel, aux_pwr_channel],
    );

    // TM channel setup
    let tm_channel = TMC.init(Channel::new());
    let cmd_channel = CMDC.init(Channel::new());

    // first battery
    let bat_1_tmp = Tmp100::new(temp_sensor_i2c, Resolution::BITS12, Addr0State::Floating)
        .await
        .inspect_err(|e| error!("could not establish connection to bat 1 temp sensor: {}", e));
    let bat_1 = Battery::new(bat_1_tmp.ok(), bat_1_watch.receiver().unwrap().as_dyn(), tm_channel.dyn_sender()).await;

    // second battery
    let bat_2_tmp = Tmp100::new(temp_sensor_i2c, Resolution::BITS12, Addr0State::Floating)
        .await
        .inspect_err(|e| error!("could not establish connection to bat 2 temp sensor: {}", e));
    let bat_2 = Battery::new(bat_2_tmp.ok(), bat_2_watch.receiver().unwrap().as_dyn(), tm_channel.dyn_sender()).await;

    // aux power
    let aux_pwr = AuxPwr::new(aux_pwr_watch.receiver().unwrap().as_dyn(), tm_channel.dyn_sender()).await;

    // debug leds not used at the moment (might disrupt can)
    let _led1 = Output::new(p.PB7, Level::Low, Speed::Low);
    let _led2 = Output::new(p.PB8, Level::Low, Speed::Low);

    // set can standby pin to low
    let _can_standby = Output::new(p.PA10, Level::Low, Speed::Low);
    //let _can_2_standby = Output::new(p.PB2, Level::High, Speed::Low);

    // can setup
    let can_configurator = CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs);
    let mut rodos_can_configurator = RodosCanInterface::new(can_configurator, RODOS_DEVICE_ID);

    rodos_can_configurator
        .set_bitrate(1_000_000)
        .add_receive_topic(tmtc_definitions::telecommands::Telecommand::ID as u16, None)
        .unwrap();

    let (can_receiver, can_sender, _interface) = rodos_can_configurator.activate(
        TX_BUF.init(TxBuf::<TX_BUF_SIZE>::new()),
        RX_BUF.init(RxBuf::<RX_BUF_SIZE>::new()),
    ).split_buffered();

    // Main control loop setup
    let control_loop = ControlLoop::spawn(
        source_flip_flop,
        sink_ctrl,
        cmd_channel.dyn_receiver(),
    );

    spawner.must_spawn(petter(watchdog));
    spawner.must_spawn(adc_thread(adc));
    spawner.must_spawn(ctrl_thread(control_loop));
    spawner.must_spawn(battery_thread(bat_1));
    spawner.must_spawn(battery_thread(bat_2));
    spawner.must_spawn(aux_pwr_thread(aux_pwr));
    spawner.must_spawn(tm_thread(can_sender, tm_channel.receiver()));
    spawner.must_spawn(tc_thread(can_receiver, cmd_channel.sender()));
    
    // wait until all other threads finished (never)
    core::future::pending::<()>().await;
}
