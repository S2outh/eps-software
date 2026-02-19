#![no_std]
#![no_main]
#![feature(variant_count)]
#![feature(type_alias_impl_trait)]
#![feature(iter_collect_into)]
#![feature(iterator_try_collect)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)] // This feature is incomplete but beeing used in a benign context

mod adc;
mod control_loop;
#[allow(dead_code)]
mod pwr_src;

use adc::AdcCtrl;
use control_loop::ControlLoop;
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
    adc::{Adc, AdcChannel, AdcConfig},
    bind_interrupts,
    can::{
        self, BufferedFdCanReceiver, BufferedFdCanSender, CanConfigurator, RxFdBuf, TxFdBuf,
        frame::FdFrame,
    },
    gpio::{Level, Output, Speed},
    i2c::{self, I2c, Master},
    mode::Async,
    peripherals::{self, FDCAN1, IWDG},
    rcc::{self, mux::Fdcansel},
    time::khz,
    wdg::IndependentWatchdog,
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    channel::{Channel, DynamicSender, Receiver, Sender},
    mutex::Mutex,
    watch::{DynReceiver, Watch},
};
use embassy_time::{Duration, Ticker, Timer};
use south_common::{
    tmtc_system::{TMValue, TelemetryContainer, TelemetryDefinition, telemetry_container},
    can_config::CanPeriphConfig,
    definitions::telecommands,
    definitions::telemetry::eps as tm, types::Telecommand,
};
use static_cell::StaticCell;

use crate::{
    adc::AdcCtrlChannel,
    pwr_src::{aux_pwr, battery},
};

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

// General setup stuff
const WATCHDOG_TIMEOUT_US: u32 = 300_000;
const WATCHDOG_PETTING_INTERVAL_US: u32 = WATCHDOG_TIMEOUT_US / 2;

// TM container
type EpsTMContainer = telemetry_container!(tm);

// static concurrency sync management types
static ITW: StaticCell<Watch<ThreadModeRawMutex, i16, 1>> = StaticCell::new();
static B1W: StaticCell<Watch<ThreadModeRawMutex, i16, 1>> = StaticCell::new();
static B2W: StaticCell<Watch<ThreadModeRawMutex, i16, 1>> = StaticCell::new();
static APW: StaticCell<Watch<ThreadModeRawMutex, i16, 1>> = StaticCell::new();

const TM_CHANNEL_BUF_SIZE: usize = 5;
const CMD_CHANNEL_BUF_SIZE: usize = 5;
static TMC: StaticCell<Channel<ThreadModeRawMutex, EpsTMContainer, TM_CHANNEL_BUF_SIZE>> =
    StaticCell::new();
static CMDC: StaticCell<Channel<ThreadModeRawMutex, Telecommand, CMD_CHANNEL_BUF_SIZE>> =
    StaticCell::new();

// static peripherals
static I2C: StaticCell<Mutex<ThreadModeRawMutex, I2c<'static, Async, Master>>> = StaticCell::new();

// can configuration
const RX_BUF_SIZE: usize = 500;
const TX_BUF_SIZE: usize = 30;

static RX_BUF: StaticCell<RxFdBuf<RX_BUF_SIZE>> = StaticCell::new();
static TX_BUF: StaticCell<TxFdBuf<TX_BUF_SIZE>> = StaticCell::new();

/// Watchdog petting task
#[embassy_executor::task]
async fn petter(mut watchdog: IndependentWatchdog<'static, IWDG>) {
    loop {
        watchdog.pet();
        Timer::after_micros(WATCHDOG_PETTING_INTERVAL_US.into()).await;
    }
}

// Internal temperature tm task
#[embassy_executor::task]
pub async fn internal_temp_thread(
    tm_sender: DynamicSender<'static, EpsTMContainer>,
    mut temp_receiver: DynReceiver<'static, i16>,
) {
    const INTERNAL_TEMP_LOOP_LEN: Duration = Duration::from_secs(2);
    let mut ticker = Ticker::every(INTERNAL_TEMP_LOOP_LEN);
    loop {
        let container =
            EpsTMContainer::new(&tm::InternalTemperature, &temp_receiver.get().await).unwrap();
        tm_sender.send(container).await;

        ticker.next().await;
    }
}

// tm sending task
#[embassy_executor::task]
pub async fn tm_thread(
    mut can_sender: BufferedFdCanSender,
    tm_channel: Receiver<'static, ThreadModeRawMutex, EpsTMContainer, TM_CHANNEL_BUF_SIZE>,
) {
    loop {
        let container = tm_channel.receive().await;
        match FdFrame::new_standard(container.id(), container.bytes()) {
            Ok(frame) => can_sender.write(frame).await,
            Err(e) => error!("error constructing can message: {}", e),
        }
    }
}

// tc receiving task
#[embassy_executor::task]
pub async fn tc_thread(
    can_receiver: BufferedFdCanReceiver,
    tc_channel: Sender<'static, ThreadModeRawMutex, Telecommand, TM_CHANNEL_BUF_SIZE>,
) {
    loop {
        match can_receiver.receive().await {
            Ok(envelope) => match Telecommand::read(envelope.frame.data()) {
                Ok((_, cmd)) => tc_channel.send(cmd).await,
                Err(_) => error!("error parsing tc"),
            },
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

    const FW_VERSION: &str = env!("FW_VERSION");
    const FW_HASH: &str = env!("FW_HASH");

    info!(
        "Launching: FW version={} hash={}",
        FW_VERSION,
        FW_HASH
    );

    // unleash independent watchdog
    let mut watchdog = IndependentWatchdog::new(p.IWDG, WATCHDOG_TIMEOUT_US);
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
    let mut adc_config = AdcConfig::default();
    adc_config.resolution = Some(embassy_stm32::adc::Resolution::BITS12);
    // 16x oversampling
    adc_config.oversampling_ratio = Some(embassy_stm32::adc::Ovsr::MUL16); // 2^n oversampling steps: 2^3 = 16
    adc_config.oversampling_shift = Some(embassy_stm32::adc::Ovss::SHIFT4); // right shift of oversampling reg, usually n+1: avg = sum >> n+1
    adc_config.oversampling_enable = Some(true); // enable oversampling feature

    let adc_periph = Adc::new_with_config(p.ADC1, adc_config);

    let internal_temperature_watch = ITW.init(Watch::new());
    let bat_1_watch = B1W.init(Watch::new());
    let bat_2_watch = B2W.init(Watch::new());
    let aux_pwr_watch = APW.init(Watch::new());

    let bat_1_channel = AdcCtrlChannel::new(
        p.PA4.degrade_adc(),
        bat_1_watch.dyn_sender(),
        adc::conversion::calculate_voltage_10mv,
    );
    let bat_2_channel = AdcCtrlChannel::new(
        p.PA3.degrade_adc(),
        bat_2_watch.dyn_sender(),
        adc::conversion::calculate_voltage_10mv,
    );
    let aux_pwr_channel = AdcCtrlChannel::new(
        p.PA2.degrade_adc(),
        aux_pwr_watch.dyn_sender(),
        adc::conversion::calculate_voltage_10mv,
    );

    let adc = AdcCtrl::new(
        adc_periph,
        p.DMA1_CH1,
        internal_temperature_watch.dyn_sender(),
        [bat_1_channel, bat_2_channel, aux_pwr_channel],
    );

    // TM channel setup
    let tm_channel = TMC.init(Channel::new());
    let cmd_channel = CMDC.init(Channel::new());

    // first battery
    let bat_1_tmp = Tmp100::new(temp_sensor_i2c, Resolution::BITS12, Addr0State::Floating)
        .await
        .inspect_err(|e| error!("could not establish connection to bat 1 temp sensor: {}", e));
    let bat_1 = Battery::new(
        bat_1_tmp.ok(),
        bat_1_watch.dyn_receiver().unwrap(),
        tm_channel.dyn_sender(),
        &tm::Bat1Temperature,
        &tm::Bat1Voltage,
    )
    .await;

    // second battery
    let bat_2_tmp = Tmp100::new(temp_sensor_i2c, Resolution::BITS12, Addr0State::Floating)
        .await
        .inspect_err(|e| error!("could not establish connection to bat 2 temp sensor: {}", e));
    let bat_2 = Battery::new(
        bat_2_tmp.ok(),
        bat_2_watch.dyn_receiver().unwrap(),
        tm_channel.dyn_sender(),
        &tm::Bat2Temperature,
        &tm::Bat2Voltage,
    )
    .await;

    // aux power
    let aux_pwr = AuxPwr::new(
        aux_pwr_watch.dyn_receiver().unwrap(),
        tm_channel.dyn_sender(),
    )
    .await;

    // debug leds not used at the moment (might disrupt can)
    let _led1 = Output::new(p.PB7, Level::Low, Speed::Low);
    let _led2 = Output::new(p.PB8, Level::Low, Speed::Low);

    // set can standby pin to low
    let _can_standby = Output::new(p.PA10, Level::Low, Speed::Low);
    //let _can_2_standby = Output::new(p.PB2, Level::High, Speed::Low);

    // -- CAN configuration
    let mut can_configurator =
        CanPeriphConfig::new(CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs));

    can_configurator
        .add_receive_topic(telecommands::Telecommand.id())
        .unwrap();

    let can_interface = can_configurator.activate(
        TX_BUF.init(TxFdBuf::<TX_BUF_SIZE>::new()),
        RX_BUF.init(RxFdBuf::<RX_BUF_SIZE>::new()),
    );

    // Main control loop setup
    let control_loop = ControlLoop::spawn(
        source_flip_flop,
        sink_ctrl,
        cmd_channel.dyn_receiver(),
        tm_channel.dyn_sender(),
    );

    spawner.must_spawn(petter(watchdog));

    spawner.must_spawn(adc::adc_thread(adc));
    spawner.must_spawn(control_loop::ctrl_thread(control_loop));

    spawner.must_spawn(battery::battery_thread(bat_1));
    spawner.must_spawn(battery::battery_thread(bat_2));
    spawner.must_spawn(aux_pwr::aux_pwr_thread(aux_pwr));

    spawner.must_spawn(internal_temp_thread(
        tm_channel.dyn_sender(),
        internal_temperature_watch.dyn_receiver().unwrap(),
    ));
    spawner.must_spawn(tm_thread(can_interface.writer(), tm_channel.receiver()));
    spawner.must_spawn(tc_thread(can_interface.reader(), cmd_channel.sender()));

    // wait until all other threads finished (never)
    core::future::pending::<()>().await;
}
