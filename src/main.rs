#![no_std]
#![no_main]
#![feature(variant_count)]
#![feature(type_alias_impl_trait)]
#![feature(iter_collect_into)]
#![feature(iterator_try_collect)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)] // This feature is incomplete but beeing used in a benign context

mod control_loop;
#[allow(dead_code)]
mod pwr_src;
mod sensor_threads;

use control_loop::ControlLoop;
use cortex_m::peripheral::SCB;
use defmt::{error, expect, info};
use pwr_src::{d_flip_flop::DFlipFlop, sink_ctrl::SinkCtrl, tmp100_drv::*};

use embassy_executor::Spawner;
use embassy_stm32::{
    Config, bind_interrupts,
    can::{
        self, CanConfigurator, RxFdBuf, TxFdBuf,
    },
    dma,
    exti::{self, ExtiInput},
    gpio::{Level, Output, Pull, Speed},
    i2c::{self, I2c, Master},
    interrupt,
    mode::Async,
    peripherals::{self, DMA1_CH2, DMA1_CH3, FDCAN1, IWDG},
    rcc::{self, mux::Fdcansel},
    time::mhz,
    wdg::IndependentWatchdog,
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    mutex::Mutex,
};
use embassy_time::Timer;
use south_common::{
    chell::ChellDefinition,
    configs::can_config::CanPeriphConfig,
    definitions::{internal_msgs, telemetry::eps as tm},
    gen_obdh_types,
    obdh::EmptyFunc,
    types::eps::{EPSCommand, FlipFlopInput},
};
use static_cell::StaticCell;

use crate::pwr_src::ina3221_drv::Ina;

use {defmt_rtt as _, panic_probe as _};

// bind interrupts
bind_interrupts!(struct Irqs {
    I2C2_3 => i2c::EventInterruptHandler<peripherals::I2C2>, i2c::ErrorInterruptHandler<peripherals::I2C2>;
    DMA1_CHANNEL2_3 => dma::InterruptHandler<DMA1_CH2>, dma::InterruptHandler<DMA1_CH3>;

    EXTI4_15 => exti::InterruptHandler<interrupt::typelevel::EXTI4_15>;

    TIM16_FDCAN_IT0 => can::IT0InterruptHandler<FDCAN1>;
    TIM17_FDCAN_IT1 => can::IT1InterruptHandler<FDCAN1>;

    // TIM16_FDCAN_IT0 => can::IT0InterruptHandler<FDCAN2>;
    // TIM17_FDCAN_IT1 => can::IT1InterruptHandler<FDCAN2>;
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
gen_obdh_types!(Eps, tm, cmd => EPSCommand);

// internal messaging channels
static COM_CHANNELS: EpsComChannels = EpsComChannels::new(3);

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

// control loop task
#[embassy_executor::task]
pub async fn ctrl_thread(mut control_loop: ControlLoop<'static>) -> ! {
    control_loop.run().await
}

// Kill eps if remove before flight is put back in
#[embassy_executor::task]
async fn safety(mut safety_off: ExtiInput<'static, Async>) {
    safety_off.wait_for_low().await;
    SCB::sys_reset();
}

#[embassy_executor::task]
pub async fn can_receiver_task(mut can_receiver: EpsCanReceiver) -> ! {
    can_receiver.run().await
}

#[embassy_executor::task]
pub async fn can_sender_task(mut can_sender: EpsCanSender) -> ! {
    can_sender.run().await
}

/// program entry
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    config.rcc = get_rcc_config();
    let p = embassy_stm32::init(config);

    const FW_VERSION: &str = env!("FW_VERSION");
    const FW_HASH: &str = env!("FW_HASH");

    info!("Launching: FW version={} hash={}", FW_VERSION, FW_HASH);

    // remove before flight pin
    let mut safety_off = ExtiInput::new(p.PB4, p.EXTI4, Pull::None, Irqs);
    safety_off.wait_for_high().await;

    // unleash independent watchdog
    let mut watchdog = IndependentWatchdog::new(p.IWDG, WATCHDOG_TIMEOUT_US);
    watchdog.unleash();

    // i2c setup
    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = mhz(1);
    let i2c = I2C.init(Mutex::new(I2c::new(
        p.I2C2, p.PA7, p.PA6, p.DMA1_CH2, p.DMA1_CH3, Irqs, i2c_config,
    )));

    // flip flop
    let source_flip_flop = DFlipFlop::new(
        p.PB6, p.PC14, // bat 1
        p.PB7, p.PB9, // bat 2
        p.PB8, p.PC15, // aux pwr
        p.PB5,  // clk
    );

    // sink ctrl
    let sink_ctrl = SinkCtrl::new(
        p.PA5,  // carrier
        p.PA3,  // umbilical
        p.PC6,  // rocketlst 1
        p.PA9,  // rocketlst 2
        p.PA8,  // SensorLower
        p.PA15, // RocketHD
        p.PA4,  // Pyro
    );

    // first battery temp sensor
    let bat_1_tmp = Tmp100::new(i2c, Resolution::BITS12, pwr_src::tmp100_drv::A0::Floating)
        .await
        .inspect_err(|e| error!("could not establish connection to bat 1 temp sensor: {}", e));

    // second battery temp sensor
    let bat_2_tmp = Tmp100::new(i2c, Resolution::BITS12, pwr_src::tmp100_drv::A0::Low)
        .await
        .inspect_err(|e| error!("could not establish connection to bat 2 temp sensor: {}", e));

    // ina
    let ina = expect!(
        Ina::new(i2c, pwr_src::ina3221_drv::A0::Ground).await,
        "could not establish connection to ina"
    );

    // debug leds not used at the moment
    let _led1 = Output::new(p.PB3, Level::Low, Speed::Low);

    // -- CAN configuration

    // can 1 configuration
    let mut can_configurator =
        CanPeriphConfig::new(CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs));

    // can 2 configuration
    // let mut can_configurator =
    //     CanPeriphConfig::new(CanConfigurator::new(p.FDCAN2, p.PB0, p.PB1, Irqs));

    can_configurator
        .add_receive_topic(internal_msgs::Telecommand.id())
        .unwrap();

    let can_instance = can_configurator.activate(
        TX_BUF.init(TxFdBuf::<TX_BUF_SIZE>::new()),
        RX_BUF.init(RxFdBuf::<RX_BUF_SIZE>::new()),
    );

    // set can standby pin to low
    let _can_1_standby = Output::new(p.PA10, Level::Low, Speed::Low);
    // let _can_2_standby = Output::new(p.PB2, Level::Low, Speed::Low);

    // Setup can sender and receiver runners
    let can_receiver = EpsCanReceiver::new(can_instance.reader(), &COM_CHANNELS, EmptyFunc);

    let can_sender = EpsCanSender::new(can_instance.writer(), &COM_CHANNELS);

    // Main control loop setup
    let control_loop = ControlLoop::spawn(
        source_flip_flop,
        sink_ctrl,
        COM_CHANNELS.get_tc_receiver(),
        COM_CHANNELS.get_tm_sender(),
    );

    spawner.spawn(petter(watchdog).unwrap());
    spawner.spawn(safety(safety_off).unwrap());

    spawner.spawn(ctrl_thread(control_loop).unwrap());

    if let Ok(tmp) = bat_1_tmp {
        spawner.spawn(
            sensor_threads::bat_temp_thread(
                COM_CHANNELS.get_tm_sender(),
                tmp,
                &tm::Bat1Temperature,
                FlipFlopInput::Bat1,
            )
            .unwrap(),
        );
    }

    if let Ok(tmp) = bat_2_tmp {
        spawner.spawn(
            sensor_threads::bat_temp_thread(
                COM_CHANNELS.get_tm_sender(),
                tmp,
                &tm::Bat2Temperature,
                FlipFlopInput::Bat2,
            )
            .unwrap(),
        );
    }

    spawner.spawn(sensor_threads::ina_thread(COM_CHANNELS.get_tm_sender(), ina).unwrap());

    spawner.spawn(can_sender_task(can_sender).unwrap());
    spawner.spawn(can_receiver_task(can_receiver).unwrap());

    // wait until all other threads finished (never)
    core::future::pending::<()>().await;
}
