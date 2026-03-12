#![no_std]
#![no_main]
#![feature(variant_count)]
#![feature(type_alias_impl_trait)]
#![feature(iter_collect_into)]
#![feature(iterator_try_collect)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)] // This feature is incomplete but beeing used in a benign context

mod control_loop;
mod sensor_threads;
#[allow(dead_code)]
mod pwr_src;

use defmt::{error, info, expect};
use control_loop::ControlLoop;
use pwr_src::{
    tmp100_drv::*,
    d_flip_flop::DFlipFlop,
    sink_ctrl::SinkCtrl,
};

use embassy_executor::Spawner;
use embassy_stm32::{
    Config,
    bind_interrupts,
    can::{
        self, BufferedFdCanReceiver, BufferedFdCanSender, CanConfigurator, RxFdBuf, TxFdBuf,
        frame::FdFrame,
    },
    gpio::{Level, Output, Speed},
    i2c::{self, I2c, Master},
    mode::Async,
    peripherals::{self, FDCAN2, IWDG},
    rcc::{self, mux::Fdcansel},
    time::mhz,
    wdg::IndependentWatchdog,
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    channel::{Channel, Receiver, Sender},
    mutex::Mutex,
};
use embassy_time::Timer;
use south_common::{
    tmtc_system::{TMValue, TelemetryDefinition, fd_compat_telemetry_container},
    configs::can_config::CanPeriphConfig,
    definitions::internal_msgs,
    definitions::telemetry::eps as tm, types::Telecommand,
};
use static_cell::StaticCell;

use crate::pwr_src::ina3221_drv::Ina;

use {defmt_rtt as _, panic_probe as _};

// bind interrupts
bind_interrupts!(struct Irqs {
    I2C2_3 => i2c::EventInterruptHandler<peripherals::I2C2>, i2c::ErrorInterruptHandler<peripherals::I2C2>;

    // TIM16_FDCAN_IT0 => can::IT0InterruptHandler<FDCAN1>;
    // TIM17_FDCAN_IT1 => can::IT1InterruptHandler<FDCAN1>;
    
    TIM16_FDCAN_IT0 => can::IT0InterruptHandler<FDCAN2>;
    TIM17_FDCAN_IT1 => can::IT1InterruptHandler<FDCAN2>;
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
type EpsTMContainer = fd_compat_telemetry_container!(tm);

// static concurrency sync management types
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

// tm sending task
#[embassy_executor::task]
pub async fn tm_thread(
    mut can_sender: BufferedFdCanSender,
    tm_channel: Receiver<'static, ThreadModeRawMutex, EpsTMContainer, TM_CHANNEL_BUF_SIZE>,
) {
    loop {
        let container = tm_channel.receive().await;
        match FdFrame::new_standard(container.id(), container.fd_bytes()) {
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

    // i2c setup
    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = mhz(1);
    let i2c = I2C.init(Mutex::new(I2c::new(
        p.I2C2, p.PA7, p.PA6, Irqs, p.DMA1_CH2, p.DMA1_CH3, i2c_config,
    )));

    // flip flop
    let source_flip_flop = DFlipFlop::new(
        p.PB6, p.PC14,  // bat 1
        p.PB7, p.PB9,   // bat 2
        p.PB8, p.PC15,  // aux pwr
        p.PB5,          // clk
    );

    // sink ctrl
    let sink_ctrl = SinkCtrl::new(
        p.PA5, // carrier
        p.PA3, // umbilical
        p.PC6, // rocketlst 1
        p.PA9, // rocketlst 2
        p.PA8, // SensorLower
        p.PA15,// RocketHD
        p.PA4  // BackupSink
    );

    // TM channel setup
    let tm_channel = TMC.init(Channel::new());
    let cmd_channel = CMDC.init(Channel::new());

    // first battery temp sensor
    let bat_1_tmp = Tmp100::new(i2c, Resolution::BITS12, pwr_src::tmp100_drv::A0::Floating)
        .await
        .inspect_err(|e| error!("could not establish connection to bat 1 temp sensor: {}", e));

    // second battery temp sensor
    let bat_2_tmp = Tmp100::new(i2c, Resolution::BITS12, pwr_src::tmp100_drv::A0::Low)
        .await
        .inspect_err(|e| error!("could not establish connection to bat 2 temp sensor: {}", e));

    // ina
    let ina = expect!(Ina::new(i2c, pwr_src::ina3221_drv::A0::Ground).await, "could not establish connection to ina");

    // debug leds not used at the moment
    let _led1 = Output::new(p.PB3, Level::Low, Speed::Low);

    // -- CAN configuration

    // can 1 configuration
    // let mut can_configurator =
    //     CanPeriphConfig::new(CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs));
    
    // can 2 configuration
    let mut can_configurator =
        CanPeriphConfig::new(CanConfigurator::new(p.FDCAN2, p.PB0, p.PB1, Irqs));

    can_configurator
        .add_receive_topic(internal_msgs::Telecommand.id())
        .unwrap();

    let can_interface = can_configurator.activate(
        TX_BUF.init(TxFdBuf::<TX_BUF_SIZE>::new()),
        RX_BUF.init(RxFdBuf::<RX_BUF_SIZE>::new()),
    );

    // set can standby pin to low
    // let _can_1_standby = Output::new(p.PA10, Level::Low, Speed::Low);
    let _can_2_standby = Output::new(p.PB2, Level::Low, Speed::Low);

    // Main control loop setup
    let control_loop = ControlLoop::spawn(
        source_flip_flop,
        sink_ctrl,
        cmd_channel.dyn_receiver(),
        tm_channel.dyn_sender(),
    );

    spawner.must_spawn(petter(watchdog));

    spawner.must_spawn(control_loop::ctrl_thread(control_loop));

    if let Ok(tmp) = bat_1_tmp {
        spawner.must_spawn(sensor_threads::bat_temp_thread(tm_channel.dyn_sender(), tmp, &tm::Bat1Temperature));
    }

    if let Ok(tmp) = bat_2_tmp {
        spawner.must_spawn(sensor_threads::bat_temp_thread(tm_channel.dyn_sender(), tmp, &tm::Bat2Temperature));
    }

    spawner.must_spawn(sensor_threads::ina_thread(tm_channel.dyn_sender(), ina));

    spawner.must_spawn(tm_thread(can_interface.writer(), tm_channel.receiver()));
    spawner.must_spawn(tc_thread(can_interface.reader(), cmd_channel.sender()));

    // wait until all other threads finished (never)
    core::future::pending::<()>().await;
}
