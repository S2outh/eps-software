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
mod util;

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
    adc::{Adc, AdcChannel},
    bind_interrupts,
    can::{self, CanConfigurator},
    gpio::{Level, Output, Speed},
    i2c::{self, I2c, Master},
    mode::Async,
    peripherals::{self, DMA1_CH1, FDCAN1, IWDG},
    rcc::{self, mux::Fdcansel},
    time::khz,
    wdg::IndependentWatchdog,
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex, watch::Watch};
use embassy_time::Timer;
use static_cell::StaticCell;

use crate::adc::AdcCtrlChannel;

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
// static concurrency sync management types
static ITW: StaticCell<Watch<ThreadModeRawMutex, i16, 1>> = StaticCell::new();
static B1W: StaticCell<Watch<ThreadModeRawMutex, i16, 1>> = StaticCell::new();
static B2W: StaticCell<Watch<ThreadModeRawMutex, i16, 1>> = StaticCell::new();
static APW: StaticCell<Watch<ThreadModeRawMutex, i16, 1>> = StaticCell::new();

// static peripherals
static I2C: StaticCell<Mutex<ThreadModeRawMutex, I2c<'static, Async, Master>>> = StaticCell::new();

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

    let internal_temperature_watch = ITW.init(Watch::<ThreadModeRawMutex, i16, 1>::new());
    let bat_1_watch = B1W.init(Watch::<ThreadModeRawMutex, i16, 1>::new());
    let bat_2_watch = B2W.init(Watch::<ThreadModeRawMutex, i16, 1>::new());
    let aux_pwr_watch = APW.init(Watch::<ThreadModeRawMutex, i16, 1>::new());

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

    // first battery
    let bat_1_tmp = Tmp100::new(temp_sensor_i2c, Resolution::BITS12, Addr0State::Floating)
        .await
        .inspect_err(|e| error!("could not establish connection to bat 1 temp sensor: {}", e));
    let bat_1 = Battery::new(bat_1_tmp.ok(), bat_1_watch.receiver().unwrap().as_dyn()).await;

    // aux power
    let aux_pwr = AuxPwr::new(aux_pwr_watch.receiver().unwrap().as_dyn()).await;

    // debug leds not used at the moment (might disrupt can)
    let _led1 = Output::new(p.PB7, Level::Low, Speed::Low);
    let _led2 = Output::new(p.PB8, Level::Low, Speed::Low);

    // set can standby pin to low
    let _can_standby = Output::new(p.PA10, Level::Low, Speed::Low);
    //let _can_2_standby = Output::new(p.PB2, Level::High, Speed::Low);

    // Main control loop setup
    let can_configurator = CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs);
    let mut control_loop = ControlLoop::spawn(
        source_flip_flop,
        sink_ctrl,
        bat_1,
        aux_pwr,
        internal_temperature_watch.receiver().unwrap().as_dyn(),
        can_configurator,
    );

    spawner.must_spawn(petter(watchdog));
    spawner.must_spawn(adc_thread(adc));

    loop {
        control_loop.run().await
    }
}
