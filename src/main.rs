#![no_std]
#![no_main]

#[allow(dead_code)]

mod pwr_src;
mod control_loop;
mod adc;

use pwr_src::{battery::{Battery, tmp100_drv::*}, aux_pwr::AuxPwr, d_flip_flop::DFlipFlop};
use embassy_futures::join::join3;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex, watch::Watch};
use embassy_time::Timer;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    adc::{Adc, AdcChannel}, bind_interrupts, can::{self, CanConfigurator}, gpio::{Input, Level, Output, Speed}, i2c::{self, I2c}, peripherals::{self, FDCAN1, IWDG}, rcc::{self, mux::Fdcansel}, time::khz, wdg::IndependentWatchdog, Config
};


use adc::EPSAdc;

use control_loop::ControlLoop;

use crate::pwr_src::sink_ctrl::SinkCtrl;

use {defmt_rtt as _, panic_probe as _};

const ADC_LOOP_LEN_MS: u64 = 50;

// bin can interrupts
bind_interrupts!(struct Irqs {
    I2C2_3 => i2c::EventInterruptHandler<peripherals::I2C2>, i2c::ErrorInterruptHandler<peripherals::I2C2>;
    TIM16_FDCAN_IT0 => can::IT0InterruptHandler<FDCAN1>;
    TIM17_FDCAN_IT1 => can::IT1InterruptHandler<FDCAN1>;
});

/// Watchdog petting task
async fn petter(mut watchdog: IndependentWatchdog<'_, IWDG>) {
    loop {
        watchdog.pet();
        Timer::after_millis(200).await;
    }
}

/// config rcc for higher sysclock and fdcan periph clock to make sure
/// all messages can be received without package drop
fn get_rcc_config() -> rcc::Config {
    let mut rcc_config = rcc::Config::default();
    rcc_config.hsi = Some(rcc::Hsi { sys_div: rcc::HsiSysDiv::DIV1 });
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

/// program entry
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    config.rcc = get_rcc_config();
    let p = embassy_stm32::init(config);
    info!("Launching");

    // independent watchdog with timeout 300 MS
    let mut watchdog = IndependentWatchdog::new(p.IWDG, 300_000);
    watchdog.unleash();

    // ADC setup
    let adc_periph = Adc::new(p.ADC1);
    let internal_temperature_watch = Watch::<NoopRawMutex, i16, 1>::new();
    let bat_1_channel = p.PA4.degrade_adc();
    let bat_1_watch = Watch::<NoopRawMutex, i16, 1>::new();
    let bat_2_channel = p.PA3.degrade_adc();
    let bat_2_watch = Watch::<NoopRawMutex, i16, 1>::new();
    let aux_pwr_channel = p.PA2.degrade_adc();
    let aux_pwr_watch = Watch::<NoopRawMutex, i16, 1>::new();
    let mut adc = EPSAdc::new(adc_periph,
        p.DMA1_CH1,
        bat_1_channel,
        bat_2_channel,
        aux_pwr_channel,
        internal_temperature_watch.sender().as_dyn(),
        bat_1_watch.sender().as_dyn(),
        bat_2_watch.sender().as_dyn(),
        aux_pwr_watch.sender().as_dyn(),
    );
    let adc_future = adc.run(ADC_LOOP_LEN_MS);
    
    // i2c for temperature sensors
    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = khz(400);
    let temp_sensor_i2c = Mutex::new(I2c::new(p.I2C2, p.PA7, p.PA6, Irqs, p.DMA1_CH2, p.DMA1_CH3, i2c_config));

    // flip flop
    let source_flip_flop = DFlipFlop::new(p.PB3, p.PB4, p.PB5, p.PB6);

    // sink ctrl
    let sink_ctrl = SinkCtrl::new(p.PA5, p.PA9, p.PA15);

    // first battery
    let bat_1_tmp = Tmp100::new(&temp_sensor_i2c, Resolution::BITS12, Addr0State::Floating).await.unwrap();
    let bat_1_stat = Input::new(p.PC14, embassy_stm32::gpio::Pull::None);
    let bat_1 = Battery::new(bat_1_tmp, bat_1_watch.receiver().unwrap().as_dyn(), bat_1_stat).await;

    // aux power
    let aux_pwr_stat = Input::new(p.PC15, embassy_stm32::gpio::Pull::None);
    let aux_pwr = AuxPwr::new(aux_pwr_watch.receiver().unwrap().as_dyn(), aux_pwr_stat).await;

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
        can_configurator
    );
    let control_loop_future = control_loop.run();

    join3(petter(watchdog), adc_future, control_loop_future).await;
}
