#![no_std]
#![no_main]

#[allow(dead_code)]

mod battery;

use battery::{Battery, tmp100_drv::*, d_flip_flop::DFlipFlop};
use embassy_time::Timer;

use core::cell::RefCell;

use rodos_can_interface::{RodosCanInterface, receiver::RodosCanReceiver, sender::RodosCanSender};
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    adc::{Adc, AdcChannel, SampleTime}, bind_interrupts, can::{self, CanConfigurator, RxBuf, TxBuf}, gpio::{Input, Level, Output, Speed}, i2c::{self, I2c}, peripherals::{self, FDCAN1, IWDG}, rcc::{self, mux::Fdcansel}, time::Hertz, wdg::IndependentWatchdog, Config
};
// use embedded_io_async::Write;


use {defmt_rtt as _, panic_probe as _};

use static_cell::StaticCell;

const RODOS_DEVICE_ID: u8 = 0x02;
const RODOS_CMD_TOPIC_ID: u16 = 8000;
// const RODOS_SND_TOPIC_ID: u16 = 4001;
// 
 const RODOS_MAX_RAW_MSG_LEN: usize = 25;
// 
const RX_BUF_SIZE: usize = 500;
const TX_BUF_SIZE: usize = 30;

static RX_BUF: StaticCell<embassy_stm32::can::RxBuf<RX_BUF_SIZE>> = StaticCell::new();
static TX_BUF: StaticCell<embassy_stm32::can::TxBuf<TX_BUF_SIZE>> = StaticCell::new();

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
    rcc_config.hsi = true;
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
    let mut p = embassy_stm32::init(config);
    info!("Launching");

    // independent watchdog with timeout 300 MS
    // let mut watchdog = IndependentWatchdog::new(p.IWDG, 300_000);
    // watchdog.unleash();

    // let mut test_pin = Output::new(p.PA5, Level::Low, Speed::Medium);
    let mut adc = Adc::new(p.ADC1);
    adc.set_resolution(embassy_stm32::adc::Resolution::BITS12);
    let mut bat_1_adc_ch = p.PA4.degrade_adc();
    let mut vbat = adc.enable_vbat().degrade_adc();
    let mut vrefint = adc.enable_vrefint().degrade_adc();
    let mut vtemp = adc.enable_temperature().degrade_adc();
    let mut measurements = [0u16; 4];
    // lots of testing to do here
    adc.read(
        &mut p.DMA1_CH2,
        [
            (&mut bat_1_adc_ch, SampleTime::CYCLES160_5),
            (&mut vbat, SampleTime::CYCLES160_5),
            (&mut vrefint, SampleTime::CYCLES160_5),
            (&mut vtemp, SampleTime::CYCLES160_5),
        ]
        .into_iter(),
        &mut measurements
    ).await;
    
    let temp_sensor_i2c = RefCell::new(I2c::new(p.I2C2, p.PA7, p.PA6, Irqs, p.DMA1_CH1, p.DMA1_CH2, Hertz::khz(400), i2c::Config::default()));
    let bat_1_tmp = Tmp100::new(&temp_sensor_i2c, Resolution::Res9Bit, Addr0State::Floating).await.unwrap();
    let bat_1_enable = DFlipFlop::new(p.PB3, p.PB6);
    let bat_1_stat = Input::new(p.PC14, embassy_stm32::gpio::Pull::None);
    let bat_1 = Battery::new(bat_1_tmp, bat_1_enable, bat_1_stat).await;

    // these led's turn off can
    let _led1 = Output::new(p.PB7, Level::Low, Speed::Medium);
    let _led2 = Output::new(p.PB8, Level::Low, Speed::Medium);

    // let mut counter = 4;

    // -- CAN configuration
    let mut rodos_can_configurator = RodosCanInterface::new(
        CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs),
        RODOS_DEVICE_ID,
    );

    rodos_can_configurator
        .set_bitrate(1_000_000)
        .add_receive_topic(RODOS_CMD_TOPIC_ID, None).unwrap();

    let (mut can_reader, _can_sender, _active_instance) = rodos_can_configurator.split_buffered::<16, RODOS_MAX_RAW_MSG_LEN, TX_BUF_SIZE, RX_BUF_SIZE>(
        TX_BUF.init(TxBuf::<TX_BUF_SIZE>::new()),
        RX_BUF.init(RxBuf::<RX_BUF_SIZE>::new()),
    );

    // set can standby pin to low
    let _can_standby = Output::new(p.PA10, Level::Low, Speed::Low);
    let _can_standby2 = Output::new(p.PB2, Level::High, Speed::Low);

    loop {
        // led2.toggle();
        // info!("current state: {}", bat_1_stat.get_level());
        // info!("current voltage: {}", adc.blocking_read(&mut bat_1_adc_ch));
        // led1.set_level((counter & 1 == 1).into());
        // led2.set_level(((counter >> 1) & 1 == 1).into());
        // counter -= 1;
        // if counter < 0 {
        //     // bat_1_sw.toggle();
        //     // bat_1_clk.set_high();
        //     // Timer::after_millis(1).await;
        //     // bat_1_clk.set_low();
        //     counter = 4;
        // }
        //
  
        match can_reader.receive().await {
            Ok(frame) => {
                if frame.data()[0] != 0x00 { //power subsystemid
                    info!("wrong cmd");
                    continue;
                }
                info!("command: {}", frame.data()[1]);
                // payload length is an u16
                let payload_length = (frame.data()[3] as usize) << 8 | (frame.data()[2] as usize);
                info!("payload_len: {}", payload_length);
                info!("payload: {}", frame.data()[4..4+payload_length]);

                let _ = frame.device();
            }
            Err(e) => error!("error in frame! {}", e),
        };
    }
}
