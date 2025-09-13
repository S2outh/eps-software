#![no_std]
#![no_main]

mod tmp100_drv;

use embassy_time::Timer;
// use rodos_can_interface::{RodosCanInterface, receiver::RodosCanReceiver, sender::RodosCanSender};
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    adc::Adc, bind_interrupts, can::{self, CanConfigurator, RxBuf, TxBuf}, gpio::{Input, Level, Output, Speed}, i2c::{self, I2c}, interrupt::typelevel::I2C2_3, peripherals, rcc::{self, mux::Fdcansel}, time::Hertz, Config
};
// use embedded_io_async::Write;
// use heapless::Vec;

use crate::tmp100_drv::{Addr0State, Resolution, Tmp100};

use {defmt_rtt as _, panic_probe as _};

// use static_cell::StaticCell;

// const RODOS_DEVICE_ID: u8 = 0x01;
// const RODOS_REC_TOPIC_ID: u16 = 4000;
// const RODOS_SND_TOPIC_ID: u16 = 4001;
// 
// const RODOS_MAX_RAW_MSG_LEN: usize = 247;
// 
// const RX_BUF_SIZE: usize = 500;
// const TX_BUF_SIZE: usize = 30;

// static RX_BUF: StaticCell<embassy_stm32::can::RxBuf<RX_BUF_SIZE>> = StaticCell::new();
// static TX_BUF: StaticCell<embassy_stm32::can::TxBuf<TX_BUF_SIZE>> = StaticCell::new();

// bin can interrupts
bind_interrupts!(struct Irqs {
    I2C2_3 => i2c::EventInterruptHandler<peripherals::I2C2>, i2c::ErrorInterruptHandler<peripherals::I2C2>;
//     TIM16_FDCAN_IT0 => can::IT0InterruptHandler<FDCAN1>;
//     TIM17_FDCAN_IT1 => can::IT1InterruptHandler<FDCAN1>;
});

/// config rcc for higher sysclock and fdcan periph clock to make sure
/// all messages can be received without package drop
// fn get_rcc_config() -> rcc::Config {
//     let mut rcc_config = rcc::Config::default();
//     rcc_config.hsi = true;
//     rcc_config.sys = rcc::Sysclk::PLL1_R;
//     rcc_config.pll = Some(rcc::Pll {
//         source: rcc::PllSource::HSI,
//         prediv: rcc::PllPreDiv::DIV1,
//         mul: rcc::PllMul::MUL8,
//         divp: None,
//         divq: Some(rcc::PllQDiv::DIV2),
//         divr: Some(rcc::PllRDiv::DIV2),
//     });
//     rcc_config.mux.fdcansel = Fdcansel::PLL1_Q;
//     rcc_config
// }

/// program entry
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let config = Config::default();
    // config.rcc = get_rcc_config();
    let p = embassy_stm32::init(config);
    info!("Launching");

    // let mut test_pin = Output::new(p.PA5, Level::Low, Speed::Medium);
    // let mut bat_1_sw = Output::new(p.PB3, Level::Low, Speed::Medium);
    // let mut bat_1_clk = Output::new(p.PB6, Level::Low, Speed::Medium);
    // let mut adc = Adc::new(p.ADC1);
    // let mut bat_1_adc_ch = p.PA4;//.degrade_adc();
    // let bat_1_stat = Input::new(p.PC14, embassy_stm32::gpio::Pull::None);
    
    let temp_sensor_i2c = I2c::new(p.I2C2, p.PA7, p.PA6, Irqs, p.DMA1_CH1, p.DMA1_CH2, Hertz::khz(400), i2c::Config::default());
    let mut bat_1_tmp = Tmp100::new(temp_sensor_i2c, Resolution::Res9Bit, Addr0State::Floating).await.unwrap();

    // let mut led1 = Output::new(p.PB7, Level::High, Speed::Medium);
    // let mut led2 = Output::new(p.PB8, Level::Low, Speed::Medium);

    // let mut counter = 4;

    // -- CAN configuration
    // let mut rodos_can_configurator = RodosCanInterface::new(
    //     CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs),
    //     RODOS_DEVICE_ID,
    // );

    // rodos_can_configurator
    //     .set_bitrate(1_000_000)
    //     .add_receive_topic(RODOS_REC_TOPIC_ID, None).unwrap();

    // let (can_reader, can_sender, _active_instance) = rodos_can_configurator.split_buffered::<4, RODOS_MAX_RAW_MSG_LEN, TX_BUF_SIZE, RX_BUF_SIZE>(
    //     TX_BUF.init(TxBuf::<TX_BUF_SIZE>::new()),
    //     RX_BUF.init(RxBuf::<RX_BUF_SIZE>::new()),
    // );

    // set can standby pin to low
    // let _can_standby = Output::new(p.PA10, Level::Low, Speed::Low);

    loop {
        let tmp_tenth_deg = bat_1_tmp.read_temp().await.unwrap();
        info!("tmp: {}", tmp_tenth_deg);
    //     led1.set_level((counter & 1 == 1).into());
    //     led2.set_level(((counter >> 1) & 1 == 1).into());
    //     info!("current state: {}", bat_1_stat.get_level());
    //     info!("current voltage: {}", adc.blocking_read(&mut bat_1_adc_ch));
    //     counter -= 1;
    //     if counter < 0 {
    //         bat_1_sw.toggle();
    //         bat_1_clk.set_high();
    //         Timer::after_millis(1).await;
    //         bat_1_clk.set_low();
    //         counter = 15;
    //     }
        Timer::after_secs(1).await;
    }
}
