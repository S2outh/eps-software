#![no_std]
#![no_main]

use core::cmp::min;

use embassy_time::Timer;
use embedded_hal::adc::Channel;
use rodos_can_interface::{RodosCanInterface, receiver::RodosCanReceiver, sender::RodosCanSender};
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_stm32::{
    adc::{Adc, AdcChannel}, bind_interrupts, can::{self, CanConfigurator, RxBuf, TxBuf}, gpio::{Input, Level, Output, Speed}, mode::Async, peripherals::*, rcc::{self, mux::Fdcansel}, usart::{self, Uart, UartRx, UartTx}, Config
};
use embedded_io_async::Write;
use heapless::Vec;

use {defmt_rtt as _, panic_probe as _};

use static_cell::StaticCell;

const RODOS_DEVICE_ID: u8 = 0x01;
const RODOS_REC_TOPIC_ID: u16 = 4000;
const RODOS_SND_TOPIC_ID: u16 = 4001;

const RODOS_MAX_RAW_MSG_LEN: usize = 247;

const RX_BUF_SIZE: usize = 500;
const TX_BUF_SIZE: usize = 30;

static RX_BUF: StaticCell<embassy_stm32::can::RxBuf<RX_BUF_SIZE>> = StaticCell::new();
static TX_BUF: StaticCell<embassy_stm32::can::TxBuf<TX_BUF_SIZE>> = StaticCell::new();

// bin can interrupts
bind_interrupts!(struct Irqs {
    TIM16_FDCAN_IT0 => can::IT0InterruptHandler<FDCAN1>;
    TIM17_FDCAN_IT1 => can::IT1InterruptHandler<FDCAN1>;
    USART3_4_5_6_LPUART1 => usart::InterruptHandler<USART5>;
    // USART2_LPUART2 => usart::InterruptHandler<USART2>;
});

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
    let config = Config::default();
    // config.rcc = get_rcc_config();
    let p = embassy_stm32::init(config);
    info!("Launching");

    // let mut test_pin = Output::new(p.PA5, Level::Low, Speed::Medium);
    let mut bat_1_sw = Output::new(p.PB3, Level::Low, Speed::Medium);
    let mut bat_1_clk = Output::new(p.PB6, Level::Low, Speed::Medium);
    let mut adc = Adc::new(p.ADC1);
    let mut bat_1_adc_ch = p.PA4.degrade_adc();
    let bat_1_stat = Input::new(p.PC14, embassy_stm32::gpio::Pull::None);

    let mut led1 = Output::new(p.PB7, Level::High, Speed::Medium);
    let mut led2 = Output::new(p.PB8, Level::Low, Speed::Medium);

    let mut counter = 4;

    // -- CAN configuration
    // let (can_reader, can_sender, _active_instance) = RodosCanInterface::new(
    //     CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs),
    //     TX_BUF.init(TxBuf::<TX_BUF_SIZE>::new()),
    //     RX_BUF.init(RxBuf::<RX_BUF_SIZE>::new()),
    //     1_000_000,
    //     RODOS_DEVICE_ID,
    //     &[(RODOS_REC_TOPIC_ID, None)], // Some(0x46)
    // )
    // .split::<4, RODOS_MAX_RAW_MSG_LEN>();

    // set can standby pin to low
    // let _can_standby = Output::new(p.PA10, Level::Low, Speed::Low);

    loop {
        led1.set_level((counter & 1 == 1).into());
        led2.set_level(((counter >> 1) & 1 == 1).into());
        info!("current state: {}", bat_1_stat.get_level());
        info!("current voltage: {}", adc.blocking_read(&mut bat_1_adc_ch));
        counter -= 1;
        if counter < 0 {
            bat_1_sw.toggle();
            bat_1_clk.set_high();
            Timer::after_millis(1).await;
            bat_1_clk.set_low();
            counter = 15;
        }
        Timer::after_secs(1).await;
    }
}
