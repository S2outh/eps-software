use embassy_stm32::{gpio::{Level, Output, Pin, Speed}, Peri};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::Timer;


pub struct DFlipFlop<'a, 'd> {
    d_pin: Output<'d>,
    clk_pin: &'a Mutex<NoopRawMutex, Output<'d>>,
}

impl<'a, 'd> DFlipFlop<'a, 'd> {
    pub fn new(d_periph: Peri<'d, impl Pin>, clk_pin: &'a Mutex<NoopRawMutex, Output<'d>>) -> Self {
        let d_pin = Output::new(d_periph, Level::Low, Speed::Medium);
        Self { d_pin, clk_pin }
    }
    pub async fn set(&mut self, level: Level) {
        let mut clk_pin = self.clk_pin.lock().await;
        self.d_pin.set_level(level);
        clk_pin.set_high();
        Timer::after_micros(10).await;
        clk_pin.set_low();
    }
}
