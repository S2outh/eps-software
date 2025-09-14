use embassy_stm32::gpio::{Level, Output, Pin, Speed};
use embassy_time::Timer;


pub struct DFlipFlop<'a> {
    d_pin: Output<'a>,
    clk_pin: Output<'a>,
}

impl<'a> DFlipFlop<'a> {
    pub fn new(d_periph: impl Pin, clk_periph: impl Pin) -> Self {
        let d_pin = Output::new(d_periph, Level::Low, Speed::Medium);
        let clk_pin = Output::new(clk_periph, Level::Low, Speed::Medium);
        Self { d_pin, clk_pin }
    }
    pub async fn set(&mut self, level: Level) {
        self.d_pin.set_level(level);
        self.clk_pin.set_high();
        Timer::after_millis(10).await;
        self.clk_pin.set_low();
    }
}
