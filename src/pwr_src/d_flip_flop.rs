use embassy_stm32::{gpio::{Level, Output, Pin, Speed}, Peri};
use embassy_time::Timer;

#[repr(u8)]
pub enum FlipFlopState {
    Off,
    Bat1,
    Bat2,
    AuxPwr,
}

pub struct DFlipFlop<'d> {
    d_bat_1: Output<'d>,
    d_bat_2: Output<'d>,
    d_aux_pwr: Output<'d>,
    clk: Output<'d>,
}

impl<'d> DFlipFlop<'d> {
    pub fn new(
        d_bat_1: Peri<'d, impl Pin>,
        d_bat_2: Peri<'d, impl Pin>,
        d_aux_pwr: Peri<'d, impl Pin>,
        clk: Peri<'d, impl Pin>,
        ) -> Self {
        let d_bat_1 = Output::new(d_bat_1, Level::Low, Speed::High);
        let d_bat_2 = Output::new(d_bat_2, Level::Low, Speed::High);
        let d_aux_pwr = Output::new(d_aux_pwr, Level::Low, Speed::High);
        let clk = Output::new(clk, Level::Low, Speed::High);
        Self { d_bat_1, d_bat_2, d_aux_pwr, clk }
    }
    pub async fn set(&mut self, state: FlipFlopState) {
        match state {
            FlipFlopState::Off => {
                self.d_bat_1.set_low();
                self.d_bat_2.set_low();
                self.d_aux_pwr.set_low();
            },
            FlipFlopState::Bat1 => {
                self.d_bat_1.set_high();
                self.d_bat_2.set_low();
                self.d_aux_pwr.set_low();
            },
            FlipFlopState::Bat2 => {
                self.d_bat_1.set_low();
                self.d_bat_2.set_high();
                self.d_aux_pwr.set_low();
            }
            FlipFlopState::AuxPwr => {
                self.d_bat_1.set_low();
                self.d_bat_2.set_low();
                self.d_aux_pwr.set_high();
            },
        }
        self.clk.set_high();
        Timer::after_micros(10).await;
        self.clk.set_low();
    }
}
