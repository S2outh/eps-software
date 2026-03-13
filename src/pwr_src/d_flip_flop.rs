use embassy_stm32::{
    Peri,
    gpio::{Input, Level, Output, Pin, Pull, Speed},
};
use embassy_time::Timer;
use south_common::types::eps::FlipFlopInput;

pub struct DFlipFlop<'d> {
    bat_1_on: bool,
    bat_2_on: bool,
    aux_pwr_on: bool,

    bat_1: RawFlipFlop<'d>,
    bat_2: RawFlipFlop<'d>,
    aux_pwr: RawFlipFlop<'d>,

    clk: Output<'d>,
}
struct RawFlipFlop<'d> {
    d_pin: Output<'d>,
    state_pin: Input<'d>,
}

impl<'d> DFlipFlop<'d> {
    pub fn new(
        d_bat_1: Peri<'d, impl Pin>,
        state_bat_1: Peri<'d, impl Pin>,
        d_bat_2: Peri<'d, impl Pin>,
        state_bat_2: Peri<'d, impl Pin>,
        d_aux_pwr: Peri<'d, impl Pin>,
        state_aux_pwr: Peri<'d, impl Pin>,
        clk: Peri<'d, impl Pin>,
    ) -> Self {
        let d_bat_1 = Output::new(d_bat_1, Level::Low, Speed::High);
        let state_bat_1 = Input::new(state_bat_1, Pull::None);
        let d_bat_2 = Output::new(d_bat_2, Level::Low, Speed::High);
        let state_bat_2 = Input::new(state_bat_2, Pull::None);
        let d_aux_pwr = Output::new(d_aux_pwr, Level::Low, Speed::High);
        let state_aux_pwr = Input::new(state_aux_pwr, Pull::None);

        let clk = Output::new(clk, Level::Low, Speed::High);
        Self {
            bat_1_on: true,
            bat_2_on: true,
            aux_pwr_on: true,
            bat_1: RawFlipFlop {
                d_pin: d_bat_1,
                state_pin: state_bat_1,
            },
            bat_2: RawFlipFlop {
                d_pin: d_bat_2,
                state_pin: state_bat_2,
            },
            aux_pwr: RawFlipFlop {
                d_pin: d_aux_pwr,
                state_pin: state_aux_pwr,
            },
            clk,
        }
    }
    pub fn is_enabled(&self, input: FlipFlopInput) -> bool {
        match input {
            FlipFlopInput::Bat1 => self.bat_1.state_pin.is_high(),
            FlipFlopInput::Bat2 => self.bat_2.state_pin.is_high(),
            FlipFlopInput::AuxPwr => self.aux_pwr.state_pin.is_high(),
        }
    }
    pub async fn clock(&mut self) {
        self.clk.set_high();
        Timer::after_micros(10).await;
        self.clk.set_low();
    }
    pub async fn update(&mut self) {
        if self.bat_1_on != self.is_enabled(FlipFlopInput::Bat1)
            || self.bat_2_on != self.is_enabled(FlipFlopInput::Bat2)
            || self.aux_pwr_on != self.is_enabled(FlipFlopInput::AuxPwr)
        {
            self.bat_1.d_pin.set_level((!self.bat_1_on).into());
            self.bat_2.d_pin.set_level((!self.bat_2_on).into());
            self.aux_pwr.d_pin.set_level((!self.aux_pwr_on).into());
            self.clock().await
        }
    }
    pub async fn set(&mut self, channel: FlipFlopInput) {
        match channel {
            FlipFlopInput::Bat1 => self.bat_1_on = true,
            FlipFlopInput::Bat2 => self.bat_2_on = true,
            FlipFlopInput::AuxPwr => self.aux_pwr_on = true,
        }
        self.update().await
    }
    pub async fn reset(&mut self, channel: FlipFlopInput) {
        match channel {
            FlipFlopInput::Bat1 => self.bat_1_on = false,
            FlipFlopInput::Bat2 => self.bat_2_on = false,
            FlipFlopInput::AuxPwr => self.aux_pwr_on = false,
        }
        self.update().await
    }
}
