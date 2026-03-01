use defmt::Format;
use embassy_stm32::{
    Peri,
    gpio::{Input, Level, Output, Pin, Pull, Speed},
};
use embassy_time::Timer;
use south_common::types::eps::FlipFlopState;

#[repr(u8)]
#[derive(Format, Clone, Copy)]
pub enum FlipFlopInput {
    Bat1,
    Bat2,
    AuxPwr,
}

pub struct DFlipFlop<'d> {
    state: FlipFlopState,
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
        let state = FlipFlopState::On;
        let d_bat_1 = Output::new(d_bat_1, Level::Low, Speed::High);
        let state_bat_1 = Input::new(state_bat_1, Pull::None);
        let d_bat_2 = Output::new(d_bat_2, Level::Low, Speed::High);
        let state_bat_2 = Input::new(state_bat_2, Pull::None);
        let d_aux_pwr = Output::new(d_aux_pwr, Level::Low, Speed::High);
        let state_aux_pwr = Input::new(state_aux_pwr, Pull::None);

        let clk = Output::new(clk, Level::Low, Speed::High);
        Self {
            state,
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
    pub fn get_state(&self) -> FlipFlopState {
        self.state
    }
    pub async fn clock(&mut self) {
        self.clk.set_high();
        Timer::after_micros(10).await;
        self.clk.set_low();
    }
    pub fn map_state(&self) -> (bool, bool, bool) {
        match self.state {
            FlipFlopState::On => (true, true, true),
            FlipFlopState::Bat1 => (true, false, false),
            FlipFlopState::Bat2 => (false, true, false),
            FlipFlopState::AuxPwr => (false, false, true),
        }
    }
    pub async fn update(&mut self) {
        let (bat_1_on, bat_2_on, aux_pwr_on) = self.map_state();
        if bat_1_on != self.is_enabled(FlipFlopInput::Bat1)
            || bat_2_on != self.is_enabled(FlipFlopInput::Bat2)
            || aux_pwr_on != self.is_enabled(FlipFlopInput::AuxPwr)
        {
            self.bat_1.d_pin.set_level((!bat_1_on).into());
            self.bat_2.d_pin.set_level((!bat_2_on).into());
            self.aux_pwr.d_pin.set_level((!aux_pwr_on).into());
            self.clock().await
        }
    }
    pub async fn set(&mut self, state: FlipFlopState) {
        self.state = state;
        self.update().await
    }
}
