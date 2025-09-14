pub mod tmp100_drv;
pub mod d_flip_flop;
use embassy_stm32::gpio::Input;
use tmp100_drv::Tmp100;
use d_flip_flop::DFlipFlop;

pub struct Battery<'a, 'd> {
    temp_probe: Tmp100<'a, 'd>,
    bat_enable: DFlipFlop<'d>,
    bat_status: Input<'d>,
}

impl<'a, 'd> Battery<'a, 'd> {
    pub async fn new(temp_probe: Tmp100<'a, 'd>, bat_enable: DFlipFlop<'d>, bat_status: Input<'d>) -> Self {
        Self {
            temp_probe,
            bat_enable,
            bat_status
        }
    }
}
