use embassy_stm32::gpio::Input;
use embassy_sync::watch::DynReceiver;
use super::d_flip_flop::DFlipFlop;

pub struct AuxPwr<'a, 'd> {
    adc_recv: DynReceiver<'a, i16>,
    aux_pwr_enable: DFlipFlop<'a, 'd>,
    aux_pwr_status: Input<'d>,
}

impl<'a, 'd> AuxPwr<'a, 'd> {
    pub async fn new(
        adc_recv: DynReceiver<'a, i16>,
        aux_pwr_enable: DFlipFlop<'a, 'd>,
        aux_pwr_status: Input<'d>)
        -> Self {
        Self {
            adc_recv,
            aux_pwr_enable,
            aux_pwr_status,
        }
    }
    pub fn is_enabled(&self) -> bool {
        self.aux_pwr_status.is_low()
    }
    pub async fn get_voltage(&mut self) -> i16 {
        self.adc_recv.get().await
    }
    pub async fn enable(&mut self) {
        self.aux_pwr_enable.set(embassy_stm32::gpio::Level::High).await;
    }
    pub async fn disable(&mut self) {
        self.aux_pwr_enable.set(embassy_stm32::gpio::Level::Low).await;
    }
}
