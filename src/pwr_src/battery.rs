pub mod tmp100_drv;
use embassy_stm32::gpio::Input;
use embassy_sync::watch::DynReceiver;
use tmp100_drv::Tmp100;
use super::d_flip_flop::DFlipFlop;

pub struct Battery<'a, 'd> {
    temp_probe: Tmp100<'a, 'd>,
    adc_recv: DynReceiver<'a, i16>,
    bat_enable: DFlipFlop<'a, 'd>,
    bat_status: Input<'d>,
}

impl<'a, 'd> Battery<'a, 'd> {
    pub async fn new(temp_probe: Tmp100<'a, 'd>,
        adc_recv: DynReceiver<'a, i16>,
        bat_enable: DFlipFlop<'a, 'd>,
        bat_status: Input<'d>)
        -> Self {
        Self {
            temp_probe,
            adc_recv,
            bat_enable,
            bat_status,
        }
    }
    pub async fn get_temperature(&mut self) -> i16 {
        self.temp_probe.read_temp().await.unwrap_or(-100)
    }
    pub fn is_enabled(&self) -> bool {
        self.bat_status.is_low()
    }
    pub async fn get_voltage(&mut self) -> i16 {
        self.adc_recv.get().await
    }
    pub async fn enable(&mut self) {
        self.bat_enable.set(embassy_stm32::gpio::Level::High).await;
    }
    pub async fn disable(&mut self) {
        self.bat_enable.set(embassy_stm32::gpio::Level::Low).await;
    }
}
