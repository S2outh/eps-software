pub mod tmp100_drv;
use embassy_stm32::gpio::Input;
use embassy_sync::watch::DynReceiver;
use tmp100_drv::Tmp100;

pub struct Battery<'a, 'd> {
    temp_probe: Option<Tmp100<'a, 'd>>,
    adc_recv: DynReceiver<'a, i16>,
    bat_status: Input<'d>,
}

impl<'a, 'd> Battery<'a, 'd> {
    pub async fn new(
        temp_probe: Option<Tmp100<'a, 'd>>,
        adc_recv: DynReceiver<'a, i16>,
        bat_status: Input<'d>)
        -> Self {
        Self {
            temp_probe,
            adc_recv,
            bat_status,
        }
    }
    pub async fn get_temperature(&mut self) -> Option<i16> {
        Some(self.temp_probe.as_mut()?.read_temp().await.ok()?)
    }
    pub fn is_enabled(&self) -> bool {
        self.bat_status.is_high()
    }
    pub async fn get_voltage(&mut self) -> i16 {
        self.adc_recv.get().await
    }
}
