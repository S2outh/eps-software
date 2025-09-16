pub mod tmp100_drv;
pub mod d_flip_flop;
use embassy_stm32::gpio::Input;
use embassy_sync::watch::DynReceiver;
use embassy_time::Timer;
use tmp100_drv::Tmp100;
use d_flip_flop::DFlipFlop;

pub struct Battery<'a, 'd> {
    temp_probe: Tmp100<'a, 'd>,
    adc_recv: DynReceiver<'a, i32>,
    bat_enable: DFlipFlop<'d>,
    bat_status: Input<'d>,
    telemetry: BatteryTelemetry,
}
pub struct BatteryTelemetry {
    temperature_tenth_deg: i32,
    voltage_10mv: i32,
    enabled: bool,
}

impl<'a, 'd> Battery<'a, 'd> {
    pub async fn new(temp_probe: Tmp100<'a, 'd>,
        adc_recv: DynReceiver<'a, i32>,
        bat_enable: DFlipFlop<'d>,
        bat_status: Input<'d>)
        -> Self {
        Self {
            temp_probe,
            adc_recv,
            bat_enable,
            bat_status,
            telemetry: BatteryTelemetry {
                temperature_tenth_deg: -100,
                voltage_10mv: -100,
                enabled: false
            }
        }
    }
    pub async fn run(&mut self, loop_millis: u64) {
        loop {
            // seperate thread architecture to add internal logic later
            self.telemetry.temperature_tenth_deg =
                self.temp_probe.read_temp().await.unwrap_or(-100);
            self.telemetry.enabled =
                self.bat_status.is_low();
            self.telemetry.voltage_10mv =
                self.adc_recv.get().await;
            Timer::after_millis(loop_millis).await;
        }
    }
    pub fn get_telemetry(&self) -> &BatteryTelemetry {
        &self.telemetry
    }
    pub async fn enable(&mut self) {
        self.bat_enable.set(embassy_stm32::gpio::Level::High).await;
    }
    pub async fn disable(&mut self) {
        self.bat_enable.set(embassy_stm32::gpio::Level::Low).await;
    }
}
