use embassy_sync::{channel::DynamicSender, watch::DynReceiver};

use embassy_time::Timer;
use heapless::Vec;
use south_common::{TMValue, DynTelemetryDefinition, telemetry};

use crate::EpsTelem;

// Aux pwr task
#[embassy_executor::task]
pub async fn aux_pwr_thread(mut aux_pwr: AuxPwr<'static>) {
    const AUX_LOOP_LEN_MS: u64 = 500;
    loop {
        aux_pwr.run().await;
        Timer::after_millis(AUX_LOOP_LEN_MS).await;
    }
}

pub struct AuxPwr<'a> {
    adc_recv: DynReceiver<'a, i16>,
    tm_sender: DynamicSender<'a, EpsTelem>
}

impl<'a> AuxPwr<'a> {
    pub async fn new(
        adc_recv: DynReceiver<'a, i16>,
        tm_sender: DynamicSender<'a, EpsTelem>
    ) -> Self {
        Self {
            adc_recv,
            tm_sender
        }
    }
    async fn get_voltage(&mut self) -> i16 {
        self.adc_recv.get().await
    }
    pub async fn run(&mut self) {
        let tm_data = Vec::from_array(self.get_voltage().await.to_bytes());
        self.tm_sender.send((telemetry::eps::AuxPowerVoltage.id(), tm_data)).await;
    }
}
