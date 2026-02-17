use embassy_sync::{channel::DynamicSender, watch::DynReceiver};

use embassy_time::{Duration, Instant, Timer};
use south_common::definitions::telemetry::eps as tm;

use crate::EpsTMContainer;

// Aux pwr task
#[embassy_executor::task]
pub async fn aux_pwr_thread(mut aux_pwr: AuxPwr<'static>) {
    const AUX_LOOP_LEN: Duration = Duration::from_millis(500);
    let mut loop_time = Instant::now();
    loop {
        aux_pwr.run().await;
        loop_time += AUX_LOOP_LEN;
        Timer::at(loop_time).await;
    }
}

pub struct AuxPwr<'a> {
    adc_recv: DynReceiver<'a, i16>,
    tm_sender: DynamicSender<'a, EpsTMContainer>,
}

impl<'a> AuxPwr<'a> {
    pub async fn new(
        adc_recv: DynReceiver<'a, i16>,
        tm_sender: DynamicSender<'a, EpsTMContainer>,
    ) -> Self {
        Self {
            adc_recv,
            tm_sender,
        }
    }
    async fn get_voltage(&mut self) -> i16 {
        self.adc_recv.get().await
    }
    pub async fn run(&mut self) {
        let container =
            EpsTMContainer::new(&tm::AuxPowerVoltage, &self.get_voltage().await).unwrap();
        self.tm_sender.send(container).await;
    }
}
