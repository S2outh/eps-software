mod factory_calibrated_values;
mod util;

use embassy_time::{Duration, Ticker};
use util::Sortable;

use derive_more::Constructor;
use embassy_stm32::{
    Peri,
    adc::{Adc, AdcChannel, AnyAdcChannel, RxDma, SampleTime},
    peripherals::{ADC1, DMA1_CH1},
};
use embassy_sync::watch::DynSender;
use heapless::Vec;

// Adc reading task
#[embassy_executor::task]
pub async fn adc_thread(mut adc: AdcCtrl<'static, 'static, DMA1_CH1, 4>) {
    const ADC_LOOP_LEN: Duration = Duration::from_millis(100);
    let mut ticker = Ticker::every(ADC_LOOP_LEN);
    loop {
        adc.run().await;
        ticker.next().await;
    }
}

#[derive(Constructor)]
pub struct AdcCtrlChannel<'a> {
    channel: AnyAdcChannel<'a, ADC1>,
    sender: DynSender<'a, i16>,
    conversion_func: fn(u16) -> i16,
}

pub mod conversion {
    use super::factory_calibrated_values::FactoryCalibratedValues;
    use embassy_sync::lazy_lock::LazyLock;

    static CALIB: LazyLock<FactoryCalibratedValues> =
        LazyLock::new(|| FactoryCalibratedValues::new());

    // datasheet reference conditions
    const VREF_10MV: i32 = 3_30;
    const VREF_CALIB_10MV: i32 = 3_00;
    const TS_1_VAL_TENTH_DEG: i32 = 30_0;
    const TS_2_VAL_TENTH_DEG: i32 = 130_0;
    const TS_REL_VAL_TENTH_DEG: i32 = TS_2_VAL_TENTH_DEG - TS_1_VAL_TENTH_DEG;

    const RAW_VALUE_RANGE_X100: i32 = 4096_00;

    // voltage divider
    const R1_OHM: i32 = 100;
    const R2_OHM: i32 = 10;
    const V_DIVIDER_MULT: i32 = (R1_OHM + R2_OHM) / R2_OHM;

    pub fn calculate_temperature_tenth_deg(measurement: u16) -> i16 {
        let temp_measurement_x10 = 10 * measurement as i32;
        let temp_calibrated_measurement = temp_measurement_x10 * VREF_10MV / VREF_CALIB_10MV;
        let calib = CALIB.get();
        let temp_tenth_deg = TS_REL_VAL_TENTH_DEG
            * (temp_calibrated_measurement - calib.ts_cal_1_x10)
            / calib.ts_cal_rel_x10
            + TS_1_VAL_TENTH_DEG;
        temp_tenth_deg as i16
    }

    pub fn calculate_voltage_10mv(measurement: u16) -> i16 {
        let vbat_1_measurement_x100 = 100 * measurement as i32;
        let voltage_mv =
            vbat_1_measurement_x100 * V_DIVIDER_MULT * VREF_10MV / RAW_VALUE_RANGE_X100;
        voltage_mv as i16
    }
}

pub struct AdcCtrl<'a, 'd, D: RxDma<ADC1>, const N: usize> {
    adc: Adc<'d, ADC1>,
    dma_channel: Peri<'d, D>,
    // adc channels
    channels: Vec<AdcCtrlChannel<'a>, N>,
}

impl<'a, 'd, D: RxDma<ADC1>, const N: usize> AdcCtrl<'a, 'd, D, N> {
    pub fn new(
        adc: Adc<'d, ADC1>,
        dma_channel: Peri<'d, D>,
        temp_sender: DynSender<'a, i16>,
        external_channels: [AdcCtrlChannel<'a>; N - 1],
    ) -> Self {
        let temp_channel = AdcCtrlChannel::new(
            adc.enable_temperature().degrade_adc(),
            temp_sender,
            conversion::calculate_temperature_tenth_deg,
        );
        let mut channels: Vec<AdcCtrlChannel<'a>, N> = external_channels.into_iter().collect();
        channels.push(temp_channel).ok();
        channels.sort_by(|c1, c2| {
            c1.channel
                .get_hw_channel()
                .cmp(&c2.channel.get_hw_channel())
        });

        Self {
            adc,
            dma_channel,
            channels,
        }
    }

    async fn measure(&mut self) -> Vec<u16, N> {
        let mut measurements = [0u16; N];
        let sequence = self
            .channels
            .iter_mut()
            .map(|c| (&mut c.channel, SampleTime::CYCLES160_5));

        self.adc
            .read(self.dma_channel.reborrow(), sequence, &mut measurements)
            .await;

        Vec::from_array(measurements)
    }
    fn convert(&self, values: Vec<u16, N>) -> Vec<i16, N> {
        self.channels
            .iter()
            .zip(values)
            .map(|(c, v)| (c.conversion_func)(v))
            .collect()
    }
    fn send(&self, values: Vec<i16, N>) {
        self.channels
            .iter()
            .zip(values)
            .for_each(|(c, v)| c.sender.send(v));
    }

    pub async fn run(&mut self) {
        let raw_values = self.measure().await;
        let converted_values = self.convert(raw_values);
        self.send(converted_values);
    }
}
