mod calib;

use embassy_stm32::{adc::{Adc, AdcChannel, AnyAdcChannel, RxDma, SampleTime}, peripherals::ADC1};
use calib::FactoryCalibratedValues;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, watch::{DynSender, Sender}};
use embassy_time::Timer;

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

const AUX_PWR_CH_POS: usize = 0;
const BAT_2_CH_POS: usize = 1;
const BAT_1_CH_POS: usize = 2;
const TEMP_CH_POS: usize = 3;

pub struct EPSAdc<'a, 'd, D: RxDma<ADC1>> {
    adc: Adc<'d, ADC1>,
    dma_channel: D,
    calib: FactoryCalibratedValues,
    // adc channels
    temp_channel: AnyAdcChannel<ADC1>,
    bat_1_channel: AnyAdcChannel<ADC1>,
    bat_2_channel: AnyAdcChannel<ADC1>,
    aux_pwr_channel: AnyAdcChannel<ADC1>,
    // watchers
    temp_sender: DynSender<'a, i32>,
    bat_1_sender: DynSender<'a, i32>,
    bat_2_sender: DynSender<'a, i32>,
    aux_pwr_sender: DynSender<'a, i32>,
}

impl<'a, 'd, D: RxDma<ADC1>> EPSAdc<'a, 'd, D> {
    pub fn new(mut adc: Adc<'d, ADC1>, dma_channel: D,
            bat_1_channel: AnyAdcChannel<ADC1>,
            bat_2_channel: AnyAdcChannel<ADC1>,
            aux_pwr_channel: AnyAdcChannel<ADC1>,
            temp_sender: DynSender<'a, i32>,
            bat_1_sender: DynSender<'a, i32>,
            bat_2_sender: DynSender<'a, i32>,
            aux_pwr_sender: DynSender<'a, i32>,
        ) -> Self {
        let calib = FactoryCalibratedValues::new();

        adc.set_resolution(embassy_stm32::adc::Resolution::BITS12);
        // 16x oversampling
        adc.set_oversampling_ratio(0x03);
        adc.set_oversampling_shift(0x04);
        adc.oversampling_enable(true);

        let temp_channel = adc.enable_temperature().degrade_adc();

        Self { adc, dma_channel, calib,
            temp_channel,
            bat_1_channel,
            bat_2_channel,
            aux_pwr_channel,
            temp_sender,
            bat_1_sender,
            bat_2_sender,
            aux_pwr_sender
        }
    }
    fn calculate_temperature_tenth_deg(&self, measurement: u16) -> i32 {
        let temp_measurement_x10 = 10 * measurement as i32;
        let temp_calibrated_measurement = temp_measurement_x10 * VREF_10MV / VREF_CALIB_10MV;
        TS_REL_VAL_TENTH_DEG * (temp_calibrated_measurement - self.calib.ts_cal_1_x10)
            / self.calib.ts_cal_rel_x10 + TS_1_VAL_TENTH_DEG
    }
    fn calculate_voltage_10mv(&self, measurement: u16) -> i32 {
        let vbat_1_measurement_x100 = 100 * measurement as i32;
        vbat_1_measurement_x100 * V_DIVIDER_MULT * VREF_10MV / RAW_VALUE_RANGE_X100
    }
    pub async fn measure(&mut self) -> (i32, i32, i32, i32) {
        let mut measurements = [0u16; 4];
        self.adc.read(
            &mut self.dma_channel,
            [
                (&mut self.aux_pwr_channel, SampleTime::CYCLES160_5),
                (&mut self.bat_2_channel, SampleTime::CYCLES160_5),
                (&mut self.bat_1_channel, SampleTime::CYCLES160_5),
                (&mut self.temp_channel, SampleTime::CYCLES160_5),
            ]
            .into_iter(),
            &mut measurements
        ).await;
        
        let internal_temperature = self.calculate_temperature_tenth_deg(measurements[TEMP_CH_POS]);
        let bat_1 = self.calculate_voltage_10mv(measurements[BAT_1_CH_POS]);
        let bat_2 = self.calculate_voltage_10mv(measurements[BAT_2_CH_POS]);
        let aux_pwr = self.calculate_voltage_10mv(measurements[AUX_PWR_CH_POS]);
        (bat_1, bat_2, aux_pwr, internal_temperature)
    }
    pub async fn run(&mut self, loop_millis: u64) {
        loop {
            let (bat_1, bat_2, aux_pwr, internal_temperature) = self.measure().await;
            self.bat_1_sender.send(bat_1);
            self.bat_2_sender.send(bat_2);
            self.aux_pwr_sender.send(aux_pwr);
            self.temp_sender.send(internal_temperature);
            Timer::after_millis(loop_millis).await;
        }
    }
}
