mod calib;

use defmt::info;
use embassy_stm32::{adc::{Adc, AdcChannel, AnyAdcChannel, RxDma, SampleTime}, peripherals::ADC1};
use calib::FactoryCalibratedValues;

// datasheet reference conditions
const VREF_VAL_CV: i32 = 3_00;
const TS_1_VAL_TENTH_DEG: i32 = 30_0;
const TS_2_VAL_TENTH_DEG: i32 = 130_0;
const TS_REL_VAL_TENTH_DEG: i32 = TS_2_VAL_TENTH_DEG - TS_1_VAL_TENTH_DEG;

const RAW_VALUE_RANGE_X100: i32 = 4096_00;

// voltage divider
const R1_OHM: i32 = 100;
const R2_OHM: i32 = 10;
const V_DIVIDER_MULT: i32 = (R1_OHM + R2_OHM) / R2_OHM;

const VBAT_1_CH_POS: usize = 0;
const VTEMP_CH_POS: usize = 1;
const VREF_CH_POS: usize = 2;

pub struct EPSAdc<'d, D: RxDma<ADC1>> {
    adc: Adc<'d, ADC1>,
    dma_channel: D,
    calib: FactoryCalibratedValues,
    bat_1_channel: AnyAdcChannel<ADC1>,
    // bat_2_channel: AnyAdcChannel<ADC1>,
    // ext_channel: AnyAdcChannel<ADC1>,
    vtemp_channel: AnyAdcChannel<ADC1>,
    vref_channel: AnyAdcChannel<ADC1>,
}

impl<'d, D: RxDma<ADC1>> EPSAdc<'d, D> {
    pub fn new(mut adc: Adc<'d, ADC1>, dma_channel: D, bat_1_channel: AnyAdcChannel<ADC1>) -> Self {
        let calib = FactoryCalibratedValues::new();

        adc.set_resolution(embassy_stm32::adc::Resolution::BITS12);
        // 16x oversampling
        adc.set_oversampling_ratio(0x03);
        adc.set_oversampling_shift(0x04);
        adc.oversampling_enable(true);

        let vtemp_channel = adc.enable_temperature().degrade_adc();
        let vref_channel = adc.enable_vrefint().degrade_adc();

        Self { adc, dma_channel, calib, bat_1_channel, vtemp_channel, vref_channel }
    }
    pub async fn measure(&mut self) {
        let mut measurements = [0u16; 3];
        // lots of testing to do here
        self.adc.read(
            &mut self.dma_channel,
            [
                (&mut self.bat_1_channel, SampleTime::CYCLES160_5),
                (&mut self.vtemp_channel, SampleTime::CYCLES160_5),
                (&mut self.vref_channel, SampleTime::CYCLES160_5),
            ]
            .into_iter(),
            &mut measurements
        ).await;

        // Reference adc voltage for recurring calibration
        let vref_measurement_x100 = 100 * measurements[VREF_CH_POS] as i32;
        
        // Measuring different adc channels
        let vbat_1_measurement_x100 = 100 * measurements[VBAT_1_CH_POS] as i32;
        let vtemp_measurement_x10 = 10 * measurements[VTEMP_CH_POS] as i32;

        let vref_mv = VREF_VAL_CV * self.calib.v_refint_x100 / vref_measurement_x100;
        info!("vref: {}", vref_mv);

        let temp_calibrated_measurement = vtemp_measurement_x10 * self.calib.v_refint_x100 / vref_measurement_x100;
        let temp_tenth_deg = TS_REL_VAL_TENTH_DEG * (temp_calibrated_measurement - self.calib.ts_cal_1_x10)
            / self.calib.ts_cal_rel_x10 + TS_1_VAL_TENTH_DEG;
        info!("temp: {}", temp_tenth_deg);
        
        let vbat_mv = vbat_1_measurement_x100 * vref_mv / RAW_VALUE_RANGE_X100;
        info!("vbat: {}", vbat_mv * V_DIVIDER_MULT);
    }
}
