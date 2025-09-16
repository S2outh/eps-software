use core::ptr::read_volatile;

const TS_CAL_1_REG: usize = 0x1FFF_75A8;
const TS_CAL_2_REG: usize = 0x1FFF_75CA;

pub struct FactoryCalibratedValues {
    pub ts_cal_1_x10: i32,
    pub ts_cal_rel_x10: i32,
}
impl FactoryCalibratedValues {
    pub fn new() -> Self {
        unsafe {
            let ts_cal_1_x10 = 10 * read_volatile(TS_CAL_1_REG as *const u16) as i32;
            let ts_cal_2_x10 = 10 * read_volatile(TS_CAL_2_REG as *const u16) as i32;
            let ts_cal_rel_x10 = ts_cal_2_x10 - ts_cal_1_x10;
            Self { ts_cal_1_x10, ts_cal_rel_x10 }
        }
    }
}
