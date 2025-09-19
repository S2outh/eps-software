use embassy_stm32::{gpio::{Level, Output, Pin, Speed}, Peri};

#[repr(u8)]
pub enum Sink {
    Mainboard,
    RocketLST,
    RocketHD,
}

pub struct SinkCtrl<'d> {
    mb_enable: Output<'d>,
    lst_enable: Output<'d>,
    rhd_enable: Output<'d>,
}

impl<'d> SinkCtrl<'d> {
    pub fn new(
        mb_enable: Peri<'d, impl Pin>,
        lst_enable: Peri<'d, impl Pin>,
        rhd_enable: Peri<'d, impl Pin>,
    ) -> Self {
        let mb_enable = Output::new(mb_enable, Level::Low, Speed::High);
        let lst_enable = Output::new(lst_enable, Level::Low, Speed::High);
        let rhd_enable = Output::new(rhd_enable, Level::Low, Speed::High);
        Self { mb_enable, lst_enable, rhd_enable }
    }
    pub fn enable(&mut self, sink: Sink) {
        match sink {
            Sink::Mainboard => self.mb_enable.set_high(),
            Sink::RocketLST => self.lst_enable.set_high(),
            Sink::RocketHD => self.rhd_enable.set_high(),
        }
    }
    pub fn disable(&mut self, sink: Sink) {
        match sink {
            Sink::Mainboard => self.mb_enable.set_low(),
            Sink::RocketLST => self.lst_enable.set_low(),
            Sink::RocketHD => self.rhd_enable.set_low(),
        }
    }
    pub fn get(&self, sink: Sink) -> bool {
        match sink {
            Sink::Mainboard => self.mb_enable.is_set_high(),
            Sink::RocketLST => self.lst_enable.is_set_high(),
            Sink::RocketHD => self.rhd_enable.is_set_high(),
        }
    }
}
