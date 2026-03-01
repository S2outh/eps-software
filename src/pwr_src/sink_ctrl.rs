use embassy_stm32::{
    Peri,
    gpio::{Level, Output, Pin, Speed},
};
use south_common::types::Sink;

macro_rules! sink {
    ($($sink:ident),*) => {paste::paste!{
        pub struct SinkCtrl<'d> {$(
            [<$sink:snake>]: Output<'d>,
        )*}
        impl<'d> SinkCtrl<'d> {
            pub fn new($(
                [<$sink:snake>]: Peri<'d, impl Pin>,
            )*) -> Self {
                $(let [<$sink:snake>] = Output::new([<$sink:snake>], Level::High, Speed::Medium);)*
                Self {$(
                    [<$sink:snake>],
                )*}
            }
            fn get(&mut self, sink: Sink) -> &mut Output<'d> {
                match sink {$(
                    Sink::$sink => &mut self.[<$sink:snake>],
                )*}
            }
        }
    }};
}
sink!(
    Carrier,
    RocketLST,
    GPS,
    ExternalCamera,
    SensorLower,
    RocketHD,
    BackupSink
);

impl<'d> SinkCtrl<'d> {
    pub fn enable(&mut self, sink: Sink) {
        self.get(sink).set_high()
    }
    pub fn disable(&mut self, sink: Sink) {
        self.get(sink).set_low()
    }
    pub fn is_enabled(&mut self, sink: Sink) -> bool {
        self.get(sink).is_set_high()
    }
}
