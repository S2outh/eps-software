use defmt::Format;

use crate::pwr_src::{d_flip_flop::FlipFlopState, sink_ctrl::Sink};

const POWER_CMD_SUBSYSTEM_ID: u8 = 0x00;

#[derive(Format)]
pub(super) struct ParseError(&'static str);

pub(super) enum Telecommand {
    SetSource(FlipFlopState),
    EnableSink(Sink),
    DisableSink(Sink),
}

// these parse only for pure repr[u8] enums. which these are.
impl FlipFlopState {
    pub(super) fn from_u8(uint: u8) -> Result<FlipFlopState, ParseError> {
        if uint >= 4 {
            Err(ParseError("flipflop state out of bounds"))
        } else { unsafe { Ok(core::mem::transmute(uint)) }}
    }
}

impl Sink {
    pub(super) fn from_u8(uint: u8) -> Result<Sink, ParseError> {
        if uint >= 3 {
            Err(ParseError("sink id out of bounds"))
        } else { unsafe { Ok(core::mem::transmute(uint)) }}
    }
}

impl Telecommand {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 5 { return Err(ParseError("length of data to short")); }
        if data[0] != POWER_CMD_SUBSYSTEM_ID { return Err(ParseError("cmd for other subsystem")); }
        // payload length is an u16
        let payload_length = (data[3] as usize) << 8 | (data[2] as usize);
        let payload = &data[4..4+payload_length];
        Ok(match data[1] {
            0x00 => Self::SetSource(FlipFlopState::from_u8(payload[0])?),
            0x01 => Self::EnableSink(Sink::from_u8(payload[0])?),
            0x02 => Self::DisableSink(Sink::from_u8(payload[0])?),
            _ => return Err(ParseError("command id out of bounds"))
        })
    }
}
