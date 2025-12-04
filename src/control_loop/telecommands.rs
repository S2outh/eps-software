use defmt::Format;

use crate::pwr_src::{d_flip_flop::FlipFlopState, sink_ctrl::Sink};

const POWER_CMD_SUBSYSTEM_ID: u8 = 0x01;

#[derive(Format)]
pub struct ParseError(&'static str);

#[derive(Format)]
pub enum Telecommand {
    SetSource(FlipFlopState, Option<u8>),
    EnableSink(Sink, Option<u8>),
    DisableSink(Sink, Option<u8>),
}

// these parse only for pure repr[u8] enums. which these are.
impl FlipFlopState {
    fn from_u8(uint: u8) -> Result<FlipFlopState, ParseError> {
        if uint >= core::mem::variant_count::<FlipFlopState>() as u8 {
            Err(ParseError("flipflop state out of bounds"))
        } else {
            unsafe { Ok(core::mem::transmute(uint)) }
        }
    }
}

impl Sink {
    fn from_u8(uint: u8) -> Result<Sink, ParseError> {
        if uint >= core::mem::variant_count::<FlipFlopState>() as u8 {
            Err(ParseError("sink id out of bounds"))
        } else {
            unsafe { Ok(core::mem::transmute(uint)) }
        }
    }
}

impl Telecommand {
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < 5 {
            return Err(ParseError("length of data to short"));
        }
        if data[0] != POWER_CMD_SUBSYSTEM_ID {
            return Err(ParseError("cmd for other subsystem"));
        }
        let payload_length = data[2] as usize;
        let payload = &data[3..3 + payload_length];
        Ok(match data[1] {
            0x00 => Self::SetSource(FlipFlopState::from_u8(payload[0])?, payload.get(1).copied()),
            0x01 => Self::EnableSink(Sink::from_u8(payload[0])?, payload.get(1).copied()),
            0x02 => Self::DisableSink(Sink::from_u8(payload[0])?, payload.get(1).copied()),
            _ => return Err(ParseError("command id out of bounds")),
        })
    }
}
