//! Definitions and implementations of extended messages.
//!
//! See [6.5].

pub mod chunked;
pub mod extended_control;
use byteorder::{ByteOrder, LittleEndian};
use heapless::Vec;
use proc_bitfield::bitfield;

use crate::protocol_layer::message::data::sink_capabilities::SinkPowerDataObject;
use crate::protocol_layer::message::data::source_capabilities::PowerDataObject;
use crate::protocol_layer::message::header::ExtendedMessageType;

/// Types of extended messages.
///
/// TODO: Add missing types as per [6.5] and [Table 6.53].
#[derive(Debug, Clone)]
#[non_exhaustive]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[allow(unused)]
pub enum Extended {
    /// Source Capabilities Extended Data Block (SCEDB, Table 6.50) as received:
    /// 25 bytes, or 24 from a Source without the EPR PDP byte.
    SourceCapabilitiesExtended(Vec<u8, 25>),
    /// Get_Manufacturer_Info Data Block (Table 6.56): Target, Ref.
    GetManufacturerInfo([u8; 2]),
    /// Manufacturer_Info Data Block (Table 6.57) as received: VID, PID and a
    /// string of up to 22 bytes (4..26 bytes).
    ManufacturerInfo(Vec<u8, 26>),
    /// SOP Status Data Block (SDB, Table 6.51) as received: 7 bytes, or 5..6
    /// from a Source of an earlier revision.
    Status(Vec<u8, 7>),
    /// Extended control message payload.
    ExtendedControl(extended_control::ExtendedControl),
    /// EPR source capabilities list.
    EprSourceCapabilities(Vec<PowerDataObject, 16>),
    /// EPR sink capabilities list.
    EprSinkCapabilities(Vec<SinkPowerDataObject, 7>),
    /// Sink Capabilities Extended Data Block (SKEDB, Table 6.61), raw bytes.
    SinkCapabilitiesExtended([u8; 24]),
    /// PPS Status Data Block (PPSSDB, Table 6.58), raw bytes.
    PpsStatus([u8; 4]),
    /// Unknown data type.
    Unknown,
}

impl Extended {
    /// Size of the extended payload in bytes.
    pub fn data_size(&self) -> u16 {
        match self {
            Self::SourceCapabilitiesExtended(block) => block.len() as u16,
            Self::GetManufacturerInfo(block) => block.len() as u16,
            Self::ManufacturerInfo(block) => block.len() as u16,
            Self::Status(block) => block.len() as u16,
            Self::ExtendedControl(_payload) => 2,
            Self::EprSourceCapabilities(pdos) => (pdos.len() * core::mem::size_of::<u32>()) as u16,
            Self::EprSinkCapabilities(pdos) => (pdos.len() * core::mem::size_of::<u32>()) as u16,
            Self::SinkCapabilitiesExtended(block) => block.len() as u16,
            Self::PpsStatus(block) => block.len() as u16,
            Self::Unknown => 0,
        }
    }

    /// Serialize message data to a slice, returning the number of written bytes.
    pub fn to_bytes(&self, payload: &mut [u8]) -> usize {
        match self {
            Self::Unknown => 0,
            Self::SourceCapabilitiesExtended(block) => {
                payload[..block.len()].copy_from_slice(block);
                block.len()
            }
            Self::GetManufacturerInfo(block) => {
                payload[..block.len()].copy_from_slice(block);
                block.len()
            }
            Self::ManufacturerInfo(block) => {
                payload[..block.len()].copy_from_slice(block);
                block.len()
            }
            Self::Status(block) => {
                payload[..block.len()].copy_from_slice(block);
                block.len()
            }
            Self::ExtendedControl(control) => control.to_bytes(payload),
            Self::SinkCapabilitiesExtended(block) => {
                payload[..block.len()].copy_from_slice(block);
                block.len()
            }
            Self::PpsStatus(block) => {
                payload[..block.len()].copy_from_slice(block);
                block.len()
            }
            Self::EprSourceCapabilities(pdos) => {
                let mut written = 0;
                for pdo in pdos {
                    let raw = match pdo {
                        PowerDataObject::FixedSupply(p) => p.0,
                        PowerDataObject::Battery(p) => p.0,
                        PowerDataObject::VariableSupply(p) => p.0,
                        PowerDataObject::Augmented(a) => match a {
                            crate::protocol_layer::message::data::source_capabilities::Augmented::Spr(p) => p.0,
                            crate::protocol_layer::message::data::source_capabilities::Augmented::Epr(p) => p.0,
                            crate::protocol_layer::message::data::source_capabilities::Augmented::SprAvs(p) => p.0,
                            crate::protocol_layer::message::data::source_capabilities::Augmented::Unknown(p) => *p,
                        },
                        PowerDataObject::Unknown(p) => p.0,
                    };
                    LittleEndian::write_u32(&mut payload[written..written + 4], raw);
                    written += 4;
                }
                written
            }
            Self::EprSinkCapabilities(pdos) => {
                let mut written = 0;
                for pdo in pdos {
                    LittleEndian::write_u32(&mut payload[written..written + 4], pdo.to_raw());
                    written += 4;
                }
                written
            }
        }
    }
}

impl Extended {
    /// An information block a Sink asks for, from a complete Data Block. Table
    /// 6.48 (Data Size): bytes beyond the expected fields are Ignored; section
    /// 6.5: fields a later revision adds are Ignored, so a 24-byte PD R3.0 SCEDB
    /// is valid, and so is a 5- or 6-byte SDB. Shorter blocks are `None`.
    pub fn information(message_type: ExtendedMessageType, payload: &[u8]) -> Option<Self> {
        match message_type {
            ExtendedMessageType::SourceCapabilitiesExtended if payload.len() >= 24 => Some(
                Self::SourceCapabilitiesExtended(Vec::from_slice(&payload[..payload.len().min(25)]).ok()?),
            ),
            ExtendedMessageType::ManufacturerInfo if payload.len() >= 4 => Some(Self::ManufacturerInfo(
                Vec::from_slice(&payload[..payload.len().min(26)]).ok()?,
            )),
            ExtendedMessageType::Status if payload.len() >= 5 => {
                Some(Self::Status(Vec::from_slice(&payload[..payload.len().min(7)]).ok()?))
            }
            _ => None,
        }
    }
}

bitfield! {
    /// Extended message header.
    ///
    /// Chunked messages are currently unsupported.
    #[derive(Clone, Copy, PartialEq, Eq, Default)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    pub struct ExtendedHeader(pub u16): Debug, FromStorage, IntoStorage {
        /// Payload size in bytes.
        pub data_size: u16 @ 0..=8,
        /// Request chunk flag.
        pub request_chunk: bool @ 10,
        /// Chunk number of this extended message.
        pub chunk_number: u8 @ 11..=14,
        /// Whether the message is chunked.
        pub chunked: bool @ 15,
    }
}

impl ExtendedHeader {
    /// Create a new, unchunked extended header for a given payload size.
    pub fn new(data_size: u16) -> Self {
        Self(0).with_data_size(data_size)
    }

    /// Serialize the extended header into the buffer, returning bytes written.
    pub fn to_bytes(self, buf: &mut [u8]) -> usize {
        LittleEndian::write_u16(buf, self.0);
        2
    }

    /// Parse an extended header from bytes.
    pub fn from_bytes(buf: &[u8]) -> Self {
        assert!(buf.len() >= 2);
        Self(LittleEndian::read_u16(buf))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol_layer::message::{Message, Payload};

    #[test]
    fn information_blocks_parse_alike_in_every_path() {
        // Single Chunk: header (Source_Capabilities_Extended / Manufacturer_Info,
        // 7 Data Objects, PD3), Extended Header (Chunked, Data Size), block, padding.
        for (kind, size, expected) in [
            (1u8, 24usize, Some(24usize)),
            (1, 25, Some(25)),
            (1, 26, Some(25)),
            (1, 23, None),
            (7, 4, Some(4)),
            (7, 26, Some(26)),
            (7, 3, None),
        ] {
            let block: std::vec::Vec<u8> = (1..=size as u8).collect();
            let mut bytes = std::vec![0xa0 | kind, 0xf1, size as u8, 0x80];
            bytes.extend_from_slice(&block);
            bytes.resize(30, 0);
            let message_type = ExtendedMessageType::from(kind);
            let parsed = match Message::from_bytes(&bytes).unwrap().payload {
                Some(Payload::Extended(extended)) => extended,
                _ => std::panic!("extended payload"),
            };
            for extended in [
                parsed,
                Message::parse_extended_payload(message_type, &block),
                Extended::information(message_type, &block).unwrap_or(Extended::Unknown),
            ] {
                match (extended, expected) {
                    (Extended::SourceCapabilitiesExtended(got), Some(n)) if kind == 1 => {
                        std::assert_eq!(got.as_slice(), &block[..n])
                    }
                    (Extended::ManufacturerInfo(got), Some(n)) if kind == 7 => {
                        std::assert_eq!(got.as_slice(), &block[..n])
                    }
                    (Extended::Unknown, None) => {}
                    (other, _) => std::panic!("{kind} {size}: {other:?}"),
                }
            }
        }
    }

    #[test]
    fn get_manufacturer_info_serializes_one_padded_object() {
        let block = Extended::GetManufacturerInfo([0, 0]);
        let mut payload = [0xff; 4];
        std::assert_eq!((block.data_size(), block.to_bytes(&mut payload)), (2, 2));
        std::assert_eq!(payload[..2], [0, 0]);
    }
}
