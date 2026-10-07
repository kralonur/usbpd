//! The protocol layer is controlled by the policy engine, and commands the PHY layer.
//!
//! Handles
//! - construction of messages,
//! - message timers and timeouts,
//! - message retry counters,
//! - reset operation,
//! - error handling,
//! - state behaviour.
//!
//! At this point in time, the protocol layer does not support extended messages.

pub mod message;

use core::future::Future;
use core::marker::PhantomData;

use byteorder::{ByteOrder, LittleEndian};
use embassy_futures::select::{Either, select};
use heapless::Vec;
use message::Message;
use message::data::{Data, request};
use message::extended::extended_control::ExtendedControlMessageType;
use message::header::{
    ControlMessageType, DataMessageType, ExtendedMessageType, Header, MessageType, SpecificationRevision,
};
use usbpd_traits::{Driver, DriverRxError, DriverTxError};

use crate::PowerRole;
use crate::counters::{Counter, CounterType, Error as CounterError};
use crate::protocol_layer::message::data::epr_mode::EprModeDataObject;
use crate::protocol_layer::message::data::source_capabilities::SourceCapabilities;
use crate::protocol_layer::message::extended::Extended;
use crate::protocol_layer::message::{ParseError, Payload};
use crate::timers::{Timer, TimerType};

/// Maximum message size including headers and payload.
const MAX_MESSAGE_SIZE: usize = 272;

/// Size of the message header in bytes.
const MSG_HEADER_SIZE: usize = 2;

/// Size of the extended message header in bytes.
const EXT_HEADER_SIZE: usize = 2;

/// Errors that can occur in the protocol layer.
#[derive(thiserror::Error, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ProtocolError {
    /// An error occured during data reception.
    #[error("RX error")]
    RxError(#[from] RxError),
    /// An error occured during data transmission.
    #[error("TX error")]
    TxError(#[from] TxError),
    /// Transmission failed after the maximum number of allowed retries.
    #[error("transmit retries (`{0}`) exceeded")]
    TransmitRetriesExceeded(u8),
    /// The driver did not send the first Message of a Sink-initiated AMS
    /// (SinkTxNG or a received SOP Message, sections 7.2/7.3). No counters change.
    #[error("transmit deferred")]
    TransmitDeferred,
    /// An unexpected message was received.
    #[error("unexpected message")]
    UnexpectedMessage,
}

/// Errors that can occur during reception of data.
#[derive(thiserror::Error, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RxError {
    /// Port partner requested soft reset.
    #[error("soft reset")]
    SoftReset,
    /// Driver reported a hard reset.
    #[error("hard reset")]
    HardReset,
    /// A timeout during message reception.
    #[error("receive timeout")]
    ReceiveTimeout,
    /// An unsupported message was received.
    #[error("unsupported message")]
    UnsupportedMessage,
    /// A Chunk of a multi-Chunk Message was received without a Chunking layer.
    /// Section 7.31.15.1: answer Not_Supported after ChunkingNotSupportedTimer.
    #[error("multi-chunk message without chunking layer")]
    ChunkingNotSupported,
    /// A message parsing error occured.
    #[error("parse error")]
    ParseError(#[from] ParseError),
    /// The received acknowledgement does not match the last transmitted message's ID.
    #[error("wrong tx id `{0}` acknowledged")]
    AcknowledgeMismatch(u8),
}

/// Errors that can occur during transmission of data.
#[derive(thiserror::Error, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TxError {
    /// Driver reported a hard reset.
    #[error("hard reset")]
    HardReset,
    /// unchunked_extended_messages_supported must be false (library uses chunked mode).
    #[error("unchunked extended messages not supported")]
    UnchunkedExtendedMessagesNotSupported,
    /// AVS voltage LSB 2 bits must be zero per USB PD 3.2 Table 6.26.
    #[error("AVS voltage alignment invalid")]
    AvsVoltageAlignmentInvalid,
}

#[derive(Debug)]
struct Counters {
    _busy: Counter,
    _caps: Counter, // Unused, optional.
    _discover_identity: Counter,
    rx_message: Option<Counter>,
    tx_message: Counter,
    retry: Counter,
}

impl Default for Counters {
    fn default() -> Self {
        Counters {
            _busy: Counter::new(CounterType::Busy),
            _caps: Counter::new(CounterType::Caps),
            _discover_identity: Counter::new(CounterType::DiscoverIdentity),
            rx_message: None,
            tx_message: Counter::new(CounterType::MessageId),
            retry: Counter::new(CounterType::Retry),
        }
    }
}

/// The USB PD protocol layer.
#[derive(Debug)]
pub(crate) struct ProtocolLayer<DRIVER: Driver, TIMER: Timer> {
    driver: DRIVER,
    counters: Counters,
    default_header: Header,
    supported_revision: SpecificationRevision,
    revision_locked: bool,
    chunking: bool,
    extended_rx_buffer: Vec<u8, MAX_MESSAGE_SIZE>,
    extended_rx_expected: Option<(ExtendedMessageType, u16, u8)>,
    _timer: PhantomData<TIMER>,
}

impl<DRIVER: Driver, TIMER: Timer> ProtocolLayer<DRIVER, TIMER> {
    /// Create a new protocol layer from a driver and default header.
    pub fn new(driver: DRIVER, default_header: Header) -> Self {
        Self {
            driver,
            counters: Default::default(),
            default_header,
            supported_revision: default_header.spec_revision().expect("valid initial header"),
            revision_locked: false,
            chunking: true,
            extended_rx_buffer: Vec::new(),
            extended_rx_expected: None,
            _timer: PhantomData,
        }
    }

    /// Exclusive explicit-contract extension; no RX future is active here.
    /// The driver returns only after normal ownership transfer has completed.
    pub async fn discover_cable(&mut self) -> Result<usbpd_traits::CableReport, ProtocolError> {
        let report = self
            .driver
            .discover_cable(
                self.counters.tx_message.value(),
                self.counters.rx_message.as_ref().map(|counter| counter.value()),
            )
            .await
            .map_err(|error| match error {
                DriverRxError::Discarded => RxError::ReceiveTimeout,
                DriverRxError::HardReset => RxError::HardReset,
            })?;
        self.counters.tx_message.set(report.next_tx_id);
        self.counters.rx_message = report.last_rx_id.map(|id| {
            let mut counter = Counter::new(CounterType::MessageId);
            counter.set(id);
            counter
        });
        Ok(report)
    }

    /// Select whether multi-Chunk Messages are assembled (the default). A port
    /// without a Chunking layer acknowledges and reports them instead, so the
    /// policy engine can apply section 7.31.15.1 (section 9.2.7).
    pub fn set_chunking(&mut self, chunking: bool) {
        self.chunking = chunking;
        self.reset_chunked_rx();
    }

    /// Reset the protocol layer.
    pub fn reset(&mut self) {
        self.counters = Default::default();
    }

    // Soft_Reset only resets counters. Revision detection restarts at a
    // Hard Reset (or a new ProtocolLayer after detach/error recovery).
    fn reset_revision(&mut self) {
        self.revision_locked = false;
        self.default_header.set_spec_revision(self.supported_revision);
    }

    fn detect_revision(&mut self, header: Header) -> Result<(), RxError> {
        if self.revision_locked {
            return Ok(());
        }
        let source_request = matches!(self.default_header.port_power_role(), PowerRole::Source)
            && header.message_type() == MessageType::Data(DataMessageType::Request);
        let sink_capabilities = matches!(self.default_header.port_power_role(), PowerRole::Sink)
            && header.message_type() == MessageType::Data(DataMessageType::SourceCapabilities);
        if source_request || sink_capabilities {
            let supported = u8::from(self.supported_revision).max(1);
            let peer = u8::from(header.spec_revision()?).max(1);
            self.default_header
                .set_spec_revision(SpecificationRevision::try_from(supported.min(peer))?);
            self.revision_locked = source_request;
        }
        Ok(())
    }

    /// Allows tests to access the driver directly.
    #[cfg(test)]
    pub fn driver(&mut self) -> &mut DRIVER {
        &mut self.driver
    }

    /// Access the default header directly.
    pub fn header(&self) -> &Header {
        &self.default_header
    }

    /// Change the header's data role after a data role swap
    /// FIXME: Use this after a data role swap
    #[allow(unused)]
    pub fn set_header_data_role(&mut self, role: crate::DataRole) {
        self.default_header.set_port_data_role(role);
    }

    fn get_message_buffer() -> [u8; MAX_MESSAGE_SIZE] {
        [0u8; MAX_MESSAGE_SIZE]
    }

    /// Get a timer future for a given type.
    pub fn get_timer(timer_type: TimerType) -> impl Future<Output = ()> {
        TimerType::get_timer::<TIMER>(timer_type)
    }

    /// Receive a simple (non-chunked) message from the driver.
    /// Used by wait_for_good_crc to avoid recursion with chunked message handling.
    async fn receive_simple(&mut self) -> Result<Message, RxError> {
        loop {
            let mut buffer = Self::get_message_buffer();

            let length = match self.driver.receive(&mut buffer).await {
                Ok(length) => length,
                Err(DriverRxError::Discarded) => continue,
                Err(DriverRxError::HardReset) => {
                    self.reset_revision();
                    return Err(RxError::HardReset);
                }
            };

            let message = Message::from_bytes(&buffer[..length])?;
            return Ok(message);
        }
    }

    /// Wait until a GoodCrc message is received, or a timeout occurs.
    async fn wait_for_good_crc(&mut self) -> Result<(), RxError> {
        trace!("Wait for GoodCrc");

        let timeout_fut = Self::get_timer(TimerType::CRCReceive);
        let receive_fut = async {
            let message = self.receive_simple().await?;

            if matches!(
                message.header.message_type(),
                MessageType::Control(ControlMessageType::GoodCRC)
            ) {
                trace!(
                    "Received GoodCrc, TX message count: {}, expected: {}",
                    message.header.message_id(),
                    self.counters.tx_message.value()
                );
                if message.header.message_id() == self.counters.tx_message.value() {
                    // See spec, [6.7.1.1]
                    self.counters.retry.reset();
                    _ = self.counters.tx_message.increment();
                    Ok(())
                } else {
                    Err(RxError::AcknowledgeMismatch(message.header.message_id()))
                }
            } else if matches!(message.header.message_type(), MessageType::Control(_)) {
                Err(ParseError::InvalidControlMessageType(message.header.message_type_raw()).into())
            } else {
                Err(ParseError::InvalidMessageType(message.header.message_type_raw()).into())
            }
        };

        match select(timeout_fut, receive_fut).await {
            Either::First(_) => Err(RxError::ReceiveTimeout),
            Either::Second(receive_result) => receive_result,
        }
    }

    /// Validate an outgoing message for spec compliance.
    ///
    /// This catches common mistakes when constructing messages:
    /// - unchunked_extended_messages_supported should always be false
    /// - AVS voltage LSB 2 bits should be zero (per USB PD 3.2 Table 6.26)
    ///
    /// Only validates outgoing messages - never called when parsing received data.
    /// Returns an error if validation fails, allowing the caller to handle it appropriately.
    fn validate_outgoing_message(message: &Message) -> Result<(), TxError> {
        if let Some(Payload::Data(message::data::Data::Request(power_source))) = &message.payload {
            use message::data::request::PowerSource;
            match power_source {
                PowerSource::FixedVariableSupply(rdo) => {
                    if rdo.unchunked_extended_messages_supported() {
                        return Err(TxError::UnchunkedExtendedMessagesNotSupported);
                    }
                }
                PowerSource::Pps(rdo) => {
                    if rdo.unchunked_extended_messages_supported() {
                        return Err(TxError::UnchunkedExtendedMessagesNotSupported);
                    }
                }
                PowerSource::EprRequest(epr) => {
                    // Check the raw RDO for validation
                    let rdo_bits = epr.rdo;
                    let unchunked = (rdo_bits >> 23) & 1 == 1;
                    if unchunked {
                        return Err(TxError::UnchunkedExtendedMessagesNotSupported);
                    }

                    // Check if this looks like an AVS request (bits 30-31 = 00, bits 28-29 = 11)
                    let is_avs = ((rdo_bits >> 30) & 0x3 == 0) && ((rdo_bits >> 28) & 0x3 == 3);
                    if is_avs {
                        let voltage = (rdo_bits >> 9) & 0xFFF;
                        if (voltage as u16) & 0x3 != 0 {
                            return Err(TxError::AvsVoltageAlignmentInvalid);
                        }
                    }
                }
                _ => {}
            }
        }
        Ok(())
    }

    async fn transmit_inner(&mut self, buffer: &[u8]) -> Result<(), TxError> {
        loop {
            match self.driver.transmit(buffer).await {
                Ok(_) => return Ok(()),
                Err(DriverTxError::HardReset) => return Err(TxError::HardReset),
                // Software-retry drivers re-offer a deferred Message like a discarded one.
                Err(DriverTxError::Discarded | DriverTxError::Deferred | DriverTxError::NotAcknowledged) => {
                    // Retry transmission.
                }
            }
        }
    }

    /// Transmit a message.
    ///
    // GoodCrc message transmission is handled separately.
    // See `transmit_good_crc()` instead.
    pub async fn transmit(&mut self, message: Message) -> Result<(), ProtocolError> {
        if matches!(self.default_header.port_power_role(), PowerRole::Sink)
            && message.header.message_type() == MessageType::Data(DataMessageType::Request)
        {
            self.revision_locked = true;
        }
        assert_ne!(
            message.header.message_type(),
            MessageType::Control(ControlMessageType::GoodCRC)
        );

        // Validate outgoing message for spec compliance
        Self::validate_outgoing_message(&message)?;

        trace!("Transmit message: {:?}", message);

        let mut buffer = Self::get_message_buffer();
        let size = message.to_bytes(&mut buffer);

        if DRIVER::HAS_AUTO_RETRY {
            // Hardware handles retries and verifies GoodCRC reception.
            // Call driver.transmit() directly (not transmit_inner()) because
            // Discarded here means all hardware retries exhausted — no point
            // retrying in software.
            match self.driver.transmit(&buffer[..size]).await {
                Ok(()) => {
                    self.counters.retry.reset();
                    _ = self.counters.tx_message.increment();
                    trace!("Transmit success (hardware retry)");
                    Ok(())
                }
                Err(DriverTxError::HardReset) => Err(TxError::HardReset.into()),
                Err(DriverTxError::Deferred) => Err(ProtocolError::TransmitDeferred),
                Err(DriverTxError::Discarded | DriverTxError::NotAcknowledged) => {
                    Err(ProtocolError::TransmitRetriesExceeded(self.counters.retry.max_value()))
                }
            }
        } else {
            // Software retry loop
            self.counters.retry.reset();

            loop {
                match self.transmit_inner(&buffer[..size]).await {
                    Ok(_) => match self.wait_for_good_crc().await {
                        Ok(()) => {
                            trace!("Transmit success");
                            return Ok(());
                        }
                        Err(RxError::ReceiveTimeout) => match self.counters.retry.increment() {
                            Ok(_) => {
                                // Retry transmission, until the retry counter is exceeded.
                            }
                            Err(CounterError::Exceeded) => {
                                return Err(ProtocolError::TransmitRetriesExceeded(self.counters.retry.max_value()));
                            }
                        },
                        Err(other) => return Err(other.into()),
                    },
                    Err(other) => return Err(other.into()),
                }
            }
        }
    }

    /// Send a GoodCrc message to the port partner.
    async fn transmit_good_crc(&mut self, revision: SpecificationRevision) -> Result<(), ProtocolError> {
        trace!(
            "Transmit message GoodCrc for RX message count: {}",
            self.counters.rx_message.unwrap().value()
        );

        let mut buffer = Self::get_message_buffer();

        let size = Message::new(Header::new_control(
            self.default_header.with_spec_revision(revision),
            self.counters.rx_message.unwrap(), // A message must have been received before.
            ControlMessageType::GoodCRC,
        ))
        .to_bytes(&mut buffer);

        Ok(self.transmit_inner(&buffer[..size]).await?)
    }

    /// Handle acknowledgement and retransmission detection for a received message.
    ///
    /// Returns `Ok(true)` if this was a retransmission (caller should continue to next message),
    /// `Ok(false)` if this is a new message to process, or `Err` on failure.
    async fn handle_rx_ack(&mut self, message: &Message) -> Result<bool, RxError> {
        let is_good_crc = matches!(
            message.header.message_type(),
            MessageType::Control(ControlMessageType::GoodCRC)
        );

        let is_retransmission = if is_good_crc {
            false
        } else {
            self.update_rx_message_counter(message)
        };

        if !DRIVER::HAS_AUTO_GOOD_CRC && !is_good_crc {
            let revision = if matches!(
                message.header.spec_revision()?,
                SpecificationRevision::R1_0 | SpecificationRevision::R2_0
            ) {
                SpecificationRevision::R2_0
            } else {
                SpecificationRevision::R3_X
            };
            match self.transmit_good_crc(revision).await {
                Ok(()) => {}
                Err(ProtocolError::TxError(TxError::HardReset)) => return Err(RxError::HardReset),
                Err(_) => return Err(RxError::UnsupportedMessage),
            }
        }

        Ok(is_retransmission)
    }

    /// Reset chunked message reception state.
    fn reset_chunked_rx(&mut self) {
        self.extended_rx_buffer.clear();
        self.extended_rx_expected = None;
    }

    /// Receive a message, assembling chunked extended messages as needed.
    async fn receive_message_inner(&mut self) -> Result<Message, RxError> {
        loop {
            let mut buffer = Self::get_message_buffer();

            let length = match self.driver.receive(&mut buffer).await {
                Ok(length) => length,
                Err(DriverRxError::Discarded) => continue,
                Err(DriverRxError::HardReset) => {
                    self.reset_revision();
                    return Err(RxError::HardReset);
                }
            };

            // Parse header early to handle chunking.
            let header = Header::from_bytes(&buffer[..MSG_HEADER_SIZE])?;
            let message_type = header.message_type();

            if matches!(message_type, MessageType::Extended(_)) {
                let ext_header_end = MSG_HEADER_SIZE + EXT_HEADER_SIZE;
                let ext_header =
                    message::extended::ExtendedHeader::from_bytes(&buffer[MSG_HEADER_SIZE..ext_header_end]);
                let payload = &buffer[ext_header_end..length];
                let total_size = ext_header.data_size();
                let chunked = ext_header.chunked();
                let chunk_number = ext_header.chunk_number();
                let msg_type = match message_type {
                    MessageType::Extended(mt) => mt,
                    _ => unreachable!(),
                };

                if chunked {
                    trace!(
                        "Received chunked extended message {:?}, chunk {}, size {}",
                        message_type,
                        chunk_number,
                        payload.len()
                    );

                    // Update RX counters and acknowledge.
                    let tmp_message = Message { header, payload: None };
                    if self.handle_rx_ack(&tmp_message).await? {
                        continue; // Retransmission
                    }
                    // Table 6.48: Chunk Number 10..15 is Invalid; ignore the Message.
                    if chunk_number > 9 {
                        continue;
                    }
                    if !self.chunking {
                        // A stray Chunk request is not part of a received multi-Chunk Message.
                        if ext_header.request_chunk() {
                            return Err(RxError::UnsupportedMessage);
                        }
                        if chunk_number > 0 || payload.len() < total_size as usize {
                            return Err(RxError::ChunkingNotSupported);
                        }
                    }

                    let (expected_total, expected_next) = match self.extended_rx_expected {
                        Some((ty, total, next)) if ty == msg_type => (total, next),
                        _ => (total_size, 0),
                    };

                    // Ensure chunks arrive in order.
                    if expected_next != 0 && chunk_number != expected_next {
                        self.reset_chunked_rx();
                        return Err(RxError::UnsupportedMessage);
                    }

                    if chunk_number == 0 || expected_next == 0 {
                        self.extended_rx_buffer.clear();
                        self.extended_rx_expected = Some((msg_type, total_size, 1));
                    } else {
                        self.extended_rx_expected = Some((msg_type, expected_total, expected_next + 1));
                    }

                    if self.extended_rx_buffer.len() + payload.len() > self.extended_rx_buffer.capacity() {
                        self.reset_chunked_rx();
                        return Err(RxError::UnsupportedMessage);
                    }
                    if self.extended_rx_buffer.extend_from_slice(payload).is_err() {
                        self.reset_chunked_rx();
                        return Err(RxError::UnsupportedMessage);
                    }

                    if self.extended_rx_buffer.len() < total_size as usize {
                        // Need more chunks - send chunk request per spec 6.12.2.1.2.4
                        let next_chunk = self
                            .extended_rx_expected
                            .as_ref()
                            .map(|(_, _, next)| *next)
                            .unwrap_or(1);
                        self.transmit_chunk_request(msg_type, next_chunk).await?;
                        continue;
                    }

                    // All chunks received, parse payload.
                    let ext_payload = &self.extended_rx_buffer[..total_size as usize];
                    let parsed_payload = match msg_type {
                        ExtendedMessageType::ExtendedControl if ext_payload.len() >= 2 => {
                            Payload::Extended(message::extended::Extended::ExtendedControl(
                                message::extended::extended_control::ExtendedControl::from_bytes(ext_payload),
                            ))
                        }
                        ExtendedMessageType::EprSourceCapabilities => {
                            Payload::Extended(message::extended::Extended::EprSourceCapabilities(
                                ext_payload
                                    .chunks_exact(4)
                                    .map(|buf| {
                                        message::data::source_capabilities::parse_raw_pdo(LittleEndian::read_u32(buf))
                                    })
                                    .collect(),
                            ))
                        }
                        // Table 6.58: the PPSSDB is exactly 4 bytes.
                        ExtendedMessageType::PpsStatus if ext_payload.len() == 4 => {
                            Payload::Extended(message::extended::Extended::PpsStatus([
                                ext_payload[0],
                                ext_payload[1],
                                ext_payload[2],
                                ext_payload[3],
                            ]))
                        }
                        _ => Payload::Extended(
                            message::extended::Extended::information(msg_type, ext_payload)
                                .unwrap_or(message::extended::Extended::Unknown),
                        ),
                    };

                    self.extended_rx_expected = None;
                    let mut message = Message::new(header);
                    message.payload = Some(parsed_payload);

                    trace!("Received assembled extended message {:?}", message);
                    return Ok(message);
                }
            }

            // Non-extended or unchunked extended messages.
            let message = Message::from_bytes(&buffer[..length])?;

            if message.header.message_type() == MessageType::Control(ControlMessageType::SoftReset) {
                // Sections 9.2.5.2.2 / 6.1.3.1: Accept a startup reset at the
                // peer's supported revision, without locking the first Request's
                // revision detection or changing an already negotiated revision.
                if !self.revision_locked {
                    let peer = u8::from(message.header.spec_revision()?).max(1);
                    let supported = u8::from(self.supported_revision).max(1);
                    self.default_header
                        .set_spec_revision(SpecificationRevision::try_from(supported.min(peer))?);
                }
                return Err(RxError::SoftReset);
            }

            // Acknowledge/deduplicate even reserved types before responding as
            // unsupported. A retransmission must not trigger another reply.
            if self.handle_rx_ack(&message).await? {
                continue; // Retransmission
            }
            self.detect_revision(message.header)?;
            if matches!(
                message.header.message_type(),
                MessageType::Control(ControlMessageType::Reserved) | MessageType::Data(DataMessageType::Reserved)
            ) {
                trace!("Unsupported message type in header: {:?}", message.header);
                return Err(RxError::UnsupportedMessage);
            }

            trace!("Received message {:?}", message);
            return Ok(message);
        }
    }

    /// Receive a message.
    pub async fn receive_message(&mut self) -> Result<Message, ProtocolError> {
        self.receive_message_inner().await.map_err(|err| err.into())
    }

    /// Updates the received message counter.
    ///
    /// If receiving the first message after protocol layer reset, copy its ID.
    /// Otherwise, compare the received ID with the stored ID. If they are equal, this is a retransmission.
    ///
    /// Returns `true`, if this was a retransmission.
    fn update_rx_message_counter(&mut self, rx_message: &Message) -> bool {
        match self.counters.rx_message.as_mut() {
            None => {
                trace!(
                    "Received first message after protocol layer reset with RX counter value: {}",
                    rx_message.header.message_id()
                );
                self.counters.rx_message = Some(Counter::new_from_value(
                    CounterType::MessageId,
                    rx_message.header.message_id(),
                ));
                false
            }
            Some(counter) => {
                if rx_message.header.message_id() == counter.value() {
                    trace!("Received retransmission of RX counter value: {}", counter.value());
                    true
                } else {
                    counter.set(rx_message.header.message_id());
                    false
                }
            }
        }
    }

    /// Wait until a message of one of the chosen types is received, or a timeout occurs.
    pub async fn receive_message_type(
        &mut self,
        message_types: &[MessageType],
        timer_type: TimerType,
    ) -> Result<Message, ProtocolError> {
        // GoodCrc message reception is handled separately.
        // See `wait_for_good_crc()` instead.
        for message_type in message_types {
            assert_ne!(*message_type, MessageType::Control(ControlMessageType::GoodCRC));
        }

        // Section 7.31.2: GoodCRC reception, not delayed driver confirmation,
        // starts SenderResponseTimer. Keep all other timer origins unchanged.
        let elapsed = self.driver.sender_response_elapsed_ms();
        let timeout_fut = async move {
            if matches!(timer_type, TimerType::SenderResponse) {
                TIMER::after_millis(usbpd_traits::SENDER_RESPONSE_MS.saturating_sub(elapsed)).await;
            } else {
                Self::get_timer(timer_type).await;
            }
        };
        let receive_fut = async {
            loop {
                match self.receive_message_inner().await {
                    Ok(message) => {
                        if matches!(
                            message.header.message_type(),
                            MessageType::Control(ControlMessageType::GoodCRC)
                        ) {
                            continue;
                        }
                        return if message_types.contains(&message.header.message_type()) {
                            Ok(message)
                        } else {
                            Err(ProtocolError::UnexpectedMessage)
                        };
                    }
                    Err(RxError::ParseError(_)) => unreachable!(),
                    Err(other) => return Err(other.into()),
                }
            }
        };

        match select(timeout_fut, receive_fut).await {
            Either::First(_) => Err(RxError::ReceiveTimeout.into()),
            Either::Second(receive_result) => receive_result,
        }
    }

    /// Perform a hard-reset procedure.
    ///
    // See spec, [6.7.1.1]
    pub async fn hard_reset(&mut self) -> Result<(), ProtocolError> {
        self.counters.tx_message.reset();
        self.counters.retry.reset();

        loop {
            match self.driver.transmit_hard_reset().await {
                Ok(_) | Err(DriverTxError::HardReset) => break,
                Err(DriverTxError::Discarded | DriverTxError::Deferred | DriverTxError::NotAcknowledged) => (),
            }
        }

        self.reset_revision();
        trace!("Performed hard reset");
        Ok(())
    }

    /// BIST Carrier Mode through the driver (section 5.4.1). `Ok(false)`: the
    /// driver does not support it and sent nothing.
    pub async fn transmit_bist_carrier(&mut self) -> Result<bool, ProtocolError> {
        match self.driver.transmit_bist_carrier().await {
            Ok(()) => Ok(true),
            Err(DriverTxError::HardReset) => Err(TxError::HardReset.into()),
            Err(DriverTxError::Discarded | DriverTxError::Deferred | DriverTxError::NotAcknowledged) => Ok(false),
        }
    }

    /// Wait for VBUS to be available.
    pub async fn wait_for_vbus(&mut self) {
        self.driver.wait_for_vbus().await
    }

    /// Transmit a control message of the provided type.
    pub async fn transmit_control_message(&mut self, message_type: ControlMessageType) -> Result<(), ProtocolError> {
        let message = Message::new(Header::new_control(
            self.default_header,
            self.counters.tx_message,
            message_type,
        ));

        self.transmit(message).await
    }

    /// Transmit an extended control message of the provided type.
    pub async fn transmit_extended_control_message(
        &mut self,
        message_type: ExtendedControlMessageType,
    ) -> Result<(), ProtocolError> {
        // Per USB PD spec 6.2.1.1.2: for extended messages, num_objects must be non-zero.
        // ExtendedControl = 2-byte extended header + 2-byte data = 4 bytes = 1 data object.
        let mut message = Message::new(Header::new_extended(
            self.default_header,
            self.counters.tx_message,
            ExtendedMessageType::ExtendedControl,
            1,
        ));

        message.payload = Some(Payload::Extended(Extended::ExtendedControl(
            message::extended::extended_control::ExtendedControl::default().with_message_type(message_type),
        )));

        self.transmit(message).await
    }

    /// Transmit an EPR mode data message.
    pub async fn transmit_epr_mode(
        &mut self,
        action: message::data::epr_mode::Action,
        data: u8,
    ) -> Result<(), ProtocolError> {
        let header = Header::new_data(
            self.default_header,
            self.counters.tx_message,
            DataMessageType::EprMode,
            1,
        );

        let mdo = EprModeDataObject::default().with_action(action).with_data(data);

        self.transmit(Message::new_with_data(header, Data::EprMode(mdo))).await
    }

    /// Transmit a chunk request message per USB PD spec 6.12.2.1.2.4.
    ///
    /// A chunk request is an extended message with:
    /// - The same message type as the chunked message being received
    /// - Extended header with: chunked=1, request_chunk=1, chunk_number=requested_chunk, data_size=0
    async fn transmit_chunk_request(
        &mut self,
        message_type: ExtendedMessageType,
        chunk_number: u8,
    ) -> Result<(), RxError> {
        trace!("Transmit chunk request for {:?} chunk {}", message_type, chunk_number);

        // Build extended header for chunk request
        let ext_header = message::extended::ExtendedHeader::default()
            .with_chunked(true)
            .with_request_chunk(true)
            .with_chunk_number(chunk_number);

        // Build message header - num_objects = 1 for the extended header word
        let header = Header::new_extended(self.default_header, self.counters.tx_message, message_type, 1);

        // Build message bytes manually
        let mut buffer = Self::get_message_buffer();
        let mut offset = header.to_bytes(&mut buffer);
        offset += ext_header.to_bytes(&mut buffer[offset..]);
        // Pad to 4-byte Data Object boundary per USB PD spec.
        // Extended header is 2 bytes, so add 2 bytes padding to complete the Data Object.
        // Buffer is already zeroed, so just advance offset.
        offset += 2;

        // Transmit and wait for GoodCRC
        if DRIVER::HAS_AUTO_RETRY {
            match self.driver.transmit(&buffer[..offset]).await {
                Ok(()) => {
                    self.counters.retry.reset();
                    _ = self.counters.tx_message.increment();
                    Ok(())
                }
                Err(DriverTxError::HardReset) => Err(RxError::HardReset),
                Err(DriverTxError::Discarded | DriverTxError::Deferred | DriverTxError::NotAcknowledged) => {
                    Err(RxError::ReceiveTimeout)
                }
            }
        } else {
            match self.transmit_inner(&buffer[..offset]).await {
                Ok(_) => self.wait_for_good_crc().await,
                Err(TxError::HardReset) => Err(RxError::HardReset),
                Err(TxError::UnchunkedExtendedMessagesNotSupported | TxError::AvsVoltageAlignmentInvalid) => {
                    unreachable!("validation should happen before transmit_inner")
                }
            }
        }
    }

    /// Transmit a Revision Message (section 6.4.11) in response to Get_Revision.
    pub async fn transmit_revision(&mut self, revision_data_object: u32) -> Result<(), ProtocolError> {
        let header = Header::new_data(
            self.default_header,
            self.counters.tx_message,
            DataMessageType::Revision,
            1,
        );
        self.transmit(Message::new_with_data(header, Data::Revision(revision_data_object)))
            .await
    }

    /// Transmit Sink_Capabilities_Extended (section 6.5.16) in response to
    /// Get_Sink_Cap_Extended. Sinks here never set Unchunked Extended Messages
    /// Supported, so section 6.5.1.1 (Table 6.49) requires Chunked. The 24-byte
    /// SKEDB is a single Chunk 0 (no Chunking layer): 2 + 24 bytes padded with
    /// 00h to 7 Data Objects (Table 6.48).
    pub async fn transmit_sink_capabilities_extended(&mut self, block: [u8; 24]) -> Result<(), ProtocolError> {
        let header = Header::new_extended(
            self.default_header,
            self.counters.tx_message,
            ExtendedMessageType::SinkCapabilitiesExtended,
            7,
        );
        let mut message = Message::new(header);
        message.payload = Some(Payload::Extended(Extended::SinkCapabilitiesExtended(block)));
        self.transmit(message).await
    }

    /// Sink Alert: one ADO (sections 6.4.5, 9.2.8.3.1).
    pub async fn transmit_sink_alert(&mut self, ado: u32) -> Result<(), ProtocolError> {
        let header = Header::new_data(self.default_header, self.counters.tx_message, DataMessageType::Alert, 1);
        self.transmit(Message::new_with_data(header, Data::Alert(ado))).await
    }

    /// SOP Status: Chunked Chunk 0, seven bytes padded to three Data Objects
    /// (Tables 6.48, 6.51). No multi-Chunk assembly is needed.
    pub async fn transmit_status(&mut self, block: [u8; 7]) -> Result<(), ProtocolError> {
        let header = Header::new_extended(
            self.default_header,
            self.counters.tx_message,
            ExtendedMessageType::Status,
            3,
        );
        let mut message = Message::new(header);
        message.payload = Some(Payload::Extended(Extended::Status(
            heapless::Vec::from_slice(&block).expect("7 bytes"),
        )));
        self.transmit(message).await
    }

    /// Transmit Get_Manufacturer_Info (section 6.5.7) for the Port (Target 0,
    /// Ref Reserved 0, Table 6.56): Chunked Chunk 0 of 2 bytes in one Data
    /// Object (Table 6.48; Sinks never set Unchunked Extended Messages Supported).
    pub async fn transmit_get_manufacturer_info(&mut self) -> Result<(), ProtocolError> {
        let header = Header::new_extended(
            self.default_header,
            self.counters.tx_message,
            ExtendedMessageType::GetManufacturerInfo,
            1,
        );
        let mut message = Message::new(header);
        message.payload = Some(Payload::Extended(Extended::GetManufacturerInfo([0, 0])));
        self.transmit(message).await
    }

    /// Transmit sink capabilities in response to Get_Sink_Cap.
    ///
    /// Per USB PD Spec R3.2 Section 6.4.1.6, sinks respond to Get_Sink_Cap messages
    /// with a Sink_Capabilities message containing PDOs describing what power levels
    /// the sink can operate at.
    pub async fn transmit_sink_capabilities(
        &mut self,
        capabilities: message::data::sink_capabilities::SinkCapabilities,
    ) -> Result<(), ProtocolError> {
        let num_objects = capabilities.num_objects();
        let header = Header::new_data(
            self.default_header,
            self.counters.tx_message,
            DataMessageType::SinkCapabilities,
            num_objects,
        );

        self.transmit(Message::new_with_data(header, Data::SinkCapabilities(capabilities)))
            .await
    }

    /// Transmit EPR sink capabilities in response to EPR_Get_Sink_Cap.
    ///
    /// Per USB PD Spec R3.2 Section 8.3.3.3.10, sinks respond to EPR_Get_Sink_Cap
    /// messages with an EPR_Sink_Capabilities message.
    pub async fn transmit_epr_sink_capabilities(
        &mut self,
        capabilities: message::data::sink_capabilities::SinkCapabilities,
    ) -> Result<(), ProtocolError> {
        // Convert SinkCapabilities PDOs to the extended message format
        let pdos: heapless::Vec<_, 7> = capabilities.0.iter().cloned().collect();
        let extended_payload = message::extended::Extended::EprSinkCapabilities(pdos);

        let header = Header::new_extended(
            self.default_header,
            self.counters.tx_message,
            ExtendedMessageType::EprSinkCapabilities,
            0, // num_objects is Reserved (0) for unchunked extended messages per spec 6.2.1.1.2
        );

        let mut message = Message::new(header);
        message.payload = Some(Payload::Extended(extended_payload));

        self.transmit(message).await
    }

    /// Transmit the device's Source Capabilities
    ///
    /// Could be sent from a Source or Dual Role Device
    pub async fn transmit_source_capabilities(
        &mut self,
        source_capabilities: &SourceCapabilities,
    ) -> Result<(), ProtocolError> {
        // Only sources can send capabilities
        debug_assert!(matches!(self.default_header.port_power_role(), PowerRole::Source));
        if source_capabilities.has_epr_pdo_in_spr_positions() {
            return Err(ProtocolError::TxError(TxError::HardReset));
        }

        let header = Header::new_data(
            self.default_header,
            self.counters.tx_message,
            DataMessageType::SourceCapabilities,
            source_capabilities.0.len() as u8, // Raw cast OK since since len has domain of [0, 8]
        );

        let message = Message::new_with_data(header, Data::SourceCapabilities(source_capabilities.clone()));

        self.transmit(message).await
    }

    /// Transmit the device's EPR Source Capabilities
    ///
    /// Could be sent from a Source or Dual Role Device
    pub async fn transmit_epr_source_capabilities(
        &mut self,
        source_capabilities: &SourceCapabilities,
    ) -> Result<(), ProtocolError> {
        debug_assert!(matches!(self.default_header.port_power_role(), PowerRole::Source));

        let pdos: heapless::Vec<_, 16> = source_capabilities.0.iter().cloned().collect();
        let extended_payload = message::extended::Extended::EprSourceCapabilities(pdos);

        let header = Header::new_extended(
            self.default_header,
            self.counters.tx_message,
            ExtendedMessageType::EprSourceCapabilities,
            0, // num_objects is 0 for extended messages
        );

        let mut message = Message::new(header);
        message.payload = Some(Payload::Extended(extended_payload));

        self.transmit(message).await
    }
}

#[repr(transparent)]
#[derive(Debug)]
/// The USB PD Protocol Layer for a `Sink`
pub(crate) struct SinkProtocolLayer<DRIVER: Driver, TIMER: Timer>(ProtocolLayer<DRIVER, TIMER>);

impl<DRIVER: Driver, TIMER: Timer> core::ops::Deref for SinkProtocolLayer<DRIVER, TIMER> {
    type Target = ProtocolLayer<DRIVER, TIMER>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<DRIVER: Driver, TIMER: Timer> core::ops::DerefMut for SinkProtocolLayer<DRIVER, TIMER> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<DRIVER: Driver, TIMER: Timer> SinkProtocolLayer<DRIVER, TIMER> {
    /// Create a new protocol layer from a driver and default header.
    pub fn new(driver: DRIVER, default_header: Header) -> Self {
        Self(ProtocolLayer::new(driver, default_header))
    }

    /// Wait for the source to provide its capabilities.
    pub async fn wait_for_source_capabilities(&mut self) -> Result<Message, ProtocolError> {
        // Only sinks can await capabilities.
        debug_assert!(matches!(self.default_header.port_power_role(), PowerRole::Sink));

        self.receive_message_type(
            &[
                MessageType::Data(message::header::DataMessageType::SourceCapabilities),
                MessageType::Extended(ExtendedMessageType::EprSourceCapabilities),
            ],
            TimerType::SinkWaitCap,
        )
        .await
    }

    /// Request a certain power level from the source.
    pub async fn request_power(&mut self, power_source_request: request::PowerSource) -> Result<(), ProtocolError> {
        // Only sinks can request from a supply.
        debug_assert!(matches!(self.default_header.port_power_role(), PowerRole::Sink));

        let message_type = power_source_request.message_type();
        let num_objects = power_source_request.num_objects();
        let header = Header::new_data(self.default_header, self.counters.tx_message, message_type, num_objects);

        self.transmit(Message::new_with_data(header, Data::Request(power_source_request)))
            .await
    }
}

#[repr(transparent)]
#[derive(Debug)]
/// The USB PD Protocol Layer for a `Source`
pub(crate) struct SourceProtocolLayer<DRIVER: Driver, TIMER: Timer>(ProtocolLayer<DRIVER, TIMER>);

impl<DRIVER: Driver, TIMER: Timer> core::ops::Deref for SourceProtocolLayer<DRIVER, TIMER> {
    type Target = ProtocolLayer<DRIVER, TIMER>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<DRIVER: Driver, TIMER: Timer> core::ops::DerefMut for SourceProtocolLayer<DRIVER, TIMER> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<DRIVER: Driver, TIMER: Timer> SourceProtocolLayer<DRIVER, TIMER> {
    /// Create a new protocol layer from a driver and default header.
    pub fn new(driver: DRIVER, default_header: Header) -> Self {
        Self(ProtocolLayer::new(driver, default_header))
    }

    /// Wait for the sink to request a capability with a Request Message.
    pub async fn wait_for_request(&mut self) -> Result<Message, ProtocolError> {
        // Only sources await a sink power request
        debug_assert!(matches!(self.default_header.port_power_role(), PowerRole::Source));

        self.receive_message_type(
            &[MessageType::Data(message::header::DataMessageType::Request)],
            TimerType::SenderResponse,
        )
        .await
    }

    /// Wait for the sink to request a capability with an EPR_Request Message
    pub async fn wait_for_epr_request(&mut self) -> Result<Message, ProtocolError> {
        // Only sources await a sink power request
        debug_assert!(matches!(self.default_header.port_power_role(), PowerRole::Source));

        self.receive_message_type(
            &[MessageType::Data(message::header::DataMessageType::EprRequest)],
            TimerType::SenderResponse,
        )
        .await
    }
}

#[cfg(test)]
mod tests {

    use core::iter::zip;

    use super::ProtocolLayer;
    use super::message::data::Data;
    use super::message::data::source_capabilities::SourceCapabilities;
    use super::message::header::Header;
    use crate::dummy::{
        DUMMY_CAPABILITIES, DummyDriver, DummyTimer, MAX_DATA_MESSAGE_SIZE, get_dummy_source_capabilities,
    };
    use crate::protocol_layer::message::Payload;

    fn get_protocol_layer() -> ProtocolLayer<DummyDriver<MAX_DATA_MESSAGE_SIZE>, DummyTimer> {
        ProtocolLayer::new(
            DummyDriver::new(),
            Header::new_template(
                crate::DataRole::Ufp,
                crate::PowerRole::Sink,
                super::message::header::SpecificationRevision::R3_X,
            ),
        )
    }

    #[tokio::test]
    async fn revision_lock_survives_soft_reset_but_not_hard_reset() {
        use super::message::header::{ControlMessageType, DataMessageType, SpecificationRevision};
        let mut protocol = get_protocol_layer();
        let mut caps = DUMMY_CAPABILITIES;
        caps[0] = (caps[0] & !0xc0) | 0x40;
        protocol.driver.inject_received_data(&caps);
        protocol.receive_message().await.unwrap();
        assert_eq!(protocol.driver.probe_transmitted_data()[0] & 0xc0, 0x40); // software GoodCRC
        protocol.driver.inject_received_data(&[0x61, 0x01]); // Request ID0 ACK
        let header = Header::new_data(
            *protocol.header(),
            protocol.counters.tx_message,
            DataMessageType::Request,
            1,
        );
        let request = super::message::Message::new_with_data(
            header,
            Data::Request(super::request::PowerSource::FixedVariableSupply(
                super::request::FixedVariableSupply(0x1100_280a),
            )),
        );
        protocol.transmit(request).await.unwrap();
        protocol.driver.probe_transmitted_data();
        protocol.driver.inject_received_data(&[0x8d, 0x01]);
        assert!(matches!(
            protocol.receive_message().await,
            Err(super::ProtocolError::RxError(super::RxError::SoftReset))
        ));
        assert_eq!(protocol.header().0 & 0xc0, 0x40); // locked revision never changes
        protocol.reset(); // Soft_Reset counter boundary, not revision boundary
        assert!(protocol.revision_locked);
        for chunked in [false, true] {
            // Single-chunk extended frame with four bytes of opaque payload.
            protocol.driver.inject_received_data(&[
                0xa2,
                if chunked { 0xa3 } else { 0xa1 },
                4,
                if chunked { 0x80 } else { 0 },
                0,
                0,
                0,
                0,
                0,
                0,
            ]);
            protocol.receive_message().await.unwrap();
            assert_eq!(protocol.driver.probe_transmitted_data()[0] & 0xc0, 0x80);
            assert_eq!(protocol.header().0 & 0xc0, 0x40); // ACK revision does not renegotiate SOP
        }
        protocol.driver.inject_received_data(&[0x81, 0x00]); // GoodCRC ID0 after counter reset
        protocol
            .transmit_control_message(ControlMessageType::Accept)
            .await
            .unwrap();
        assert_eq!(protocol.driver.probe_transmitted_data()[0] & 0xc0, 0x40);
        protocol.hard_reset().await.unwrap(); // dummy driver only; no physical signaling
        assert!(!protocol.revision_locked);
        protocol
            .detect_revision(
                header
                    .with_spec_revision(SpecificationRevision::R3_X)
                    .with_message_type_raw(DataMessageType::SourceCapabilities as u8),
            )
            .unwrap();
        assert_eq!(protocol.header().0 & 0xc0, 0x80);
    }

    #[test]
    fn source_revision_detection_is_bounded_by_supported_revision() {
        use super::message::header::{DataMessageType, SpecificationRevision};
        let header = Header::new_template(
            crate::DataRole::Dfp,
            crate::PowerRole::Source,
            SpecificationRevision::R2_0,
        );
        let mut protocol: ProtocolLayer<DummyDriver<MAX_DATA_MESSAGE_SIZE>, DummyTimer> =
            ProtocolLayer::new(DummyDriver::new(), header);
        protocol
            .detect_revision(
                header
                    .with_spec_revision(SpecificationRevision::R3_X)
                    .with_num_objects(1)
                    .with_message_type_raw(DataMessageType::Request as u8),
            )
            .unwrap();
        assert!(protocol.revision_locked);
        assert_eq!(protocol.header().0 & 0xc0, 0x40);
        protocol.reset();
        assert!(protocol.revision_locked);
    }

    #[tokio::test]
    async fn sender_response_uses_time_remaining_since_goodcrc() {
        use usbpd_traits::{Driver, DriverRxError, DriverTxError};

        use super::message::header::ControlMessageType;
        use crate::timers::{Timer, TimerType};
        std::thread_local! {
            static REQUESTED_MS: core::cell::Cell<u64> = const { core::cell::Cell::new(u64::MAX) };
        }
        struct ElapsedDriver(u64);
        impl Driver for ElapsedDriver {
            async fn wait_for_vbus(&mut self) {}
            async fn receive(&mut self, _: &mut [u8]) -> Result<usize, DriverRxError> {
                core::future::pending().await
            }
            async fn transmit(&mut self, _: &[u8]) -> Result<(), DriverTxError> {
                Ok(())
            }
            async fn transmit_hard_reset(&mut self) -> Result<(), DriverTxError> {
                Ok(())
            }
            fn sender_response_elapsed_ms(&self) -> u64 {
                self.0
            }
        }
        struct RecordingTimer;
        impl Timer for RecordingTimer {
            async fn after_millis(ms: u64) {
                REQUESTED_MS.with(|value| value.set(ms));
            }
        }
        for (timer, elapsed, remaining) in [
            (TimerType::SenderResponse, 0, 30),
            (TimerType::SenderResponse, 15, 15),
            (TimerType::SenderResponse, 29, 1),
            (TimerType::SenderResponse, 30, 0),
            (TimerType::SenderResponse, 45, 0),
            (TimerType::SinkWaitCap, 15, 465),
        ] {
            let mut protocol =
                ProtocolLayer::<_, RecordingTimer>::new(ElapsedDriver(elapsed), *get_protocol_layer().header());
            assert!(matches!(
                protocol
                    .receive_message_type(
                        &[super::message::header::MessageType::Control(ControlMessageType::Accept)],
                        timer,
                    )
                    .await,
                Err(super::ProtocolError::RxError(super::RxError::ReceiveTimeout))
            ));
            REQUESTED_MS.with(|value| assert_eq!(value.get(), remaining));
        }
    }

    #[tokio::test]
    async fn test_it() {
        let mut protocol_layer = get_protocol_layer();

        protocol_layer.driver.inject_received_data(&DUMMY_CAPABILITIES);
        let message = protocol_layer.receive_message().await.unwrap();

        if let Some(Payload::Data(Data::SourceCapabilities(SourceCapabilities(caps)))) = message.payload {
            for (cap, dummy_cap) in zip(caps, get_dummy_source_capabilities()) {
                assert_eq!(cap, dummy_cap);
            }
        } else {
            panic!()
        }
    }
}
