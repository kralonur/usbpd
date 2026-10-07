//! Regression tests for backported protocol fixes.

use super::*;
use crate::dummy::{DUMMY_CAPABILITIES, DummyDriver, DummyTimer, MAX_DATA_MESSAGE_SIZE};

fn protocol() -> ProtocolLayer<DummyDriver<MAX_DATA_MESSAGE_SIZE>, DummyTimer> {
    ProtocolLayer::new(
        DummyDriver::new(),
        Header::new_template(crate::DataRole::Ufp, PowerRole::Sink, SpecificationRevision::R3_X),
    )
}

#[tokio::test]
async fn malformed_frames_are_dropped_while_waiting_for_a_response() {
    let mut protocol = protocol();
    // Invalid specification revision, then an Accept from the source.
    protocol.driver.inject_received_data(&[0xe3, 0x01]);
    protocol.driver.inject_received_data(&[0xa3, 0x01]);
    let message = protocol
        .receive_message_type(
            &[MessageType::Control(ControlMessageType::Accept)],
            TimerType::SenderResponse,
        )
        .await
        .unwrap();
    std::assert_eq!(
        message.header.message_type(),
        MessageType::Control(ControlMessageType::Accept)
    );
}

#[tokio::test]
async fn truncated_extended_frame_does_not_panic() {
    let mut protocol = protocol();
    protocol.driver.inject_received_data(&[0xa2, 0x81]);
    protocol.driver.inject_received_data(&[0xa3, 0x01]);
    protocol
        .receive_message_type(
            &[MessageType::Control(ControlMessageType::Accept)],
            TimerType::SenderResponse,
        )
        .await
        .unwrap();
}

#[tokio::test]
async fn layer_resets_clear_receive_and_chunk_state() {
    for hard in [false, true] {
        let mut protocol = protocol();
        protocol.counters.rx_message = Some(Counter::new_from_value(CounterType::MessageId, 4));
        protocol.counters.tx_message.set(3);
        protocol.counters.retry.set(1);
        protocol.extended_rx_buffer.extend_from_slice(&[1, 2]).unwrap();
        protocol.extended_rx_expected = Some((ExtendedMessageType::Status, 40, 1));
        protocol.revision_locked = true;
        protocol.pending_rx.push_back(Message::new(*protocol.header())).unwrap();
        if hard {
            protocol.hard_reset().await.unwrap();
        } else {
            protocol.reset();
        }
        std::assert!(protocol.counters.rx_message.is_none());
        std::assert_eq!(protocol.counters.tx_message.value(), 0);
        std::assert_eq!(protocol.counters.retry.value(), 0);
        std::assert!(protocol.extended_rx_buffer.is_empty());
        std::assert!(protocol.extended_rx_expected.is_none());
        std::assert!(protocol.pending_rx.is_empty());
        std::assert_eq!(protocol.revision_locked, !hard);
    }
}

#[tokio::test]
async fn soft_reset_is_acknowledged_without_renegotiating_a_locked_revision() {
    let mut protocol = protocol();
    protocol.default_header.set_spec_revision(SpecificationRevision::R2_0);
    protocol.revision_locked = true;
    protocol.counters.rx_message = Some(Counter::new_from_value(CounterType::MessageId, 0));
    protocol.driver.inject_received_data(&[0xad, 0x01]);
    std::assert!(matches!(
        protocol.receive_message().await,
        Err(ProtocolError::RxError(RxError::SoftReset))
    ));
    let ack = Message::from_bytes(&protocol.driver.probe_transmitted_data()).unwrap();
    std::assert_eq!(
        ack.header.message_type(),
        MessageType::Control(ControlMessageType::GoodCRC)
    );
    std::assert!(matches!(ack.header.spec_revision(), Ok(SpecificationRevision::R3_X)));
    std::assert!(matches!(
        protocol.header().spec_revision(),
        Ok(SpecificationRevision::R2_0)
    ));
}

#[tokio::test]
async fn software_retry_exhaustion_advances_message_id() {
    let mut protocol = protocol();
    // Wrong GoodCRC IDs must retry, not escape as an acknowledgement error.
    for _ in 0..3 {
        protocol.driver.inject_received_data(&[0x81, 0x02]);
    }
    std::assert!(matches!(
        protocol
            .transmit_control_message(ControlMessageType::GetSourceCap)
            .await,
        Err(ProtocolError::TransmitRetriesExceeded(2))
    ));
    std::assert_eq!(protocol.counters.tx_message.value(), 1);
    for _ in 0..3 {
        let message = Message::from_bytes(&protocol.driver.probe_transmitted_data()).unwrap();
        std::assert_eq!(message.header.message_id(), 0);
    }
}

#[test]
fn avs_alignment_validation_uses_the_requested_pdo() {
    use message::data::source_capabilities::{Augmented, EprAdjustableVoltageSupply, FixedSupply, PowerDataObject};
    let protocol = protocol();
    let header = Header::new_data(
        *protocol.header(),
        protocol.counters.tx_message,
        DataMessageType::EprRequest,
        2,
    );
    // Object 8 is AVS; object-position bits are not an APDO type discriminator.
    let request = request::EprRequestDataObject {
        rdo: 0x8000_0200,
        pdo: PowerDataObject::Augmented(Augmented::Epr(EprAdjustableVoltageSupply(0xd000_0000))),
    };
    let message = Message::new_with_data(header, Data::Request(request::PowerSource::EprRequest(request)));
    std::assert!(matches!(
        ProtocolLayer::<DummyDriver<MAX_DATA_MESSAGE_SIZE>, DummyTimer>::validate_outgoing_message(&message),
        Err(TxError::AvsVoltageAlignmentInvalid)
    ));
    // Object 3 is fixed; its position must not cause AVS validation.
    let request = request::EprRequestDataObject {
        rdo: 0x3000_0200,
        pdo: PowerDataObject::FixedSupply(FixedSupply(0)),
    };
    let message = Message::new_with_data(header, Data::Request(request::PowerSource::EprRequest(request)));
    std::assert!(
        ProtocolLayer::<DummyDriver<MAX_DATA_MESSAGE_SIZE>, DummyTimer>::validate_outgoing_message(&message).is_ok()
    );
}

struct HardwareDriver(DriverTxError);
impl Driver for HardwareDriver {
    const HAS_AUTO_GOOD_CRC: bool = true;
    const HAS_AUTO_RETRY: bool = true;
    async fn wait_for_vbus(&mut self) {}
    async fn receive(&mut self, _: &mut [u8]) -> Result<usize, DriverRxError> {
        core::future::pending().await
    }
    async fn transmit(&mut self, _: &[u8]) -> Result<(), DriverTxError> {
        Err(self.0)
    }
    async fn transmit_hard_reset(&mut self) -> Result<(), DriverTxError> {
        Ok(())
    }
}

#[tokio::test]
async fn hardware_retry_exhaustion_advances_id_but_deferral_does_not() {
    for (error, expected) in [
        (DriverTxError::Discarded, 1),
        (DriverTxError::NotAcknowledged, 1),
        (DriverTxError::Deferred, 0),
    ] {
        let mut protocol = ProtocolLayer::<_, DummyTimer>::new(HardwareDriver(error), *protocol().header());
        let result = protocol
            .transmit_control_message(ControlMessageType::GetSourceCap)
            .await;
        std::assert!(if expected == 0 {
            matches!(result, Err(ProtocolError::TransmitDeferred))
        } else {
            matches!(result, Err(ProtocolError::TransmitRetriesExceeded(2)))
        });
        std::assert_eq!(protocol.counters.tx_message.value(), expected);
    }
}

#[tokio::test]
async fn partner_message_during_goodcrc_wait_is_acknowledged_and_delivered_once() {
    let mut protocol = protocol();
    protocol.default_header.set_spec_revision(SpecificationRevision::R2_0);
    protocol.revision_locked = true;
    protocol.driver.inject_received_data(&DUMMY_CAPABILITIES);
    protocol.driver.inject_received_data(&DUMMY_CAPABILITIES);
    protocol.driver.inject_received_data(&[0x81, 0]);
    protocol
        .transmit_control_message(ControlMessageType::GetSourceCap)
        .await
        .unwrap();
    for kind in [
        ControlMessageType::GetSourceCap,
        ControlMessageType::GoodCRC,
        ControlMessageType::GoodCRC,
    ] {
        let message = Message::from_bytes(&protocol.driver.probe_transmitted_data()).unwrap();
        std::assert_eq!(message.header.message_type(), MessageType::Control(kind));
    }
    let message = protocol.receive_message().await.unwrap();
    std::assert!(matches!(
        message.payload,
        Some(Payload::Data(Data::SourceCapabilities(_)))
    ));
    std::assert!(matches!(
        protocol.header().spec_revision(),
        Ok(SpecificationRevision::R2_0)
    ));
    std::assert!(matches!(
        protocol
            .receive_message_type(
                &[MessageType::Data(DataMessageType::SourceCapabilities)],
                TimerType::SenderResponse,
            )
            .await,
        Err(ProtocolError::RxError(RxError::ReceiveTimeout))
    ));
}
