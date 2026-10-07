//! Tests for the policy engine.

use super::Sink;
use crate::counters::{Counter, CounterType};
use crate::dummy::{DUMMY_CAPABILITIES, DummyDriver, DummySinkDevice, DummyTimer, MAX_DATA_MESSAGE_SIZE};
use crate::protocol_layer::message::data::Data;
use crate::protocol_layer::message::data::epr_mode::Action;
use crate::protocol_layer::message::data::request::PowerSource;
use crate::protocol_layer::message::data::source_capabilities::{PowerDataObject, SourceCapabilities};
use crate::protocol_layer::message::header::{
    ControlMessageType, DataMessageType, ExtendedMessageType, Header, MessageType,
};
use crate::protocol_layer::message::{Message, Payload};
use crate::sink::device_policy_manager::PpsStatusOutcome;
use crate::sink::policy_engine::State;

fn get_policy_engine() -> Sink<DummyDriver<MAX_DATA_MESSAGE_SIZE>, DummyTimer, DummySinkDevice> {
    Sink::new(DummyDriver::new(), DummySinkDevice {})
}

fn source_capabilities_fixture() -> SourceCapabilities {
    let Some(Payload::Data(Data::SourceCapabilities(capabilities))) =
        Message::from_bytes(&DUMMY_CAPABILITIES).unwrap().payload
    else {
        std::panic!("capabilities fixture");
    };
    capabilities
}

fn simulate_source_control_message<DPM: crate::sink::device_policy_manager::DevicePolicyManager>(
    policy_engine: &mut Sink<DummyDriver<MAX_DATA_MESSAGE_SIZE>, DummyTimer, DPM>,
    control_message_type: ControlMessageType,
    message_id: u8,
) {
    let header = *policy_engine.protocol_layer.header();
    let mut buf = [0u8; MAX_DATA_MESSAGE_SIZE];

    Message::new(Header::new_control(
        header,
        Counter::new_from_value(CounterType::MessageId, message_id),
        control_message_type,
    ))
    .to_bytes(&mut buf);
    policy_engine.protocol_layer.driver().inject_received_data(&buf);
}

/// Get a header template for simulating source messages (Source/Dfp roles).
/// This flips the roles from the sink's perspective to simulate messages from the source.
fn get_source_header_template() -> Header {
    use crate::protocol_layer::message::header::SpecificationRevision;
    use crate::{DataRole, PowerRole};

    // Source messages have Source/Dfp roles (opposite of sink's Sink/Ufp)
    Header::new_template(DataRole::Dfp, PowerRole::Source, SpecificationRevision::R3_X)
}

/// Simulate an EPR Mode data message from the source with proper API.
/// Returns the serialized bytes for assertion.
fn simulate_source_epr_mode_message<DPM: crate::sink::device_policy_manager::DevicePolicyManager>(
    policy_engine: &mut Sink<DummyDriver<MAX_DATA_MESSAGE_SIZE>, DummyTimer, DPM>,
    action: Action,
    message_id: u8,
) -> heapless::Vec<u8, MAX_DATA_MESSAGE_SIZE> {
    use crate::protocol_layer::message::data::epr_mode::EprModeDataObject;

    let source_header = get_source_header_template();
    let header = Header::new_data(
        source_header,
        Counter::new_from_value(CounterType::MessageId, message_id),
        DataMessageType::EprMode,
        1, // 1 data object (the EprModeDataObject)
    );

    let epr_mode = EprModeDataObject::default().with_action(action);
    let message = Message::new_with_data(header, Data::EprMode(epr_mode));

    let mut buf = [0u8; MAX_DATA_MESSAGE_SIZE];
    let len = message.to_bytes(&mut buf);
    policy_engine.protocol_layer.driver().inject_received_data(&buf[..len]);

    let mut result = heapless::Vec::new();
    result.extend_from_slice(&buf[..len]).unwrap();
    result
}

/// Simulate an EprKeepAliveAck extended control message from the source.
/// Returns the serialized bytes for assertion.
fn simulate_epr_keep_alive_ack<DPM: crate::sink::device_policy_manager::DevicePolicyManager>(
    policy_engine: &mut Sink<DummyDriver<MAX_DATA_MESSAGE_SIZE>, DummyTimer, DPM>,
    message_id: u8,
) -> heapless::Vec<u8, MAX_DATA_MESSAGE_SIZE> {
    use crate::protocol_layer::message::Payload;
    use crate::protocol_layer::message::extended::Extended;
    use crate::protocol_layer::message::extended::extended_control::{ExtendedControl, ExtendedControlMessageType};

    let source_header = get_source_header_template();
    // Create extended message header (num_objects=0 as used in transmit_extended_control_message)
    let header = Header::new_extended(
        source_header,
        Counter::new_from_value(CounterType::MessageId, message_id),
        ExtendedMessageType::ExtendedControl,
        0,
    );

    // Create the message with proper payload
    let mut message = Message::new(header);
    message.payload = Some(Payload::Extended(Extended::ExtendedControl(
        ExtendedControl::default().with_message_type(ExtendedControlMessageType::EprKeepAliveAck),
    )));

    // Serialize and inject
    let mut buf = [0u8; MAX_DATA_MESSAGE_SIZE];
    let len = message.to_bytes(&mut buf);
    policy_engine.protocol_layer.driver().inject_received_data(&buf[..len]);

    let mut result = heapless::Vec::new();
    result.extend_from_slice(&buf[..len]).unwrap();
    result
}

#[tokio::test]
async fn declined_change_preserves_established_request_and_default_wait_retry() {
    use crate::protocol_layer::message::data::request::FixedVariableSupply;
    for response in [ControlMessageType::Reject, ControlMessageType::Wait] {
        let mut engine = get_policy_engine();
        let established = PowerSource::FixedVariableSupply(FixedVariableSupply(0x1000_280a));
        let attempted = PowerSource::FixedVariableSupply(FixedVariableSupply(0x2000_280a));
        engine.contract = super::Contract::Explicit;
        engine.established_request = Some(established);
        engine.source_capabilities = Some(source_capabilities_fixture());
        engine.state = State::SelectCapability(attempted);
        simulate_source_control_message(&mut engine, ControlMessageType::GoodCRC, 0);
        simulate_source_control_message(&mut engine, response, 1);
        engine.run_step().await.unwrap();
        let State::Ready(PowerSource::FixedVariableSupply(current), after_wait) = engine.state else {
            std::panic!("declined request did not return to Ready");
        };
        std::assert_eq!(current.0, 0x1000_280a);
        std::assert_eq!(after_wait, response == ControlMessageType::Wait);
        if after_wait {
            engine.run_step().await.unwrap();
            std::assert!(
                matches!(engine.state, State::SelectCapability(PowerSource::FixedVariableSupply(rdo))
                if rdo.0 == 0x2000_280a)
            );
        } else {
            std::assert!(engine.wait_request.is_none());
        }
    }
}

#[tokio::test]
async fn test_negotiation() {
    // Instantiated in `Discovery` state
    let mut policy_engine = get_policy_engine();

    // Provide capabilities
    policy_engine
        .protocol_layer
        .driver()
        .inject_received_data(&DUMMY_CAPABILITIES);

    // `Discovery` -> `WaitForCapabilities`
    policy_engine.run_step().await.unwrap();

    // `WaitForCapabilities` -> `EvaluateCapabilities`
    policy_engine.run_step().await.unwrap();

    let good_crc = Message::from_bytes(&policy_engine.protocol_layer.driver().probe_transmitted_data()).unwrap();
    assert!(matches!(
        good_crc.header.message_type(),
        MessageType::Control(ControlMessageType::GoodCRC)
    ));

    // Simulate `GoodCrc` with ID 0.
    simulate_source_control_message(&mut policy_engine, ControlMessageType::GoodCRC, 0);

    // `EvaluateCapabilities` -> `SelectCapability`
    policy_engine.run_step().await.unwrap();

    // Simulate `Accept` message.
    simulate_source_control_message(&mut policy_engine, ControlMessageType::Accept, 1);

    // `SelectCapability` -> `TransitionSink`
    policy_engine.run_step().await.unwrap();

    let request_capabilities =
        Message::from_bytes(&policy_engine.protocol_layer.driver().probe_transmitted_data()).unwrap();
    assert!(matches!(
        request_capabilities.header.message_type(),
        MessageType::Data(DataMessageType::Request)
    ));

    // Simulate `PsRdy` message.
    simulate_source_control_message(&mut policy_engine, ControlMessageType::PsRdy, 2);

    // `TransitionSink` -> `Ready`
    policy_engine.run_step().await.unwrap();
    assert!(matches!(policy_engine.state, State::Ready(..)));

    let good_crc = Message::from_bytes(&policy_engine.protocol_layer.driver().probe_transmitted_data()).unwrap();
    assert!(matches!(
        good_crc.header.message_type(),
        MessageType::Control(ControlMessageType::GoodCRC)
    ));
}

#[tokio::test]
async fn test_epr_negotiation() {
    use crate::dummy::{DUMMY_SPR_CAPS_EPR_CAPABLE, DummySinkEprDevice};

    // Create policy engine with EPR-capable DPM
    let mut policy_engine: Sink<DummyDriver<MAX_DATA_MESSAGE_SIZE>, DummyTimer, DummySinkEprDevice> =
        Sink::new(DummyDriver::new(), DummySinkEprDevice::new());

    // === Phase 1: Initial SPR Negotiation ===
    // Using same flow as test_negotiation
    eprintln!("Starting test");

    policy_engine
        .protocol_layer
        .driver()
        .inject_received_data(&DUMMY_SPR_CAPS_EPR_CAPABLE);

    // Discovery -> WaitForCapabilities
    eprintln!("run_step 1");
    policy_engine.run_step().await.unwrap();

    // WaitForCapabilities -> EvaluateCapabilities
    eprintln!("run_step 2");
    policy_engine.run_step().await.unwrap();

    eprintln!("Probing first GoodCRC");
    let good_crc = Message::from_bytes(&policy_engine.protocol_layer.driver().probe_transmitted_data()).unwrap();
    eprintln!("Got first GoodCRC");
    assert!(matches!(
        good_crc.header.message_type(),
        MessageType::Control(ControlMessageType::GoodCRC)
    ));

    // Simulate GoodCRC with ID 0
    simulate_source_control_message(&mut policy_engine, ControlMessageType::GoodCRC, 0);

    // EvaluateCapabilities -> SelectCapability
    policy_engine.run_step().await.unwrap();

    // Simulate Accept
    simulate_source_control_message(&mut policy_engine, ControlMessageType::Accept, 1);

    // SelectCapability -> TransitionSink
    policy_engine.run_step().await.unwrap();

    let request_capabilities =
        Message::from_bytes(&policy_engine.protocol_layer.driver().probe_transmitted_data()).unwrap();
    assert!(matches!(
        request_capabilities.header.message_type(),
        MessageType::Data(DataMessageType::Request)
    ));

    // Simulate PsRdy
    simulate_source_control_message(&mut policy_engine, ControlMessageType::PsRdy, 2);

    // TransitionSink -> Ready
    policy_engine.run_step().await.unwrap();
    eprintln!("State after last run_step: {:?}", policy_engine.state);
    assert!(matches!(policy_engine.state, State::Ready(..)));

    eprintln!(
        "Has transmitted data: {}",
        policy_engine.protocol_layer.driver().has_transmitted_data()
    );
    let good_crc = Message::from_bytes(&policy_engine.protocol_layer.driver().probe_transmitted_data()).unwrap();
    eprintln!("Got first GoodCRC: {:?}", good_crc.header.message_type());
    assert!(matches!(
        good_crc.header.message_type(),
        MessageType::Control(ControlMessageType::GoodCRC)
    ));

    // Probe any remaining messages from Phase 1
    while policy_engine.protocol_layer.driver().has_transmitted_data() {
        let msg = Message::from_bytes(&policy_engine.protocol_layer.driver().probe_transmitted_data()).unwrap();
        eprintln!("Draining leftover message: {:?}", msg.header.message_type());
    }

    eprintln!("\n=== Phase 1 Complete: SPR negotiation at 20V ===\n");

    // === Phase 2: EPR Mode Entry ===
    // Per spec 8.3.3.26.2, EPR mode entry flow:
    // 1. Sink sends EPR_Mode (Enter), starts SenderResponseTimer
    // 2. Source sends EnterAcknowledged
    // 3. Source performs cable discovery (we skip this in test)
    // 4. Source sends EnterSucceeded
    eprintln!("=== Phase 2: EPR Mode Entry ===");

    // Ready -> EprModeEntry (DPM triggers EnterEprMode event)
    policy_engine.run_step().await.unwrap();
    eprintln!("State after DPM event: {:?}", policy_engine.state);

    // Inject GoodCRC for EPR_Mode (Enter) that will be transmitted
    simulate_source_control_message(&mut policy_engine, ControlMessageType::GoodCRC, 1);

    // Inject EPR_Mode (EnterAcknowledged) using proper API
    // From capture line 78-81: RAW[6]: AA 19 00 00 00 02 (Source, msg_id=4)
    let epr_enter_ack_bytes = simulate_source_epr_mode_message(
        &mut policy_engine,
        Action::EnterAcknowledged,
        4, // message_id from capture
    );
    // Assert bytes match the real capture
    assert_eq!(
        &epr_enter_ack_bytes[..],
        &[0xAA, 0x19, 0x00, 0x00, 0x00, 0x02],
        "EPR EnterAcknowledged bytes should match capture"
    );

    // EprModeEntry: sends EPR_Mode (Enter), receives EnterAcknowledged -> EprEntryWaitForResponse
    match policy_engine.run_step().await {
        Ok(_) => eprintln!("EprModeEntry run_step succeeded"),
        Err(e) => eprintln!("EprModeEntry run_step failed: {:?}", e),
    }
    eprintln!("State after EprModeEntry: {:?}", policy_engine.state);

    // Probe EPR_Mode (Enter) message
    let epr_enter_bytes = policy_engine.protocol_layer.driver().probe_transmitted_data();
    let epr_enter = Message::from_bytes(&epr_enter_bytes).unwrap();
    eprintln!("Probed message type: {:?}", epr_enter.header.message_type());
    assert!(matches!(
        epr_enter.header.message_type(),
        MessageType::Data(DataMessageType::EprMode)
    ));
    if let Some(Payload::Data(Data::EprMode(mode))) = epr_enter.payload {
        assert_eq!(mode.action(), Action::Enter);
    } else {
        panic!("Expected EprMode Enter payload");
    }
    // Assert EPR_Mode Enter bytes match capture line 71-74: RAW[6]: 8A 14 00 00 00 01
    // Note: Our test starts from different state so message_id may differ
    eprintln!("EPR_Mode Enter bytes: {:02X?}", &epr_enter_bytes[..]);

    // Probe GoodCRC for EnterAcknowledged
    let good_crc = Message::from_bytes(&policy_engine.protocol_layer.driver().probe_transmitted_data()).unwrap();
    eprintln!("Probed GoodCRC for EnterAck: {:?}", good_crc.header.message_type());
    assert!(matches!(
        good_crc.header.message_type(),
        MessageType::Control(ControlMessageType::GoodCRC)
    ));

    // Inject EPR_Mode (EnterSucceeded) using proper API
    // From capture line 85-88: RAW[6]: AA 1B 00 00 00 03 (Source, msg_id=5)
    let epr_enter_succeeded_bytes = simulate_source_epr_mode_message(
        &mut policy_engine,
        Action::EnterSucceeded,
        5, // message_id from capture
    );
    // Assert bytes match the real capture
    assert_eq!(
        &epr_enter_succeeded_bytes[..],
        &[0xAA, 0x1B, 0x00, 0x00, 0x00, 0x03],
        "EPR EnterSucceeded bytes should match capture"
    );

    // EprEntryWaitForResponse receives EnterSucceeded -> EprWaitForCapabilities
    policy_engine.run_step().await.unwrap();
    eprintln!("State after EnterSucceeded: {:?}", policy_engine.state);

    // Probe GoodCRC for EnterSucceeded
    let good_crc = Message::from_bytes(&policy_engine.protocol_layer.driver().probe_transmitted_data()).unwrap();
    assert!(matches!(
        good_crc.header.message_type(),
        MessageType::Control(ControlMessageType::GoodCRC)
    ));

    eprintln!("=== Phase 2 Complete: EPR mode entry succeeded ===\n");

    // === Phase 3: Chunked EPR Source Capabilities ===
    // This follows the real-world capture flow per USB PD spec 6.12.2.1.2:
    // 1. Source sends chunk 0 -> Sink sends GoodCRC
    // 2. Sink sends Chunk Request (chunk=1) -> Source sends GoodCRC
    // 3. Source sends chunk 1 -> Sink sends GoodCRC
    use crate::dummy::{DUMMY_EPR_SOURCE_CAPS_CHUNK_0, DUMMY_EPR_SOURCE_CAPS_CHUNK_1};

    eprintln!("=== Phase 3: Chunked EPR Source Capabilities ===");

    // Source sends EPR_Source_Capabilities chunk 0
    policy_engine
        .protocol_layer
        .driver()
        .inject_received_data(&DUMMY_EPR_SOURCE_CAPS_CHUNK_0);

    // Inject GoodCRC for the Chunk Request that sink will send after receiving chunk 0
    // The chunk request message ID will be based on tx_message counter
    simulate_source_control_message(&mut policy_engine, ControlMessageType::GoodCRC, 2);

    // Source sends chunk 1 after receiving the chunk request
    policy_engine
        .protocol_layer
        .driver()
        .inject_received_data(&DUMMY_EPR_SOURCE_CAPS_CHUNK_1);

    // EprWaitForCapabilities -> Protocol layer:
    // - receives chunk 0, sends GoodCRC
    // - sends chunk request, waits for GoodCRC
    // - receives chunk 1, sends GoodCRC
    // - assembles message -> EvaluateCapabilities
    policy_engine.run_step().await.unwrap();

    // Probe GoodCRC for chunk 0
    let good_crc_0 = Message::from_bytes(&policy_engine.protocol_layer.driver().probe_transmitted_data()).unwrap();
    eprintln!("Chunk 0 GoodCRC: {:?}", good_crc_0.header.message_type());
    assert!(matches!(
        good_crc_0.header.message_type(),
        MessageType::Control(ControlMessageType::GoodCRC)
    ));

    // Probe the Chunk Request message (per spec 6.12.2.1.2.4)
    // Chunk requests are parsed as ChunkedExtendedMessage error, so we use parse_extended_chunk
    let chunk_req_data = policy_engine.protocol_layer.driver().probe_transmitted_data();
    let (chunk_req_header, chunk_req_ext_header, _chunk_data) = Message::parse_extended_chunk(&chunk_req_data).unwrap();
    eprintln!(
        "Chunk Request: type={:?}, chunk_number={}, request_chunk={}",
        chunk_req_header.message_type(),
        chunk_req_ext_header.chunk_number(),
        chunk_req_ext_header.request_chunk()
    );
    assert!(
        chunk_req_header.extended(),
        "Chunk request should be an extended message"
    );
    assert!(matches!(
        chunk_req_header.message_type(),
        MessageType::Extended(ExtendedMessageType::EprSourceCapabilities)
    ));
    assert!(chunk_req_ext_header.request_chunk(), "Should be a chunk request");
    assert_eq!(chunk_req_ext_header.chunk_number(), 1, "Should request chunk 1");

    // Probe GoodCRC for chunk 1
    let good_crc_1 = Message::from_bytes(&policy_engine.protocol_layer.driver().probe_transmitted_data()).unwrap();
    eprintln!("Chunk 1 GoodCRC: {:?}", good_crc_1.header.message_type());
    assert!(matches!(
        good_crc_1.header.message_type(),
        MessageType::Control(ControlMessageType::GoodCRC)
    ));

    eprintln!("=== Phase 3 Complete: EPR caps assembled (with chunk request per spec) ===\n");

    // === Phase 4: EPR Power Negotiation ===
    // Selects PDO#8 (28V @ 5A = 140W)
    eprintln!("=== Phase 4: EPR Power Negotiation ===");

    // EvaluateCapabilities -> DPM.request() selects EPR PDO#8 (28V) -> SelectCapability
    policy_engine.run_step().await.unwrap();
    eprintln!("State after evaluate: {:?}", policy_engine.state);

    // Inject GoodCRC for the EprRequest that will be transmitted
    // Note: TX message counter is now 3 (after chunk request in Phase 3 incremented it from 2 to 3)
    simulate_source_control_message(&mut policy_engine, ControlMessageType::GoodCRC, 3);

    // Also inject Accept message that SelectCapability will wait for after transmitting
    simulate_source_control_message(&mut policy_engine, ControlMessageType::Accept, 0);

    // SelectCapability -> sends EprRequest, waits for Accept -> TransitionSink
    policy_engine.run_step().await.unwrap();

    // Probe the EPR Request
    let epr_request = Message::from_bytes(&policy_engine.protocol_layer.driver().probe_transmitted_data()).unwrap();
    eprintln!(
        "EPR Request: {:?} (message_id={})",
        epr_request.header.message_type(),
        epr_request.header.message_id()
    );
    assert!(matches!(
        epr_request.header.message_type(),
        MessageType::Data(DataMessageType::EprRequest)
    ));

    // Verify EPR Request selects PDO#8 (28V)
    if let Some(Payload::Data(Data::Request(PowerSource::EprRequest(epr)))) = &epr_request.payload {
        let object_pos = epr.object_position();
        eprintln!("EPR Request: PDO#{} (RDO=0x{:08X})", object_pos, epr.rdo);
        assert_eq!(object_pos, 8, "Should request PDO#8 (28V) to match real capture");

        // Verify it's the 28V PDO
        if let PowerDataObject::FixedSupply(fixed) = epr.pdo {
            assert_eq!(fixed.raw_voltage(), 560, "28V = 560 * 50mV");
            assert_eq!(fixed.raw_max_current(), 500, "5A = 500 * 10mA");
        }
    } else {
        panic!("Expected EprRequest payload");
    }

    // Drain any leftover messages
    eprintln!(
        "Has transmitted data before drain: {}",
        policy_engine.protocol_layer.driver().has_transmitted_data()
    );
    while policy_engine.protocol_layer.driver().has_transmitted_data() {
        let msg = Message::from_bytes(&policy_engine.protocol_layer.driver().probe_transmitted_data()).unwrap();
        eprintln!("Draining: {:?}", msg.header.message_type());
    }
    eprintln!(
        "Has transmitted data after drain: {}",
        policy_engine.protocol_layer.driver().has_transmitted_data()
    );

    // Inject PsRdy that TransitionSink will wait for
    simulate_source_control_message(&mut policy_engine, ControlMessageType::PsRdy, 1);

    // TransitionSink waits for PsRdy -> Ready
    policy_engine.run_step().await.unwrap();

    // Probe any GoodCRCs we transmitted (for Accept and PsRdy messages we received)
    while policy_engine.protocol_layer.driver().has_transmitted_data() {
        let msg = Message::from_bytes(&policy_engine.protocol_layer.driver().probe_transmitted_data()).unwrap();
        eprintln!("Phase 4 transmitted: {:?}", msg.header.message_type());
        assert!(matches!(
            msg.header.message_type(),
            MessageType::Control(ControlMessageType::GoodCRC)
        ));
    }

    // Verify we're in Ready state with EPR power
    assert!(matches!(policy_engine.state, State::Ready(..)));
    eprintln!("Final state: {:?}", policy_engine.state);

    eprintln!("=== Phase 4 Complete: EPR power negotiation at 28V/5A (140W) ===\n");

    // === Phase 5: EPR Keep-Alive ===
    // Per USB PD spec 8.3.3.3.11, sink must send EprKeepAlive periodically in EPR mode.
    // Real capture shows multiple keep-alive exchanges after EPR contract (lines 145-214).
    // We manually transition to EprKeepAlive state to test this flow, simulating multiple
    // keep-alive cycles to verify the sink continues sending them.
    eprintln!("=== Phase 5: EPR Keep-Alive (multiple cycles) ===");

    // Test multiple keep-alive cycles to verify the sink keeps sending them
    // Real capture shows 7 keep-alive exchanges - we'll test 3 to verify the pattern
    // From capture (lines 152-183), EprKeepAliveAck messages have Source/Dfp roles.
    let mut sink_tx_counter = 4u8; // Sink TX counter after EPR Request
    let mut source_tx_counter = 2u8; // Source TX counter (increments with each message source sends)

    // Expected EprKeepAliveAck bytes from capture (for first 3 cycles):
    // Cycle 1 (msg_id=2): B0 95 02 80 04 00
    // Cycle 2 (msg_id=3): B0 97 02 80 04 00
    // Cycle 3 (msg_id=4): B0 99 02 80 04 00
    // Note: Header changes based on msg_id, but extended header (02 80) and payload (04 00) stay same

    for cycle in 1..=3 {
        eprintln!("--- Keep-Alive cycle {} ---", cycle);

        // Manually set state to EprKeepAlive (normally triggered by SinkEPRKeepAliveTimer in Ready state)
        if let State::Ready(power_source, _) = policy_engine.state.clone() {
            policy_engine.state = State::EprKeepAlive(power_source);
        } else {
            panic!("Expected Ready state before keep-alive cycle {}", cycle);
        }

        // Inject GoodCRC for the EprKeepAlive message that will be transmitted
        simulate_source_control_message(&mut policy_engine, ControlMessageType::GoodCRC, sink_tx_counter);
        sink_tx_counter = sink_tx_counter.wrapping_add(1);

        // Inject EprKeepAliveAck response from source with correct message ID
        let keep_alive_ack_bytes = simulate_epr_keep_alive_ack(&mut policy_engine, source_tx_counter);
        eprintln!("  EprKeepAliveAck bytes: {:02X?}", &keep_alive_ack_bytes[..]);
        // Verify payload matches capture pattern
        // The extended header data_size=2 (low byte 0x02), and the chunked bit may or may not be set
        // (spec allows both). Real capture shows chunked=true (0x80), but our impl uses chunked=false (0x00)
        // Payload: 04 00 (ExtendedControl with EprKeepAliveAck type)
        assert_eq!(keep_alive_ack_bytes[2] & 0x1F, 0x02, "data_size should be 2");
        assert_eq!(
            &keep_alive_ack_bytes[4..],
            &[0x04, 0x00],
            "EprKeepAliveAck payload should match capture"
        );
        source_tx_counter = source_tx_counter.wrapping_add(1);

        // EprKeepAlive sends keep-alive, receives ack -> Ready
        policy_engine.run_step().await.unwrap();

        // Probe the EprKeepAlive message
        let keep_alive_bytes = policy_engine.protocol_layer.driver().probe_transmitted_data();
        let keep_alive = Message::from_bytes(&keep_alive_bytes).unwrap();
        eprintln!("  EprKeepAlive sent: {:?}", keep_alive.header.message_type());
        assert!(matches!(
            keep_alive.header.message_type(),
            MessageType::Extended(ExtendedMessageType::ExtendedControl)
        ));

        // Verify it's actually an EprKeepAlive message
        if let Some(Payload::Extended(crate::protocol_layer::message::extended::Extended::ExtendedControl(ctrl))) =
            &keep_alive.payload
        {
            assert_eq!(
                ctrl.message_type(),
                crate::protocol_layer::message::extended::extended_control::ExtendedControlMessageType::EprKeepAlive,
                "Expected EprKeepAlive message type"
            );
        } else {
            panic!("Expected ExtendedControl payload with EprKeepAlive");
        }
        // Verify EprKeepAlive payload matches capture pattern
        // From capture (e.g. line 145-148): 90 9A 02 80 03 00
        // Extended header data_size=2, Payload: 03 00 (EprKeepAlive type)
        // Note: chunked bit may differ between our impl (0x00) and capture (0x80)
        assert_eq!(keep_alive_bytes[2] & 0x1F, 0x02, "data_size should be 2");
        assert_eq!(
            &keep_alive_bytes[4..],
            &[0x03, 0x00],
            "EprKeepAlive payload should match capture"
        );

        // Probe GoodCRC for EprKeepAliveAck
        let good_crc = Message::from_bytes(&policy_engine.protocol_layer.driver().probe_transmitted_data()).unwrap();
        assert!(matches!(
            good_crc.header.message_type(),
            MessageType::Control(ControlMessageType::GoodCRC)
        ));

        // Verify we're back in Ready state (ready for next keep-alive cycle)
        assert!(matches!(policy_engine.state, State::Ready(..)));
        eprintln!("  Returned to Ready state");
    }

    eprintln!("=== Phase 5 Complete: {} EPR keep-alive cycles succeeded ===\n", 3);
    eprintln!("=== Full EPR negotiation test PASSED ===");
}

/// Records PD3 Alerts; optionally a port without a Chunking layer and not EPR capable.
#[derive(Default)]
struct ReadyRecorder<const CHUNKING: bool> {
    alerts: std::vec::Vec<u32>,
}
impl<const CHUNKING: bool> crate::sink::device_policy_manager::DevicePolicyManager for ReadyRecorder<CHUNKING> {
    const CHUNKING: bool = CHUNKING;
    const EPR: bool = CHUNKING;
    async fn alert(&mut self, alert_data_object: u32) {
        self.alerts.push(alert_data_object);
    }
    fn sink_capabilities_extended(&self) -> Option<[u8; 24]> {
        CHUNKING.then(|| core::array::from_fn(|i| i as u8 + 1))
    }
}

fn ready_engine<const CHUNKING: bool>() -> Sink<DummyDriver<MAX_DATA_MESSAGE_SIZE>, DummyTimer, ReadyRecorder<CHUNKING>>
{
    use crate::protocol_layer::message::data::request::FixedVariableSupply;
    let mut engine = Sink::new(DummyDriver::new(), ReadyRecorder::default());
    let established = PowerSource::FixedVariableSupply(FixedVariableSupply(0x1000_280a));
    engine.contract = super::Contract::Explicit;
    engine.established_request = Some(established);
    engine.source_capabilities = Some(source_capabilities_fixture());
    engine.state = State::Ready(established, false);
    engine
}
/// PD3 source/DFP header word: kind, Data Objects, MessageID, extended flag.
fn source_header(kind: u16, objects: u16, id: u16, extended: bool) -> u16 {
    (extended as u16) << 15 | objects << 12 | id << 9 | 0x1a0 | kind
}
fn inject(
    engine: &mut Sink<
        DummyDriver<MAX_DATA_MESSAGE_SIZE>,
        DummyTimer,
        impl crate::sink::device_policy_manager::DevicePolicyManager,
    >,
    header: u16,
    body: &[u8],
) {
    let mut bytes = header.to_le_bytes().to_vec();
    bytes.extend_from_slice(body);
    engine.protocol_layer.driver().inject_received_data(&bytes);
}
fn transmitted(
    engine: &mut Sink<
        DummyDriver<MAX_DATA_MESSAGE_SIZE>,
        DummyTimer,
        impl crate::sink::device_policy_manager::DevicePolicyManager,
    >,
) -> std::vec::Vec<MessageType> {
    let mut kinds = std::vec::Vec::new();
    while engine.protocol_layer.driver().has_transmitted_data() {
        let bytes = engine.protocol_layer.driver().probe_transmitted_data();
        kinds.push(Message::from_bytes(&bytes).unwrap().header.message_type());
    }
    kinds
}
const GOOD_CRC: MessageType = MessageType::Control(ControlMessageType::GoodCRC);

#[tokio::test]
async fn ready_get_revision_returns_the_revision_message() {
    let mut engine = ready_engine::<true>();
    inject(&mut engine, source_header(24, 0, 1, false), &[]);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::GiveRevision(_)));
    simulate_source_control_message(&mut engine, ControlMessageType::GoodCRC, 0);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::Ready(..)));
    engine.protocol_layer.driver().probe_transmitted_data(); // GoodCRC for Get_Revision
    let revision = Message::from_bytes(&engine.protocol_layer.driver().probe_transmitted_data()).unwrap();
    std::assert_eq!(
        revision.header.message_type(),
        MessageType::Data(DataMessageType::Revision)
    );
    std::assert_eq!(revision.header.num_objects(), 1);
    std::assert!(matches!(
        revision.payload,
        Some(Payload::Data(Data::Revision(0x3212_0000)))
    ));
}

#[tokio::test]
async fn ready_get_sink_cap_extended_returns_one_padded_chunk() {
    let mut engine = ready_engine::<true>();
    inject(&mut engine, source_header(22, 0, 1, false), &[]);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::GiveSinkCapExtended(..)));
    simulate_source_control_message(&mut engine, ControlMessageType::GoodCRC, 0);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::Ready(..)));
    engine.protocol_layer.driver().probe_transmitted_data(); // GoodCRC for Get_Sink_Cap_Extended
    let bytes = engine.protocol_layer.driver().probe_transmitted_data();
    // Extended, 7 Data Objects, PD3, Sink/UFP; Chunked, Chunk 0, Data Size 24.
    std::assert_eq!(bytes.len(), 30);
    std::assert_eq!(u16::from_le_bytes([bytes[0], bytes[1]]) & 0xf1ff, 0xf08f);
    std::assert_eq!(bytes[2..4], [0x18, 0x80]);
    std::assert_eq!(bytes[4..28], core::array::from_fn::<u8, 24, _>(|i| i as u8 + 1));
    std::assert_eq!(bytes[28..], [0, 0], "Table 6.48: padded with 00h");
    // Without a Device Policy Manager SKEDB the request is unsupported.
    let mut engine = ready_engine::<false>();
    inject(&mut engine, source_header(22, 0, 1, false), &[]);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::SendNotSupported(_)));
}

#[tokio::test]
async fn ready_dr_swap_and_vconn_swap_are_rejected_not_unsupported() {
    // Section 7.12.1: the DR_Swap recipient Shall send Accept, Wait or Reject.
    // Section 7.13: a VCONN_Swap recipient that is not the VCONN Source Rejects.
    for kind in [9, 11] {
        let mut engine = ready_engine::<true>();
        inject(&mut engine, source_header(kind, 0, 1, false), &[]);
        engine.run_step().await.unwrap();
        std::assert!(matches!(engine.state, State::RejectSwap(_)), "{kind}");
        simulate_source_control_message(&mut engine, ControlMessageType::GoodCRC, 0);
        engine.run_step().await.unwrap();
        std::assert!(matches!(engine.state, State::Ready(..)));
        std::assert_eq!(
            transmitted(&mut engine),
            [GOOD_CRC, MessageType::Control(ControlMessageType::Reject)]
        );
    }
}

#[tokio::test]
async fn bist_test_data_at_vsafe5v_sends_only_goodcrc_until_hard_reset() {
    // Section 9.2.26.4.2: the fixture contract is object 1 (vSafe5V).
    let mut engine = ready_engine::<true>();
    let mut bist = 0x8000_0000u32.to_le_bytes().to_vec();
    bist.extend([0x55; 24]);
    inject(&mut engine, source_header(3, 7, 1, false), &bist);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::BistTestData));
    inject(&mut engine, source_header(8, 0, 2, false), &[]); // Get_Sink_Cap
    inject(&mut engine, source_header(13, 0, 0, false), &[]); // Soft_Reset
    for _ in 0..2 {
        engine.run_step().await.unwrap();
        std::assert!(matches!(engine.state, State::BistTestData));
    }
    std::assert!(transmitted(&mut engine).iter().all(|kind| *kind == GOOD_CRC));
    // Above vSafe5V (object 2) the same request gets GoodCRC only.
    let mut engine = ready_engine::<true>();
    let nine = PowerSource::FixedVariableSupply(crate::protocol_layer::message::data::request::FixedVariableSupply(
        0x2000_280a,
    ));
    engine.established_request = Some(nine);
    engine.state = State::Ready(nine, false);
    inject(&mut engine, source_header(3, 7, 1, false), &bist);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::Ready(..)));
}

#[tokio::test]
async fn bist_carrier_mode_at_vsafe5v_transitions_to_default_without_hard_reset() {
    // Section 9.2.26.4.1.1: carrier, then PE_SNK_Transition_to_default and
    // PE_SNK_Startup; no Hard Reset, no contract until fresh Source_Capabilities.
    let mut engine = ready_engine::<true>();
    engine.protocol_layer.driver().bist_carrier = true;
    inject(
        &mut engine,
        source_header(3, 1, 1, false),
        &0x5000_0000u32.to_le_bytes(),
    );
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::BistCarrierMode(_)));
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::WaitForCapabilities));
    std::assert_eq!(engine.protocol_layer.driver().bist_carriers, 1);
    std::assert!(engine.established_request.is_none());
    std::assert!(engine.source_capabilities.is_none());
    std::assert_eq!(transmitted(&mut engine), [GOOD_CRC], "no Hard Reset or Message");
    // A driver without carrier support sends nothing and Ready continues.
    let mut engine = ready_engine::<true>();
    inject(
        &mut engine,
        source_header(3, 1, 1, false),
        &0x5000_0000u32.to_le_bytes(),
    );
    engine.run_step().await.unwrap();
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::Ready(..)));
    std::assert_eq!(engine.protocol_layer.driver().bist_carriers, 0);
}

#[tokio::test]
async fn ready_alert_bist_and_not_supported_get_goodcrc_only() {
    let mut engine = ready_engine::<true>();
    inject(
        &mut engine,
        source_header(6, 1, 1, false),
        &0x1000_0000u32.to_le_bytes(),
    );
    inject(
        &mut engine,
        source_header(3, 1, 2, false),
        &0x9000_0000u32.to_le_bytes(),
    ); // Shared Test Mode Entry
    inject(&mut engine, source_header(16, 0, 3, false), &[]);
    for _ in 0..3 {
        engine.run_step().await.unwrap();
        std::assert!(matches!(engine.state, State::Ready(..)));
    }
    std::assert_eq!(engine.device_policy_manager.alerts, [0x1000_0000]);
    std::assert_eq!(transmitted(&mut engine), [GOOD_CRC; 3]);
}

#[tokio::test]
async fn epr_messages_are_unsupported_outside_epr_or_for_a_non_epr_sink() {
    // EPR_Mode outside EPR Mode is not an exit notification.
    let mut engine = ready_engine::<true>();
    simulate_source_epr_mode_message(&mut engine, Action::Exit, 1);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::SendNotSupported(_)));
    // A sink that is not EPR capable does not answer EPR_Get_Sink_Cap.
    let mut engine = ready_engine::<false>();
    inject(&mut engine, source_header(16, 1, 1, true), &[0x02, 0x80, 0x02, 0x00]);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::SendNotSupported(_)));
    // A truncated ECDB is unsupported rather than a parser panic.
    let mut engine = ready_engine::<true>();
    inject(&mut engine, source_header(16, 0, 1, true), &[0x00, 0x00]);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::SendNotSupported(_)));
}

#[tokio::test]
async fn without_chunking_layer_multi_chunk_messages_wait_then_get_not_supported() {
    let mut engine = ready_engine::<false>();
    let mut chunk = (0x8000u16 | 40).to_le_bytes().to_vec();
    chunk.extend([0x5a; 26]);
    inject(&mut engine, source_header(30, 7, 1, true), &chunk);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::ChunkReceived(_)));
    std::assert_eq!(transmitted(&mut engine), [GOOD_CRC], "no Chunk request");
    engine.run_step().await.unwrap(); // ChunkingNotSupportedTimer
    std::assert!(matches!(engine.state, State::SendNotSupported(_)));
    // A stray Chunk request is unsupported immediately.
    inject(&mut engine, source_header(30, 1, 2, true), &[0x00, 0x8c, 0, 0]);
    engine.state = State::Ready(engine.established_request.unwrap(), false);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::SendNotSupported(_)));
    // Table 6.48: Invalid Chunk Number is Ignored; the next Message is serviced.
    engine.state = State::Ready(engine.established_request.unwrap(), false);
    inject(
        &mut engine,
        source_header(30, 2, 3, true),
        &[0x28, 0xe0, 0, 0, 0, 0, 0, 0],
    );
    inject(&mut engine, source_header(8, 0, 4, false), &[]);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::GiveSinkCap(..)));
}

/// PHY with hardware GoodCRC/retry whose first `defer` transmissions are not
/// sent (SinkTxNG or a received SOP Message, sections 7.2/7.3).
struct DeferringDriver {
    defer: usize,
    tx: std::vec::Vec<std::vec::Vec<u8>>,
}
impl usbpd_traits::Driver for DeferringDriver {
    const HAS_AUTO_GOOD_CRC: bool = true;
    const HAS_AUTO_RETRY: bool = true;
    async fn wait_for_vbus(&mut self) {}
    async fn receive(&mut self, _buffer: &mut [u8]) -> Result<usize, usbpd_traits::DriverRxError> {
        core::future::pending().await
    }
    async fn transmit(&mut self, data: &[u8]) -> Result<(), usbpd_traits::DriverTxError> {
        if self.defer > 0 {
            self.defer -= 1;
            return Err(usbpd_traits::DriverTxError::Deferred);
        }
        self.tx.push(data.to_vec());
        Ok(())
    }
    async fn transmit_hard_reset(&mut self) -> Result<(), usbpd_traits::DriverTxError> {
        std::panic!("a deferred AMS start never escalates to Hard Reset")
    }
}
#[derive(Default)]
struct DeferredRecorder {
    deferred: Option<PowerSource>,
}
impl crate::sink::device_policy_manager::DevicePolicyManager for DeferredRecorder {
    async fn request_deferred(&mut self, requested: &PowerSource) {
        self.deferred = Some(*requested);
    }
}

#[tokio::test]
async fn deferred_sink_initiated_request_returns_to_ready_without_using_a_message_id() {
    use crate::protocol_layer::message::data::request::FixedVariableSupply;
    let mut engine: Sink<DeferringDriver, DummyTimer, DeferredRecorder> = Sink::new(
        DeferringDriver {
            defer: 1,
            tx: std::vec::Vec::new(),
        },
        DeferredRecorder::default(),
    );
    let established = PowerSource::FixedVariableSupply(FixedVariableSupply(0x1000_280a));
    let attempted = PowerSource::FixedVariableSupply(FixedVariableSupply(0x2000_280a));
    engine.contract = super::Contract::Explicit;
    engine.established_request = Some(established);
    engine.state = State::SelectCapability(attempted);
    engine.run_step().await.unwrap();
    let State::Ready(PowerSource::FixedVariableSupply(current), false) = engine.state else {
        std::panic!("deferred request did not return to Ready");
    };
    std::assert_eq!(current.0, 0x1000_280a);
    std::assert!(matches!(engine.device_policy_manager.deferred,
        Some(PowerSource::FixedVariableSupply(rdo)) if rdo.0 == 0x2000_280a));
    std::assert!(engine.protocol_layer.driver().tx.is_empty());
    // Nothing was sent: the next Request still carries MessageID 0.
    engine.protocol_layer.request_power(attempted).await.unwrap();
    let sent = engine.protocol_layer.driver().tx[0].clone();
    std::assert_eq!((sent[1] >> 1) & 7, 0);
}

#[tokio::test]
async fn deferred_request_without_explicit_contract_is_not_turned_into_ready() {
    use crate::protocol_layer::message::data::request::FixedVariableSupply;
    let mut engine: Sink<DeferringDriver, DummyTimer, DeferredRecorder> = Sink::new(
        DeferringDriver {
            defer: 1,
            tx: std::vec::Vec::new(),
        },
        DeferredRecorder::default(),
    );
    let attempted = PowerSource::FixedVariableSupply(FixedVariableSupply(0x2000_280a));
    engine.state = State::SelectCapability(attempted);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::SelectCapability(_)));
    std::assert!(engine.device_policy_manager.deferred.is_none());
}

/// Asks for the Source's PPS status once and records each outcome.
#[derive(Default)]
struct PpsStatusRecorder {
    ask: bool,
    outcomes: std::vec::Vec<PpsStatusOutcome>,
}
impl crate::sink::device_policy_manager::DevicePolicyManager for PpsStatusRecorder {
    const CHUNKING: bool = false;
    const EPR: bool = false;
    async fn pps_status(&mut self, outcome: PpsStatusOutcome) {
        self.outcomes.push(outcome);
    }
    async fn get_event(&mut self, _capabilities: &SourceCapabilities) -> crate::sink::device_policy_manager::Event {
        if core::mem::take(&mut self.ask) {
            crate::sink::device_policy_manager::Event::GetPpsStatus
        } else {
            core::future::pending().await
        }
    }
}
const PPS_CONTRACT: PowerSource = PowerSource::Pps(crate::protocol_layer::message::data::request::Pps(0x4002_d064));

fn pps_engine(established: PowerSource) -> Sink<DummyDriver<MAX_DATA_MESSAGE_SIZE>, DummyTimer, PpsStatusRecorder> {
    let mut engine = Sink::new(
        DummyDriver::new(),
        PpsStatusRecorder {
            ask: true,
            outcomes: std::vec::Vec::new(),
        },
    );
    engine.contract = super::Contract::Explicit;
    engine.established_request = Some(established);
    engine.source_capabilities = Some(source_capabilities_fixture());
    engine.state = State::Ready(established, false);
    engine
}

/// Run Ready -> PE_SNK_Get_PPS_Status -> Ready with `reply` queued after the
/// GoodCRC (MessageID `id`) for Get_PPS_Status; returns the transmitted Messages.
async fn get_pps_status(
    engine: &mut Sink<DummyDriver<MAX_DATA_MESSAGE_SIZE>, DummyTimer, PpsStatusRecorder>,
    id: u8,
    reply: Option<(u16, &[u8])>,
) -> Result<std::vec::Vec<std::vec::Vec<u8>>, super::Error> {
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::GetPpsStatus(_)));
    simulate_source_control_message(engine, ControlMessageType::GoodCRC, id);
    if let Some((header, body)) = reply {
        inject(engine, header, body);
    }
    engine.run_step().await?;
    let mut sent = std::vec::Vec::new();
    while engine.protocol_layer.driver().has_transmitted_data() {
        sent.push(engine.protocol_layer.driver().probe_transmitted_data().to_vec());
    }
    Ok(sent)
}

#[tokio::test]
async fn get_pps_status_reports_the_ppssdb_and_returns_to_ready() {
    let mut engine = pps_engine(PPS_CONTRACT);
    // Chunked, Chunk 0, Data Size 4; 9.00 V, 2.00 A, PTF normal, OMF CL; 00h padding.
    let body = [0x04, 0x80, 0xc2, 0x01, 0x28, 0x0a, 0, 0];
    let sent = get_pps_status(&mut engine, 0, Some((source_header(12, 2, 1, true), &body)))
        .await
        .unwrap();
    std::assert!(matches!(engine.state, State::Ready(PowerSource::Pps(_), false)));
    std::assert_eq!(
        engine.device_policy_manager.outcomes,
        [PpsStatusOutcome::Status([0xc2, 0x01, 0x28, 0x0a])]
    );
    // Get_PPS_Status: Control, no Data Objects, PD3, Sink/UFP (section 6.3.20).
    std::assert_eq!(sent[0].len(), 2);
    std::assert_eq!(u16::from_le_bytes([sent[0][0], sent[0][1]]) & 0xf1ff, 0x0094);
    std::assert_eq!(Message::from_bytes(&sent[1]).unwrap().header.message_type(), GOOD_CRC);
    std::assert_eq!(sent.len(), 2);
}

#[tokio::test]
async fn get_pps_status_not_supported_or_timeout_returns_to_ready() {
    let mut engine = pps_engine(PPS_CONTRACT);
    get_pps_status(&mut engine, 0, Some((source_header(16, 0, 1, false), &[])))
        .await
        .unwrap();
    std::assert!(matches!(engine.state, State::Ready(..)));
    // Figure 9.37: SenderResponseTimer timeout is a normal exit, never a reset.
    engine.device_policy_manager.ask = true;
    get_pps_status(&mut engine, 1, None).await.unwrap();
    std::assert!(matches!(engine.state, State::Ready(..)));
    std::assert_eq!(
        engine.device_policy_manager.outcomes,
        [PpsStatusOutcome::NotSupported, PpsStatusOutcome::Timeout]
    );
}

#[tokio::test]
async fn get_pps_status_malformed_or_unexpected_reply_is_a_protocol_error() {
    // Data Size 3 is not a PPSSDB (Table 6.58).
    let mut engine = pps_engine(PPS_CONTRACT);
    let body = [0x03, 0x80, 0xc2, 0x01, 0x28, 0, 0, 0];
    get_pps_status(&mut engine, 0, Some((source_header(12, 2, 1, true), &body)))
        .await
        .unwrap();
    std::assert!(matches!(engine.state, State::SendSoftReset));
    std::assert!(engine.device_policy_manager.outcomes.is_empty());
    // Accept is not an answer to Get_PPS_Status.
    let mut engine = pps_engine(PPS_CONTRACT);
    get_pps_status(&mut engine, 0, Some((source_header(3, 0, 1, false), &[])))
        .await
        .unwrap();
    std::assert!(matches!(engine.state, State::SendSoftReset));
    std::assert!(engine.device_policy_manager.outcomes.is_empty());
}

#[tokio::test]
async fn get_pps_status_needs_a_pd3_pps_contract() {
    use crate::protocol_layer::message::data::request::FixedVariableSupply;
    let mut engine = pps_engine(PowerSource::FixedVariableSupply(FixedVariableSupply(0x1000_280a)));
    engine.run_step().await.unwrap();
    std::assert!(matches!(
        engine.state,
        State::Ready(PowerSource::FixedVariableSupply(_), false)
    ));
    std::assert_eq!(engine.device_policy_manager.outcomes, [PpsStatusOutcome::NotSent]);
    std::assert!(!engine.protocol_layer.driver().has_transmitted_data());
}

#[tokio::test]
async fn deferred_get_pps_status_returns_to_ready_without_using_a_message_id() {
    let mut engine: Sink<DeferringDriver, DummyTimer, PpsStatusRecorder> = Sink::new(
        DeferringDriver {
            defer: 1,
            tx: std::vec::Vec::new(),
        },
        PpsStatusRecorder::default(),
    );
    engine.contract = super::Contract::Explicit;
    engine.established_request = Some(PPS_CONTRACT);
    engine.state = State::GetPpsStatus(PPS_CONTRACT);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::Ready(PowerSource::Pps(_), false)));
    std::assert_eq!(engine.device_policy_manager.outcomes, [PpsStatusOutcome::Deferred]);
    std::assert!(engine.protocol_layer.driver().tx.is_empty());
    engine
        .protocol_layer
        .transmit_control_message(ControlMessageType::GetPpsStatus)
        .await
        .unwrap();
    std::assert_eq!((engine.protocol_layer.driver().tx[0][1] >> 1) & 7, 0);
}

const SOFT_RESET: MessageType = MessageType::Control(ControlMessageType::SoftReset);

#[tokio::test]
async fn startup_soft_reset_uses_peer_revision_without_locking_capability_detection() {
    for revision in [1u8, 2] {
        let mut engine = get_policy_engine();
        engine.state = State::WaitForCapabilities;
        inject(
            &mut engine,
            source_header(13, 0, 0, false) & !0xc0 | u16::from(revision) << 6,
            &[],
        );
        engine.run_step().await.unwrap();
        std::assert!(matches!(engine.state, State::SoftReset));
        std::assert_eq!(transmitted(&mut engine), [GOOD_CRC]);
        simulate_source_control_message(&mut engine, ControlMessageType::GoodCRC, 0);
        engine.run_step().await.unwrap();
        std::assert!(matches!(engine.state, State::WaitForCapabilities));
        let accept = engine.protocol_layer.driver().probe_transmitted_data();
        let accept = Message::from_bytes(&accept).unwrap();
        std::assert_eq!(
            accept.header.message_type(),
            MessageType::Control(ControlMessageType::Accept)
        );
        std::assert_eq!(accept.header.message_id(), 0);
        std::assert_eq!(u8::from(accept.header.spec_revision().unwrap()), revision);
        // Section 6.1.3.1: the first Request still derives its revision from
        // fresh Source_Capabilities, not from the startup reset.
        let fresh = 3 - revision;
        let mut caps = DUMMY_CAPABILITIES;
        caps[0] = (caps[0] & !0xc0) | fresh << 6;
        engine.protocol_layer.driver().inject_received_data(&caps);
        engine.run_step().await.unwrap();
        std::assert!(matches!(engine.state, State::EvaluateCapabilities(_)));
        std::assert_eq!(u8::from(engine.protocol_layer.header().spec_revision().unwrap()), fresh);
    }
}

#[tokio::test]
async fn ready_unexpected_accept_reject_ps_rdy_or_wait_initiates_soft_reset() {
    // Table 7.1: Recognized, Supported, Unexpected in PE_SNK_Ready -> Soft_Reset.
    for kind in [3, 4, 6, 12] {
        let mut engine = ready_engine::<false>();
        inject(&mut engine, source_header(kind, 0, 5, false), &[]);
        engine.run_step().await.unwrap();
        std::assert!(matches!(engine.state, State::SendSoftReset), "kind {kind}");
        std::assert_eq!(transmitted(&mut engine), [GOOD_CRC]);
        // Section 9.2.5.2.1: protocol reset, Soft_Reset with MessageID 0, Accept -> Wait_for_Capabilities.
        simulate_source_control_message(&mut engine, ControlMessageType::GoodCRC, 0);
        inject(&mut engine, source_header(3, 0, 0, false), &[]);
        engine.run_step().await.unwrap();
        std::assert!(matches!(engine.state, State::WaitForCapabilities));
        let soft_reset = engine.protocol_layer.driver().probe_transmitted_data();
        let soft_reset = Message::from_bytes(&soft_reset).unwrap();
        std::assert_eq!(soft_reset.header.message_type(), SOFT_RESET);
        std::assert_eq!(soft_reset.header.message_id(), 0);
        std::assert_eq!(transmitted(&mut engine), [GOOD_CRC], "GoodCRC for the Accept");
    }
}

#[tokio::test]
async fn send_soft_reset_escalates_to_hard_reset_and_never_repeats_itself() {
    // Section 9.2.5.2.1: SenderResponseTimer timeout -> PE_SNK_Hard_Reset.
    let mut engine = ready_engine::<false>();
    engine.state = State::SendSoftReset;
    simulate_source_control_message(&mut engine, ControlMessageType::GoodCRC, 0);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::HardReset));
    // Section 7.7: a Protocol Error during the Soft Reset -> Hard Reset.
    let mut engine = ready_engine::<false>();
    engine.state = State::SendSoftReset;
    simulate_source_control_message(&mut engine, ControlMessageType::GoodCRC, 0);
    inject(&mut engine, source_header(6, 0, 0, false), &[]);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::HardReset));
    // An Unrecognized Message there is not answered with another Soft_Reset either.
    let mut engine = ready_engine::<false>();
    engine.state = State::SendSoftReset;
    simulate_source_control_message(&mut engine, ControlMessageType::GoodCRC, 0);
    inject(&mut engine, source_header(31, 0, 0, false), &[]);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::HardReset));
    // Section 9.2.5.2.2: the Port Partner's own Soft_Reset wins.
    let mut engine = ready_engine::<false>();
    engine.state = State::SendSoftReset;
    simulate_source_control_message(&mut engine, ControlMessageType::GoodCRC, 0);
    inject(&mut engine, source_header(13, 0, 0, false), &[]);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::SoftReset));
}

#[tokio::test]
async fn unsupported_message_during_request_ams_initiates_soft_reset_not_a_new_request() {
    let mut engine = ready_engine::<false>();
    let established = engine.established_request.unwrap();
    engine.state = State::SelectCapability(established);
    simulate_source_control_message(&mut engine, ControlMessageType::GoodCRC, 0);
    inject(&mut engine, source_header(31, 0, 1, false), &[]);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::SendSoftReset));
    // A Source_Capabilities instead of Accept is an Unexpected Message (Figure 9.18 note 1).
    let mut engine = ready_engine::<false>();
    engine.state = State::SelectCapability(established);
    simulate_source_control_message(&mut engine, ControlMessageType::GoodCRC, 0);
    engine.protocol_layer.driver().inject_received_data(&DUMMY_CAPABILITIES);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::SendSoftReset));
}

/// Hardware-retry PHY whose next `nack` transmissions get no GoodCRC.
struct NackDriver {
    nack: usize,
    tx: std::vec::Vec<std::vec::Vec<u8>>,
}
impl usbpd_traits::Driver for NackDriver {
    const HAS_AUTO_GOOD_CRC: bool = true;
    const HAS_AUTO_RETRY: bool = true;
    async fn wait_for_vbus(&mut self) {}
    async fn receive(&mut self, _buffer: &mut [u8]) -> Result<usize, usbpd_traits::DriverRxError> {
        core::future::pending().await
    }
    async fn transmit(&mut self, data: &[u8]) -> Result<(), usbpd_traits::DriverTxError> {
        self.tx.push(data.to_vec());
        if self.nack > 0 {
            self.nack -= 1;
            return Err(usbpd_traits::DriverTxError::NotAcknowledged);
        }
        Ok(())
    }
    async fn transmit_hard_reset(&mut self) -> Result<(), usbpd_traits::DriverTxError> {
        Ok(())
    }
}

#[tokio::test]
async fn not_acknowledged_message_soft_resets_and_a_not_acknowledged_soft_reset_hard_resets() {
    use crate::protocol_layer::message::data::request::FixedVariableSupply;
    let established = PowerSource::FixedVariableSupply(FixedVariableSupply(0x1000_280a));
    let mut engine: Sink<NackDriver, DummyTimer, DummySinkDevice> = Sink::new(
        NackDriver {
            nack: 2,
            tx: std::vec::Vec::new(),
        },
        DummySinkDevice {},
    );
    engine.contract = super::Contract::Explicit;
    engine.established_request = Some(established);
    engine.state = State::SelectCapability(established);
    // Section 7.1.1: no GoodCRC after nRetryCount retries -> PE_SNK_Send_Soft_Reset.
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::SendSoftReset));
    // Section 7.1.1: a Soft_Reset that fails -> Hard Reset.
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::HardReset));
    let tx = &engine.protocol_layer.driver().tx;
    std::assert_eq!(tx.len(), 2);
    std::assert_eq!(tx[1][0] & 0x1f, 13);
    std::assert_eq!((tx[1][1] >> 1) & 7, 0, "Soft_Reset uses MessageID 0");
}

/// Ready -> unexpected Accept -> PE_SNK_Send_Soft_Reset -> Accept (MessageID 0)
/// -> PE_SNK_Wait_for_Capabilities, recovering.
async fn recovering_engine() -> Sink<DummyDriver<MAX_DATA_MESSAGE_SIZE>, DummyTimer, ReadyRecorder<false>> {
    let mut engine = ready_engine::<false>();
    inject(&mut engine, source_header(3, 0, 5, false), &[]);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::SendSoftReset) && engine.recovering);
    simulate_source_control_message(&mut engine, ControlMessageType::GoodCRC, 0);
    inject(&mut engine, source_header(3, 0, 0, false), &[]);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::WaitForCapabilities) && engine.recovering);
    transmitted(&mut engine);
    engine
}

/// Fresh Source_Capabilities (MessageID 1) -> our Request (MessageID 1) acknowledged.
async fn renegotiate(engine: &mut Sink<DummyDriver<MAX_DATA_MESSAGE_SIZE>, DummyTimer, ReadyRecorder<false>>) {
    let mut capabilities = DUMMY_CAPABILITIES;
    capabilities[1] |= 1 << 1;
    engine.protocol_layer.driver().inject_received_data(&capabilities);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::EvaluateCapabilities(_)));
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::SelectCapability(_)));
    simulate_source_control_message(engine, ControlMessageType::GoodCRC, 1);
}

#[tokio::test]
async fn a_protocol_error_while_recovering_hard_resets_instead_of_a_second_soft_reset() {
    // Section 7.1.1: the Soft Reset did not correct the error. PS_RDY instead of
    // fresh capabilities after our Soft_Reset's Accept.
    let mut engine = recovering_engine().await;
    inject(&mut engine, source_header(6, 0, 1, false), &[]);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::HardReset));
    // An Unrecognized Message instead of the renegotiated Request's answer.
    let mut engine = recovering_engine().await;
    renegotiate(&mut engine).await;
    inject(&mut engine, source_header(31, 0, 2, false), &[]);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::HardReset));
    // An Unexpected one (Source_Capabilities, Figure 9.18 note 1).
    let mut engine = recovering_engine().await;
    renegotiate(&mut engine).await;
    let mut capabilities = DUMMY_CAPABILITIES;
    capabilities[1] |= 2 << 1;
    engine.protocol_layer.driver().inject_received_data(&capabilities);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::HardReset));
    // An Unrecognized Message while waiting for capabilities is not ignored either.
    let mut engine = recovering_engine().await;
    inject(&mut engine, source_header(31, 0, 1, false), &[]);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::HardReset));
    // A partner Soft_Reset is still accepted (section 9.2.5.2.2).
    let mut engine = recovering_engine().await;
    inject(&mut engine, source_header(13, 0, 0, false), &[]);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::SoftReset) && engine.recovering);
}

#[tokio::test]
async fn a_not_acknowledged_message_while_recovering_hard_resets() {
    use crate::protocol_layer::message::data::request::FixedVariableSupply;
    let established = PowerSource::FixedVariableSupply(FixedVariableSupply(0x1000_280a));
    let mut engine: Sink<NackDriver, DummyTimer, DummySinkDevice> = Sink::new(
        NackDriver {
            nack: 1,
            tx: std::vec::Vec::new(),
        },
        DummySinkDevice {},
    );
    engine.contract = super::Contract::Explicit;
    engine.established_request = Some(established);
    engine.state = State::SelectCapability(established);
    engine.recovering = true;
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::HardReset));
    std::assert_eq!(engine.protocol_layer.driver().tx.len(), 1, "no Soft_Reset");
}

#[tokio::test]
async fn ps_rdy_reject_or_wait_end_recovery_so_a_later_error_soft_resets_again() {
    // PS_RDY of the renegotiated contract.
    let mut engine = recovering_engine().await;
    renegotiate(&mut engine).await;
    inject(&mut engine, source_header(3, 0, 2, false), &[]);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::TransitionSink(_)) && engine.recovering);
    inject(&mut engine, source_header(6, 0, 3, false), &[]);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::Ready(..)) && !engine.recovering);
    inject(&mut engine, source_header(3, 0, 4, false), &[]);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::SendSoftReset));
    // Reject or Wait under the kept Explicit Contract.
    for kind in [4, 12] {
        let mut engine = recovering_engine().await;
        renegotiate(&mut engine).await;
        inject(&mut engine, source_header(kind, 0, 2, false), &[]);
        engine.run_step().await.unwrap();
        std::assert!(
            matches!(engine.state, State::Ready(..)) && !engine.recovering,
            "kind {kind}"
        );
        // Before any Explicit Contract they lead to Wait_for_Capabilities, also
        // not recovering: a later error there is answered with Soft_Reset again.
        let mut engine = recovering_engine().await;
        renegotiate(&mut engine).await;
        engine.contract = super::Contract::Safe5V;
        inject(&mut engine, source_header(kind, 0, 2, false), &[]);
        engine.run_step().await.unwrap();
        std::assert!(
            matches!(engine.state, State::WaitForCapabilities) && !engine.recovering,
            "kind {kind}"
        );
        inject(&mut engine, source_header(6, 0, 3, false), &[]);
        engine.run_step().await.unwrap();
        std::assert!(matches!(engine.state, State::SendSoftReset), "kind {kind}");
    }
}

#[tokio::test]
async fn a_protocol_error_while_power_transitions_hard_resets() {
    // Table 7.1, section 9.2.4.6: during PE_SNK_Transition_Sink.
    let mut engine = ready_engine::<false>();
    let established = engine.established_request.unwrap();
    engine.state = State::TransitionSink(established);
    inject(&mut engine, source_header(3, 0, 1, false), &[]);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::HardReset));
    // PSTransitionTimer timeout.
    let mut engine = ready_engine::<false>();
    engine.state = State::TransitionSink(established);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::HardReset));
}

/// Asks the queued information requests in order and records each outcome.
#[derive(Default)]
struct QueryRecorder {
    ask: std::vec::Vec<crate::sink::device_policy_manager::PartnerQuery>,
    outcomes: std::vec::Vec<(
        crate::sink::device_policy_manager::PartnerQuery,
        crate::sink::device_policy_manager::QueryOutcome,
    )>,
}
impl crate::sink::device_policy_manager::DevicePolicyManager for QueryRecorder {
    const CHUNKING: bool = false;
    const EPR: bool = false;
    async fn query(
        &mut self,
        query: crate::sink::device_policy_manager::PartnerQuery,
        outcome: crate::sink::device_policy_manager::QueryOutcome,
    ) {
        self.outcomes.push((query, outcome));
    }
    async fn get_event(&mut self, _capabilities: &SourceCapabilities) -> crate::sink::device_policy_manager::Event {
        if self.ask.is_empty() {
            core::future::pending().await
        } else {
            crate::sink::device_policy_manager::Event::Query(self.ask.remove(0))
        }
    }
}

fn query_engine(
    ask: crate::sink::device_policy_manager::PartnerQuery,
) -> Sink<DummyDriver<MAX_DATA_MESSAGE_SIZE>, DummyTimer, QueryRecorder> {
    use crate::protocol_layer::message::data::request::FixedVariableSupply;
    let mut engine = Sink::new(
        DummyDriver::new(),
        QueryRecorder {
            ask: std::vec![ask],
            outcomes: std::vec::Vec::new(),
        },
    );
    let established = PowerSource::FixedVariableSupply(FixedVariableSupply(0x1000_280a));
    engine.contract = super::Contract::Explicit;
    engine.established_request = Some(established);
    engine.source_capabilities = Some(source_capabilities_fixture());
    engine.state = State::Ready(established, false);
    engine
}

/// Run Ready -> query state -> Ready with `reply` queued after the GoodCRC for
/// the request; returns the transmitted Messages.
async fn query(
    engine: &mut Sink<DummyDriver<MAX_DATA_MESSAGE_SIZE>, DummyTimer, QueryRecorder>,
    reply: Option<(u16, &[u8])>,
) -> std::vec::Vec<std::vec::Vec<u8>> {
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::Query(..)));
    simulate_source_control_message(engine, ControlMessageType::GoodCRC, 0);
    if let Some((header, body)) = reply {
        inject(engine, header, body);
    }
    engine.run_step().await.unwrap();
    let mut sent = std::vec::Vec::new();
    while engine.protocol_layer.driver().has_transmitted_data() {
        sent.push(engine.protocol_layer.driver().probe_transmitted_data().to_vec());
    }
    sent
}

/// One Chunked Chunk 0 extended reply body: Extended Header, block, 00h padding.
fn chunk(block: &[u8]) -> (u16, std::vec::Vec<u8>) {
    let mut body = std::vec![block.len() as u8, 0x80];
    body.extend_from_slice(block);
    while body.len() % 4 != 0 {
        body.push(0);
    }
    ((body.len() / 4) as u16, body)
}

#[tokio::test]
async fn get_source_cap_extended_reports_the_scedb_and_returns_to_ready() {
    use crate::sink::device_policy_manager::PartnerQuery::SourceCapabilitiesExtended as Sce;
    use crate::sink::device_policy_manager::QueryOutcome;
    // Section 6.5: a 24-byte PD R3.0 block, the 25-byte block, and a longer one
    // whose extra byte is Ignored (Table 6.48 Data Size).
    for (size, kept) in [(24usize, 24usize), (25, 25), (26, 25)] {
        let mut engine = query_engine(Sce);
        let block: std::vec::Vec<u8> = (1..=size as u8).collect();
        let (objects, body) = chunk(&block);
        let sent = query(&mut engine, Some((source_header(1, objects, 1, true), &body))).await;
        std::assert!(matches!(engine.state, State::Ready(..)));
        // Get_Source_Cap_Extended: Control, no Data Objects, PD3, Sink/UFP.
        std::assert_eq!(sent[0].len(), 2);
        std::assert_eq!(u16::from_le_bytes([sent[0][0], sent[0][1]]) & 0xf1ff, 0x0091);
        std::assert_eq!(Message::from_bytes(&sent[1]).unwrap().header.message_type(), GOOD_CRC);
        let QueryOutcome::Answer(answer) = &engine.device_policy_manager.outcomes[0].1 else {
            std::panic!("no answer")
        };
        std::assert_eq!(answer.as_slice(), &block[..kept]);
    }
}

#[tokio::test]
async fn get_revision_reports_the_rmdo() {
    use crate::sink::device_policy_manager::PartnerQuery::Revision;
    use crate::sink::device_policy_manager::QueryOutcome;
    let mut engine = query_engine(Revision);
    let sent = query(
        &mut engine,
        Some((source_header(12, 1, 1, false), &0x3118_0000u32.to_le_bytes())),
    )
    .await;
    std::assert_eq!(u16::from_le_bytes([sent[0][0], sent[0][1]]) & 0xf1ff, 0x0098);
    std::assert!(matches!(engine.state, State::Ready(..)));
    std::assert_eq!(
        engine.device_policy_manager.outcomes,
        [(
            Revision,
            QueryOutcome::Answer(heapless::Vec::from_slice(&[0, 0, 0x18, 0x31]).unwrap())
        )]
    );
}

#[tokio::test]
async fn get_manufacturer_info_asks_for_the_port_and_reports_the_block() {
    use crate::sink::device_policy_manager::PartnerQuery::ManufacturerInfo;
    use crate::sink::device_policy_manager::QueryOutcome;
    let mut engine = query_engine(ManufacturerInfo);
    let block = *b"\x34\x12\x78\x56ACME 65W\0";
    let (objects, body) = chunk(&block);
    let sent = query(&mut engine, Some((source_header(7, objects, 1, true), &body))).await;
    // Extended, one Data Object, PD3, Sink/UFP, type 00110b; Chunked Chunk 0
    // of 2 bytes: Target 0 (Port), Ref 0 (Reserved), Table 6.56.
    std::assert_eq!(sent[0].len(), 6);
    std::assert_eq!(u16::from_le_bytes([sent[0][0], sent[0][1]]) & 0xf1ff, 0x9086);
    std::assert_eq!(sent[0][2..], [0x02, 0x80, 0, 0]);
    std::assert!(matches!(engine.state, State::Ready(..)));
    std::assert_eq!(
        engine.device_policy_manager.outcomes,
        [(
            ManufacturerInfo,
            QueryOutcome::Answer(heapless::Vec::from_slice(&block).unwrap())
        )]
    );
}

#[tokio::test]
async fn queries_not_supported_or_timeout_return_to_ready() {
    use crate::sink::device_policy_manager::{PartnerQuery, QueryOutcome};
    for ask in [
        PartnerQuery::SourceCapabilitiesExtended,
        PartnerQuery::Revision,
        PartnerQuery::ManufacturerInfo,
    ] {
        let mut engine = query_engine(ask);
        query(&mut engine, Some((source_header(16, 0, 1, false), &[]))).await;
        std::assert!(matches!(engine.state, State::Ready(..)));
        // A SenderResponseTimer timeout is a normal exit, never a reset.
        engine.device_policy_manager.ask.push(ask);
        engine.run_step().await.unwrap();
        simulate_source_control_message(&mut engine, ControlMessageType::GoodCRC, 1);
        engine.run_step().await.unwrap();
        std::assert!(matches!(engine.state, State::Ready(..)));
        std::assert_eq!(
            engine.device_policy_manager.outcomes,
            [(ask, QueryOutcome::NotSupported), (ask, QueryOutcome::Timeout)]
        );
    }
}

#[tokio::test]
async fn malformed_or_unexpected_query_answers_are_a_protocol_error() {
    use crate::sink::device_policy_manager::PartnerQuery;
    // A 23-byte SCEDB, a 3-byte Manufacturer_Info, a Revision without an RMDO
    // (a Wait header), and an Accept.
    let short_scedb = chunk(&[0; 23]);
    let short_info = chunk(&[0; 3]);
    let cases: [(PartnerQuery, u16, &[u8]); 4] = [
        (
            PartnerQuery::SourceCapabilitiesExtended,
            source_header(1, short_scedb.0, 1, true),
            &short_scedb.1,
        ),
        (
            PartnerQuery::ManufacturerInfo,
            source_header(7, short_info.0, 1, true),
            &short_info.1,
        ),
        (PartnerQuery::Revision, source_header(12, 0, 1, false), &[]),
        (PartnerQuery::Revision, source_header(3, 0, 1, false), &[]),
    ];
    for (ask, header, body) in cases {
        let mut engine = query_engine(ask);
        query(&mut engine, Some((header, body))).await;
        std::assert!(matches!(engine.state, State::SendSoftReset), "{ask:?}");
        std::assert!(engine.device_policy_manager.outcomes.is_empty());
    }
}

#[tokio::test]
async fn queries_need_pd3() {
    use crate::sink::device_policy_manager::{PartnerQuery, QueryOutcome};
    let mut engine = query_engine(PartnerQuery::Revision);
    // PD2 Source_Capabilities select PD2 before the first Request (section 6.2.1.1.5).
    let mut caps = DUMMY_CAPABILITIES;
    caps[0] = (caps[0] & !0xc0) | 0x40;
    engine.protocol_layer.driver().inject_received_data(&caps);
    engine.protocol_layer.receive_message().await.unwrap();
    engine.protocol_layer.driver().probe_transmitted_data();
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::Ready(..)));
    std::assert_eq!(
        engine.device_policy_manager.outcomes,
        [(PartnerQuery::Revision, QueryOutcome::NotSent)]
    );
    std::assert!(!engine.protocol_layer.driver().has_transmitted_data());
}

#[tokio::test]
async fn deferred_query_returns_to_ready_without_using_a_message_id() {
    use crate::protocol_layer::message::data::request::FixedVariableSupply;
    use crate::sink::device_policy_manager::{PartnerQuery, QueryOutcome};
    let mut engine: Sink<DeferringDriver, DummyTimer, QueryRecorder> = Sink::new(
        DeferringDriver {
            defer: 1,
            tx: std::vec::Vec::new(),
        },
        QueryRecorder::default(),
    );
    let established = PowerSource::FixedVariableSupply(FixedVariableSupply(0x1000_280a));
    engine.contract = super::Contract::Explicit;
    engine.established_request = Some(established);
    engine.state = State::Query(established, PartnerQuery::ManufacturerInfo);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::Ready(..)));
    std::assert_eq!(
        engine.device_policy_manager.outcomes,
        [(PartnerQuery::ManufacturerInfo, QueryOutcome::Deferred)]
    );
    std::assert!(engine.protocol_layer.driver().tx.is_empty());
    engine.protocol_layer.transmit_get_manufacturer_info().await.unwrap();
    std::assert_eq!((engine.protocol_layer.driver().tx[0][1] >> 1) & 7, 0);
}

#[tokio::test]
async fn get_source_info_reports_one_or_two_sidos_and_ignores_more() {
    use crate::sink::device_policy_manager::PartnerQuery::SourceInfo;
    use crate::sink::device_policy_manager::QueryOutcome;
    // R3.1 Sources send SIDO1 only; R3.2 V1.2 adds SIDO2; further objects are Ignored.
    for objects in [
        &[0x8a8c_8c8cu32][..],
        &[0x8a8c_8c8c, 0x0001_1918],
        &[0x8a8c_8c8c, 0x0001_1918, 7],
    ] {
        let mut engine = query_engine(SourceInfo);
        let body: std::vec::Vec<u8> = objects.iter().flat_map(|o| o.to_le_bytes()).collect();
        let sent = query(
            &mut engine,
            Some((source_header(11, objects.len() as u16, 1, false), &body)),
        )
        .await;
        // Get_Source_Info: Control 10111b, PD3, Sink/UFP.
        std::assert_eq!(u16::from_le_bytes([sent[0][0], sent[0][1]]) & 0xf1ff, 0x0097);
        std::assert!(matches!(engine.state, State::Ready(..)));
        let kept = objects.len().min(2) * 4;
        std::assert_eq!(
            engine.device_policy_manager.outcomes,
            [(
                SourceInfo,
                QueryOutcome::Answer(heapless::Vec::from_slice(&body[..kept]).unwrap())
            )]
        );
    }
}

#[tokio::test]
async fn get_status_reports_the_sdb_and_accepts_older_shorter_blocks() {
    use crate::sink::device_policy_manager::PartnerQuery::Status;
    use crate::sink::device_policy_manager::QueryOutcome;
    // Section 6.5: a 5-byte PD R3.0 SDB is valid; bytes past the 7 are Ignored.
    for (size, kept) in [(5usize, 5usize), (7, 7), (8, 7)] {
        let mut engine = query_engine(Status);
        let block: std::vec::Vec<u8> = (1..=size as u8).collect();
        let (objects, body) = chunk(&block);
        let sent = query(&mut engine, Some((source_header(2, objects, 1, true), &body))).await;
        // Get_Status: Control 10010b, PD3, Sink/UFP.
        std::assert_eq!(u16::from_le_bytes([sent[0][0], sent[0][1]]) & 0xf1ff, 0x0092);
        std::assert!(matches!(engine.state, State::Ready(..)));
        std::assert_eq!(
            engine.device_policy_manager.outcomes,
            [(
                Status,
                QueryOutcome::Answer(heapless::Vec::from_slice(&block[..kept]).unwrap())
            )]
        );
    }
    // A 4-byte block is not an SDB: a Protocol Error.
    let mut engine = query_engine(Status);
    let (objects, body) = chunk(&[0; 4]);
    query(&mut engine, Some((source_header(2, objects, 1, true), &body))).await;
    std::assert!(matches!(engine.state, State::SendSoftReset));
}

#[derive(Default)]
struct SinkAlertRecorder {
    ask: Option<u32>,
    sent: usize,
    not_sent: usize,
    finished: usize,
    statuses: usize,
}
impl crate::sink::device_policy_manager::DevicePolicyManager for SinkAlertRecorder {
    fn status(&self) -> Option<[u8; 7]> {
        Some([0, 2, 0, 0, 0, 0, 0])
    }
    async fn status_sent(&mut self, block: [u8; 7]) {
        std::assert_eq!(block, [0, 2, 0, 0, 0, 0, 0]);
        self.statuses += 1;
    }
    async fn sink_alert_sent(&mut self, _ado: u32) {
        self.sent += 1;
    }
    async fn sink_alert_not_sent(&mut self, _ado: u32) {
        self.not_sent += 1;
    }
    async fn sink_alert_finished(&mut self) {
        self.finished += 1;
    }
    async fn get_event(&mut self, _: &SourceCapabilities) -> crate::sink::device_policy_manager::Event {
        match self.ask.take() {
            Some(ado) => crate::sink::device_policy_manager::Event::SendSinkAlert(ado),
            None => core::future::pending().await,
        }
    }
}
fn sink_alert_engine() -> Sink<DummyDriver<MAX_DATA_MESSAGE_SIZE>, DummyTimer, SinkAlertRecorder> {
    let mut engine = Sink::new(
        DummyDriver::new(),
        SinkAlertRecorder {
            ask: Some(1 << 29),
            ..Default::default()
        },
    );
    engine.contract = super::Contract::Explicit;
    engine.established_request = Some(PPS_CONTRACT);
    engine.source_capabilities = Some(source_capabilities_fixture());
    engine.state = State::Ready(PPS_CONTRACT, false);
    engine
}

#[tokio::test]
async fn sink_alert_sends_ado_then_waits_for_get_status_and_reports_snapshot() {
    let mut engine = sink_alert_engine();
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::SendSinkAlert(..)));
    simulate_source_control_message(&mut engine, ControlMessageType::GoodCRC, 0);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::WaitForGetStatus(_)));
    std::assert_eq!(engine.device_policy_manager.sent, 1);
    std::assert_eq!(
        &engine.protocol_layer.driver().probe_transmitted_data()[..],
        &[0x86, 0x10, 0, 0, 0, 0x20]
    );
    simulate_source_control_message(&mut engine, ControlMessageType::GetStatus, 1);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::GiveStatus(..)));
    std::assert_eq!(engine.device_policy_manager.finished, 1);
    simulate_source_control_message(&mut engine, ControlMessageType::GoodCRC, 1);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::Ready(..)));
    std::assert_eq!(engine.device_policy_manager.statuses, 1);
    let mut found = false;
    while engine.protocol_layer.driver().has_transmitted_data() {
        let bytes = engine.protocol_layer.driver().probe_transmitted_data();
        if bytes.len() == 14 {
            std::assert_eq!(&bytes[..], &[0x82, 0xb2, 7, 0x80, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0]);
            found = true;
        }
    }
    std::assert!(found);
}

#[tokio::test]
async fn deferred_sink_alert_keeps_message_id_and_does_not_clear_event_bits() {
    let mut engine: Sink<DeferringDriver, DummyTimer, SinkAlertRecorder> = Sink::new(
        DeferringDriver {
            defer: 1,
            tx: std::vec::Vec::new(),
        },
        SinkAlertRecorder::default(),
    );
    engine.contract = super::Contract::Explicit;
    engine.state = State::SendSinkAlert(PPS_CONTRACT, 1 << 29);
    engine.run_step().await.unwrap();
    std::assert!(matches!(engine.state, State::Ready(..)));
    std::assert_eq!(engine.device_policy_manager.sent, 0);
    std::assert_eq!(engine.device_policy_manager.not_sent, 1);
    std::assert!(engine.protocol_layer.driver().tx.is_empty());
    engine.protocol_layer.transmit_sink_alert(1 << 29).await.unwrap();
    std::assert_eq!((engine.protocol_layer.driver().tx[0][1] >> 1) & 7, 0);
}

#[tokio::test]
async fn sink_alert_timeout_is_ready_and_unexpected_reply_is_soft_reset() {
    for reply in [None, Some(ControlMessageType::GetSinkCap)] {
        let mut engine = sink_alert_engine();
        engine.run_step().await.unwrap();
        simulate_source_control_message(&mut engine, ControlMessageType::GoodCRC, 0);
        engine.run_step().await.unwrap();
        if let Some(reply) = reply {
            simulate_source_control_message(&mut engine, reply, 1);
        }
        engine.run_step().await.unwrap();
        std::assert_eq!(engine.device_policy_manager.finished, 1);
        std::assert_eq!(engine.device_policy_manager.statuses, 0);
        if reply.is_some() {
            std::assert!(matches!(engine.state, State::SendSoftReset));
        } else {
            std::assert!(matches!(engine.state, State::Ready(..)));
        }
    }
}

#[tokio::test]
async fn sink_alert_pd2_and_sink_reserved_events_are_not_sent() {
    use crate::protocol_layer::message::header::SpecificationRevision;
    for ado in [0, 1 << 26, 1 << 25, 1 << 31, 1 << 29] {
        let mut engine = sink_alert_engine();
        engine.device_policy_manager.ask = Some(ado);
        if ado == 1 << 29 {
            engine.protocol_layer = crate::protocol_layer::SinkProtocolLayer::new(
                DummyDriver::new(),
                Header::new_template(
                    crate::DataRole::Ufp,
                    crate::PowerRole::Sink,
                    SpecificationRevision::R2_0,
                ),
            );
        }
        engine.run_step().await.unwrap();
        std::assert!(matches!(engine.state, State::Ready(..)));
        std::assert_eq!(engine.device_policy_manager.not_sent, 1);
        std::assert!(!engine.protocol_layer.driver().has_transmitted_data());
    }
}
