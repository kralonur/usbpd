//! Guard against role changes, truncated seven-object replies and invented identity VDOs.
use super::*;
use crate::sink::device_policy_manager::{PartnerQuery, QueryOutcome};

#[tokio::test]
async fn port_queries_keep_the_contract_and_all_received_objects() {
    for objects in [
        std::vec![0x3601_912c, 0x0002_d0c8, 0x8b41_90c8, 0xc0dc_212c],
        std::vec![0x3601_912c; 7],
    ] {
        let body: std::vec::Vec<u8> = objects.iter().flat_map(|raw: &u32| raw.to_le_bytes()).collect();
        let mut engine = query_engine(PartnerQuery::SinkCapabilities);
        let established = engine.established_request;
        let sent = query(
            &mut engine,
            Some((source_header(4, objects.len() as u16, 1, false), &body)),
        )
        .await;
        assert_eq!(sent[0], [0x88, 0x00]);
        assert!(
            sent[1..]
                .iter()
                .all(|frame| frame.len() == 2 && u16::from_le_bytes([frame[0], frame[1]]) & 0xf01f == 1)
        );
        assert_eq!(
            engine.device_policy_manager.outcomes,
            [(
                PartnerQuery::SinkCapabilities,
                QueryOutcome::Answer(heapless::Vec::from_slice(&body).unwrap())
            )]
        );
        assert!(matches!((engine.established_request, established),
            (Some(PowerSource::FixedVariableSupply(actual)), Some(PowerSource::FixedVariableSupply(before))) if actual.0 == before.0));
        assert!(matches!(engine.state, State::Ready(..)));
    }

    // Four message objects means only three identity VDOs after the VDM header.
    let body: std::vec::Vec<u8> = [0xff00_a041u32, 0xd040_04e8, 0, 0x6860_0000]
        .into_iter()
        .flat_map(u32::to_le_bytes)
        .collect();
    let mut engine = query_engine(PartnerQuery::DiscoverIdentity);
    let established = engine.established_request;
    let sent = query(&mut engine, Some((source_header(15, 4, 1, false), &body))).await;
    assert_eq!(sent[0], [0x8f, 0x10, 0x01, 0xa8, 0x00, 0xff]);
    assert!(
        sent[1..]
            .iter()
            .all(|frame| frame.len() == 2 && u16::from_le_bytes([frame[0], frame[1]]) & 0xf01f == 1)
    );
    assert_eq!(
        engine.device_policy_manager.outcomes,
        [(
            PartnerQuery::DiscoverIdentity,
            QueryOutcome::Answer(heapless::Vec::from_slice(&body).unwrap())
        )]
    );
    assert!(matches!((engine.established_request, established),
        (Some(PowerSource::FixedVariableSupply(actual)), Some(PowerSource::FixedVariableSupply(before))) if actual.0 == before.0));
    assert!(matches!(engine.state, State::Ready(..)));
}

#[tokio::test]
async fn identity_nak_busy_and_timeout_return_to_ready_without_retry() {
    for (vdm, outcome) in [(0xff00_a081u32, QueryOutcome::Nak), (0xff00_a0c1, QueryOutcome::Busy)] {
        let mut engine = query_engine(PartnerQuery::DiscoverIdentity);
        query(&mut engine, Some((source_header(15, 1, 1, false), &vdm.to_le_bytes()))).await;
        assert_eq!(
            engine.device_policy_manager.outcomes,
            [(PartnerQuery::DiscoverIdentity, outcome)]
        );
        assert!(matches!(engine.state, State::Ready(..)));
    }
    for ask in [PartnerQuery::SinkCapabilities, PartnerQuery::DiscoverIdentity] {
        let mut engine = query_engine(ask);
        query(&mut engine, None).await;
        assert_eq!(engine.device_policy_manager.outcomes, [(ask, QueryOutcome::Timeout)]);
        assert!(matches!(engine.state, State::Ready(..)));
    }
}

#[tokio::test]
async fn identity_uses_the_common_version_on_later_scans() {
    for version in [0x2000u32, 0] {
        let mut engine = query_engine(PartnerQuery::DiscoverIdentity);
        let body: std::vec::Vec<u8> = [0xff00_8041 | version, 0xd040_04e8, 0, 0x6860_0000]
            .into_iter()
            .flat_map(u32::to_le_bytes)
            .collect();
        query(&mut engine, Some((source_header(15, 4, 1, false), &body))).await;
        engine.device_policy_manager.ask.push(PartnerQuery::DiscoverIdentity);
        engine.run_step().await.unwrap();
        simulate_source_control_message(&mut engine, ControlMessageType::GoodCRC, 1);
        inject(&mut engine, source_header(15, 4, 2, false), &body);
        engine.run_step().await.unwrap();
        let request = engine.protocol_layer.driver().probe_transmitted_data().to_vec();
        assert_eq!(
            u32::from_le_bytes(request[2..6].try_into().unwrap()),
            0xff00_8001 | version
        );
        assert!(matches!(engine.state, State::Ready(..)));
    }
}

#[tokio::test]
async fn identity_rejects_wrong_command_and_missing_mandatory_vdos() {
    for objects in [std::vec![0xff00_a042u32, 0, 0, 0], std::vec![0xff00_a041, 0, 0]] {
        let body: std::vec::Vec<u8> = objects.iter().flat_map(|raw| raw.to_le_bytes()).collect();
        let mut engine = query_engine(PartnerQuery::DiscoverIdentity);
        query(
            &mut engine,
            Some((source_header(15, objects.len() as u16, 1, false), &body)),
        )
        .await;
        assert!(engine.device_policy_manager.outcomes.is_empty());
        assert!(matches!(engine.state, State::SendSoftReset));
    }
}
