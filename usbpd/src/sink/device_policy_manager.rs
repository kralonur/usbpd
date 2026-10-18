//! The device policy manager (DPM) allows a device to control the policy engine, and be informed about status changes.
//!
//! For example, through the DPM, a device can request certain source capabilities (voltage, current),
//! or renegotiate the power contract.
use core::future::Future;

use crate::protocol_layer::message::data::{epr_mode, request, sink_capabilities, source_capabilities};
use crate::units::Power;

/// Events that the device policy manager can send to the policy engine.
#[derive(Debug)]
pub enum Event {
    /// Empty event.
    None,
    /// Request SPR source capabilities.
    RequestSprSourceCapabilities,
    /// Request EPR source capabilities (when already in EPR mode).
    ///
    /// Sends EprGetSourceCap extended control message.
    /// See [8.3.3.8.1]
    RequestEprSourceCapabilities,
    /// Enter EPR mode with the specified operational PDP.
    ///
    /// Initiates EPR mode entry sequence (EPR_Mode Enter -> EnterAcknowledged -> EnterSucceeded).
    /// After successful entry, source automatically sends EPR_Source_Capabilities.
    ///
    /// Per USB PD spec 6.4.10, the Data field in EPR_Mode(Enter) shall be set to the
    /// EPR Sink Operational PDP. For example, a 28V × 5A = 140W device should pass 140W.
    ///
    /// See spec Table 8.39: "Steps for Entering EPR Mode (Success)"
    EnterEprMode(Power),
    /// Exit EPR mode (sink-initiated).
    ///
    /// Sends EPR_Mode (Exit) message to source, then waits for Source_Capabilities.
    /// After receiving caps, negotiation proceeds as normal SPR negotiation.
    /// See spec Table 8.46: "Steps for Exiting EPR Mode (Sink Initiated)"
    ExitEprMode,
    /// Explicit bounded cable discovery, through the exclusive driver handoff.
    DiscoverCable,
    /// Request a certain power level.
    RequestPower(request::PowerSource),
    /// Query the Source's PPS status (PE_SNK_Get_PPS_Status, section 9.2.11.3).
    ///
    /// Acted on only under PD3 with a PPS contract; otherwise the outcome is
    /// [`PpsStatusOutcome::NotSent`].
    GetPpsStatus,
    /// Ask the Source for information (Table 7.4 Recommended initiators). Acted
    /// on only under PD3; otherwise the outcome is [`QueryOutcome::NotSent`].
    Query(PartnerQuery),
    /// A detected non-Battery Sink status change (Table 6.25), from PD3 Ready:
    /// OVP, input change, thermal operating-condition change or OTP (bits 30..27).
    /// Battery and Extended Alert Events are not implemented by this path.
    SendSinkAlert(u32),
}

impl Event {
    /// Enter with the SKEDB operational PDP in whole watts (section 6.4.10).
    pub fn enter_epr_mode(operational_pdp_w: u8) -> Self {
        Self::EnterEprMode(Power::new::<uom::si::power::watt>(u32::from(operational_pdp_w)))
    }
}

/// A Sink-initiated information request from PE_SNK_Ready. Each sends one
/// Message, runs SenderResponseTimer (VDMResponseTimer for identity), and
/// returns to Ready on the answer or timeout.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PartnerQuery {
    /// Get_Sink_Cap -> the source-mode partner's SPR Sink_Capabilities.
    SinkCapabilities,
    /// SOP Structured VDM Discover Identity, without a data-role swap (PD3).
    DiscoverIdentity,
    /// Get_Source_Cap_Extended -> Source_Capabilities_Extended
    /// (PE_SNK_Get_Source_Cap_Ext, section 9.2.9.1.1).
    SourceCapabilitiesExtended,
    /// Get_Revision -> Revision (PE_Get_Revision, section 9.2.16.1.1).
    Revision,
    /// Get_Manufacturer_Info for the Port (Target 0, Table 6.56) ->
    /// Manufacturer_Info (PE_Get_Manufacturer_Info, section 9.2.14.1.1).
    ManufacturerInfo,
    /// Get_Source_Info -> Source_Info (PE_SNK_Get_Source_Info, section 9.2.10.1.1).
    SourceInfo,
    /// Get_Status -> Status (PE_Get_Status, section 9.2.11.1.1).
    Status,
    /// Get_Battery_Cap for Battery `reference` (Table 6.53) ->
    /// Battery_Capabilities (PE_Get_Battery_Cap, section 9.2.12.1.1).
    BatteryCapabilities(u8),
    /// Get_Battery_Status for Battery `reference` (Table 6.54) ->
    /// Battery_Status (PE_Get_Battery_Status, section 9.2.13.1.1).
    BatteryStatus(u8),
}

/// Outcome of a [`PartnerQuery`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum QueryOutcome {
    /// The answer's Data Block as received: the SCEDB (Table 6.50), the
    /// Manufacturer_Info block (Table 6.57), the SDB (Table 6.51), the BCDB
    /// (Table 6.55), or the RMDO (Table 6.31) / SIDO1 and SIDO2 (Tables 6.29,
    /// 6.30) / BSDO (Table 6.24) in little-endian byte order. Port queries retain
    /// up to seven PDOs or identity objects, including the structured VDM header.
    Answer(heapless::Vec<u8, 28>),
    /// The partner declined Structured VDM discovery with NAK.
    Nak,
    /// The partner answered Structured VDM discovery with BUSY; not retried.
    Busy,
    /// The Source answered Not_Supported.
    NotSupported,
    /// SenderResponseTimer expired.
    Timeout,
    /// The request was not sent (driver `Deferred`, sections 7.2/7.3).
    Deferred,
    /// Not started: the negotiated revision is not PD3.
    NotSent,
}

/// Outcome of a Get_PPS_Status request (sections 7.16, 9.2.11.3.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PpsStatusOutcome {
    /// PPS_Status received: the raw PPS Status Data Block (Table 6.58).
    Status([u8; 4]),
    /// The Source answered Not_Supported.
    NotSupported,
    /// SenderResponseTimer expired (Figure 9.37).
    Timeout,
    /// Get_PPS_Status was not sent (driver `Deferred`, sections 7.2/7.3).
    Deferred,
    /// Not started: no PD3 PPS contract.
    NotSent,
}

/// Trait for the device policy manager.
///
/// This entity commands the policy engine and enforces device policy.
pub trait DevicePolicyManager {
    /// Whether received multi-Chunk Extended Messages are assembled. A port
    /// with no such requirement May omit the Chunking layer (section 7.31.15):
    /// it then answers Not_Supported after ChunkingNotSupportedTimer.
    const CHUNKING: bool = true;

    /// Whether this sink is EPR capable. A sink that is not answers EPR
    /// requests and capabilities as unsupported (Table 7.4).
    const EPR: bool = true;

    /// Revision Message Data Object (Table 6.31) returned for Get_Revision.
    fn revision(&self) -> u32 {
        0x3212_0000
    }

    /// Sink Capabilities Extended Data Block (Table 6.61) returned for PD3
    /// Get_Sink_Cap_Extended. The default `None` answers Not_Supported.
    fn sink_capabilities_extended(&self) -> Option<[u8; 24]> {
        None
    }

    /// Current SOP Status Data Block (Table 6.51); None declines Get_Status.
    fn status(&self) -> Option<[u8; 7]> {
        None
    }

    /// Successful Status transmission: clear acknowledged OCP/OTP/OVP event
    /// flags, preserving later events and the real-time CV/CL flag (Table 6.51).
    fn status_sent(&mut self, _block: [u8; 7]) -> impl Future<Output = ()> {
        async {}
    }

    /// Alert acknowledged: clear the reported ADO event bits (section 7.14.1).
    fn sink_alert_sent(&mut self, _ado: u32) -> impl Future<Output = ()> {
        async {}
    }

    /// Alert was not sent (PD2, or proven unsent arbitration deferral).
    fn sink_alert_not_sent(&mut self, _ado: u32) -> impl Future<Output = ()> {
        async {}
    }

    /// The bounded wait for Get_Status ended (section 9.2.8.3.2).
    fn sink_alert_finished(&mut self) -> impl Future<Output = ()> {
        async {}
    }

    /// Inform the device of a received PD3 Alert (PE_SNK_Source_Alert_Received).
    /// The Alert AMS has no mandatory response; the default ignores the ADO.
    fn alert(&mut self, _alert_data_object: u32) -> impl Future<Output = ()> {
        async {}
    }

    /// Inform the device about source capabilities, e.g. after a request.
    fn inform(&mut self, _source_capabilities: &source_capabilities::SourceCapabilities) -> impl Future<Output = ()> {
        async {}
    }

    /// Request a power source.
    ///
    /// Defaults to 5 V at maximum current.
    fn request(
        &mut self,
        source_capabilities: &source_capabilities::SourceCapabilities,
    ) -> impl Future<Output = request::PowerSource> {
        async {
            request::PowerSource::new_fixed(
                request::CurrentRequest::Highest,
                request::VoltageRequest::Safe5V,
                source_capabilities,
            )
            .unwrap()
        }
    }

    /// Notify policy that a Request was declined. The default preserves the
    /// timed retry after Wait; return false to require policy-driven
    /// retries instead. Reject is never automatically retried.
    fn request_declined(
        &mut self,
        _requested: &request::PowerSource,
        _response: crate::protocol_layer::message::header::ControlMessageType,
    ) -> impl Future<Output = bool> {
        core::future::ready(true)
    }

    /// Notify policy that a Sink-initiated Request was not sent under an
    /// Explicit Contract (driver `Deferred`: SinkTxNG or a received SOP Message,
    /// sections 7.2/7.3). The engine returns to Ready with the established
    /// contract; the default does nothing.
    fn request_deferred(&mut self, _requested: &request::PowerSource) -> impl Future<Output = ()> {
        async {}
    }

    /// Inform the device of the outcome of PE_SNK_Get_PPS_Status (section 9.2.11.3.1).
    fn pps_status(&mut self, _outcome: PpsStatusOutcome) -> impl Future<Output = ()> {
        async {}
    }

    /// Inform the device of the outcome of a [`PartnerQuery`] on exit from its state.
    fn query(&mut self, _query: PartnerQuery, _outcome: QueryOutcome) -> impl Future<Output = ()> {
        async {}
    }

    /// Notify the device that it shall transition to a new power level.
    ///
    /// The device is informed about the request that was accepted by the source.
    fn transition_power(&mut self, _accepted: &request::PowerSource) -> impl Future<Output = ()> {
        async {}
    }

    /// Notify the device that a hard reset has occurred.
    ///
    /// Per USB PD Spec R3.2 Section 8.3.3.3.9, on entry to PE_SNK_Transition_to_default:
    /// - The sink shall transition to default power level (vSafe5V)
    /// - Local hardware should be reset
    /// - Port data role should be set to UFP
    ///
    /// The device should prepare for VBUS going to vSafe0V and then back to vSafe5V.
    /// This callback should return when the device has reached the default level.
    fn hard_reset(&mut self) -> impl Future<Output = ()> {
        async {}
    }

    /// Notify the device that EPR mode entry failed.
    ///
    /// Per USB PD Spec R3.2 Section 8.3.3.26.2.1, when the source responds with
    /// EPR_Mode (Enter Failed), the sink transitions to soft reset. This callback
    /// informs the DPM of the failure reason before the soft reset occurs.
    ///
    /// The failure reasons are defined in Table 6.50 and include:
    /// - Cable not EPR capable
    /// - Source failed to become VCONN source
    /// - EPR capable bit not set in RDO
    /// - Source unable to enter EPR mode (sink may retry later)
    /// - EPR capable bit not set in PDO
    fn epr_mode_entry_failed(&mut self, _reason: epr_mode::DataEnterFailed) -> impl Future<Output = ()> {
        async {}
    }

    /// Optional notification for policies opting into DiscoverCable. Default
    /// policies never request discovery and intentionally ignore its reports;
    /// a policy that requests discovery overrides this to receive the result.
    fn cable_report(&mut self, _report: usbpd_traits::CableReport) -> impl Future<Output = ()> {
        core::future::ready(())
    }

    /// Get the sink's power capabilities.
    ///
    /// Per USB PD Spec R3.2 Section 6.4.1.6, sinks respond to Get_Sink_Cap messages
    /// with a Sink_Capabilities message containing PDOs describing what power levels
    /// the sink can operate at.
    ///
    /// All sinks shall minimally offer one PDO at vSafe5V. The default implementation
    /// returns a single 5V @ 100mA PDO.
    fn sink_capabilities(&self) -> sink_capabilities::SinkCapabilities {
        // Default: 5V @ 100mA (1A = 100 * 10mA)
        sink_capabilities::SinkCapabilities::new_vsafe5v_only(100)
    }

    /// EPR Sink_Capabilities, including zero padding through position seven
    /// and EPR objects starting at position eight (section 6.5.15.1).
    /// This is separate from the seven-object SPR advertisement.
    fn epr_sink_capabilities(&self) -> sink_capabilities::SinkCapabilities {
        self.sink_capabilities()
    }

    /// Successful EPR mode entry or commanded exit. Hard Reset is reported
    /// through `hard_reset` instead; Soft Reset does not leave EPR mode.
    fn epr_mode_changed(&mut self, _active: bool) -> impl Future<Output = ()> {
        async {}
    }

    /// Entry ended without a source failure reason (timeout or unsent Enter).
    fn epr_mode_entry_aborted(&mut self, _deferred: bool) -> impl Future<Output = ()> {
        async {}
    }

    /// The policy engine gets and evaluates device policy events when ready.
    ///
    /// By default, this is a future that never resolves.
    ///
    /// <div class="warning">
    /// The function must be safe to cancel. To determine whether your own methods are cancellation safe,
    /// look for the location of uses of .await. This is because when an asynchronous method is cancelled,
    /// that always happens at an .await. If your function behaves correctly even if it is restarted while waiting
    /// at an .await, then it is cancellation safe.
    /// </div>
    fn get_event(
        &mut self,
        _source_capabilities: &source_capabilities::SourceCapabilities,
    ) -> impl Future<Output = Event> {
        async { core::future::pending().await }
    }
}
