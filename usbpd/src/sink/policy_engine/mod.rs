//! Policy engine for the implementation of a sink.
use core::marker::PhantomData;

use embassy_futures::select::{Either, Either3, select, select3};
use uom::si::power::watt;
use usbpd_traits::Driver;

use super::device_policy_manager::DevicePolicyManager;
use crate::counters::Counter;
use crate::protocol_layer::message::data::epr_mode::{self, Action};
use crate::protocol_layer::message::data::request::PowerSource;
use crate::protocol_layer::message::data::source_capabilities::SourceCapabilities;
use crate::protocol_layer::message::data::{Data, request};
use crate::protocol_layer::message::extended::extended_control::ExtendedControlMessageType;
use crate::protocol_layer::message::header::{
    ControlMessageType, DataMessageType, ExtendedMessageType, Header, MessageType, SpecificationRevision,
};
use crate::protocol_layer::message::{Payload, extended};
use crate::protocol_layer::{ProtocolError, RxError, SinkProtocolLayer, TxError};
use crate::sink::device_policy_manager::{Event, PartnerQuery, PpsStatusOutcome, QueryOutcome};
use crate::timers::{Timer, TimerType};
use crate::{DataRole, PowerRole, units};

#[cfg(test)]
mod tests;

/// Sink capability
#[derive(Debug, Clone, Copy, PartialEq)]
enum Mode {
    /// The classic mode of PD operation where explicit contracts are negotiaged using SPR (A)PDOs.
    Spr,
    /// A Power Delivery mode of operation where maximum allowable voltage is 48V.
    Epr,
}

#[derive(Debug, Clone, Copy, Default)]
enum Contract {
    #[default]
    Safe5V,
    _Implicit, // FIXME: Only present after fast role swap, yet unsupported. Limited to max. type C current.
    TransitionToExplicit,
    Explicit,
}

/// Sink states.
#[derive(Debug, Clone)]
enum State {
    // States of the policy engine as given by the specification.
    /// Default state at startup.
    Startup,
    Discovery,
    WaitForCapabilities,
    EvaluateCapabilities(SourceCapabilities),
    SelectCapability(request::PowerSource),
    TransitionSink(request::PowerSource),
    /// Ready state. The bool indicates if we entered due to receiving a Wait message,
    /// which requires running SinkRequestTimer before allowing re-request.
    Ready(request::PowerSource, bool),
    SendNotSupported(request::PowerSource),
    /// PE_DRS_UFP_DFP_Reject_Swap or PE_VCS_Reject_VCONN_Swap: Reject a DR_Swap
    /// (UFP-only Sink) or a VCONN_Swap (this Sink is not the VCONN Source in Ready).
    RejectSwap(request::PowerSource),
    /// PE_BIST_Test_Mode (section 9.2.26.4.2.1): only GoodCRC until Hard Reset.
    BistTestData,
    /// PE_BIST_Carrier_Mode (section 9.2.26.4.1.1).
    BistCarrierMode(request::PowerSource),
    /// A Chunk of a multi-Chunk Message arrived without a Chunking layer.
    ChunkReceived(request::PowerSource),
    GiveRevision(request::PowerSource),
    GiveSinkCapExtended(request::PowerSource, [u8; 24]),
    GiveStatus(request::PowerSource, [u8; 7]),
    SendSinkAlert(request::PowerSource, u32),
    WaitForGetStatus(request::PowerSource),
    GetPpsStatus(request::PowerSource),
    /// PE_SNK_Get_Source_Cap_Ext, PE_Get_Revision or PE_Get_Manufacturer_Info.
    Query(request::PowerSource, PartnerQuery),
    DiscoverCable(request::PowerSource),
    SendSoftReset,
    SoftReset,
    HardReset,
    TransitionToDefault,
    /// Give sink capabilities. The Mode indicates whether to send Sink_Capabilities (Spr)
    /// or EPR_Sink_Capabilities (Epr) per spec 8.3.3.3.10.
    GiveSinkCap(Mode, request::PowerSource),
    GetSourceCap(Mode, request::PowerSource),

    // EPR states
    EprModeEntry(request::PowerSource, units::Power),
    EprWaitForCapabilities(request::PowerSource),
    EprSendExit(request::PowerSource),
    EprExitReceived(request::PowerSource),
    EprKeepAlive(request::PowerSource),
}

/// Implementation of the sink policy engine.
/// See spec, [8.3.3.3]
#[derive(Debug)]
pub struct Sink<DRIVER: Driver, TIMER: Timer, DPM: DevicePolicyManager> {
    device_policy_manager: DPM,
    protocol_layer: SinkProtocolLayer<DRIVER, TIMER>,
    contract: Contract,
    hard_reset_counter: Counter,
    source_capabilities: Option<SourceCapabilities>,
    mode: Mode,
    state: State,
    /// Tracks whether a Get_Source_Cap request is pending.
    /// Per USB PD Spec R3.2 Section 8.3.3.3.8, in EPR mode, receiving a
    /// Source_Capabilities message that was not requested via Get_Source_Cap
    /// shall trigger a Hard Reset.
    get_source_cap_pending: bool,
    established_request: Option<request::PowerSource>,
    wait_request: Option<request::PowerSource>,
    /// From a Soft Reset (sent or accepted) until the renegotiated Request ends
    /// (PS_RDY, Reject or Wait). A further Protocol Error then means the Soft
    /// Reset did not correct the error: Hard Reset (section 7.1.1).
    recovering: bool,

    _timer: PhantomData<TIMER>,
}

/// Errors that can occur in the sink policy engine state machine.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The port partner is unresponsive.
    PortPartnerUnresponsive,
    /// A protocol error has occured.
    Protocol(ProtocolError),
}

impl From<ProtocolError> for Error {
    fn from(protocol_error: ProtocolError) -> Self {
        Error::Protocol(protocol_error)
    }
}

impl<DRIVER: Driver, TIMER: Timer, DPM: DevicePolicyManager> Sink<DRIVER, TIMER, DPM> {
    /// Create a fresh protocol layer with initial state.
    fn new_protocol_layer(driver: DRIVER) -> SinkProtocolLayer<DRIVER, TIMER> {
        let header = Header::new_template(DataRole::Ufp, PowerRole::Sink, SpecificationRevision::R3_X);
        let mut protocol_layer = SinkProtocolLayer::new(driver, header);
        protocol_layer.set_chunking(DPM::CHUNKING);
        protocol_layer
    }

    /// Create a new sink policy engine with a given `driver`.
    pub fn new(driver: DRIVER, device_policy_manager: DPM) -> Self {
        Self {
            device_policy_manager,
            protocol_layer: Self::new_protocol_layer(driver),
            state: State::Discovery,
            contract: Default::default(),
            hard_reset_counter: Counter::new(crate::counters::CounterType::HardReset),
            source_capabilities: None,
            mode: Mode::Spr,
            get_source_cap_pending: false,
            established_request: None,
            wait_request: None,
            recovering: false,
            _timer: PhantomData,
        }
    }

    /// Set a new driver when re-attached.
    pub fn re_attach(&mut self, driver: DRIVER) {
        self.protocol_layer = Self::new_protocol_layer(driver);
    }

    /// Run a single step in the policy engine state machine.
    async fn run_step(&mut self) -> Result<(), Error> {
        let result = self.update_state().await;
        self.note_recovery();
        if result.is_ok() {
            return Ok(());
        }

        if let Err(Error::Protocol(protocol_error)) = result {
            // Sections 7.2/7.3: an unsent Sink-initiated Request leaves the
            // Explicit Contract unchanged; return to Ready, where a Message that
            // caused the discard is received next.
            if let (
                State::SelectCapability(requested),
                ProtocolError::TransmitDeferred,
                Contract::Explicit,
                Some(established),
            ) = (&self.state, &protocol_error, self.contract, self.established_request)
            {
                let requested = *requested;
                self.device_policy_manager.request_deferred(&requested).await;
                self.state = State::Ready(established, false);
                return Ok(());
            }
            // The same for an unsent Get_PPS_Status (PE_SNK_Get_PPS_Status).
            if let (State::GetPpsStatus(power_source), ProtocolError::TransmitDeferred) = (&self.state, &protocol_error)
            {
                let power_source = *power_source;
                self.device_policy_manager.pps_status(PpsStatusOutcome::Deferred).await;
                self.state = State::Ready(power_source, false);
                return Ok(());
            }
            // A proven unsent Alert keeps its event bits; no automatic retry.
            if let (State::SendSinkAlert(power_source, ado), ProtocolError::TransmitDeferred) =
                (&self.state, &protocol_error)
            {
                let (power_source, ado) = (*power_source, *ado);
                self.device_policy_manager.sink_alert_not_sent(ado).await;
                self.state = State::Ready(power_source, false);
                return Ok(());
            }
            // The same for an unsent information request.
            if let (State::Query(power_source, query), ProtocolError::TransmitDeferred) = (&self.state, &protocol_error)
            {
                let (power_source, query) = (*power_source, *query);
                self.device_policy_manager.query(query, QueryOutcome::Deferred).await;
                self.state = State::Ready(power_source, false);
                return Ok(());
            }
            // The same for an unsent Get_Source_Cap or EPR_Get_Source_Cap
            // (PE_SNK_Get_Source_Cap): no AMS started, so nothing is awaited.
            if let (State::GetSourceCap(_, power_source), ProtocolError::TransmitDeferred) =
                (&self.state, &protocol_error)
            {
                let power_source = *power_source;
                self.get_source_cap_pending = false;
                self.state = State::Ready(power_source, false);
                return Ok(());
            }
            if let (State::EprModeEntry(power_source, _), ProtocolError::TransmitDeferred) =
                (&self.state, &protocol_error)
            {
                let power_source = *power_source;
                self.device_policy_manager.epr_mode_entry_aborted(true).await;
                self.state = State::Ready(power_source, false);
                return Ok(());
            }
            // Section 8.3.3.1: a Discarded first Message is retried after the
            // incoming Message. Ready handles that Message, still in EPR Mode,
            // and restarts SinkEPRKeepAliveTimer; the DPM asks to exit again.
            if let (
                State::EprKeepAlive(power_source) | State::EprSendExit(power_source),
                ProtocolError::TransmitDeferred,
            ) = (&self.state, &protocol_error)
            {
                self.state = State::Ready(*power_source, false);
                return Ok(());
            }
            let new_state = match (&self.mode, &self.state, protocol_error) {
                // Handle when hard reset is signaled by the driver itself.
                (_, _, ProtocolError::RxError(RxError::HardReset) | ProtocolError::TxError(TxError::HardReset)) => {
                    Some(State::TransitionToDefault)
                }

                // Section 6.4.3.1: in BIST Test Data Mode nothing but Hard Reset is
                // acted on, including a Soft_Reset or a malformed Message.
                (_, State::BistTestData, _) => Some(State::BistTestData),

                // Handle when soft reset is signaled by the driver itself.
                (_, _, ProtocolError::RxError(RxError::SoftReset)) => Some(State::SoftReset),

                // Per spec 6.3.13: If the Soft_Reset Message fails, a Hard Reset shall be initiated.
                // This handles the case where we're trying to send/receive a soft reset and it fails.
                (_, State::SoftReset | State::SendSoftReset, ProtocolError::TransmitRetriesExceeded(_)) => {
                    Some(State::HardReset)
                }

                // Sections 7.7, 9.2.5.2.1: in PE_SNK_Send_Soft_Reset, a SenderResponseTimer
                // timeout or any Protocol Error (no Accept) leads to Hard Reset, never
                // another Soft_Reset. A received Soft_Reset is handled above.
                (_, State::SendSoftReset, _) => Some(State::HardReset),

                // Section 7.1.1: while recovering, any further Protocol Error while
                // waiting for capabilities or for the Request's answer means the
                // Soft Reset did not correct the error: Hard Reset, never a second
                // Soft_Reset. (Timeouts: arms below.)
                (_, State::WaitForCapabilities | State::SelectCapability(_), ref error)
                    if self.recovering && !matches!(error, ProtocolError::RxError(RxError::ReceiveTimeout)) =>
                {
                    Some(State::HardReset)
                }

                (_, State::GiveSinkCap(Mode::Epr, _), ProtocolError::RxError(RxError::ReceiveTimeout)) => {
                    Some(State::SendSoftReset)
                }
                (_, State::EprWaitForCapabilities(_), _) => Some(State::HardReset),

                // Per spec 8.3.3.3.3: SinkWaitCapTimer timeout triggers Hard Reset.
                (_, State::WaitForCapabilities, ProtocolError::RxError(RxError::ReceiveTimeout)) => {
                    Some(State::HardReset)
                }

                // Per spec 8.3.3.3.5: SenderResponseTimer timeout triggers Hard Reset.
                (_, State::SelectCapability(_), ProtocolError::RxError(RxError::ReceiveTimeout)) => {
                    Some(State::HardReset)
                }

                // Per USB PD Spec R3.2 Section 8.3.3.3.6 and Table 6.72:
                // Any Protocol Error during power transition (PE_SNK_Transition_Sink state)
                // shall trigger a Hard Reset, not a Soft Reset.
                (_, State::TransitionSink(_), _) => Some(State::HardReset),

                // Table 7.1: during an AMS with power not transitioning, an Unsupported or
                // Unrecognized Message (or a Chunk this port cannot assemble) is also a
                // Protocol Error answered with Soft_Reset, not a repeat of the AMS.
                (
                    _,
                    State::SelectCapability(_) | State::GetPpsStatus(_) | State::Query(..) | State::WaitForGetStatus(_),
                    ProtocolError::RxError(RxError::UnsupportedMessage | RxError::ChunkingNotSupported),
                ) => Some(State::SendSoftReset),

                // Unexpected messages indicate a protocol error and demand a soft reset.
                // Per spec 6.8.1 Table 6.72 (for non-power-transitioning states).
                // Note: This must come AFTER TransitionSink check above.
                (_, _, ProtocolError::UnexpectedMessage) => Some(State::SendSoftReset),

                // Per spec Table 6.72: Unsupported messages in Ready state get Not_Supported response.
                (_, State::Ready(power_source, _), ProtocolError::RxError(RxError::UnsupportedMessage)) => {
                    Some(State::SendNotSupported(*power_source))
                }

                // Section 9.2.7: a Chunk of a multi-Chunk Message without a Chunking layer.
                (_, State::Ready(power_source, _), ProtocolError::RxError(RxError::ChunkingNotSupported)) => {
                    Some(State::ChunkReceived(*power_source))
                }

                // Per spec 6.6.9.1: Transmission failure (no GoodCRC after retries) triggers Soft Reset.
                // Note: If we're in SoftReset/SendSoftReset state, this is caught above and escalates to Hard Reset.
                (_, _, ProtocolError::TransmitRetriesExceeded(_)) => Some(State::SendSoftReset),

                // Unhandled protocol errors - log and continue.
                // Note: Unrequested Source_Capabilities in EPR mode is handled in Ready state
                // by checking get_source_cap_pending flag (per spec 8.3.3.3.8).
                (_, _, error) => {
                    error!("Protocol error {:?} in sink state transition", error);
                    None
                }
            };

            if let Some(state) = new_state {
                self.state = state
            }
            self.note_recovery();

            Ok(())
        } else {
            error!("Unrecoverable result {:?} in sink state transition", result);
            result
        }
    }

    /// Track `recovering` from the state just entered.
    fn note_recovery(&mut self) {
        match self.state {
            State::SoftReset | State::SendSoftReset => self.recovering = true,
            State::Ready(..) | State::Startup | State::TransitionToDefault => self.recovering = false,
            _ => {}
        }
    }

    /// Run the sink's state machine continuously.
    ///
    /// The loop is only broken for unrecoverable errors, for example if the port partner is unresponsive.
    pub async fn run(&mut self) -> Result<(), Error> {
        loop {
            self.run_step().await?;
        }
    }

    /// Wait for source capabilities message (either Source_Capabilities or EPR_Source_Capabilities).
    ///
    /// Per USB PD Spec R3.2 Section 8.3.3.3.3 (PE_SNK_Wait_for_Capabilities):
    /// - In SPR Mode: Source_Capabilities Message is received
    /// - In EPR Mode: EPR_Source_Capabilities Message is received
    ///
    /// EPR Mode persists through Soft Reset (unlike Hard Reset which exits EPR per spec 6.8.3.2).
    /// Per spec section 6.4.1.2.2, after a Soft Reset while in EPR Mode, the source sends
    /// EPR_Source_Capabilities. Therefore this function must handle both message types.
    async fn wait_for_source_capabilities(
        protocol_layer: &mut SinkProtocolLayer<DRIVER, TIMER>,
    ) -> Result<SourceCapabilities, Error> {
        let message = protocol_layer.wait_for_source_capabilities().await?;
        trace!("Source capabilities: {:?}", message);

        let capabilities = match message.payload {
            Some(Payload::Data(Data::SourceCapabilities(caps))) => caps,
            Some(Payload::Extended(extended::Extended::EprSourceCapabilities(pdos))) => SourceCapabilities(pdos),
            _ => return Err(Error::Protocol(ProtocolError::UnexpectedMessage)),
        };

        Ok(capabilities)
    }

    async fn update_state(&mut self) -> Result<(), Error> {
        let new_state = match &self.state {
            State::Startup => {
                self.contract = Default::default();
                self.established_request = None;
                self.wait_request = None;
                self.protocol_layer.reset();
                self.mode = Mode::Spr;

                State::Discovery
            }
            State::Discovery => {
                self.protocol_layer.wait_for_vbus().await;
                self.source_capabilities = None;

                State::WaitForCapabilities
            }
            State::WaitForCapabilities => {
                State::EvaluateCapabilities(Self::wait_for_source_capabilities(&mut self.protocol_layer).await?)
            }
            State::EvaluateCapabilities(capabilities) => {
                // Sink now knows that it is attached.
                self.source_capabilities = Some(capabilities.clone());

                self.hard_reset_counter.reset();

                let request = self
                    .device_policy_manager
                    .request(self.source_capabilities.as_ref().unwrap())
                    .await;

                State::SelectCapability(request)
            }
            State::SelectCapability(power_source) => {
                // Section 6.4.10.2: even SPR PDOs require EPR_Request in EPR mode.
                if (self.mode == Mode::Epr) != matches!(power_source, PowerSource::EprRequest(_)) {
                    return Err(ProtocolError::UnexpectedMessage.into());
                }
                self.protocol_layer.request_power(*power_source).await?;

                let message_type = self
                    .protocol_layer
                    .receive_message_type(
                        &[
                            MessageType::Control(ControlMessageType::Accept),
                            MessageType::Control(ControlMessageType::Wait),
                            MessageType::Control(ControlMessageType::Reject),
                        ],
                        TimerType::SenderResponse,
                    )
                    .await?
                    .header
                    .message_type();

                let MessageType::Control(control_message_type) = message_type else {
                    unreachable!()
                };

                let retry_wait = if matches!(
                    control_message_type,
                    ControlMessageType::Reject | ControlMessageType::Wait
                ) {
                    // The renegotiation ended: a Soft Reset corrected the error.
                    self.recovering = false;
                    self.device_policy_manager
                        .request_declined(power_source, control_message_type)
                        .await
                } else {
                    false
                };
                match (self.contract, control_message_type) {
                    (_, ControlMessageType::Accept) => State::TransitionSink(*power_source),
                    (Contract::Safe5V, ControlMessageType::Wait | ControlMessageType::Reject) => {
                        State::WaitForCapabilities
                    }
                    (Contract::Explicit, ControlMessageType::Reject) => {
                        self.wait_request = None;
                        State::Ready(self.established_request.expect("explicit contract"), false)
                    }
                    (Contract::Explicit, ControlMessageType::Wait) => {
                        self.wait_request = retry_wait.then_some(*power_source);
                        State::Ready(self.established_request.expect("explicit contract"), retry_wait)
                    }
                    _ => unreachable!(),
                }
            }
            State::TransitionSink(power_source) => {
                self.protocol_layer
                    .receive_message_type(
                        &[MessageType::Control(ControlMessageType::PsRdy)],
                        match self.mode {
                            Mode::Epr => TimerType::PSTransitionEpr,
                            Mode::Spr => TimerType::PSTransitionSpr,
                        },
                    )
                    .await?;

                self.contract = Contract::TransitionToExplicit;
                self.device_policy_manager.transition_power(power_source).await;
                self.established_request = Some(*power_source);
                self.wait_request = None;
                State::Ready(*power_source, false)
            }
            State::Ready(power_source, after_wait) => {
                // TODO: Entry: Init. and run DiscoverIdentityTimer(4)
                // TODO: Entry: Send GetSinkCap message if sink supports fast role swap
                // TODO: Exit: If initiating an AMS, notify protocol layer
                //
                // Timers implemented:
                // - SinkRequestTimer: Per spec 8.3.3.3.7, after receiving Wait, wait tSinkRequest
                //   before allowing re-request. On timeout, transition to SelectCapability.
                // - SinkPPSPeriodicTimer: triggers SelectCapability in SPR PPS mode
                // - SinkEPRKeepAliveTimer: triggers EprKeepAlive in EPR mode
                self.contract = Contract::Explicit;

                let receive_fut = self.protocol_layer.receive_message();
                let event_fut = self
                    .device_policy_manager
                    .get_event(self.source_capabilities.as_ref().unwrap());
                let pps_periodic_fut = async {
                    match power_source {
                        PowerSource::Pps(_) => TimerType::get_timer::<TIMER>(TimerType::SinkPPSPeriodic).await,
                        _ => core::future::pending().await,
                    }
                };
                let epr_keep_alive_fut = async {
                    match self.mode {
                        Mode::Epr => TimerType::get_timer::<TIMER>(TimerType::SinkEPRKeepAlive).await,
                        Mode::Spr => core::future::pending().await,
                    }
                };
                // Per spec 8.3.3.3.7: SinkRequestTimer runs concurrently when re-entering
                // Ready after a Wait response. On timeout, transition to SelectCapability.
                // Per spec 6.6.4.1: Ensures minimum tSinkRequest (100ms) delay before re-request.
                let sink_request_fut = async {
                    if *after_wait {
                        TimerType::get_timer::<TIMER>(TimerType::SinkRequest).await
                    } else {
                        core::future::pending().await
                    }
                };
                let timers_fut = async { select3(pps_periodic_fut, epr_keep_alive_fut, sink_request_fut).await };

                match select3(receive_fut, event_fut, timers_fut).await {
                    // A message was received.
                    Either3::First(message) => {
                        let message = message?;
                        let pd3 = matches!(
                            self.protocol_layer.header().spec_revision(),
                            Ok(SpecificationRevision::R3_X)
                        );

                        match message.header.message_type() {
                            MessageType::Data(DataMessageType::SourceCapabilities) => {
                                // Per USB PD Spec R3.2 Section 8.3.3.3.8:
                                // In EPR Mode, if a Source_Capabilities Message is received that
                                // has not been requested using a Get_Source_Cap Message, trigger Hard Reset.
                                if self.mode == Mode::Epr && !self.get_source_cap_pending {
                                    State::HardReset
                                } else {
                                    self.get_source_cap_pending = false;
                                    match message.payload {
                                        Some(Payload::Data(Data::SourceCapabilities(capabilities))) => {
                                            State::EvaluateCapabilities(capabilities)
                                        }
                                        _ => State::SendNotSupported(*power_source),
                                    }
                                }
                            }
                            MessageType::Extended(ExtendedMessageType::EprSourceCapabilities) if DPM::EPR => {
                                if let Some(Payload::Extended(extended::Extended::EprSourceCapabilities(pdos))) =
                                    message.payload
                                {
                                    self.get_source_cap_pending = false;
                                    let caps = SourceCapabilities(pdos);

                                    // Per spec 8.3.3.3.8: In EPR Mode, if EPR_Source_Capabilities
                                    // contains an EPR (A)PDO in positions 1-7 → Hard Reset
                                    if self.mode == Mode::Epr && caps.has_epr_pdo_in_spr_positions() {
                                        State::HardReset
                                    } else {
                                        State::EvaluateCapabilities(caps)
                                    }
                                } else {
                                    State::HardReset
                                }
                            }
                            // An SPR-only contract has no EPR Mode to exit; unsupported below.
                            MessageType::Data(DataMessageType::EprMode) if self.mode == Mode::Epr => {
                                if matches!(message.payload, Some(Payload::Data(Data::EprMode(mode)))
                                    if mode.action() == Action::Exit)
                                {
                                    State::EprExitReceived(*power_source)
                                } else {
                                    State::SendSoftReset
                                }
                            }
                            // PE_SNK_Source_Alert_Received: inform the DPM, no response.
                            MessageType::Data(DataMessageType::Alert) if pd3 => {
                                if let Some(Payload::Data(Data::Alert(ado))) = message.payload {
                                    self.device_policy_manager.alert(ado).await;
                                }
                                State::Ready(*power_source, *after_wait)
                            }
                            // Section 9.2.26.4: BIST Test Data and Carrier Mode only at vSafe5V
                            // (the vSafe5V Fixed PDO is always object 1). Other BIST modes, or
                            // BIST above vSafe5V, receive GoodCRC only.
                            MessageType::Data(DataMessageType::Bist) => match message.payload {
                                Some(Payload::Data(Data::Bist(bdo))) if power_source.object_position() == 1 => {
                                    match bdo >> 28 {
                                        0b1000 => State::BistTestData,
                                        0b0101 => State::BistCarrierMode(*power_source),
                                        _ => State::Ready(*power_source, *after_wait),
                                    }
                                }
                                _ => State::Ready(*power_source, *after_wait),
                            },
                            // PD2 V1.3 section 6.4.4: a port not supporting VDMs Ignores them.
                            // PD3 section 8.4/8.5 answers Not_Supported below.
                            MessageType::Data(DataMessageType::VendorDefined) if !pd3 => {
                                State::Ready(*power_source, *after_wait)
                            }
                            // PE_SNK_Not_Supported_Received: inform the DPM, no response.
                            MessageType::Control(ControlMessageType::NotSupported) if pd3 => {
                                State::Ready(*power_source, *after_wait)
                            }
                            // Section 7.25: Get_Revision responder support is Required.
                            MessageType::Control(ControlMessageType::GetRevision) if pd3 => {
                                State::GiveRevision(*power_source)
                            }
                            // Table 7.4: Get_Sink_Cap_Extended responder support is Required.
                            MessageType::Control(ControlMessageType::GetSinkCapExtended) if pd3 => {
                                match self.device_policy_manager.sink_capabilities_extended() {
                                    Some(block) => State::GiveSinkCapExtended(*power_source, block),
                                    None => State::SendNotSupported(*power_source),
                                }
                            }
                            MessageType::Control(ControlMessageType::GetStatus) if pd3 => {
                                match self.device_policy_manager.status() {
                                    Some(block) => State::GiveStatus(*power_source, block),
                                    None => State::SendNotSupported(*power_source),
                                }
                            }
                            // PD2 Ping needs GoodCRC only, not Reject/Not_Supported.
                            MessageType::Control(ControlMessageType::Ping)
                                if matches!(
                                    self.protocol_layer.header().spec_revision(),
                                    Ok(SpecificationRevision::R1_0 | SpecificationRevision::R2_0)
                                ) =>
                            {
                                State::Ready(*power_source, *after_wait)
                            }
                            // Per spec 8.3.3.3.7: Get_Sink_Cap → GiveSinkCap (send Sink_Capabilities)
                            MessageType::Control(ControlMessageType::GetSinkCap) => {
                                State::GiveSinkCap(Mode::Spr, *power_source)
                            }
                            // Section 7.12.1: the recipient of DR_Swap Shall answer Accept,
                            // Wait or Reject (PD2 V1.3 section 6.3.9 likewise). This Sink does
                            // not change Data Role and enters no SOP Modal Operation.
                            MessageType::Control(ControlMessageType::DrSwap) => State::RejectSwap(*power_source),
                            // Section 7.13: a recipient that is not presently the VCONN
                            // Source answers Reject (PD2 V1.3 section 6.3.11 likewise).
                            // Cable inspection switches VCONN off before Ready resumes.
                            // FIXME: a Sink that stays VCONN Source Shall Not Reject; it
                            // needs the Accept path.
                            MessageType::Control(ControlMessageType::VconnSwap) => State::RejectSwap(*power_source),
                            // Table 7.1: a Recognized, Supported but Unexpected Message in
                            // PE_SNK_Ready is a Protocol Error answered with Soft_Reset.
                            MessageType::Control(
                                ControlMessageType::Accept
                                | ControlMessageType::Reject
                                | ControlMessageType::PsRdy
                                | ControlMessageType::Wait,
                            ) => State::SendSoftReset,
                            // Per spec 8.3.3.3.7: EPR_Get_Sink_Cap → GiveSinkCap (send EPR_Sink_Capabilities)
                            MessageType::Extended(ExtendedMessageType::ExtendedControl) if DPM::EPR => {
                                if let Some(Payload::Extended(extended::Extended::ExtendedControl(ctrl))) =
                                    &message.payload
                                {
                                    if ctrl.try_message_type() == Ok(ExtendedControlMessageType::EprGetSinkCap) {
                                        State::GiveSinkCap(Mode::Epr, *power_source)
                                    } else {
                                        State::SendNotSupported(*power_source)
                                    }
                                } else {
                                    State::SendNotSupported(*power_source)
                                }
                            }
                            _ => State::SendNotSupported(*power_source),
                        }
                    }
                    // Event from device policy manager.
                    Either3::Second(event) => match event {
                        Event::RequestSprSourceCapabilities => State::GetSourceCap(Mode::Spr, *power_source),
                        Event::RequestEprSourceCapabilities => State::GetSourceCap(Mode::Epr, *power_source),
                        Event::EnterEprMode(pdp) if DPM::EPR && self.mode == Mode::Spr => {
                            State::EprModeEntry(*power_source, pdp)
                        }
                        Event::ExitEprMode if self.mode == Mode::Epr && power_source.object_position() <= 7 => {
                            State::EprSendExit(*power_source)
                        }
                        Event::EnterEprMode(_) | Event::ExitEprMode => State::Ready(*power_source, false),
                        Event::RequestPower(power_source) => State::SelectCapability(power_source),
                        Event::DiscoverCable => State::DiscoverCable(*power_source),
                        // Get_PPS_Status exists only in PD3 and only for a PPS contract.
                        Event::GetPpsStatus => {
                            let pd3 = matches!(
                                self.protocol_layer.header().spec_revision(),
                                Ok(SpecificationRevision::R3_X)
                            );
                            if pd3 && matches!(power_source, PowerSource::Pps(_)) {
                                State::GetPpsStatus(*power_source)
                            } else {
                                self.device_policy_manager.pps_status(PpsStatusOutcome::NotSent).await;
                                State::Ready(*power_source, false)
                            }
                        }
                        // The information requests exist only in PD3 (Table 6.5).
                        Event::Query(query) => {
                            let pd3 = matches!(
                                self.protocol_layer.header().spec_revision(),
                                Ok(SpecificationRevision::R3_X)
                            );
                            if pd3 {
                                State::Query(*power_source, query)
                            } else {
                                self.device_policy_manager.query(query, QueryOutcome::NotSent).await;
                                State::Ready(*power_source, false)
                            }
                        }
                        Event::SendSinkAlert(ado) => {
                            let pd3 = matches!(
                                self.protocol_layer.header().spec_revision(),
                                Ok(SpecificationRevision::R3_X)
                            );
                            if pd3
                                && ado != 0
                                && ado & !0x7800_0000 == 0
                                && self.device_policy_manager.status().is_some()
                            {
                                State::SendSinkAlert(*power_source, ado)
                            } else {
                                self.device_policy_manager.sink_alert_not_sent(ado).await;
                                State::Ready(*power_source, false)
                            }
                        }
                        Event::None => State::Ready(*power_source, false),
                    },
                    // Timer timeout handling
                    Either3::Third(timeout_source) => match timeout_source {
                        // PPS periodic timeout -> select capability again as keep-alive.
                        Either3::First(_) => State::SelectCapability(*power_source),
                        // EPR keep-alive timeout
                        Either3::Second(_) => State::EprKeepAlive(*power_source),
                        // SinkRequest timeout -> re-request power after Wait response
                        Either3::Third(_) => State::SelectCapability(self.wait_request.take().expect("Wait retry")),
                    },
                }
            }
            State::DiscoverCable(power_source) => {
                let report = self.protocol_layer.discover_cable().await?;
                self.device_policy_manager.cable_report(report).await;
                State::Ready(*power_source, false)
            }
            State::ChunkReceived(power_source) => {
                // PE_SNK_Chunk_Received: Not_Supported only after tChunkingNotSupported.
                TimerType::get_timer::<TIMER>(TimerType::ChunkingNotSupported).await;
                State::SendNotSupported(*power_source)
            }
            State::GiveRevision(power_source) => {
                let revision = self.device_policy_manager.revision();
                self.protocol_layer.transmit_revision(revision).await?;
                State::Ready(*power_source, false)
            }
            State::GiveSinkCapExtended(power_source, block) => {
                // PE_SNK_Give_Sink_Cap_Ext (section 9.2.9.4.1).
                self.protocol_layer.transmit_sink_capabilities_extended(*block).await?;
                State::Ready(*power_source, false)
            }
            State::GiveStatus(power_source, block) => {
                self.protocol_layer.transmit_status(*block).await?;
                self.device_policy_manager.status_sent(*block).await;
                State::Ready(*power_source, false)
            }
            State::SendSinkAlert(power_source, ado) => {
                self.protocol_layer.transmit_sink_alert(*ado).await?;
                self.device_policy_manager.sink_alert_sent(*ado).await;
                State::WaitForGetStatus(*power_source)
            }
            State::WaitForGetStatus(power_source) => {
                // Figure 9.27: bounded wait from Alert GoodCRC, never a Hard Reset
                // on timeout. Get_Status is also supported later from Ready.
                let result = self
                    .protocol_layer
                    .receive_message_type(
                        &[MessageType::Control(ControlMessageType::GetStatus)],
                        TimerType::SenderResponse,
                    )
                    .await;
                self.device_policy_manager.sink_alert_finished().await;
                match result {
                    Ok(_) => match self.device_policy_manager.status() {
                        Some(block) => State::GiveStatus(*power_source, block),
                        None => State::SendNotSupported(*power_source),
                    },
                    Err(ProtocolError::RxError(RxError::ReceiveTimeout)) => State::Ready(*power_source, false),
                    Err(error) => return Err(error.into()),
                }
            }
            State::GetPpsStatus(power_source) => {
                // PE_SNK_Get_PPS_Status (section 9.2.11.3.1): send Get_PPS_Status,
                // run SenderResponseTimer, return to Ready on PPS_Status or timeout.
                self.protocol_layer
                    .transmit_control_message(ControlMessageType::GetPpsStatus)
                    .await?;
                let result = self
                    .protocol_layer
                    .receive_message_type(
                        &[
                            MessageType::Extended(ExtendedMessageType::PpsStatus),
                            MessageType::Control(ControlMessageType::NotSupported),
                        ],
                        TimerType::SenderResponse,
                    )
                    .await;
                let outcome = match result {
                    Ok(message) => match message.payload {
                        Some(Payload::Extended(extended::Extended::PpsStatus(block))) => {
                            PpsStatusOutcome::Status(block)
                        }
                        None => PpsStatusOutcome::NotSupported,
                        // A PPS_Status without a valid 4-byte PPSSDB.
                        Some(_) => return Err(ProtocolError::UnexpectedMessage.into()),
                    },
                    Err(ProtocolError::RxError(RxError::ReceiveTimeout)) => PpsStatusOutcome::Timeout,
                    Err(e) => return Err(e.into()),
                };
                self.device_policy_manager.pps_status(outcome).await;
                State::Ready(*power_source, false)
            }
            State::Query(power_source, query) => {
                // Send the request, run SenderResponseTimer, return to Ready on the
                // answer or the timeout (sections 9.2.9.1.1, 9.2.14.1.1, 9.2.16.1.1).
                let answer = match query {
                    PartnerQuery::SourceCapabilitiesExtended => {
                        self.protocol_layer
                            .transmit_control_message(ControlMessageType::GetSourceCapExtended)
                            .await?;
                        MessageType::Extended(ExtendedMessageType::SourceCapabilitiesExtended)
                    }
                    PartnerQuery::Revision => {
                        self.protocol_layer
                            .transmit_control_message(ControlMessageType::GetRevision)
                            .await?;
                        MessageType::Data(DataMessageType::Revision)
                    }
                    PartnerQuery::ManufacturerInfo => {
                        self.protocol_layer.transmit_get_manufacturer_info().await?;
                        MessageType::Extended(ExtendedMessageType::ManufacturerInfo)
                    }
                    PartnerQuery::SourceInfo => {
                        self.protocol_layer
                            .transmit_control_message(ControlMessageType::GetSourceInfo)
                            .await?;
                        MessageType::Data(DataMessageType::SourceInfo)
                    }
                    PartnerQuery::Status => {
                        self.protocol_layer
                            .transmit_control_message(ControlMessageType::GetStatus)
                            .await?;
                        MessageType::Extended(ExtendedMessageType::Status)
                    }
                    PartnerQuery::BatteryCapabilities(reference) => {
                        self.protocol_layer.transmit_get_battery_cap(*reference).await?;
                        MessageType::Extended(ExtendedMessageType::BatteryCapabilities)
                    }
                    PartnerQuery::BatteryStatus(reference) => {
                        self.protocol_layer.transmit_get_battery_status(*reference).await?;
                        MessageType::Data(DataMessageType::BatteryStatus)
                    }
                };
                let result = self
                    .protocol_layer
                    .receive_message_type(
                        &[answer, MessageType::Control(ControlMessageType::NotSupported)],
                        TimerType::SenderResponse,
                    )
                    .await;
                let outcome = match result {
                    Ok(message) => match message.payload {
                        None => QueryOutcome::NotSupported,
                        Some(Payload::Extended(extended::Extended::SourceCapabilitiesExtended(block))) => {
                            QueryOutcome::Answer(heapless::Vec::from_slice(&block).expect("25 <= 26"))
                        }
                        Some(Payload::Extended(extended::Extended::ManufacturerInfo(block))) => {
                            QueryOutcome::Answer(block)
                        }
                        Some(Payload::Data(Data::Revision(rmdo))) => {
                            QueryOutcome::Answer(heapless::Vec::from_slice(&rmdo.to_le_bytes()).expect("4 <= 26"))
                        }
                        Some(Payload::Extended(extended::Extended::Status(block))) => {
                            QueryOutcome::Answer(heapless::Vec::from_slice(&block).expect("7 <= 26"))
                        }
                        Some(Payload::Extended(extended::Extended::BatteryCapabilities(block))) => {
                            QueryOutcome::Answer(heapless::Vec::from_slice(&block).expect("9 <= 26"))
                        }
                        Some(Payload::Data(Data::BatteryStatus(bsdo))) => {
                            QueryOutcome::Answer(heapless::Vec::from_slice(&bsdo.to_le_bytes()).expect("4 <= 26"))
                        }
                        Some(Payload::Data(Data::SourceInfo(objects))) => {
                            let mut block = heapless::Vec::new();
                            for object in objects {
                                block.extend_from_slice(&object.to_le_bytes()).expect("8 <= 26");
                            }
                            QueryOutcome::Answer(block)
                        }
                        // An answer without its valid Data Block.
                        Some(_) => return Err(ProtocolError::UnexpectedMessage.into()),
                    },
                    Err(ProtocolError::RxError(RxError::ReceiveTimeout)) => QueryOutcome::Timeout,
                    Err(e) => return Err(e.into()),
                };
                let (power_source, query) = (*power_source, *query);
                self.device_policy_manager.query(query, outcome).await;
                State::Ready(power_source, false)
            }
            State::SendNotSupported(power_source) => {
                // PD2 V1.3 section 6.8.1 uses Reject; PD3 section 6.3.16
                // introduces Not_Supported.
                let response = match self
                    .protocol_layer
                    .header()
                    .spec_revision()
                    .expect("validated protocol header")
                {
                    SpecificationRevision::R1_0 | SpecificationRevision::R2_0 => ControlMessageType::Reject,
                    SpecificationRevision::R3_X => ControlMessageType::NotSupported,
                };
                self.protocol_layer.transmit_control_message(response).await?;

                State::Ready(*power_source, false)
            }
            State::BistCarrierMode(power_source) => {
                // The driver runs the carrier and stops it within tBISTContMode
                // (BISTContModeTimer, section 7.31.6.2).
                if self.protocol_layer.transmit_bist_carrier().await? {
                    // PE_SNK_Transition_to_default (section 9.2.4.9) without Hard
                    // Reset: protocol reset, no contract, then PE_SNK_Startup. VBUS
                    // stays at vSafe5V (the only level BIST runs at), so the
                    // Discovery VBUS wait is already satisfied.
                    self.contract = Default::default();
                    self.established_request = None;
                    self.wait_request = None;
                    self.protocol_layer.reset();
                    self.mode = Mode::Spr;
                    self.source_capabilities = None;
                    State::WaitForCapabilities
                } else {
                    State::Ready(*power_source, false)
                }
            }
            State::BistTestData => {
                // Section 6.4.3.1: received Messages get GoodCRC (protocol layer)
                // and are not acted on; no timers, DPM events or responses. Only
                // Hard Reset (an error from the protocol layer) leaves this state.
                let _ = self.protocol_layer.receive_message().await?;
                State::BistTestData
            }
            State::RejectSwap(power_source) => {
                self.protocol_layer
                    .transmit_control_message(ControlMessageType::Reject)
                    .await?;
                State::Ready(*power_source, false)
            }
            State::SendSoftReset => {
                self.protocol_layer.reset();

                self.protocol_layer
                    .transmit_control_message(ControlMessageType::SoftReset)
                    .await?;

                self.protocol_layer
                    .receive_message_type(
                        &[MessageType::Control(ControlMessageType::Accept)],
                        TimerType::SenderResponse,
                    )
                    .await?;

                State::WaitForCapabilities
            }
            State::SoftReset => {
                // Section 7.7 / 9.2.5.2.2: reset SOP counters before Accept.
                // Do not reset again afterward: Accept consumes TX ID0.
                self.protocol_layer.reset();
                self.wait_request = None;
                self.protocol_layer
                    .transmit_control_message(ControlMessageType::Accept)
                    .await?;

                State::WaitForCapabilities
            }
            State::HardReset => {
                // Per USB PD Spec R3.2 Section 8.3.3.3.8 (PE_SNK_Hard_Reset):
                // Entry conditions:
                // - PSTransitionTimer timeout (when HardResetCounter <= nHardResetCount)
                // - Hard reset request from Device Policy Manager
                // - EPR mode and EPR_Source_Capabilities message with EPR PDO in pos. 1..7
                // - Source_Capabilities message not requested by Get_Source_Cap
                // - SinkWaitCapTimer timeout (when HardResetCounter <= nHardResetCount)
                //
                // On entry: Request Hard Reset Signaling AND increment HardResetCounter

                // Increment counter first - returns Err when counter > nHardResetCount.
                // Per spec 8.3.3.3.8: If HardResetCounter > nHardResetCount (> 2),
                // the Sink shall assume that the Source is non-responsive.
                // With counter max_value = 3, we allow 3 hard reset attempts (counter 1, 2, 3)
                // before wrap returns Err.
                if self.hard_reset_counter.increment().is_err() {
                    return Err(Error::PortPartnerUnresponsive);
                }

                // Transmit Hard Reset Signaling
                self.protocol_layer.hard_reset().await?;

                State::TransitionToDefault
            }
            State::TransitionToDefault => {
                // Per USB PD Spec R3.2 Section 8.3.3.3.9 (PE_SNK_Transition_to_default):
                // This state is entered when:
                // - Hard Reset Signaling is detected (received or transmitted)
                // - From PE_SNK_Hard_Reset after hard reset is complete
                //
                // On entry:
                // - Indicate to DPM that Sink shall transition to default
                // - Request reset of local hardware
                // - Request DPM that Port Data Role is set to UFP
                //
                // Transition to PE_SNK_Startup when:
                // - DPM indicates Sink has reached default level

                // Notify DPM about hard reset (DPM should transition to default power level)
                self.device_policy_manager.hard_reset().await;

                // Reset protocol layer (per spec 6.8.3: "Protocol Layers shall be reset as for Soft Reset")
                self.protocol_layer.reset();

                // Reset EPR mode (per spec 6.8.3.2: "Hard Reset shall cause EPR Mode to be exited")
                self.mode = Mode::Spr;

                // Reset contract to default
                self.contract = Contract::Safe5V;

                // Clear cached source capabilities
                self.source_capabilities = None;

                State::Startup
            }
            State::GiveSinkCap(response_mode, power_source) => {
                // Per USB PD Spec R3.2 Section 8.3.3.3.10:
                // - Send Sink_Capabilities when Get_Sink_Cap was received
                // - Send EPR_Sink_Capabilities when EPR_Get_Sink_Cap was received
                let sink_caps = self.device_policy_manager.sink_capabilities();
                match response_mode {
                    Mode::Spr => {
                        self.protocol_layer.transmit_sink_capabilities(sink_caps).await?;
                    }
                    Mode::Epr => {
                        self.protocol_layer
                            .transmit_epr_sink_capabilities(self.device_policy_manager.epr_sink_capabilities())
                            .await?;
                    }
                }

                State::Ready(*power_source, false)
            }
            State::GetSourceCap(requested_mode, power_source) => {
                // Per USB PD Spec R3.2 Section 8.3.3.3.12 (PE_SNK_Get_Source_Cap):
                // - Send Get_Source_Cap (SPR) or EPR_Get_Source_Cap (EPR)
                // - Start SenderResponseTimer
                // - On timeout or mode mismatch → Ready
                // - On matching capabilities received → EvaluateCapabilities
                //
                // Set flag before sending to track that we requested source capabilities.
                // Per spec 8.3.3.3.8, in EPR mode, receiving an unrequested
                // Source_Capabilities message triggers a Hard Reset.
                self.get_source_cap_pending = true;

                match requested_mode {
                    Mode::Spr => {
                        self.protocol_layer
                            .transmit_control_message(ControlMessageType::GetSourceCap)
                            .await?;
                    }
                    Mode::Epr => {
                        self.protocol_layer
                            .transmit_extended_control_message(
                                crate::protocol_layer::message::extended::extended_control::ExtendedControlMessageType::EprGetSourceCap,
                            )
                            .await?;
                    }
                };

                // Per spec 8.3.3.3.12: Use SenderResponseTimer (not SinkWaitCap)
                let result = self
                    .protocol_layer
                    .receive_message_type(
                        &[
                            MessageType::Data(DataMessageType::SourceCapabilities),
                            MessageType::Extended(ExtendedMessageType::EprSourceCapabilities),
                        ],
                        TimerType::SenderResponse,
                    )
                    .await;

                self.get_source_cap_pending = false;

                // Per spec 8.3.3.3.12: On timeout, inform DPM and transition to Ready
                let message = match result {
                    Ok(msg) => msg,
                    Err(ProtocolError::RxError(RxError::ReceiveTimeout)) => {
                        // Inform DPM of timeout (no capabilities received)
                        warn!("Get_Source_Cap timeout, returning to Ready");
                        self.state = State::Ready(*power_source, false);
                        return Ok(());
                    }
                    Err(e) => return Err(e.into()),
                };

                // Per spec 8.3.3.3.12:
                // - In SPR mode + SPR caps requested + Source_Capabilities received → EvaluateCapabilities
                // - In EPR mode + EPR caps requested + EPR_Source_Capabilities received → EvaluateCapabilities
                // - Mode mismatch (e.g., EPR mode but SPR caps requested) → Ready
                let received_spr = matches!(
                    message.header.message_type(),
                    MessageType::Data(DataMessageType::SourceCapabilities)
                );
                let received_epr = matches!(
                    message.header.message_type(),
                    MessageType::Extended(ExtendedMessageType::EprSourceCapabilities)
                );

                let mode_matches = (*requested_mode == Mode::Spr && self.mode == Mode::Spr && received_spr)
                    || (*requested_mode == Mode::Epr && self.mode == Mode::Epr && received_epr);

                // Extract capabilities from the message
                let capabilities = match message.payload {
                    Some(Payload::Data(Data::SourceCapabilities(caps))) => caps,
                    Some(Payload::Extended(extended::Extended::EprSourceCapabilities(pdos))) => {
                        SourceCapabilities(pdos)
                    }
                    _ => {
                        self.state = State::SendNotSupported(*power_source);
                        return Ok(());
                    }
                };

                self.device_policy_manager.inform(&capabilities).await;

                if mode_matches {
                    State::EvaluateCapabilities(capabilities)
                } else {
                    State::Ready(*power_source, false)
                }
            }
            State::EprModeEntry(power_source, operational_pdp) => {
                // Request entry into EPR mode.
                // Per spec 8.3.3.26.2.1 (PE_SNK_Send_EPR_Mode_Entry), sink sends EPR_Mode (Enter)
                // and starts SenderResponseTimer and SinkEPREnterTimer.
                //
                // Per spec 6.4.10, the Data field shall be set to the EPR Sink Operational PDP.
                //
                let pdp_watts: u8 = operational_pdp.get::<watt>() as u8;
                self.protocol_layer.transmit_epr_mode(Action::Enter, pdp_watts).await?;

                // Section 6.6.21.1: one continuous timer, anchored at Enter's
                // GoodCRC, spans both Ack and Succeeded.
                let exchange = async {
                    let ack = self
                        .protocol_layer
                        .receive_message_type(
                            &[MessageType::Data(DataMessageType::EprMode)],
                            TimerType::SenderResponse,
                        )
                        .await?;
                    let Some(Payload::Data(Data::EprMode(ack))) = ack.payload else {
                        return Err(ProtocolError::UnexpectedMessage);
                    };
                    if ack.action() != Action::EnterAcknowledged {
                        return if ack.action() == Action::EnterFailed {
                            Ok(ack)
                        } else {
                            Err(ProtocolError::UnexpectedMessage)
                        };
                    }
                    let succeeded = self
                        .protocol_layer
                        .receive_message_type(&[MessageType::Data(DataMessageType::EprMode)], TimerType::SinkEPREnter)
                        .await?;
                    match succeeded.payload {
                        Some(Payload::Data(Data::EprMode(mode))) => Ok(mode),
                        _ => Err(ProtocolError::UnexpectedMessage),
                    }
                };
                match select(TimerType::get_timer::<TIMER>(TimerType::SinkEPREnter), exchange).await {
                    Either::Second(Ok(mode)) if mode.action() == Action::EnterSucceeded => {
                        self.mode = Mode::Epr;
                        self.device_policy_manager.epr_mode_changed(true).await;
                        State::EprWaitForCapabilities(*power_source)
                    }
                    Either::Second(Ok(mode)) if mode.action() == Action::EnterFailed => {
                        self.device_policy_manager
                            .epr_mode_entry_failed(epr_mode::DataEnterFailed::from(mode.data()))
                            .await;
                        State::SendSoftReset
                    }
                    Either::Second(Err(error))
                        if !matches!(
                            error,
                            ProtocolError::UnexpectedMessage | ProtocolError::RxError(RxError::ReceiveTimeout)
                        ) =>
                    {
                        return Err(error.into());
                    }
                    _ => {
                        self.device_policy_manager.epr_mode_entry_aborted(false).await;
                        State::SendSoftReset
                    }
                }
            }
            State::EprWaitForCapabilities(_power_source) => {
                // After successful EPR mode entry, source automatically sends EPR_Source_Capabilities.
                // This may be a chunked extended message that requires assembly.
                // Wait for the capabilities and evaluate them.
                let message = self
                    .protocol_layer
                    .receive_message_type(
                        &[MessageType::Extended(ExtendedMessageType::EprSourceCapabilities)],
                        TimerType::SinkWaitCap,
                    )
                    .await?;

                match message.payload {
                    Some(Payload::Extended(extended::Extended::EprSourceCapabilities(pdos))) => {
                        let capabilities = SourceCapabilities(pdos);
                        if capabilities.has_epr_pdo_in_spr_positions() {
                            State::HardReset
                        } else {
                            State::EvaluateCapabilities(capabilities)
                        }
                    }
                    _ => {
                        error!("Expected source capabilities after EPR mode entry");
                        State::HardReset
                    }
                }
            }
            State::EprSendExit(_) => {
                // Inform partner we are exiting EPR.
                self.protocol_layer.transmit_epr_mode(Action::Exit, 0).await?;
                self.mode = Mode::Spr;
                self.device_policy_manager.epr_mode_changed(false).await;
                State::WaitForCapabilities
            }
            State::EprExitReceived(power_source) => {
                // Per USB PD Spec R3.2 Section 8.3.3.26.4.2 (PE_SNK_EPR_Mode_Exit_Received):
                // - If in an Explicit Contract with an SPR (A)PDO → WaitForCapabilities
                // - If NOT in an Explicit Contract with an SPR (A)PDO → HardReset
                //
                // SPR PDOs are in object positions 1-7, EPR PDOs are in positions 8+.
                // In EPR mode, requests use EprRequest which contains the RDO with object position.
                self.mode = Mode::Spr;

                let is_epr_pdo_contract = match power_source {
                    PowerSource::EprRequest(epr) => {
                        // Extract object position from RDO (bits 28-31)
                        epr.object_position() >= 8
                    }
                    // Non-EprRequest variants are only used in SPR mode, so always SPR PDOs
                    _ => false,
                };

                if is_epr_pdo_contract {
                    State::HardReset
                } else {
                    self.device_policy_manager.epr_mode_changed(false).await;
                    State::WaitForCapabilities
                }
            }
            State::EprKeepAlive(power_source) => {
                // Per spec 8.3.3.3.11 (PE_SNK_EPR_Keep_Alive):
                // - Entry: Send EPR_KeepAlive message, start SenderResponseTimer
                // - On EPR_KeepAlive_Ack: transition to Ready (which restarts SinkEPRKeepAliveTimer)
                // - On timeout: transition to HardReset
                self.protocol_layer
                    .transmit_extended_control_message(
                        crate::protocol_layer::message::extended::extended_control::ExtendedControlMessageType::EprKeepAlive,
                    )
                    .await?;
                match self
                    .protocol_layer
                    .receive_message_type(
                        &[MessageType::Extended(ExtendedMessageType::ExtendedControl)],
                        TimerType::SenderResponse,
                    )
                    .await
                {
                    Ok(message) => {
                        if let Some(Payload::Extended(extended::Extended::ExtendedControl(control))) = message.payload {
                            if control.try_message_type()
                                == Ok(crate::protocol_layer::message::extended::extended_control::ExtendedControlMessageType::EprKeepAliveAck)
                            {
                                self.mode = Mode::Epr;
                                State::Ready(*power_source, false)
                            } else {
                                State::SendSoftReset
                            }
                        } else {
                            State::SendSoftReset
                        }
                    }
                    Err(ProtocolError::RxError(RxError::HardReset)) => State::TransitionToDefault,
                    Err(_) => State::HardReset,
                }
            }
        };

        self.state = new_state;

        Ok(())
    }
}
