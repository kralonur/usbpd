//! Timers that are used by the protocol layer and policy engine.

/// The timer trait to implement by the user application.
pub trait Timer {
    /// Expire after the specified number of milliseconds.
    fn after_millis(milliseconds: u64) -> impl Future<Output = ()>;

    /// Expire `timer_type` after its specified `milliseconds`. Override to treat
    /// particular timers specially, matched by type rather than by duration.
    fn after_timer(timer_type: TimerType, milliseconds: u64) -> impl Future<Output = ()> {
        let _ = timer_type;
        Self::after_millis(milliseconds)
    }
}

use core::future::Future;

/// Types of timers that are used for timeouts.
#[allow(missing_docs)]
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TimerType {
    BISTContMode,
    ChunkingNotSupported,
    ChunkSenderRequest,
    ChunkSenderResponse,
    CRCReceive,
    DataResetFail,
    DataResetFailUFP,
    DiscoverIdentity,
    HardResetComplete,
    NoResponse,
    PSHardReset,
    PSSourceOffSpr,
    PSSourceOffEpr,
    PSSourceOnSpr,
    PSTransitionSpr,
    PSTransitionEpr,
    SenderResponse,
    SinkEPREnter,
    SinkEPRKeepAlive,
    SinkPPSPeriodic,
    SinkRequest,
    SinkWaitCap,
    SourceCapability,
    SourceEPRKeepAlive,
    SourcePPSComm,
    SinkTx,
    SwapSourceStart,
    VCONNDischarge,
    VCONNOn,
    VDMModeEntry,
    VDMModeExit,
    VDMResponse,
}

impl TimerType {
    /// Create a new timer for a given type.
    ///
    /// Times out after a duration that is given by the USB PD specification.
    pub fn get_timer<TIMER: Timer>(timer_type: TimerType) -> impl Future<Output = ()> {
        let milliseconds = match timer_type {
            TimerType::BISTContMode => 45,
            TimerType::ChunkingNotSupported => 45,
            TimerType::ChunkSenderRequest => 27,
            TimerType::ChunkSenderResponse => 27,
            TimerType::CRCReceive => 1,
            TimerType::DataResetFail => 350,
            TimerType::DataResetFailUFP => 500,
            TimerType::DiscoverIdentity => 45,
            TimerType::HardResetComplete => 5,
            TimerType::NoResponse => 5000,
            TimerType::PSHardReset => 30,
            TimerType::PSSourceOffSpr => 835,
            TimerType::PSSourceOffEpr => 1260,
            TimerType::PSSourceOnSpr => 435,
            TimerType::PSTransitionSpr => 500,
            TimerType::PSTransitionEpr => 925,
            TimerType::SenderResponse => usbpd_traits::SENDER_RESPONSE_MS,
            TimerType::SinkEPREnter => 500,
            TimerType::SinkEPRKeepAlive => 375,
            TimerType::SinkPPSPeriodic => 5000, // Max. 10 s
            TimerType::SinkRequest => 100,
            TimerType::SinkWaitCap => 465,
            TimerType::SourceCapability => 150,
            TimerType::SourceEPRKeepAlive => 875,
            TimerType::SourcePPSComm => 13500,
            TimerType::SinkTx => 18,
            TimerType::SwapSourceStart => 20,
            TimerType::VCONNDischarge => 200,
            TimerType::VCONNOn => 50,
            TimerType::VDMModeEntry => 45,
            TimerType::VDMModeExit => 45,
            TimerType::VDMResponse => usbpd_traits::VDM_RESPONSE_MS,
        };
        TIMER::after_timer(timer_type, milliseconds)
    }
}
