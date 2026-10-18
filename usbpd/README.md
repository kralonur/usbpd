# Library for USB PD

Modeled after the Universal Serial Bus Power Delivery Specification: USB PD R3.2 v1.1 (2024/10).

The library implements:

- A policy engine for each supported mode,
- the protocol layer, and
- the `DevicePolicyManager` trait, which allows a device user application to talk to the policy engine, and control it.

The library depends on the crate [usbpd-traits](https://crates.io/crates/usbpd-traits), which provides traits for supporting
USB PD PHYs.

## Currently supported modes

- SPR and EPR sink mode
- Includes many helpers for requesting power sources, for example
  - the highest fixed voltage,
  - a specific fixed voltage and/or current, and
  - augmented PDOs (PPS or AVS).

## Partner discovery

The sink DPM can explicitly request `PartnerQuery::SinkCapabilities` or
`PartnerQuery::DiscoverIdentity` from PD3 Ready without changing power or data
role. Replies retain up to seven raw objects; identity includes its VDM header
and only the VDOs actually received. NAK, BUSY, Not_Supported, deferral and
timeout are separate outcomes, and the common SVDM version is retained until
detach or Hard Reset. Identity uses VDMResponseTimer anchored to the request's GoodCRC.
Callers still own partner-role eligibility, voltage limits and physical safety;
these queries do not provide a source power supply or role-swap support.

## Usage

Find usage examples for different platforms and modes in the [GitHub repository](https://github.com/elagil/usbpd/tree/main/examples).

# Credit

Inherits message parsing code from [usb-pd-rs](https://github.com/fmckeogh/usb-pd-rs).
