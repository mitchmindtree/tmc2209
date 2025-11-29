//! Declaration of the TMC2209 registers and their implementations.
//!
//! Please refer to the TMC2209 datasheet for information on what each of these registers and their
//! fields mean. The register map is described under section 5 of the datasheet.
//!
//! <https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2209_datasheet_rev1.08.pdf>

#![allow(non_camel_case_types)]

use crate::data::{MicroStepResolution, StandstillMode};

// Register Traits
// --------------------------------------------------------

/// Implemented for all register types.
///
/// NOTE: This should not be implemented for custom types. If the user attempts to request a custom
/// register type from the register `Map`, the method call will hang indefinitely.
pub trait Register: Into<State> {
    const ADDRESS: Address;
}

/// Implemented for all registers that can be read from.
pub trait ReadableRegister: Register + From<u32> {}

/// Implemented for all registers that can be written to.
pub trait WritableRegister: Register + Into<u32> {}

/// An error that might occur in the case that an address could not be parsed.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
pub struct UnknownAddress;

/// An error indicating an unexpected `State`.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
pub struct UnexpectedAddress;

// Register Declarations
// --------------------------------------------------------

bitfield! {
    /// Global configuration flags
    #[derive(Clone, Copy, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct GCONF(u32);
    impl Debug;
    u16;
    /// Analog current scaling.
    ///
    /// If set to true, the driver uses the voltage supplied to `VREF` as current reference,
    /// otherwise an internal reference derived from `5VOUT` is used.
    pub i_scale_analog, set_i_scale_analog: 0;
    /// Whether to use the internal sense resistors for current measurement.
    ///
    /// The chip needs to measure the motor current, for this a sense resistor is used.
    /// If this is true, the chip internal sense resistor will be used, otherwise the external
    /// sense resistor connected to the `BRA` and `BRB` pads will be used.
    pub internal_rsense, set_internal_rsense: 1;
    /// Whether to enable spread cycle or stealth chop mode.
    ///
    /// If set to false, StealthChop PWM mode will be enabled (depending on velocity thresholds).
    /// Initially switch from off to on state while in stand still only.
    ///
    /// If set to true, SpreadCycle mode will be enabled.
    ///
    /// A high level on the `SPREAD` pin will invert this flag to switch between both chopper modes.
    pub en_spread_cycle, set_en_spread_cycle: 2;
    /// Whether to reverse the motor direction.
    ///
    /// A value of true will invert the direction of the motor.
    pub shaft, set_shaft: 3;
    /// What the index output pin shows.
    ///
    /// If set to true, the index pin outputs overtemperature prewarning flag (otpw) instead of
    /// the first microstep position of sequencer.
    pub index_otpw, set_index_otpw: 4;
    /// What the index output pin shows.
    ///
    /// If set to false, the pin will output as selected by [`GCONF::index_otpw`].
    /// If set to true, the pin shows step pulses from internal pulse generator (toggle upon each step).
    pub index_step, set_index_step: 5;
    /// What the `PDN_UART` pin does.
    ///
    /// If set to true, the `PDN_UART` input function is disabled, this bit must be set when the UART interface is used.
    /// If set to false, the `PDN_UART` pin controls standstill current reduction.
    pub pdn_disable, set_pdn_disable: 6;
    /// How the microstep resolution is selected.
    ///
    /// If true, the microstep resolution is set by the [`CHOPCONF::set_mres`] register, otherwise the
    /// microstep resolution is selected through the `MS1` and `MS2` pins.
    pub mstep_reg_select, set_mstep_reg_select: 7;
    /// Whether to filter step pulses.
    ///
    /// If set to true, software pulse generator optimization will be enabled
    /// when fullstep frequency > 750Hz (roughly). [`TSTEP`] shows filtered step time values when active.
    pub multistep_filt, set_multistep_filt: 8;
    /// Whether to enable test mode.
    ///
    /// If set to true, it will enable analog test output on ENN pin (pull down resistor off), ENN treated as enabled.
    /// `IHOLD[1..0]` selects the function of `DCO`: 0..2: T120, DAC, VDDH.
    ///
    /// Attention: Not for user, set to 0 for normal operation!
    pub test_mode, set_test_mode: 9;
}

// The GSTAT register is special, it is marked as R+WC, which is a status register that can be
// written to with 1 bit to clear a flag. For example by setting the drv_err bit to 1, it will clear
// the drv_err flag on write.
bitfield! {
    /// Global status flags register.
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct GSTAT(u32);
    impl Debug;
    u8;
    /// Indicates that the IC has been reset since the last read access to GSTAT.
    /// All regisers have been cleared to reset values.
    pub reset, _: 0;
    /// Indicates, that the driver has been shut down due to overtemperature or
    /// short circuit detection since the last read access.
    /// Read [`DRV_STATUS`] for details.
    ///
    /// The flag can only be cleared when all error conditions are cleared.
    /// To clear the flag, call `set_drv_err(true)`, and then write the register
    /// to the driver.
    pub drv_err, set_drv_err: 1;
    /// Indicates an undervoltage on the charge pump.
    /// The driver is disabled in this case.
    ///
    /// This flag is not latched and thus does not need to be cleared.
    pub uv_cp, _: 2;
}

bitfield! {
    /// Interface transmission counter.
    ///
    /// This register becomes incremented with each successful UART interface write access.
    /// Read out to check the serial transmission for lost data. Read accesses do not change
    /// the content.
    ///
    /// The counter wraps around from 255 to 0, it should be a value between 0 and 255 (inclusive).
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct IFCNT(u32);
    impl Debug;
    u8;
    pub get, set: 7, 0;
}

bitfield! {
    /// SENDDELAY for read access (time until reply is sent):
    /// - 0, 1: 8 bit times (Attention: Don’t use in multi-slave)
    /// - 2, 3: 3*8 bit times
    /// - 4, 5: 5*8 bit times
    /// - 6, 7: 7*8 bit times
    /// - 8, 9: 9*8 bit times
    /// - 10, 11: 11*8 bit times
    /// - 12, 13: 13*8 bit times
    /// - 14, 15: 15*8 bit times
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct SLAVECONF(u32);
    impl Debug;
    u8;
    pub send_delay, set_send_delay: 11, 8;
}

bitfield! {
    /// Write access programs OTP memory (one bit at a time),
    /// read access refreshes read data from OTP after a write.
    ///
    /// All OTP memory bits are cleared to 0 by default.
    /// Programming only can set bits to 1, clearing bits is **not** possible.
    ///
    /// <div class="warning">OTP = One Time Programmable, you can only program the memory once.</div>
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct OTP_PROG(u32);
    impl Debug;
    u16;
    /// Selection of OTP bit to be programmed to the selected
    /// byte location (n=0..7: programs bit n to a logic 1)
    pub otp_bit, set_otp_bit: 2, 0;
    /// Selection of OTP programming location (0, 1 or 2)
    pub otp_byte, set_otp_byte: 5, 4;
    /// Set to 0xbd to enable programming.
    ///
    /// A programming time of minimum 10ms per bit is recommended (check by reading [`OTP_READ`]).
    pub otp_magic, set_otp_magic: 15, 8;
}

bitfield! {
    /// Read OTP memory.
    ///
    /// The OTP memory holds power up defaults for certain registers. All OTP memory
    /// bits are cleared to 0 by default.
    ///
    /// For programming the OTP, see [`OTP_PROG`].
    ///
    /// Factory tuning of the clock frequency affects opt0.0 to otp0.4 (= [`OTP_READ::otp_fclktrim`]).
    /// The state of these bits therefore may differ between individual ICs.
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct OTP_READ(u32);
    impl Debug;
    u8;
    /// Reset default for FCLKTRIM
    /// - 0: lowest frequency setting
    /// - 31: highest frequency setting
    ///
    /// <div class="warning">
    /// <b>Attention</b>: This value is pre-programmed by factory clock
    /// trimming to the default clock frequency of 12MHz and
    /// differs between individual ICs! It should not be altered.
    /// </div>
    pub otp_fclktrim, _: 4, 0;
    /// Reset default for OTTRIM:
    /// - 0: `OTTRIM=0b00` (143°C)
    /// - 1: `OTTRIM=0b01` (150°C)
    /// (internal power stage temperature about 10°C above the sensor temperature limit)
    pub otp_ottrim, _: 5;
    /// Reset default for GCONF.internal_Rsense
    /// - 0: External sense resistors
    /// - 1: Internal sense resistors
    pub otp_internal_rsense, _: 6;
    /// Reset default for TBL:
    ///
    /// - 0: `TBL=0b10`
    /// - 1: `TBL=0b01`
    pub otp_tbl, _: 7;
    /// # StealthChop enabled by default:
    ///
    /// Reset default for [`PWMCONF::pwm_grad`] as defined by (0..15):
    /// | Value | PWM_GRAD | Value | PWM_GRAD |
    /// |-------|----------|-------|----------|
    /// |  0    |       14 |     8 |       40 |
    /// |  1    |       16 |     9 |       46 |
    /// |  2    |       18 |    10 |       52 |
    /// |  3    |       21 |    11 |       59 |
    /// |  4    |       24 |    12 |       67 |
    /// |  5    |       27 |    13 |       77 |
    /// |  6    |       31 |    14 |       88 |
    /// |  7    |       35 |    15 |      100 |
    ///
    /// # SpreadCycle enabled by default:
    ///
    /// This value will set the default bits `0` to `3` in the [`CHOPCONF`] register (`TOFF`).
    pub otp_pwm_grad, _: 11, 8;
    /// # StealthChop enabled by default:
    ///
    /// If `true`, `pwm_autograd` is enabled, otherwise disabled.
    ///
    /// # SpreadCycle enabled by default:
    ///
    /// This value will set the default value for 4th bit in the `CHOPCONF` register (hstrt0) (pwm_autograd=1).
    pub otp_pwm_autograd, _: 12;
    /// # StealthChop enabled by default:
    ///
    /// Reset default for TPWM_THRS as defined by (0..7):
    /// | Value | TPWM_THRS |
    /// |-------|-----------|
    /// | 0     |         0 |
    /// | 1     |       200 |
    /// | 2     |       300 |
    /// | 3     |       400 |
    /// | 4     |       500 |
    /// | 5     |       800 |
    /// | 6     |      1200 |
    /// | 7     |      4000 |
    ///
    /// # SpreadCycle enabled by default:
    ///
    /// This value will set the default bits 5 to 7 in the `CHOPCONF` register (hstrt1, hstrt2 and hend0).
    pub otp_tpwmthrs, _: 15, 13;
    /// # StealthChop enabled by default:
    ///
    /// This field will set the `PWM_OFS` to `36` (if false) or `0` (if true).
    ///
    /// # SpreadCycle enabled by default:
    ///
    /// This value will set the default value for 8th bit in the `CHOPCONF` register (hend1).
    pub otp_pwm_ofs, _: 16;
    /// Reset default for PWM_REG:
    /// - 0: `PWM_REG=0b1000`: max. 4 increments / cycle
    /// - 1: `PWM_REG=0b0010`: max. 1 increment / cycle
    pub otp_pwm_reg, _: 17;
    /// Reset default for PWM_FREQ:
    /// - 0: `PWM_FREQ=0b01=2/683`
    /// - 1: `PWM_FREQ=0b10=2/512`
    pub otp_pwm_freq, _: 18;
    /// Reset default for IHOLDDELAY
    /// - `0b00`: IHOLDDELAY= 1
    /// - `0b01`: IHOLDDELAY= 2
    /// - `0b10`: IHOLDDELAY= 4
    /// - `0b11`: IHOLDDELAY= 8
    pub otp_ihold_delay, _: 20, 19;
    /// Reset default for standstill current IHOLD (used only if
    /// current reduction enabled, e.g. pin PDN_UART low).
    /// - `0b00`: IHOLD= 16 (53% of IRUN)
    /// - `0b01`: IHOLD= 2 ( 9% of IRUN)
    /// - `0b10`: IHOLD= 8 (28% of IRUN)
    /// - `0b11`: IHOLD= 24 (78% of IRUN)
    ///
    /// (Reset default for run current IRUN=31)
    pub otp_ihold, _: 22, 21;
    /// This flag determines if the driver defaults to SpreadCycle (true) or StealthChop (false).
    ///
    /// If StealthChop is enabled by default, it will use these settings for SpreadCycle:
    /// - `HEND=0`
    /// - `HSTART=5`
    /// - `TOFF=3`
    ///
    /// If SpreadCycle is enabled by default, it will use these settings for StealthChop:
    /// - `PWM_GRAD=0`
    /// - `TPWM_THRS=0`
    /// - `PWM_OFS=36`
    /// - `pwm_autograd=1`
    pub otp_en_spread_cycle, _: 23;
}

bitfield! {
    /// Reads the state of all input pins available.
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct IOIN(u32);
    impl Debug;
    u16;
    /// Enable pin state.
    pub enn, _: 0;
    /// Microstep 1 selection pin state.
    pub ms1, _: 2;
    /// Microstep 2 selection pin state.
    pub ms2, _: 3;
    /// Diag pin state.
    pub diag, _: 4;
    /// PDN UART pin state.
    pub pdn_uart, _: 6;
    /// Step pin state.
    pub step, _: 7;
    /// Spread cycle pin state.
    pub spread_en, _: 8;
    /// Direction pin state.
    pub dir, _: 9;
    u8;
    /// Version number of the IC.
    ///
    /// `0x21` is the first version of the IC, identical numbers mean full digital compatibility.
    pub version, _: 31, 24;
}

bitfield! {
    /// Factory configuration
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct FACTORY_CONF(u32);
    impl Debug;
    u8;
    /// FCLKTRIM (Reset default: OTP)
    ///
    /// 0...31: Lowest to highest clock frequency. Check at charge pump output.
    /// The frequency span is not guaranteed, but it is tested, that tuning to 12MHz
    /// internal clock is possible. The devices come preset to 12MHz clock frequency
    /// by OTP programming.
    pub fclktrim, set_fclktrim: 4, 0;
    /// OTTRIM (Default: OTP)
    ///
    /// | Value  | OT    | OTPW  |
    /// |--------|-------|-------|
    /// | `0b00` | 143°C | 120°C |
    /// | `0b01` | 150°C | 120°C |
    /// | `0b10` | 150°C | 143°C |
    /// | `0b11` | 157°C | 143°C |
    pub ottrim, set_ottrim: 9, 8;
}

bitfield! {
    /// Driver current control
    #[derive(Clone, Copy, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct IHOLD_IRUN(u32);
    impl Debug;
    u8;
    /// Standstill current.
    ///
    /// This is the current used when the motor is not moving, but still powered.
    ///
    /// Valid values are 0=1/32, ..., 31=32/32.
    ///
    /// A value of 0 allows to choose freewheeling or coild
    /// short circuit (passive braking) for motor stand still.
    ///
    /// Consider using [`rms_current_to_vsense_cs`](crate::rms_current_to_vsense_cs)
    /// to convert a current value to the appropriate register value.
    ///
    /// Reset default: OTP
    pub ihold, set_ihold: 4, 0;
    /// Motor run current.
    ///
    /// Consider using [`rms_current_to_vsense_cs`](crate::rms_current_to_vsense_cs)
    /// to convert a current value to the appropriate register value.
    ///
    /// It is recommended to choose sense resistors in a way, that normal
    /// IRUN is between 16 and 31 for the best microstep performance.
    ///
    /// Reset default: 31
    pub irun, set_irun: 12, 8;
    /// Controls the number of clock cycles for motor power down after standstill is detected
    /// ([`DRV_STATUS::stst`] = 1) and [`TPOWERDOWN`] has expired.
    ///
    /// The smooth transition avoids a motor jerk upon power down.
    ///
    /// - 0: Instant power down
    /// - 1..15: Delay per current reduction step in multiple of 2^18 clocks
    pub ihold_delay, set_ihold_delay: 19, 16;
}

bitfield! {
    /// Sets the delay time from stand still ([`DRV_STATUS::stst`]) detection to motor current power down.
    ///
    /// Time range is about 0 to 5.6 seconds. Setting 0 is no delay, 1 a minimum delay.
    /// Further increment is in discrete steps of 2^18 clock cycles (value * 2^18 * t_CLK).
    ///
    /// Attention: A minimum setting of 2 is required to allow automatic
    /// tuning of StealthChop `PWM_OFFS_AUTO`.
    #[derive(Clone, Copy, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct TPOWERDOWN(u32);
    impl Debug;
    u8;
    pub get, set: 7, 0;
}

bitfield! {
    /// Actual measured time between two 1/256 microsteps.
    ///
    /// This is derived from the step input frequency in units of 1/`f_CLK`.
    /// `f_CLK` is [`tmc2209::INTERNAL_CLOCK_HZ`] if internal clock is used.
    ///
    /// Measured value is `2.pow(20) - 1` in case of overflow or stand still.
    /// `TSTEP` always related to 1/256 step, independent of the actual [`CHOPCONF::mres`].
    ///
    /// The `TSTEP` related threshold uses a hysteresis of 1/16 of the compare value to compensate
    /// for jitter in the clock or the step frequency:
    /// `(Txxx*15/16)-1` is the lower compare value for each `TSTEP` based comparison.
    ///
    /// This means, that the lower switching velocity equals the calculated setting, but the upper
    /// switching velocity is higher as defined by the hysteresis setting.
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct TSTEP(u32);
    impl Debug;
    u32;
    /// Returns the measured time between two 1/256 microsteps.
    pub get, _: 19, 0;
}

bitfield! {
    /// Sets the upper velocity threshold for StealthChop voltage PWM mode.
    ///
    /// While `TSTEP >= TPWMTHRS` the driver operates in StealthChop mode (if configured).
    /// When the velocity exceeds this threshold (higher velocity, lower time between steps),
    /// the driver switches to SpreadCycle mode.
    ///
    /// If the value is set to 0, it is disabled and StealthChop is used for all velocities.
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct TPWMTHRS(u32);
    impl Debug;
    u32;
    pub get, set: 19, 0;
}

bitfield! {
    /// VACTUAL allows moving the motor by UART control.
    ///
    /// It gives the motor velocity in +-(2^23) - 1 [microsteps / t].
    ///
    /// - 0: Normal operation, driver reacts to step input.
    /// - other: Motor moves with the velocity given by VACTUAL. Step pulses can be monitored via INDEX output.
    ///   The motor direction is controlled by the sign of VACTUAL.
    ///
    /// If the internal clock with 12 MHz is used, you can convert microsteps per second
    /// to the value for this register like this `(microsteps_per_second / 0.715).round() as i32`
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct VACTUAL(u32);
    impl Debug;
    i32;
    pub get, set: 23, 0;
}

bitfield! {
    /// CoolStep threshold.
    ///
    /// This is the lower threshold velocity for switching on smart energy CoolStep and StallGuard to DIAG output.
    /// (unsigned)
    ///
    /// Set this parameter to disable CoolStep at low speeds, where it cannot work reliably.
    /// The stall output signal become enabled when exceeding this velocity. It becomes disabled again once
    /// the velocity falls below this threshold.
    ///
    /// `TCOOLTHRS >= TSTEP > TPWMTHRS`
    /// - CoolStep is enabled, if configured (only with StealthChop)
    /// - Stall output signal on pin DIAG is enabled
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct TCOOLTHRS(u32);
    impl Debug;
    u32;
    pub get, set: 19, 0;
}

bitfield! {
    /// Detection threshold for stall.
    ///
    /// The StallGuard value [`SG_RESULT`] becomes compared to the double of this threshold.
    /// A stall is signaled with `SG_RESULT <= SGTHRS * 2`
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct SGTHRS(u32);
    impl Debug;
    u8;
    pub get, set: 7, 0;
}

bitfield! {
    /// StallGuard result.
    ///
    /// SG_RESULT becomes updated with each fullstep, independent of TCOOLTHRS and SGTHRS.
    /// A higher value signals a lower motor load and more torque headroom. Intended for StealthChop mode, only.
    ///
    /// Bits 9 and 0 will always show 0. Scaling to 10 bit is for compatibility to StallGuard2.
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct SG_RESULT(u32);
    impl Debug;
    u16;
    pub get, _: 9, 0;
}

bitfield! {
    /// Smart Energy Control CoolStep and StallGuard
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct COOLCONF(u32);
    impl Debug;
    u16;
    /// Minimum StallGuard value for smart current control and smart current enable.
    ///
    /// If the StallGuard4 result falls below `SEMIN * 32`, the motor current becomes
    /// increased to reduce motor load angle.
    ///
    /// A value of 0 will turn off the smart current control functionality.
    /// Valid values are 0-15.
    ///
    /// This feature is referred to as cool step.
    pub semin, set_semin: 3, 0;
    /// Current step up width.
    ///
    /// Current increment steps per measured StallGuard value:
    /// - 0: 1
    /// - 1: 2
    /// - 2: 4
    /// - 3: 8
    pub seup, set_seup: 6, 5;
    /// StallGuard hysteresis value for smart current control.
    ///
    /// If the StallGuard4 result is equal to or above `(SEMIN + SEMAX + 1) * 32`, the motor
    /// current becomes decreased to save energy.
    ///
    /// Valid values are 0-15.
    pub semax, set_semax: 11, 8;
    /// Current down step speed.
    ///
    /// - 0: For each 32 StallGuard4 values decrease by one
    /// - 1: For each 8 StallGuard4 values decrease by one
    /// - 2: For each 2 StallGuard4 values decrease by one
    /// - 3: For each StallGuard4 value decrease by one
    pub sedn, set_sedn: 14, 13;
    /// Minimum current for smart current control.
    ///
    /// - 0: 1/2 of current setting [`IHOLD_IRUN::irun`]. Attention: Use with IRUN >= 10
    /// - 1: 1/4 of current setting [`IHOLD_IRUN::irun`]. Attention: Use with IRUN >= 20
    pub seimin, set_seimin: 15;
}

bitfield! {
    /// Microstep counter.
    ///
    /// Indicates actual position in the microstep table for `CUR_A`. `CUR_B` uses an offset
    /// of 256 into the table. Reading out MSCNT allows determination of the motor position
    /// within the electrical wave.
    ///
    /// To monitor the internal step pulse execution, one can periodically read out MSCNT to grasp
    /// steps done in the previous polling interval. It wraps around from 1023 to 0.
    ///
    /// For 32 microsteps per fullstep, each step pulse will increment MSCNT by 256 / 32 = 8.
    /// With this, 10 steps would increment MSCNT by 80.
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct MSCNT(u32);
    impl Debug;
    u16;
    /// Current microstep count (0...1023).
    pub get, _: 9, 0;
}

bitfield! {
    /// Actual microstep current for motor phase A and B.
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct MSCURACT(u32);
    impl Debug;
    i16;
    /// Actual microstep current for motor phase B (sine wave) as read from the internal sine wave table
    /// (not scaled by current setting). Value ranges from -255 to 255.
    pub cur_b, _: 8, 0;
    i16;
    /// Actual microstep current for motor phase A (co-sine wave) as read from the internal sine wave table
    /// (not scaled by current setting). Value ranges from -255 to 255.
    pub cur_a, _: 24, 16;
}

bitfield! {
    /// Chopper configuration
    #[derive(Clone, Copy, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct CHOPCONF(u32);
    impl Debug;
    u32;
    /// TOFF off time and driver enable.
    ///
    /// This setting controls duration of slow decay phase
    /// N_CLK = 24 + 32 * TOFF
    ///
    /// - 0: Driver disabled, all bridges off
    /// - 1: Use only with [`CHOPCONF::tbl`] >= 2
    /// - 2..15
    ///
    /// (Default: OTP, resp. 3 in StealthChop mode)
    pub toff, set_toff: 3, 0;
    /// `0b000` ... `0b111`:
    /// Add 1, 2, ..., 8 to hysteresis low value HEND
    /// (1/512 of this setting adds to current setting)
    /// Attention: Effective HEND+HSTRT ≤ 16.
    /// Hint: Hysteresis decrement is done each 16 clocks
    ///
    /// (Default: OTP, resp. 5 in StealthChop mode)
    pub hstrt, set_hstrt: 6, 4;
    /// `0b0000` ... `0b1111`:
    /// Hysteresis is -3, -2, -1, 0, 1, ..., 12
    /// (1/512 of this setting adds to current setting)
    ///
    /// This is the hysteresis value which becomes used for the
    /// hysteresis chopper.
    ///
    /// (Default: OTP, resp. 0 in StealthChop mode)
    pub hend, set_hend: 10, 7;
    /// Sets the comparator blank time to 16, 24, 32, or 40 clocks.
    ///
    /// Note that you can not call this function with the clocks.
    /// Each of the comparator blank times corresponds to a value from 0 to 3:
    /// - 0: 16 clocks
    /// - 1: 24 clocks
    /// - 2: 32 clocks
    /// - 3: 40 clocks
    ///
    /// which you use to set the field.
    ///
    /// Hint: 0 or 1 is recommended for most applications.
    ///
    /// By default it uses OTP.
    pub tbl, set_tbl: 16, 15;
    /// Sense resistor voltage based current scaling.
    ///
    /// If set to true, the driver uses a high sensitivity, low sense resistor voltage,
    /// otherwise, a low sensitivity, high sense resistor voltage is used.
    pub vsense, set_vsense: 17;
    /// The microstep resolution per step.
    ///
    /// The resolution gives the number of microstep entries per sine quarter wave.
    /// When choosing a lower microstep resolution, the driver automatically uses
    /// microstep positions which result in a symmetrical wave.
    ///
    /// Selection is through the pins, unless it is disabled by [`GCONF::mstep_reg_select`].
    ///
    /// The values range from 0 to 8, where
    /// - 0: 256 microsteps/step
    /// - 1: 128 microsteps/step
    /// - 2: 64 microsteps/step
    /// - 3: 32 microsteps/step
    /// - 4: 16 microsteps/step
    /// - ...
    ///
    /// In general the number of microsteps can be converted to the value the driver understands
    /// by calculating `8 - log2(microsteps)`. For example, for 16 microsteps, `8 - log2(16) = 4`.
    pub from into MicroStepResolution, mres, set_mres: 27, 24;
    /// Interpolation to 256 microsteps.
    ///
    /// When enabled, the actual microstep resolution ([`CHOPCONF::mres`])
    /// becomes extrapolated to 256 microsteps for smoothest motor operation.
    ///
    /// By default, this is enabled.
    pub intpol, set_intpol: 28;
    /// Enable double edge step pulses.
    ///
    /// Enables step impulse at each step edge to reduce step frequency
    /// requirement. This mode is not compatible with the step filtering
    /// function [`GCONF::multistep_filt`].
    pub dedge, set_dedge: 29;
    /// Short to GND protection disable.
    ///
    /// If set to true, the short protection to GND will be disabled.
    pub diss2g, set_diss2g: 30;
    /// Low side short protection disable.
    ///
    /// If set to true, the short protection low side will be disabled.
    pub diss2vs, set_diss2vs: 31;
}

bitfield! {
    /// Driver Status Flags and Current Level Read Back
    ///
    /// This register contains the status flags for the driver, including error conditions
    /// like overtemperature, short circuits, and open load detection. It also provides
    /// the actual current scaling value.
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct DRV_STATUS(u32);
    impl Debug;
    u32;
    /// Overtemperature pre-warning flag
    ///
    /// If `true`, the selected overtemperature pre-warning threshold is exceeded.
    /// The overtemperature pre-warning flag is common for both bridges.
    /// Default threshold is usually 120°C (configurable).
    pub otpw, _: 0;
    /// Overtemperature flag
    ///
    /// If `true`, the selected overtemperature limit has been reached.
    /// Drivers become disabled until `otpw` is also cleared due to cooling down of the IC.
    /// The overtemperature flag is common for both bridges.
    pub ot, _: 1;
    /// Short to ground indicator phase A
    ///
    /// If `true`, short to GND detected on phase A. The driver becomes disabled.
    /// The flags stay active until the driver is disabled by software (TOFF=0) or by the ENN input.
    pub s2ga, _: 2;
    /// Short to ground indicator phase B
    ///
    /// If `true`, short to GND detected on phase B. The driver becomes disabled.
    /// The flags stay active until the driver is disabled by software (TOFF=0) or by the ENN input.
    pub s2gb, _: 3;
    /// Low side short indicator phase A
    ///
    /// If `true`, short on low-side MOSFET detected on phase A.
    /// The driver becomes disabled. The flags stay active until the driver is disabled
    /// by software (TOFF=0) or by the ENN input.
    pub s2vsa, _: 4;
    /// Low side short indicator phase B
    ///
    /// If `true`, short on low-side MOSFET detected on phase B.
    /// The driver becomes disabled. The flags stay active until the driver is disabled
    /// by software (TOFF=0) or by the ENN input.
    pub s2vsb, _: 5;
    /// Open load indicator phase A
    ///
    /// If `true`, open load detected on phase A.
    /// Hint: This is just an informative flag. The driver takes no action upon it.
    /// False detection may occur in fast motion and standstill. Check during slow motion only.
    pub ola, _: 6;
    /// Open load indicator phase B
    ///
    /// If `true`, open load detected on phase B.
    /// Hint: This is just an informative flag. The driver takes no action upon it.
    pub olb, _: 7;
    /// 120°C comparator
    ///
    /// If `true`, temperature threshold 120°C is exceeded.
    pub t120, _: 8;
    /// 143°C comparator
    ///
    /// If `true`, temperature threshold 143°C is exceeded.
    pub t143, _: 9;
    /// 150°C comparator
    ///
    /// If `true`, temperature threshold 150°C is exceeded.
    pub t150, _: 10;
    /// 157°C comparator
    ///
    /// If `true`, temperature threshold 157°C is exceeded.
    pub t157, _: 11;
    /// Actual motor current / smart energy current
    ///
    /// Actual current control scaling (CS), for monitoring the function of the automatic
    /// current scaling. Returns the actual scaling value (0..31).
    pub cs_actual, _: 20, 16;
    /// StealthChop indicator
    ///
    /// 1: Driver operates in StealthChop mode.
    /// 0: Driver operates in SpreadCycle mode.
    pub stealth, _: 30;
    /// Standstill indicator
    ///
    /// This flag indicates motor stand still in each operation mode.
    /// This occurs 2^20 clocks after the last step pulse.
    pub stst, _: 31;
}

bitfield! {
    /// Voltage PWM Mode StealthChop Configuration
    ///
    /// This register configures the StealthChop voltage PWM mode, including
    /// automatic scaling, frequency, and gradients.
    #[derive(Clone, Copy, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct PWMCONF(u32);
    impl Debug;
    u8;
    /// User defined amplitude (offset)
    ///
    /// User defined PWM amplitude offset (0-255) related to full motor current
    /// (CS_ACTUAL=31) in stand still. (Reset default=36).
    ///
    /// When using automatic scaling (`pwm_autoscale=1`) the value is used for initialization only.
    /// `PWM_OFS` = 0 will disable scaling down motor current below a motor specific lower
    /// measurement threshold.
    pub pwm_ofs, set_pwm_ofs: 7, 0;
    /// User defined amplitude gradient
    ///
    /// Velocity dependent gradient for PWM amplitude: `PWM_GRAD * 256 / TSTEP`.
    /// This value is added to `PWM_AMPL` to compensate for the velocity-dependent motor back-EMF.
    ///
    /// With automatic scaling (`pwm_autoscale=1`) the value is used for first initialization only.
    pub pwm_grad, set_pwm_grad: 15, 8;
    /// PWM frequency selection
    ///
    /// Determines the frequency of the StealthChop PWM.
    /// - `0b00`: fPWM = 2/1024 fCLK
    /// - `0b01`: fPWM = 2/683 fCLK (Standard)
    /// - `0b10`: fPWM = 2/512 fCLK
    /// - `0b11`: fPWM = 2/410 fCLK
    pub pwm_freq, set_pwm_freq: 17, 16;
    /// PWM automatic amplitude scaling
    ///
    /// - 0: User defined feed forward PWM amplitude. Current settings IRUN and IHOLD scale
    ///      PWM amplitude, but are not enforced by regulation.
    /// - 1: Enable automatic current control (Reset default).
    pub pwm_autoscale, set_pwm_autoscale: 18;
    /// PWM automatic gradient adaptation
    ///
    /// - 0: Fixed value for `PWM_GRAD`.
    /// - 1: Automatic tuning (only with `pwm_autoscale=1`). `PWM_GRAD_AUTO` is initialized
    ///      with `PWM_GRAD` and becomes optimized automatically during motion.
    pub pwm_autograd, set_pwm_autograd: 19;
    /// Standstill options (Freewheeling)
    ///
    /// Defines the behavior when motor current setting is zero (`I_HOLD=0`).
    /// - `0b00`: Normal operation
    /// - `0b01`: Freewheeling
    /// - `0b10`: Coil shorted using LS drivers (Passive Braking)
    /// - `0b11`: Coil shorted using HS drivers (Passive Braking)
    pub from into StandstillMode, freewheel, set_freewheel: 21, 20;
    /// Regulation loop gradient
    ///
    /// User defined maximum PWM amplitude change per half wave when using `pwm_autoscale=1`.
    /// - 1: 0.5 increments (slowest regulation)
    /// - 2: 1 increment (default with OTP2.1=1)
    /// - ...
    /// - 8: 4 increments (default with OTP2.1=0)
    /// - ...
    /// - 15: 7.5 increments (fastest regulation)
    pub pwm_reg, set_pwm_reg: 27, 24;
    /// PWM automatic scale amplitude limit
    ///
    /// Limit for `PWM_SCALE_AUTO` when switching back from SpreadCycle to StealthChop.
    /// This value defines the upper limit for bits 7 to 4 of the automatic current control
    /// when switching back. It can be set to reduce the current jerk during mode change.
    pub pwm_lim, set_pwm_lim: 31, 28;
}

bitfield! {
    /// Results of StealthChop amplitude regulator.
    ///
    /// These values can be used to monitor automatic PWM amplitude scaling (`255` = max voltage).
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct PWM_SCALE(u32);
    impl Debug;
    u8;
    /// Actual PWM duty cycle.
    ///
    /// This value is used for scaling the values `CUR_A` and `CUR_B` read from the sine wave table.
    pub pwm_scale_sum, _: 7, 0;
    i16;
    /// 9 Bit signed offset added to the calculated PWM duty cycle.
    ///
    /// This is the result of the automatic amplitude regulation based on current measurement.
    ///
    /// Returned values range from `-255` to `255`.
    pub pwm_scale_auto, _: 24, 16;
}

bitfield! {
    /// Automatic PWM scaling values.
    ///
    /// These automatically generated values can be read out in order to
    /// determine a default / power up setting for [`PWM_GRAD`] and [`PWM_OFS`].
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct PWM_AUTO(u32);
    impl Debug;
    u8;
    /// Automatically determined offset value.
    pub pwm_ofs_auto, _: 7, 0;
    /// Automatically determined gradient value.
    pub pwm_grad_auto, _: 23, 16;
}

// Implementation Macros
// --------------------------------------------------------

/// A macro for generating `ReadableRegister` and `WritableRegister` implementations for the
/// register types based on the `R`, `W` or `RW` prefix.
macro_rules! impl_rw {
    (RW $T:ident) => {
        impl ReadableRegister for $T {}
        impl WritableRegister for $T {}
    };
    (R $T:ident) => {
        impl ReadableRegister for $T {}
    };
    (W $T:ident) => {
        impl WritableRegister for $T {}
    };
}

macro_rules! is_readable {
    (RW) => {
        true
    };
    (R) => {
        true
    };
    (W) => {
        false
    };
}

macro_rules! is_writable {
    (RW) => {
        true
    };
    (R) => {
        false
    };
    (W) => {
        true
    };
}

macro_rules! map_indices {
    ($ix:expr, $T:ident) => {
        pub(crate) const $T: usize = $ix;
    };
    ($ix:expr, $T:ident, $($Ts:ident),*) => {
        pub(crate) const $T: usize = $ix;
        map_indices!($T + 1, $($Ts),*);
    };
}

/// A macro for generating the `Address` enum along with the `Register` trait implementations.
macro_rules! impl_registers {
    ($($RW:ident $addr:literal $T:ident $map_access:ident $map_access_mut:ident,)*) => {
        /// Generate a private, unique index for each register into the `Map`'s inner array.
        mod map_index {
            map_indices!(0, $($T),*);
        }

        /// A dynamic representation of a register's 8-bit address.
        #[repr(u8)]
        #[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
        #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
        #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
        pub enum Address {
            $(
                $T = $addr,
            )*
        }

        /// A dynamic representation of a register's 32-bit state.
        #[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
        #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
        #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
        pub enum State {
            $(
                $T($T),
            )*
        }

        /// A map of the state of all registers in the TMC2209.
        #[derive(Clone, Debug, Eq, Hash, PartialEq)]
        #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
        #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
        pub struct Map {
            arr: MapArray,
        }

        /// The inner array storing all register state.
        ///
        /// Each register is laid out in the array in the order in which they are declared in the
        /// `impl_registers` macro. The `map_index` module is used internally to map register
        /// addresses and their state to the associated elements in the array.
        type MapArray = [State; COUNT];

        /// The total number of documented registers in the TMC2209.
        ///
        /// Useful for statically allocated register maps, etc.
        pub const COUNT: usize = 0 $(+ { let _ = Address::$T; 1 })*;

        impl Map {
            /// The total number of documented registers in the TMC2209.
            pub const LEN: usize = COUNT;

            /// Read-only access to the register of the given type.
            pub fn reg<T>(&self) -> &T
            where
                T: 'static + Register,
            {
                self.state(T::ADDRESS)
                    .reg::<T>()
                    // We gaurantee that `TmcRegisters` will always have state for each register, but need
                    // to avoid generating panicking branches, so we use an infinite loop rather than
                    // unwrap.
                    .unwrap_or_else(|_| loop {})
            }

            /// Mutable access to the register of the given type.
            pub fn reg_mut<T>(&mut self) -> &mut T
            where
                T: 'static + Register,
            {
                self.state_mut(T::ADDRESS)
                    .reg_mut::<T>()
                    // We gaurantee that `TmcRegisters` will always have state for each register, but need
                    // to avoid generating panicking branches, so we use an infinite loop rather than
                    // unwrap.
                    .unwrap_or_else(|_| loop {})
            }

            /// Read-only access to the dynamic representation of the register state at the given
            /// address.
            pub fn state(&self, addr: Address) -> &State {
                match addr {
                    $(
                        // We gaurantee that `Map` will always have state for each register.
                        Address::$T => unsafe {
                            self.arr.get_unchecked(map_index::$T)
                        }
                    )*
                }
            }

            /// Mutable access to the dynamic representation of the register state at the given
            /// address.
            ///
            /// Note: This should remain private for internal use only, as the user should never be
            /// allowed to change the stored `State` to a different variant.
            fn state_mut(&mut self, addr: Address) -> &mut State {
                match addr {
                    $(
                        // We gaurantee that `Map` will always have state for each register.
                        Address::$T => unsafe {
                            self.arr.get_unchecked_mut(map_index::$T)
                        }
                    )*
                }
            }

            /// Update the given register state.
            pub fn set_state(&mut self, state: State) {
                *self.state_mut(state.addr()) = state;
            }

            // Generate the short-hand names for gaining direct access to typed register state.
            $(
                pub fn $map_access(&self) -> &$T {
                    self.reg::<$T>()
                }

                pub fn $map_access_mut(&mut self) -> &mut $T {
                    self.reg_mut::<$T>()
                }
            )*
        }

        impl Address {
            /// All register addresses.
            pub const ALL: &'static [Self] = &[
                $(
                    Self::$T,
                )*
            ];

            /// Whether or not we can send a read request to the register address.
            pub fn readable(&self) -> bool {
                match *self {
                    $(
                        Self::$T => is_readable!($RW),
                    )*
                }
            }

            /// Whether or not we can send a write request to the register address.
            pub fn writable(&self) -> bool {
                match *self {
                    $(
                        Self::$T => is_writable!($RW),
                    )*
                }
            }
        }

        impl State {
            /// Construct a register state from its address and data represented as a `u32`.
            pub fn from_addr_and_data(addr: Address, data: u32) -> Self {
                match addr {
                    $(
                        Address::$T => State::$T(<_>::from(data)),
                    )*
                }
            }

            /// Construct the default register state associated with the given address.
            pub fn from_addr_default(addr: Address) -> Self {
                match addr {
                    $(
                        Address::$T => State::$T(<_>::default()),
                    )*
                }
            }

            /// The address of the register with which this state is associated.
            pub fn addr(&self) -> Address {
                match *self {
                    $(
                        State::$T(_) => Address::$T,
                    )*
                }
            }

            /// Attempt to retrieve a reference to a register of type `R` from the dynamic register
            /// `State` representation.
            ///
            /// Returns an `Err` if the register type does not match.
            pub fn reg<R>(&self) -> Result<&R, UnexpectedAddress>
            where
                R: 'static + Register,
            {
                match *self {
                    $(
                        Self::$T(ref r) => (r as &dyn core::any::Any)
                            .downcast_ref()
                            .ok_or(UnexpectedAddress),
                    )*
                }
            }

            /// Attempt to retrieve a mutable reference to a register of type `R` from the dynamic
            /// register `State` representation.
            ///
            /// Returns an `Err` if the register type does not match.
            pub fn reg_mut<R>(&mut self) -> Result<&mut R, UnexpectedAddress>
            where
                R: 'static + Register,
            {
                match *self {
                    $(
                        Self::$T(ref mut r) => (r as &mut dyn core::any::Any)
                            .downcast_mut()
                            .ok_or(UnexpectedAddress),
                    )*
                }
            }
        }

        impl Default for Map {
            fn default() -> Self {
                let arr = [$(
                    State::$T($T::default()),
                )*];
                Map { arr }
            }
        }

        impl core::ops::Deref for Map {
            type Target = MapArray;
            fn deref(&self) -> &Self::Target {
                &self.arr
            }
        }

        #[cfg(feature = "hash")]
        impl hash32::Hash for Address {
            fn hash<H>(&self, state: &mut H)
            where
                H: hash32::Hasher,
            {
                (*self as u8).hash(state)
            }
        }

        #[cfg(feature = "hash")]
        impl hash32::Hash for State {
            fn hash<H>(&self, state: &mut H)
            where
                H: hash32::Hasher,
            {
                let u: u32 = (*self).into();
                u.hash(state)
            }
        }

        impl core::ops::Index<Address> for Map {
            type Output = State;
            fn index(&self, addr: Address) -> &Self::Output {
                self.state(addr)
            }
        }

        impl core::ops::IndexMut<Address> for Map {
            fn index_mut(&mut self, addr: Address) -> &mut Self::Output {
                self.state_mut(addr)
            }
        }

        impl From<Address> for u8 {
            fn from(address: Address) -> u8 {
                address as u8
            }
        }

        impl From<State> for u32 {
            fn from(state: State) -> u32 {
                match state {
                    $(
                        State::$T(r) => r.into(),
                    )*
                }
            }
        }

        impl core::convert::TryFrom<u8> for Address {
            type Error = UnknownAddress;
            fn try_from(u: u8) -> Result<Self, Self::Error> {
                let reg = match u {
                    $(
                        $addr => Self::$T,
                    )*
                    _ => return Err(UnknownAddress),
                };
                Ok(reg)
            }
        }

        $(
            impl From<u32> for $T {
                fn from(u: u32) -> $T {
                    $T(u)
                }
            }

            impl From<$T> for State {
                fn from(r: $T) -> Self {
                    State::$T(r)
                }
            }

            impl From<$T> for u32 {
                fn from(value: $T) -> u32 {
                    value.0 as u32
                }
            }

            impl Register for $T {
                const ADDRESS: Address = Address::$T;
            }

            impl core::convert::TryFrom<State> for $T {
                type Error = UnexpectedAddress;
                fn try_from(state: State) -> Result<Self, Self::Error> {
                    match state {
                        State::$T(s) => Ok(s),
                        _ => Err(UnexpectedAddress),
                    }
                }
            }
        )*

        $(
            impl_rw!{$RW $T}
        )*
    };
}

// Register Implementations
// --------------------------------------------------------

impl_registers! {
    // General Registers.
    RW 0x00 GCONF gconf gconf_mut,
    RW 0x01 GSTAT gstat gstat_mut,
    R  0x02 IFCNT ifcnt ifcnt_mut,
    W  0x03 SLAVECONF slaveconf slaveconf_mut,
    W  0x04 OTP_PROG otp_prog otp_prog_mut,
    R  0x05 OTP_READ otp_read otp_read_mut,
    R  0x06 IOIN ioin ioin_mut,
    RW 0x07 FACTORY_CONF factory_conf factory_conf_mut,

    // Velocity Dependent Control.
    W  0x10 IHOLD_IRUN ihold_irun ihold_irun_mut,
    W  0x11 TPOWERDOWN tpowerdown tpowerdown_mut,
    R  0x12 TSTEP tstep tstep_mut,
    W  0x13 TPWMTHRS tpwmthrs tpwmthrs_mut,
    W  0x22 VACTUAL vactual vactual_mut,

    // StallGuard Control.
    W  0x14 TCOOLTHRS tcoolthrs tcoolthrs_mut,
    W  0x40 SGTHRS sgthrs sgthrs_mut,
    R  0x41 SG_RESULT sg_result sg_result_mut,
    W  0x42 COOLCONF coolconf coolconf_mut,

    // Sequencer Registers.
    R  0x6A MSCNT mscnt mscnt_mut,
    R  0x6B MSCURACT mscuract mscuract_mut,

    // Chopper Control Registers.
    RW 0x6C CHOPCONF chopconf chopconf_mut,
    R  0x6F DRV_STATUS drv_status drv_status_mut,
    RW 0x70 PWMCONF pwmconf pwmconf_mut,
    R  0x71 PWM_SCALE pwm_scale pwm_scale_mut,
    R  0x72 PWM_AUTO pwm_auto pwm_auto_mut,
}

impl VACTUAL {
    /// Creates the `VACTUAL` register enabled for UART control but in a stopped state.
    pub const ENABLED_STOPPED: Self = VACTUAL(1);
}

// Default Register States (taken from TMC-API reference).
// --------------------------------------------------------

impl Default for GCONF {
    fn default() -> Self {
        let mut result = Self(0);

        result.set_i_scale_analog(true);
        result.set_pdn_disable(true);

        result
    }
}

impl Default for IHOLD_IRUN {
    fn default() -> Self {
        let mut result = Self(0);

        result.set_irun(31);

        result
    }
}

impl Default for CHOPCONF {
    fn default() -> Self {
        let mut result = Self(0);

        result.set_toff(3);
        result.set_hstrt(5);
        result.set_intpol(true);

        result
    }
}

impl Default for PWMCONF {
    fn default() -> Self {
        let mut result = Self(0);

        result.set_pwm_ofs(36);
        result.set_pwm_freq(1);
        result.set_pwm_autoscale(true);
        result.set_pwm_autograd(true);
        result.set_pwm_reg(1);
        result.set_pwm_lim(12);

        result
    }
}

impl Default for TPOWERDOWN {
    fn default() -> Self {
        Self(20)
    }
}

// Sanity Checks
// --------------------------------------------------------

#[test]
fn test_slaveconf() {
    let mut s = SLAVECONF(0);
    assert_eq!(s.0, 0b000000000000);
    s.set_send_delay(15);
    assert_eq!(s.0, 0b111100000000);
}

#[test]
fn test_gconf() {
    let mut g = GCONF(0);
    assert_eq!(g.0, 0b0000000000);
    g.set_i_scale_analog(true);
    assert_eq!(g.0, 0b0000000001);
    g.set_test_mode(true);
    assert_eq!(g.0, 0b1000000001);
    g = Default::default();
    assert_eq!(g.0, 0x00000041);
}

#[test]
fn test_gconf_default() {
    assert_eq!(GCONF::default(), GCONF(0x00000041));
}

#[test]
fn test_ihold_irun_default() {
    assert_eq!(IHOLD_IRUN::default(), IHOLD_IRUN(0x00001F00));
}

#[test]
fn test_chopconf_default() {
    assert_eq!(CHOPCONF::default(), CHOPCONF(0x10000053));
}

#[test]
fn test_pwmconf_default() {
    assert_eq!(PWMCONF::default(), PWMCONF(0xC10D0024));
}
