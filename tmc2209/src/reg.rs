//! Declaration of the TMC2209 registers and their implementations.
//!
//! Please refer to the TMC2209 datasheet for information on what each of these registers and their
//! fields mean. The register map is described under section 5 of the datasheet.
//!
//! https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2209_Datasheet_V103.pdf

#![allow(non_camel_case_types)]

// Register Traits
// --------------------------------------------------------

/// Implemented for all register types.
pub trait Register {
    const ADDRESS: Address;
}

/// Implemented for all registers that can be read from.
pub trait ReadableRegister: Register + From<u32> {}

/// Implemented for all registers that can be written to.
pub trait WritableRegister: Register + Into<u32> {}

/// An error that might occur in the case that an address could not be parsed.
#[derive(Debug)]
pub struct UnknownAddress;

// Register Declarations
// --------------------------------------------------------

bitfield! {
    #[derive(Clone, Copy)]
    pub struct GCONF(u32);
    impl Debug;
    u16;
    pub i_scale_analog, set_i_scale_analog: 0;
    pub internal_rsense, set_internal_rsense: 1;
    pub en_spread_cycle, set_en_spread_cycle: 2;
    pub shaft, set_shaft: 3;
    pub index_otpw, set_index_otpw: 4;
    pub index_step, set_index_step: 5;
    pub pdn_disable, set_pdn_disable: 6;
    pub mstep_reg_select, set_mstep_reg_select: 7;
    pub multistep_filt, set_multistep_filt: 8;
    pub test_mode, set_test_mode: 9;
}

bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct GSTAT(u32);
    impl Debug;
    u8;
    pub reset, _: 0;
    pub drv_err, _: 1;
    pub uv_cp, _: 2;
}

#[derive(Clone, Copy, Debug)]
pub struct IFCNT(pub u32);

bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct SLAVECONF(u32);
    impl Debug;
    u8;
    pub get, set: 11, 8;
}

bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct OTP_PROG(u32);
    impl Debug;
    u16;
    pub otp_bit, set_otp_bit: 2, 0;
    pub otp_byte, set_otp_byte: 5, 4;
    pub opt_magic, set_otp_magic: 15, 8;
}

bitfield! {
    #[derive(Clone, Copy)]
    pub struct OTP_READ(u32);
    impl Debug;
    u8;
    pub otp_fclktrim, _: 4, 0;
    pub otp_ottrim, _: 5;
    pub otp_internal_rsense, _: 6;
    pub otp_tbl, _: 7;
    pub otp_pwm_grad, _: 11, 8;
    pub otp_pwm_autograd, _: 12;
    pub otp_tpwmthrs, _: 15, 13;
    pub otp_pwm_ofs, _: 16;
    pub otp_pwm_reg, _: 17;
    pub otp_pwm_freq, _: 18;
    pub otp_iholdddelay, _: 20, 19;
    pub otp_ihold, _: 22, 21;
    pub otp_en_spread_cycle, _: 23;
}

bitfield! {
    #[derive(Clone, Copy)]
    pub struct IOIN(u32);
    impl Debug;
    u16;
    pub enn, _: 0;
    pub ms1, _: 2;
    pub ms2, _: 3;
    pub diag, _: 4;
    pub pdn_uart, _: 6;
    pub step, _: 7;
    pub spread_en, _: 8;
    pub dir, _: 9;
    u8;
    pub version, _: 31, 24;
}

bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct FACTORY_CONF(u32);
    impl Debug;
    u8;
    pub fclktrim, set_fclktrim: 4, 0;
    pub ottrim, set_ottrim: 9, 8;
}

bitfield! {
    #[derive(Clone, Copy)]
    pub struct IHOLD_IRUN(u32);
    impl Debug;
    u8;
    pub ihold, set_ihold: 4, 0;
    pub irun, set_irun: 12, 8;
    pub ihold_delay, set_ihold_delay: 19, 16;
}

#[derive(Clone, Copy, Debug, Default)]
pub struct TPOWERDOWN(pub u32);

bitfield! {
    #[derive(Clone, Copy)]
    pub struct TSTEP(u32);
    impl Debug;
    u32;
    pub get, _: 19, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct TPWMTHRS(u32);
    impl Debug;
    u32;
    pub get, set: 19, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct VACTUAL(u32);
    impl Debug;
    i32;
    pub get, set: 23, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct TCOOLTHRS(u32);
    impl Debug;
    u32;
    pub get, set: 19, 0;
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SGTHRS(pub u32);

bitfield! {
    #[derive(Clone, Copy)]
    pub struct SG_RESULT(u32);
    impl Debug;
    u16;
    pub get, _: 9, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct COOLCONF(u32);
    impl Debug;
    u16;
    pub semin0, set_semin0: 0;
    pub semin1, set_semin1: 1;
    pub semin2, set_semin2: 2;
    pub semin3, set_semin3: 3;
    pub seup0, set_seup0: 5;
    pub seup1, set_seup1: 6;
    pub semax0, set_semax0: 8;
    pub semax1, set_semax1: 9;
    pub semax2, set_semax2: 10;
    pub semax3, set_semax3: 11;
    pub sedn0, set_sedn0: 13;
    pub sedn1, set_sedn1: 14;
    pub seimin, set_seimin: 15;
}

bitfield! {
    #[derive(Clone, Copy)]
    pub struct MSCNT(u32);
    impl Debug;
    u16;
    pub get, _: 9, 0;
}

bitfield! {
    #[derive(Clone, Copy)]
    pub struct MSCURACT(u32);
    impl Debug;
    u16;
    pub cur_a, _: 8, 0;
    pub cur_b, _: 24, 16;
}

bitfield! {
    #[derive(Clone, Copy)]
    pub struct CHOPCONF(u32);
    impl Debug;
    u32;
    pub toff0, set_toff0: 0;
    pub toff1, set_toff1: 1;
    pub toff2, set_toff2: 2;
    pub toff3, set_toff3: 3;
    pub hstrt0, set_hstrt0: 4;
    pub hstrt1, set_hstrt1: 5;
    pub hstrt2, set_hstrt2: 6;
    pub hend0, set_hend0: 7;
    pub hend1, set_hend1: 8;
    pub hend2, set_hend2: 9;
    pub hend3, set_hend3: 0;
    pub tbl0, set_tbl0: 15;
    pub tbl1, set_tbl1: 16;
    pub vsense, set_vsense: 17;
    pub mres0, set_mres0: 24;
    pub mres1, set_mres1: 25;
    pub mres2, set_mres2: 26;
    pub mres3, set_mres3: 27;
    pub intpol, set_intpol: 28;
    pub dedge, set_dedge: 29;
    pub diss2g, set_diss2g: 30;
    pub diss2vs, set_diss2vs: 31;
}

bitfield! {
    #[derive(Clone, Copy)]
    pub struct DRV_STATUS(u32);
    impl Debug;
    u32;
    pub otpw, _: 0;
    pub ot, _: 1;
    pub s2ga, _: 2;
    pub s2gb, _: 3;
    pub s2vsa, _: 4;
    pub s2vsb, _: 5;
    pub ola, _: 6;
    pub olb, _: 7;
    pub t120, _: 8;
    pub t143, _: 9;
    pub t150, _: 10;
    pub t157, _: 11;
    pub cs_actual, _: 20, 16;
    pub stealth, _: 30;
    pub stst, _: 31;
}

bitfield! {
    #[derive(Clone, Copy)]
    pub struct PWMCONF(u32);
    impl Debug;
    u8;
    pub pwm_ofs, set_pwm_ofs: 7, 0;
    pub pwm_grad, set_pwm_grad: 15, 8;
    pub pwm_freq0, set_pwm_freq0: 16;
    pub pwm_freq1, set_pwm_freq1: 17;
    pub pwm_autoscale, set_pwm_autoscale: 18;
    pub pwm_autograd, set_pwm_autograd: 19;
    pub freewheel0, set_freewheel0: 20;
    pub freewheel1, set_freewheel1: 21;
    pub pwm_reg, set_pwm_reg: 27, 24;
    pub pwm_lim, set_pwm_lim: 31, 28;
}

bitfield! {
    #[derive(Clone, Copy)]
    pub struct PWM_SCALE(u32);
    impl Debug;
    u8;
    pub pwm_scale_sum, _: 7, 0;
    u16;
    pub pwm_scale_auto, _: 24, 16;
}

bitfield! {
    #[derive(Clone, Copy)]
    pub struct PWM_AUTO(u32);
    impl Debug;
    u8;
    pub pwm_ofs_auto, _: 7, 0;
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

/// A macro for generating the `Address` enum along with the `Register` trait implementations.
macro_rules! impl_registers {
    ($($RW:ident $addr:literal $T:ident,)*) => {
        #[repr(u8)]
        #[derive(Clone, Copy, Debug, Eq, PartialEq)]
        pub enum Address {
            $(
                $T = $addr,
            )*
        }

        impl Into<u8> for Address {
            fn into(self) -> u8 {
                self as u8
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
            impl Register for $T {
                const ADDRESS: Address = Address::$T;
            }

            impl Into<u32> for $T {
                fn into(self) -> u32 {
                    self.0 as u32
                }
            }

            impl From<u32> for $T {
                fn from(u: u32) -> $T {
                    $T(u)
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
    RW 0x00 GCONF,
    RW 0x01 GSTAT,
    R  0x02 IFCNT,
    W  0x03 SLAVECONF,
    W  0x04 OTP_PROG,
    R  0x05 OTP_READ,
    R  0x06 IOIN,
    RW 0x07 FACTORY_CONF,

    // Velocity Dependent Control.
    W  0x10 IHOLD_IRUN,
    W  0x11 TPOWERDOWN,
    R  0x12 TSTEP,
    W  0x13 TPWMTHRS,
    W  0x22 VACTUAL,

    // StallGuard Control.
    W  0x14 TCOOLTHRS,
    W  0x40 SGTHRS,
    R  0x41 SG_RESULT,
    W  0x42 COOLCONF,

    // Sequencer Registers.
    R  0x6A MSCNT,
    R  0x6B MSCURACT,

    // Chopper Control Registers.
    RW 0x6C CHOPCONF,
    R  0x6F DRV_STATUS,
    RW 0x70 PWMCONF,
    R  0x71 PWM_SCALE,
    R  0x72 PWM_AUTO,
}

impl VACTUAL {
    /// Creates the `VACTUAL` register enabled for UART control but in a stopped state.
    pub const ENABLED_STOPPED: Self = VACTUAL(1);
}

// Default Register States (taken from TMC-API reference).
// --------------------------------------------------------

impl Default for GCONF {
    fn default() -> Self {
        Self(0x00000041)
    }
}

impl Default for IHOLD_IRUN {
    fn default() -> Self {
        Self(0x00001F00)
    }
}

impl Default for CHOPCONF {
    fn default() -> Self {
        Self(0x10000053)
    }
}

impl Default for PWMCONF {
    fn default() -> Self {
        Self(0xC10D0024)
    }
}

// Sanity Checks
// --------------------------------------------------------

#[test]
fn test_slaveconf() {
    let mut s = SLAVECONF(0);
    assert_eq!(s.0, 0b000000000000);
    s.set(15);
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
