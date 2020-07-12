//! A small demonstration of controlling a TMC2209 via the Teensy 4.0 MCU.

#![no_std]
#![no_main]

use core::convert::TryFrom;
use embedded_hal::blocking::serial::Write;
use embedded_hal::digital::v2::{OutputPin, StatefulOutputPin, ToggleableOutputPin};
use embedded_hal::serial::Read;
use heapless::consts::U8;
use heapless::Vec;
use panic_halt as _;
use teensy4_bsp as bsp;
use teensy4_bsp::hal::gpio::IntoGpio;

const BAUD: u32 = 115_200;
const TX_FIFO_SIZE: u8 = 4;

// The UART channels.
type CompUartRx = bsp::hal::uart::Rx<bsp::hal::iomuxc::uart::module::_2>;
type TmcUartRx = bsp::hal::uart::Rx<bsp::hal::iomuxc::uart::module::_4>;
type TmcUartTx = bsp::hal::uart::Tx<bsp::hal::iomuxc::uart::module::_4>;

// Motor pins.
type Enable = bsp::hal::gpio::GPIO2IO01<bsp::hal::gpio::GPIO2, bsp::hal::gpio::Output>;

/// The TMC2209 registers that we care about.
pub struct TmcRegisters {
    gconf: tmc2209::reg::GCONF,
    vactual: tmc2209::reg::VACTUAL,
}

// VACTUAL accepts velocity as `[usteps / t]`. Not sure what t is, maybe seconds?
const DEFAULT_VELOCITY: i32 = 1024;

// `0` disables VACTUAL.
// `1` enables VACTUAL but indicates a velocity of `0`.
const VACTUAL_STOP: i32 = 1;

#[rtic::app(device = teensy4_bsp, monotonic = rtic::cyccnt::CYCCNT, peripherals = true)]
const APP: () = {
    struct Resources {
        led: bsp::LED,
        enable: Enable,
        comp_rx: CompUartRx,
        tmc_rx: TmcUartRx,
        tmc_tx: TmcUartTx,
        tmc_reader: tmc2209::Reader,
        tmc_regs: TmcRegisters,
        stepping: bool,
        velocity: i32,
        v_sign: bool,
    }

    #[init]
    fn init(mut cx: init::Context) -> init::LateResources {
        init_delay();

        // Setup the clock for rtic scheduling.
        cx.device.ccm.set_mode(bsp::hal::ccm::ClockMode::Run);
        cx.core.DWT.enable_cycle_counter();
        cx.device.ccm.pll1.set_arm_clock(
            bsp::hal::ccm::PLL1::ARM_HZ,
            &mut cx.device.ccm.handle,
            &mut cx.device.dcdc,
        );

        // UART setup.
        let uarts = cx.device.uart.clock(
            &mut cx.device.ccm.handle,
            bsp::hal::ccm::uart::ClockSelect::OSC,
            bsp::hal::ccm::uart::PrescalarSelect::DIVIDE_1,
        );

        // Computer UART channel.
        let mut comp_uart = uarts
            .uart2
            .init(cx.device.pins.p14.alt2(), cx.device.pins.p15.alt2(), BAUD)
            .unwrap();
        comp_uart.set_tx_fifo(core::num::NonZeroU8::new(TX_FIFO_SIZE));
        comp_uart.set_rx_fifo(true);
        comp_uart.set_receiver_interrupt(Some(0));
        let (comp_tx, comp_rx) = comp_uart.split();
        imxrt_uart_log::blocking::init(comp_tx, Default::default()).unwrap();

        // TMC2209 UART channel.
        let mut tmc_uart = uarts
            .uart4
            .init(cx.device.pins.p8.alt2(), cx.device.pins.p7.alt2(), BAUD)
            .unwrap();
        tmc_uart.set_tx_fifo(core::num::NonZeroU8::new(TX_FIFO_SIZE));
        tmc_uart.set_rx_fifo(true);
        tmc_uart.set_receiver_interrupt(Some(0));
        let (mut tmc_tx, tmc_rx) = tmc_uart.split();

        // Initialise tracking state for the registers we care about.
        // - Enable UART controlled velocity but in the stopped state.
        // - Enable the `pdn_disable` field necessary for UART comms.
        let mut gconf = tmc2209::reg::GCONF::default();
        let vactual = tmc2209::reg::VACTUAL::ENABLED_STOPPED;
        gconf.set_pdn_disable(true);
        tmc2209::send_write_request(0, gconf, &mut tmc_tx).unwrap();
        tmc2209::send_write_request(0, vactual, &mut tmc_tx).unwrap();
        let tmc_regs = TmcRegisters { gconf, vactual };

        // The reader that we will use for interpreting TMC2209 UART responses.
        let tmc_reader = Default::default();

        // LED setup.
        let mut led = bsp::configure_led(&mut cx.device.gpr, cx.device.pins.p13);
        led.set_high().unwrap();

        // Motor digital pins.
        let enable = cx.device.pins.p12.into_gpio().output();

        // Init step interval.
        let stepping = false;
        let velocity = DEFAULT_VELOCITY;
        let v_sign = false;

        init::LateResources {
            led,
            enable,
            comp_rx,
            tmc_tx,
            tmc_rx,
            tmc_reader,
            tmc_regs,
            stepping,
            velocity,
            v_sign,
        }
    }

    #[task(binds = LPUART2, resources = [comp_rx, enable, stepping, tmc_regs, tmc_reader, tmc_rx, tmc_tx, velocity, v_sign])]
    fn lpuart2(mut cx: lpuart2::Context) {
        let r = &mut cx.resources;
        while let Ok(b) = r.comp_rx.read() {
            if let Ok(s) = core::str::from_utf8(&[b]) {
                match s {
                    "d" => {
                        let new_shaft = !r.tmc_regs.gconf.shaft();
                        r.tmc_regs.gconf.set_shaft(new_shaft);
                        tmc2209::send_write_request(0, r.tmc_regs.gconf, r.tmc_tx).unwrap();
                    }
                    "t" => {
                        tmc2209::send_read_request::<tmc2209::reg::IFCNT, _>(0, r.tmc_tx).unwrap();
                    }
                    "r" => {
                        tmc2209::send_read_request::<tmc2209::reg::IOIN, _>(0, r.tmc_tx).unwrap();
                    }
                    "e" => {
                        r.enable.toggle().unwrap();
                        log::info!("ENN: {}", output_pin_state_str(r.enable));
                    }

                    // Velocity control. Notice that:
                    // - `1` indicates "stopped",
                    // - `0` disables VACTUAL mode so we avoid it.
                    "s" => {
                        // Toggle the sign of the velocity value (change direction).
                        *r.velocity *= -1;
                        log::info!("Velocity: {}", r.velocity);
                        if *r.stepping {
                            r.tmc_regs.vactual.set(*r.velocity);
                            tmc2209::send_write_request(0, r.tmc_regs.vactual, r.tmc_tx).unwrap();
                        }
                    }
                    "+" => {
                        *r.velocity *= 2;
                        log::info!("Velocity: {}", r.velocity);
                        if *r.stepping {
                            r.tmc_regs.vactual.set(*r.velocity);
                            tmc2209::send_write_request(0, r.tmc_regs.vactual, r.tmc_tx).unwrap();
                        }
                    }
                    "-" => {
                        if *r.velocity > VACTUAL_STOP {
                            *r.velocity /= 2;
                        }
                        log::info!("Velocity: {}", r.velocity);
                        if *r.stepping {
                            r.tmc_regs.vactual.set(*r.velocity);
                            tmc2209::send_write_request(0, r.tmc_regs.vactual, r.tmc_tx).unwrap();
                        }
                    }

                    // Stop / Start.
                    " " => {
                        *r.stepping = !*r.stepping;
                        if *r.stepping {
                            log::info!("START");
                            r.enable.set_high().unwrap();
                            r.tmc_regs.vactual.set(*r.velocity);
                        } else {
                            log::info!("STOP");
                            r.enable.set_low().unwrap();
                            r.tmc_regs.vactual.set(VACTUAL_STOP);
                        }
                        tmc2209::send_write_request(0, r.tmc_regs.vactual, r.tmc_tx).unwrap();
                    }

                    _ => log::info!("Unknown Command: {}", s),
                }
            }
        }
    }

    #[task(binds = LPUART4, resources = [tmc_rx, tmc_reader])]
    fn lpuart4(mut cx: lpuart4::Context) {
        let r = &mut cx.resources;
        while let Ok(b) = r.tmc_rx.read() {
            if let (_, Some(response)) = r.tmc_reader.read_response(&[b]) {
                match response.crc_is_valid() {
                    true => log::info!("Received valid response!"),
                    false => {
                        log::error!("Received invalid response!");
                        continue;
                    }
                }
                match response.reg_addr() {
                    Ok(tmc2209::reg::Address::IOIN) => {
                        let reg = response.register::<tmc2209::reg::IOIN>().unwrap();
                        log::info!("{:?}", reg);
                    }
                    Ok(tmc2209::reg::Address::IFCNT) => {
                        let reg = response.register::<tmc2209::reg::IFCNT>().unwrap();
                        log::info!("{:?}", reg);
                    }
                    addr => log::warn!("Unexpected register address: {:?}", addr),
                }
            }
        }
    }

    // RTIC requires that unused interrupts are declared in an extern block when
    // using software tasks; these free interrupts will be used to dispatch the
    // software tasks.
    extern "C" {
        fn LPUART8();
    }
};

// If we reach WFI on teensy 4.0 too quickly it seems to halt. Here we wait a short while in `init`
// to avoid this issue. The issue only appears to occur when rebooting the device (via the button),
// however there appears to be no issue when power cycling the device.
//
// TODO: Investigate exactly why this appears to be necessary.
fn init_delay() {
    for _ in 0..10_000_000 {
        core::sync::atomic::spin_loop_hint();
    }
}

fn output_pin_state_str<P>(p: &P) -> &str
where
    P: StatefulOutputPin,
{
    match p.is_set_high() {
        Ok(true) => "HIGH",
        Ok(false) => "LOW",
        Err(_) => "ERR",
    }
}
