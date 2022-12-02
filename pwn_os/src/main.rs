//! This is the Software running on the rp2040 itselve and is responsible for emulating the
//! keyboard.
#![no_std]
#![no_main]

use board::entry;
use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;

// for now I'm using the rp_pico
use rp_pico as board;
// If you wanna use the trinkey instead uncomment the trinkey line in the Cargo.toml and the
// line below (havent tested it yet, dont have a trinkey yet)
// use adafruit-trinkey-rp2040 as board;

// Some of the more often used Imports
use board::hal;
use hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

use pac::interrupt;

// USB Imports
use usb_device::{class_prelude::*, prelude::*};
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::KeyboardReport;
use usbd_hid::hid_class::HIDClass;

static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;

#[entry]
fn main() -> ! {
    // mutable variable für unseren Peripheral Access Crate
    let mut pac = pac::Peripherals::take().unwrap();
    // Unser Wachhund doggy
    let mut doggy = Watchdog::new(pac.WATCHDOG);

    // Clock config
    let clocks = init_clocks_and_plls(
        board::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut doggy,
    )
    .ok()
    .unwrap();

    // sio (single cycle IO block) Burschen ich hab auch keinen Plan was das ist
    let sio = Sio::new(pac.SIO);
    // Unsere werten Pins
    let pins = board::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // USB driver setup
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    // DON'T FUCK WITH THAT
    unsafe {
        // Only safe because interupts haven't been started yet
        USB_BUS = Some(usb_bus);
    }

    // Siehst du den unsafe Bums da? NICHT BERÜHREN!
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };
    let usb_hid = HIDClass::new(bus_ref, KeyboardReport::desc(), 1); // Last thing is the poll
                                                                     // rate in ms
    unsafe {
        // Also only safe cus interrupts r not started
        USB_HID = Some(usb_hid);
    }

    // Jetzt faken wir mal so ein Universelles Serielles Buss Geraet
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x046D, 0xC315))
        .manufacturer("OurPC, Inc.")
        .product("UrLastUSB")
        .serial_number("TEST")
        .device_class(0) // 0 steht für "0 Plan wofür das steht"
        .build();
    unsafe {
        // Da hammas wieder
        // Nur safe wenn die interrupts nicht gestartet sind, bla bla bla
        USB_DEVICE = Some(usb_dev);
    }

    unsafe {
        // letztes mal für's Erste
        // Des issa jetzt, der berüchtigte Interrupt
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    }

    // Jetzt nur noch core und delay
    let core = pac::CorePeripherals::take().unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

      //======================//
     //== Now we can start ==//
    //======================//

    struct Instruction {
        report: KeyboardReport,
        time_us: u32,
    }

    // ☟☟☟ Modifiers ☟☟☟

    // Meine Damen und Herren: der right Shift Bit Bit-shifter
    fn l_shift(kr: KeyboardReport) -> KeyboardReport {
        let mut modifier = kr.modifier;
        modifier |= 2;
        KeyboardReport { modifier, ..kr }
    }

    //left GUI Bit Bit-shifter, same for GUI Key and with a less cool name
    fn l_gui(kr: KeyboardReport) -> KeyboardReport {
        let mut modifier = kr.modifier;
        modifier |= 8; //not sure bout that number
        KeyboardReport { modifier, ..kr }
    }

    // left ALT Bit-shifter
    fn l_alt(kr: KeyboardReport) -> KeyboardReport {
        let mut modifier = kr.modifier;
        modifier |= 4; //not sure bout that number
        KeyboardReport { modifier, ..kr }
    }

    //left ctrl bit shifter
    fn l_ctrl(kr: KeyboardReport) -> KeyboardReport {
        let mut modifier = kr.modifier;
        modifier |= 1; //not sure bout that number
        KeyboardReport { modifier, ..kr }
    }

    // r_shift bit shifter
    fn r_shift(kr: KeyboardReport) -> KeyboardReport {
        let mut modifier = kr.modifier;
        modifier |= 32;
        KeyboardReport { modifier, ..kr }
    }

    // right GUI Bit Bit-shifter, same for GUI Key and with a less cool name
    fn r_gui(kr: KeyboardReport) -> KeyboardReport {
        let mut modifier = kr.modifier;
        modifier |= 128; //not sure bout that number
        KeyboardReport { modifier, ..kr }
    }

    //right ALT Bit-shifter
    fn r_alt(kr: KeyboardReport) -> KeyboardReport {
        let mut modifier = kr.modifier;
        modifier |= 64; //not sure bout that number
        KeyboardReport { modifier, ..kr }
    }

    //right ctrl Bit shifter
    fn r_ctrl(kr: KeyboardReport) -> KeyboardReport {
        let mut modifier = kr.modifier;
        modifier |= 16; //not sure bout that number
        KeyboardReport { modifier, ..kr }
    }


    // ☟☟☟ Simple Reports ☟☟☟

    // Gets you a simple Enter Report
    fn get_enter_report() -> KeyboardReport {
        KeyboardReport {
            keycodes: [0x28, 0, 0, 0, 0, 0],
            ..get_empty_report()
        }
    }

    //gives back an empty report
    fn get_empty_report() -> KeyboardReport {
        KeyboardReport {
            modifier: 0,
            reserved: 0,
            leds: 0,
            keycodes: [0, 0, 0, 0, 0, 0],
        }
    }

    // Makes a KeyboardReport out of a letter.
    // IMPORTANT: not all characters are supported at the moment, and I don't check if the character you
    // enter is. If it does not have a case for that character it just returns an empty report.
    fn get_letter_report(letter: char) -> KeyboardReport {
        //TODO: Stuff that shit into a Restult
        let mut modifier: u8 = 0;
        let n: u8 = match letter {
            c @ 'a'..='z' => c as u8 - b'a' + 4,
            c @ 'A'..='Z' => {
                modifier |= 2;
                c as u8 - b'A' + 4
            }
            d @ '1'..='9' => d as u8 - b'1' + 0x1E,
            '0' => 0x27,
            ' ' => 0x2C,
            _ => 0,
        };
        KeyboardReport {
            modifier,
            keycodes: [n, 0, 0, 0, 0, 0],
            ..get_empty_report()
        }
    }


    

    delay.delay_ms(1_000);

    let instruction_array = [
        Instruction {
            report: get_letter_report('a'),
            time_us: 0,
        },
        Instruction {
            report: get_letter_report(' '),
            time_us: 0,
        },
        Instruction {
            report: get_empty_report(),
            time_us: 3_000_000,
        }
    ];

    let min_time = 1_200;


      //======================//
     //== For all eternity ==//
    //======================//
    loop {
        for ins in instruction_array.as_slice(){
            push_keyboard_report(ins.report).ok().unwrap_or(0);
            delay.delay_us(if ins.time_us < min_time {min_time} else {ins.time_us} );
        }
    }


      //=================================//
     //== push and interrupt handlers ==//
    //=================================//
    /// We do this with interrupts disabled, to avoid a race hazard with the USB IRQ.
    fn push_keyboard_report(report: KeyboardReport) -> Result<usize, usb_device::UsbError> {
        critical_section::with(|_| unsafe {
            // Now interrupts are disabled, grab the global variable and, if
            // available, send it a HID report
            USB_HID.as_mut().map(|hid| hid.push_input(&report))
        })
        .unwrap()
    }

    /// This function is called whenever the USB Hardware generates an Interrupt
    /// Request.
    #[allow(non_snake_case)]
    #[interrupt]
    fn USBCTRL_IRQ() {
        unsafe {
            // Handle USB request
            let usb_dev = USB_DEVICE.as_mut().unwrap();
            let usb_hid = USB_HID.as_mut().unwrap();
            usb_dev.poll(&mut [usb_hid]);
        }
    }
}
// Des woas
