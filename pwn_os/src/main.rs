//! This is the Software running on the rp2040 itselve and is responsible for emulating the
//! keyboard.
#![no_std]
#![no_main]

use board::entry;
use defmt::*;
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
    info!("Program start");
    // mutable variable für unseren Peripheral Access Crate
    let mut pac = pac::Peripherals::take().unwrap();
    // Unser Wachhund doggy
    let mut doggy = Watchdog::new(pac.WATCHDOG);

    // Clock config
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut doggy,
    )
    .ok()
    .unwrap();

    //#[cfg(feature = "rp2040-e5")]
    //{
    // sio (single cycle IO block) Burschen ich hab auch keinen Plan was das ist
    let sio = Sio::new(pac.SIO);
    // Unsere werten Pins
    let pins = board::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    //}

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
    let usb_hid = HIDClass::new(bus_ref, KeyboardReport::desc(), 60);
    unsafe {
        // Also only safe cus interrupts r not started
        USB_HID = Some(usb_hid);
    }

    // Jetzt faken wir mal so ein Universelles Serielles Buss Geraet
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27da))
        .manufacturer("OurPC Inc.")
        .product("UrLastUSB")
        .serial_number("SorryNotSorry")
        .device_class(0) // Steht für "0 Plan wofür das steht"
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

    let mut led_pin = pins.led.into_push_pull_output();
    let delay_time: u32 = 10_000;

    loop {
        delay.delay_ms(10_000);
        led_pin.set_high().unwrap();
        delay.delay_ms(1_000);
        led_pin.set_low().unwrap();
        delay.delay_ms(1_000);
        push_keyboard_report(KeyboardReport {
            modifier: 0,
            leds: 0,
            reserved: 0,
            keycodes: [1, 0, 0, 0, 0, 0],
        })
        .ok()
        .unwrap_or(0);

        // delay.delay_ms(100);

        push_keyboard_report(KeyboardReport {
            modifier: 0,
            leds: 0,
            reserved: 0,
            keycodes: [2, 0, 0, 0, 0, 0],
        })
        .ok()
        .unwrap_or(0);
    }

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
    unsafe fn USBCTRL_IRQ() {
        // Handle USB request
        let usb_dev = USB_DEVICE.as_mut().unwrap();
        let usb_hid = USB_HID.as_mut().unwrap();
        usb_dev.poll(&mut [usb_hid]);
    }
}
// Des woas
