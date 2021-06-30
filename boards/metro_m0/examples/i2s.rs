#![no_std]
#![no_main]

#[cfg(not(feature = "use_semihosting"))]
use panic_halt as _;
#[cfg(feature = "use_semihosting")]
use panic_semihosting as _;

use bsp::hal;
use bsp::pac;
use metro_m0 as bsp;

use bsp::entry;
use hal::clock::GenericClockController;
use hal::gpio::v2::AlternateG;
use hal::i2s::{
    self,
    I2s,
};
use hal::time::KiloHertz;
use pac::Peripherals;

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );
    let pins = bsp::Pins::new(peripherals.PORT);

    let gclk0 = clocks.gclk0();

    let _tdm = I2s::tdm_master(
        peripherals.I2S,
        &mut peripherals.PM,
        clocks.i2s0(&gclk0).unwrap(),
        KiloHertz(2048), // Serial clock frequency
        8,  // Number of slots
        i2s::BitsPerSlot::_32,
        pins.d6.into_mode::<AlternateG>(), // Serial Clock (labelled tx)
        pins.d7.into_mode::<AlternateG>(), // Frame Sync (labelled rx)
        pins.d4.into_mode::<AlternateG>(), // Data in
        pins.d9.into_mode::<AlternateG>(), // Data out

        // I2S   SAMD21  Metro M0 BSP
        // -----------------
        // SD0  | PA07 | d9
        // SD1  | PA08 | d4
        // MCK0 | PA09 | d3
        // SCK0 | PA10 | d1
        // MCK1 | PB10 | mosi (SPI header)
        // SCK1 | PB11 | sck (SPI header)
        // SD0  | PA19 | d12
        // SCK0 | PA20 | d6
        // FS0  | PA21 | d7
        // FS0  | PA11 | n/a
        // FS1  | PB12 | n/a
        // SD1  | PB16 | n/a
        // MCK0 | PB17 | n/a
    );

    loop {

    }
}
