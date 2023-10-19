use crate::clock;
//use crate::pac;
use atsamd21g as pac; //temp while writing
use crate::gpio::*;
use crate::time::Hertz;

use pac::i2s::clkctrl::SLOTSIZE_A as SlotSizeBits;
use pac::i2s::serctrl::CLKSEL_A as ClockUnitID;
use crate::typelevel::NoneT;


















// I2S Clock Units
pub struct ClockUnit0;
pub struct ClockUnit1;

pub trait ClockUnit {
    const ID: ClockUnitID;
}

impl ClockUnit for ClockUnit0 {
    const ID: ClockUnitID = ClockUnitID::CLK0;
}

impl ClockUnit for ClockUnit1 {
    const ID: ClockUnitID = ClockUnitID::CLK1;
}

trait MasterClock<ClockUnit> {
    fn freq(&self) -> Hertz;
}

impl MasterClock<ClockUnit0> for clock::I2S0Clock {
    fn freq(&self) -> Hertz {
        self.freq()
    }
}

impl MasterClock<ClockUnit1> for clock::I2S1Clock {
    fn freq(&self) -> Hertz {
        self.freq()
    }
}

// I2S Serializers
enum Serializer {
    M0,
    M1,
}

trait Srz0 {}
trait Srz1 {}

trait SimplexRX {
    const ID: Serializer;

    const RECEIVE_READY_MASK: u16;
    const RECEIVE_OVERRUN_MASK: u16;
}

trait SimplexTX {
    const ID: Serializer;

    const TRANSMIT_READY_MASK: u16;
    const TRANSMIT_UNDERRUN_MASK: u16;
}

struct TX0;
impl SimplexTX for TX0 {
    const ID: Serializer = Serializer::M0;

    const TRANSMIT_READY_MASK: u16 = 1 << 8;
    const TRANSMIT_UNDERRUN_MASK: u16 = 1 << 12;
}
impl Srz0 for TX0 {}

struct TX1;
impl SimplexTX for TX1 {
    const ID: Serializer = Serializer::M1;

    const TRANSMIT_READY_MASK: u16 = 1 << 9;
    const TRANSMIT_UNDERRUN_MASK: u16 = 1 << 13;
}
impl Srz1 for TX1 {}

struct RX0;
impl SimplexRX for RX0 {
    const ID: Serializer = Serializer::M0;

    const RECEIVE_READY_MASK: u16 = 1 << 0;
    const RECEIVE_OVERRUN_MASK: u16 = 1 << 4;
}
impl Srz0 for RX0 {}

struct RX1;
impl SimplexRX for TX1 {
    const ID: Serializer = Serializer::M1;

    const RECEIVE_READY_MASK: u16 = 1 << 1;
    const RECEIVE_OVERRUN_MASK: u16 = 1 << 5;
}
impl Srz1 for RX1 {}

impl Srz0 for NoneT {}
impl Srz1 for NoneT {}

struct SerializerConfig<SR0, SR1>
where
    SR0: Srz0,
    SR1: Srz1,
{
    serializer_0: SR0,
    serializer_1: SR1,
}

// GPIO Assignment
pub trait SerialClock<ClockUnit> {}

#[cfg(any(feature = "pins-d21e", feature = "pins-d21g", feature = "pins-d21j"))]
impl SerialClock<ClockUnit0> for Pin<PA10, AlternateG> {}
#[cfg(any(feature = "pins-d21g", feature = "pins-d21j"))]
impl SerialClock<ClockUnit0> for Pin<PA20, AlternateG> {}
#[cfg(any(feature = "pins-d21g", feature = "pins-d21j"))]
impl SerialClock<ClockUnit1> for Pin<PB11, AlternateG> {}

pub trait FrameSync<ClockUnit> {}

#[cfg(any(feature = "pins-d21e", feature = "pins-d21g", feature = "pins-d21j"))]
impl FrameSync<ClockUnit0> for Pin<PA11, AlternateG> {}
#[cfg(any(feature = "pins-d21g", feature = "pins-d21j"))]
impl FrameSync<ClockUnit0> for Pin<PA21, AlternateG> {}
#[cfg(feature = "pins-d21j")]
impl FrameSync<ClockUnit1> for Pin<PB12, AlternateG> {}


// Main Functionality
pub struct I2S<MC, SP, FP, RX, TX> {
    hw: pac::I2S,
    serial_clock_pin: SP,
    frame_sync_pin: FP,
    data_in_pin: RX,
    data_out_pin: TX,
    master_clock_source: MC,
}

impl<MC, SP, FP, RX, TX> I2S<MC, SP, FP, RX, TX> {
    pub fn i2s_host<CU: ClockUnit, F: Into<Hertz>> (
        hw: pac::I2S,
        pm: &mut pac::PM,
        master_clock_source: MC,
        serial_freq: F,
        serial_clock_pin: SP,
        frame_sync_pin: FP,
        data_out_pin: TX,
    ) -> Self
    where
        MC: MasterClock<CU>,
        SP: SerialClock<CU>,
        FP: FrameSync<CU>,
    {
        //do stuff here I suppose
    }
}