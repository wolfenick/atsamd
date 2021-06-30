use crate::clock;
#[cfg(feature = "dma")]
use crate::dmac::Buffer;
use crate::gpio::v2::*;
use crate::pac;
use crate::time::Hertz;

use core::convert::From;
use core::marker::PhantomData;

#[cfg(all(feature = "samd21", feature = "dma"))]
pub use pac::dmac::chctrlb::TRIGSRC_A as DmaTriggerSource;

#[cfg(all(feature = "min-samd51g", feature = "dma"))]
pub use pac::dmac::chctrla::TRIGSRC_A as DmaTriggerSource;

pub use pac::i2s::clkctrl::SLOTSIZE_A as BitsPerSlot;
use pac::i2s::serctrl::CLKSEL_A as ClockUnitID;

// TODO This probably belongs in an I2S trait crate?
// TODO nb::WouldBlock, assuming there aren't other I2SError enums needed
#[derive(Debug)]
pub enum I2SError {
    /// An operation would block because the device is currently busy or there is no data available.
    WouldBlock,
}

/// Result for I2S operations.
pub type Result<T> = core::result::Result<T, I2SError>;

pub struct ClockUnit0;
pub struct ClockUnit1;

/// Allows compile-time associations between pins and clock units
pub trait ClockUnit {
    const ID: ClockUnitID;
}

impl ClockUnit for ClockUnit0 {
    const ID: ClockUnitID = ClockUnitID::CLK0;
}

impl ClockUnit for ClockUnit1 {
    const ID: ClockUnitID = ClockUnitID::CLK1;
}

// TODO perhaps have something like this in gpio?
pub struct ExternalClock<PinType> {
    frequency: Hertz,
    pin: PhantomData<PinType>,
}

pub trait MasterClock<ClockUnit> {
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

#[cfg(feature = "samd21")]
impl MasterClock<ClockUnit0> for ExternalClock<Pin<PA09, AlternateG>> {
    fn freq(&self) -> Hertz {
        self.frequency
    }
}

#[cfg(feature = "min-samd21j")]
impl MasterClock<ClockUnit0> for ExternalClock<Pin<PB17, AlternateG>> {
    fn freq(&self) -> Hertz {
        self.frequency
    }
}

#[cfg(feature = "min-samd21g")]
impl MasterClock<ClockUnit1> for ExternalClock<Pin<PB10, AlternateG>> {
    fn freq(&self) -> Hertz {
        self.frequency
    }
}

pub trait SerialClock<ClockUnit> {}

#[cfg(feature = "samd21")]
impl SerialClock<ClockUnit0> for Pin<PA10, AlternateG> {}
#[cfg(feature = "min-samd21g")]
impl SerialClock<ClockUnit0> for Pin<PA20, AlternateG> {}
#[cfg(feature = "min-samd21g")]
impl SerialClock<ClockUnit1> for Pin<PB11, AlternateG> {}

pub trait FrameSync<ClockUnit> {}

#[cfg(feature = "samd21")]
impl FrameSync<ClockUnit0> for Pin<PA11, AlternateG> {}
#[cfg(feature = "min-samd21g")]
impl FrameSync<ClockUnit0> for Pin<PA21, AlternateG> {}
#[cfg(feature = "min-samd21j")]
impl FrameSync<ClockUnit1> for Pin<PB12, AlternateG> {}

/// The I2S peripheral has two serializers; refer to them using this enum
pub enum Serializer {
    M0, // 'm' is the datasheet convention
    M1,
}

/// The I2S peripheral has two serializers, each can be used as either an input or an output.  The
/// SerializerOrientation trait is used to indicate which serializer is used for each direction.
pub trait SerializerOrientation {
    const TX_ID: Serializer;
    const RX_ID: Serializer;

    // Masks are for the interrupt registers
    const RECEIVE_READY_MASK: u16;
    const RECEIVE_OVERRUN_MASK: u16;
    const TRANSMIT_READY_MASK: u16;
    const TRANSMIT_UNDERRUN_MASK: u16;
}

/// Transmit from serializer 0, receive on serializer 1
pub struct Tx0Rx1;
impl SerializerOrientation for Tx0Rx1 {
    const TX_ID: Serializer = Serializer::M0;
    const RX_ID: Serializer = Serializer::M1;

    const RECEIVE_READY_MASK: u16 = 1 << 1;
    const RECEIVE_OVERRUN_MASK: u16 = 1 << 5;
    const TRANSMIT_READY_MASK: u16 = 1 << 8;
    const TRANSMIT_UNDERRUN_MASK: u16 = 1 << 12;
}

/// Transmit from serializer 1, receive on serializer 0
pub struct Tx1Rx0;
impl SerializerOrientation for Tx1Rx0 {
    const TX_ID: Serializer = Serializer::M1;
    const RX_ID: Serializer = Serializer::M0;

    const RECEIVE_READY_MASK: u16 = 1 << 0;
    const RECEIVE_OVERRUN_MASK: u16 = 1 << 4;
    const TRANSMIT_READY_MASK: u16 = 1 << 9;
    const TRANSMIT_UNDERRUN_MASK: u16 = 1 << 13;
}

// TODO make these optional, in particular the Tx one to support PDM mics
pub trait SerializerTx<SerializerOrientation> {}

#[cfg(feature = "samd21")]
impl SerializerTx<Tx0Rx1> for Pin<PA07, AlternateG> {}
#[cfg(feature = "samd21")]
impl SerializerTx<Tx1Rx0> for Pin<PA08, AlternateG> {}
#[cfg(feature = "samd21")]
impl SerializerTx<Tx0Rx1> for Pin<PA19, AlternateG> {}
#[cfg(feature = "min-samd21j")]
impl SerializerTx<Tx1Rx0> for Pin<PB16, AlternateG> {}

pub trait SerializerRx<SerializerOrientation> {}
#[cfg(feature = "samd21")]
impl SerializerRx<Tx1Rx0> for Pin<PA07, AlternateG> {}
#[cfg(feature = "samd21")]
impl SerializerRx<Tx0Rx1> for Pin<PA08, AlternateG> {}
#[cfg(feature = "samd21")]
impl SerializerRx<Tx1Rx0> for Pin<PA19, AlternateG> {}
#[cfg(feature = "min-samd21j")]
impl SerializerRx<Tx0Rx1> for Pin<PB16, AlternateG> {}

pub struct InterruptMask<T> {
    mask: u16,
    phantom: PhantomData<T>,
}

impl<T> From<u16> for InterruptMask<T> {
    fn from(mask: u16) -> InterruptMask<T> {
        InterruptMask {
            mask,
            phantom: PhantomData,
        }
    }
}

impl<T: SerializerOrientation> InterruptMask<T> {
    pub fn receive_ready(&self) -> bool {
        self.mask & T::RECEIVE_READY_MASK != 0
    }

    pub fn receive_overrrun(&self) -> bool {
        self.mask & T::RECEIVE_OVERRUN_MASK != 0
    }

    pub fn transmit_ready(&self) -> bool {
        self.mask & T::TRANSMIT_READY_MASK != 0
    }

    pub fn transmit_underrun(&self) -> bool {
        self.mask & T::TRANSMIT_UNDERRUN_MASK != 0
    }
}

#[cfg(feature = "dma")]
#[derive(Clone, Copy)]
pub struct I2sDmaBuffer(*mut u32);

#[cfg(feature = "dma")]
unsafe impl Buffer for I2sDmaBuffer {
    type Beat = u32;

    fn dma_ptr(&mut self) -> *mut Self::Beat {
        self.0
    }

    fn incrementing(&self) -> bool {
        false
    }

    fn buffer_len(&self) -> usize {
        1
    }
}

pub struct I2s<MasterClockSource, SerialClockPin, FrameSyncPin, RxPin, TxPin> {
    hw: pac::I2S,
    serial_clock_pin: SerialClockPin,
    frame_sync_pin: FrameSyncPin,
    data_in_pin: RxPin,
    data_out_pin: TxPin,
    master_clock_source: MasterClockSource,
}

// Need to support three clocking configurations:
//   gclck master clock (frequency) => serial clock is output (pin+frequency)
//   external master clock (pin+frequency) => serial clock is output (pin+frequency)
//   No master clock => serial clock is input (pin)

impl<MasterClockSource, SerialClockPin, FrameSyncPin, RxPin, TxPin>
    I2s<MasterClockSource, SerialClockPin, FrameSyncPin, RxPin, TxPin>
{
    /// master_clock_source, serial_clock_pin, and frame_sync_pin must be attached to the same clock unit
    /// TxPin and RxPin need to be connected to different serializer units
    pub fn tdm_master<ClkUnit: ClockUnit, SerializerCfg: SerializerOrientation, Freq: Into<Hertz>>(
        hw: pac::I2S,
        pm: &mut pac::PM,
        master_clock_source: MasterClockSource,
        serial_freq: Freq,
        number_of_slots: u8,
        bits_per_slot: BitsPerSlot,
        serial_clock_pin: SerialClockPin,
        frame_sync_pin: FrameSyncPin,
        data_in_pin: RxPin,
        data_out_pin: TxPin,
    ) -> Self
    where
        MasterClockSource: MasterClock<ClkUnit>,
        SerialClockPin: SerialClock<ClkUnit>,
        FrameSyncPin: FrameSync<ClkUnit>,
        RxPin: SerializerRx<SerializerCfg>,
        TxPin: SerializerTx<SerializerCfg>,
    {
        // Turn on the APB clock to the I2S peripheral
        pm.apbcmask.modify(|_, w| w.i2s_().set_bit());

        Self::reset(&hw);

        // defmt::info!("Master clock running at {:?}", master_clock_source.freq().0);

        let master_clock_divisor = (master_clock_source.freq().0 / serial_freq.into().0 - 1) as u8;
        // defmt::info!("divisor is {:?}", master_clock_divisor);

        // unsafe is due to the bits() calls
        unsafe {
            hw.clkctrl[ClkUnit::ID as usize].write(|w| {
                w
                    .mckdiv().bits(master_clock_divisor)
                    // .mcksel().mckpin() // Use MCK pin as master clock input
                    // .scksel().sckpin() // Uses SCK pin as input
                    .sckoutinv().set_bit() // Invert serial clock
                    .bitdelay().i2s() // 1-bit delay between fsync and data
                    .fswidth().bit_()
                    .nbslots().bits(number_of_slots - 1)
                    .slotsize().variant(bits_per_slot)
            });
        }

        hw.serctrl[SerializerCfg::RX_ID as usize].write(|w| w.clksel().variant(ClkUnit::ID));

        hw.serctrl[SerializerCfg::TX_ID as usize]
            .write(|w| w.clksel().variant(ClkUnit::ID).sermode().tx());

        // Synchronization doesn't seem to happen until the peripheral is enabled

        match ClkUnit::ID {
            ClockUnitID::CLK0 => {
                hw.ctrla.modify(|_, w| w.cken0().set_bit());
            }

            ClockUnitID::CLK1 => {
                hw.ctrla.modify(|_, w| w.cken1().set_bit());
            }
        }

        hw.ctrla
            .modify(|_, w| w.seren0().set_bit().seren1().set_bit());

        Self {
            hw,
            serial_clock_pin,
            frame_sync_pin: frame_sync_pin.into(),
            data_in_pin: data_in_pin.into(),
            data_out_pin: data_out_pin.into(),
            master_clock_source,
        }
    }

    pub fn send<SerializerCfg: SerializerOrientation>(&self, v: u32) -> Result<()>
    where
        RxPin: SerializerRx<SerializerCfg>,
        TxPin: SerializerTx<SerializerCfg>,
    {
        match SerializerCfg::TX_ID {
            Serializer::M0 => {
                if self.hw.intflag.read().txrdy0().bit_is_clear() {
                    return Err(I2SError::WouldBlock);
                }
                unsafe {
                    self.hw.data[0].write(|reg| reg.data().bits(v));
                }

                while self.hw.syncbusy.read().data0().bit_is_set() {}
            }

            Serializer::M1 => {
                if self.hw.intflag.read().txrdy1().bit_is_clear() {
                    return Err(I2SError::WouldBlock);
                }
                unsafe {
                    self.hw.data[1].write(|reg| reg.data().bits(v));
                }

                while self.hw.syncbusy.read().data1().bit_is_set() {}
            }
        }

        Ok(())
    }

    /// Gives the peripheral and pins back
    pub fn free(
        self,
    ) -> (
        pac::I2S,
        SerialClockPin,
        FrameSyncPin,
        RxPin,
        TxPin,
        MasterClockSource,
    ) {
        (
            self.hw,
            self.serial_clock_pin,
            self.frame_sync_pin,
            self.data_in_pin,
            self.data_out_pin,
            self.master_clock_source,
        )
    }

    /// Blocking software reset of the peripheral
    fn reset(hw: &pac::I2S) {
        hw.ctrla.write(|w| w.swrst().set_bit());

        while hw.syncbusy.read().swrst().bit_is_set() || hw.ctrla.read().swrst().bit_is_set() {}
    }

    /// Enable the peripheral
    pub fn enable(&self) {
        self.hw.ctrla.modify(|_, w| w.enable().set_bit());

        while self.hw.syncbusy.read().cken0().bit_is_set()
            || self.hw.syncbusy.read().cken1().bit_is_set()
            || self.hw.syncbusy.read().seren0().bit_is_set()
            || self.hw.syncbusy.read().seren1().bit_is_set()
            || self.hw.syncbusy.read().enable().bit_is_set()
        {}
    }

    pub fn enable_receive_ready_interrupt<SerializerCfg: SerializerOrientation>(&self)
    where
        RxPin: SerializerRx<SerializerCfg>,
        TxPin: SerializerTx<SerializerCfg>,
    {
        unsafe {
            self.hw
                .intenset
                .write(|w| w.bits(SerializerCfg::RECEIVE_READY_MASK));
        }
    }

    pub fn enable_receive_overrun_interrupt<SerializerCfg: SerializerOrientation>(&self)
    where
        RxPin: SerializerRx<SerializerCfg>,
        TxPin: SerializerTx<SerializerCfg>,
    {
        unsafe {
            self.hw
                .intenset
                .write(|w| w.bits(SerializerCfg::RECEIVE_OVERRUN_MASK));
        }
    }

    pub fn enable_transmit_ready_interrupt<SerializerCfg: SerializerOrientation>(&self)
    where
        RxPin: SerializerRx<SerializerCfg>,
        TxPin: SerializerTx<SerializerCfg>,
    {
        unsafe {
            self.hw
                .intenset
                .write(|w| w.bits(SerializerCfg::TRANSMIT_READY_MASK));
        }
    }
    pub fn enable_transmit_underrun_interrupt<SerializerCfg: SerializerOrientation>(&self)
    where
        RxPin: SerializerRx<SerializerCfg>,
        TxPin: SerializerTx<SerializerCfg>,
    {
        unsafe {
            self.hw
                .intenset
                .write(|w| w.bits(SerializerCfg::TRANSMIT_UNDERRUN_MASK));
        }
    }

    pub fn get_and_clear_interrupts<SerializerCfg: SerializerOrientation>(
        &self,
    ) -> InterruptMask<SerializerCfg>
    where
        RxPin: SerializerRx<SerializerCfg>,
        TxPin: SerializerTx<SerializerCfg>,
    {
        let ints = self.hw.intflag.read().bits();
        unsafe {
            self.hw.intflag.write(|w| w.bits(ints));
        }
        InterruptMask::from(ints)
    }

    #[cfg(feature = "dma")]
    pub fn transmit_dma_buffer<SerializerCfg: SerializerOrientation>(&self) -> I2sDmaBuffer
    where
        RxPin: SerializerRx<SerializerCfg>,
        TxPin: SerializerTx<SerializerCfg>,
    {
        I2sDmaBuffer(&self.hw.data[SerializerCfg::TX_ID as usize] as *const _ as *mut u32)
    }

    #[cfg(feature = "dma")]
    pub fn transmit_dma_trigger<SerializerCfg: SerializerOrientation>(&self) -> DmaTriggerSource
    where
        RxPin: SerializerRx<SerializerCfg>,
        TxPin: SerializerTx<SerializerCfg>,
    {
        match SerializerCfg::TX_ID {
            Serializer::M0 => DmaTriggerSource::I2S_TX_0,
            Serializer::M1 => DmaTriggerSource::I2S_TX_1,
        }
    }

    #[cfg(feature = "dma")]
    pub fn receive_dma_buffer<SerializerCfg: SerializerOrientation>(&self) -> I2sDmaBuffer
    where
        RxPin: SerializerRx<SerializerCfg>,
        TxPin: SerializerTx<SerializerCfg>,
    {
        I2sDmaBuffer(&self.hw.data[SerializerCfg::RX_ID as usize] as *const _ as *mut u32)
    }

    #[cfg(feature = "dma")]
    pub fn receive_dma_trigger<SerializerCfg: SerializerOrientation>(&self) -> DmaTriggerSource
    where
        RxPin: SerializerRx<SerializerCfg>,
        TxPin: SerializerTx<SerializerCfg>,
    {
        match SerializerCfg::RX_ID {
            Serializer::M0 => DmaTriggerSource::I2S_RX_0,
            Serializer::M1 => DmaTriggerSource::I2S_RX_1,
        }
    }
}
