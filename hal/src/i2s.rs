use crate::clock;
#[cfg(feature = "dma")]
use crate::dmac::Buffer;
use crate::gpio::*;
use crate::pac;
use crate::time::Hertz;
use crate::typelevel::NoneT;

use core::convert::From;
use core::marker::PhantomData;
use core::ops::DerefMut;
use embedded_hal::can::Frame;

#[cfg(all(feature = "samd21", feature = "dma"))]
pub use pac::dmac::chctrlb::TRIGSRC_A as DmaTriggerSource;

// TODO how about E51 variants?
#[cfg(all(feature = "samd51", feature = "dma"))]
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

#[cfg(any(feature = "pins-d21e", feature = "pins-d21g", feature = "pins-d21j"))]
impl MasterClock<ClockUnit0> for ExternalClock<Pin<PA09, AlternateG>> {
    fn freq(&self) -> Hertz {
        self.frequency
    }
}

#[cfg(feature = "pins-d21j")]
impl MasterClock<ClockUnit0> for ExternalClock<Pin<PB17, AlternateG>> {
    fn freq(&self) -> Hertz {
        self.frequency
    }
}

#[cfg(any(feature = "pins-d21g", feature = "pins-d21j"))]
impl MasterClock<ClockUnit1> for ExternalClock<Pin<PB10, AlternateG>> {
    fn freq(&self) -> Hertz {
        self.frequency
    }
}

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

#[cfg(any(feature = "pins-d21e", feature = "pins-d21g", feature = "pins-d21j"))]
impl SerializerTx<Tx0Rx1> for Pin<PA07, AlternateG> {}
#[cfg(any(feature = "pins-d21e", feature = "pins-d21g", feature = "pins-d21j"))]
impl SerializerTx<Tx1Rx0> for Pin<PA08, AlternateG> {}
#[cfg(any(feature = "pins-d21e", feature = "pins-d21g", feature = "pins-d21j"))]
impl SerializerTx<Tx0Rx1> for Pin<PA19, AlternateG> {}
#[cfg(feature = "pins-d21j")]
impl SerializerTx<Tx1Rx0> for Pin<PB16, AlternateG> {}

pub trait SerializerRx<SerializerOrientation> {}
#[cfg(any(feature = "pins-d21e", feature = "pins-d21g", feature = "pins-d21j"))]
impl SerializerRx<Tx1Rx0> for Pin<PA07, AlternateG> {}
#[cfg(any(feature = "pins-d21e", feature = "pins-d21g", feature = "pins-d21j"))]
impl SerializerRx<Tx0Rx1> for Pin<PA08, AlternateG> {}
#[cfg(any(feature = "pins-d21e", feature = "pins-d21g", feature = "pins-d21j"))]
impl SerializerRx<Tx1Rx0> for Pin<PA19, AlternateG> {}
#[cfg(feature = "pins-d21j")]
impl SerializerRx<Tx0Rx1> for Pin<PB16, AlternateG> {}

// no RX pin for I2S config
impl SerializerRx<Tx0Rx1> for NoneT {}
impl SerializerRx<Tx1Rx0> for NoneT {}

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

pub struct I2s<MCS, SC_P, FS_P, RX_P, TX_P> {
    hw: pac::I2S,
    serial_clock_pin: SC_P,
    frame_sync_pin: FS_P,
    data_in_pin: RX_P,
    data_out_pin: TX_P,
    master_clock_source: MCS,
}

// Need to support three clocking configurations:
//   gclck master clock (frequency) => serial clock is output (pin+frequency)
//   external master clock (pin+frequency) => serial clock is output (pin+frequency)
//   No master clock => serial clock is input (pin)
impl<MCS, SC_P, FS_P, RX_P, TX_P> I2s<MCS, SC_P, FS_P, RX_P, TX_P> {

    /// master_clock_source, serial_clock_pin, and frame_sync_pin must be attached to the same clock unit
    /// TxPin and RxPin need to be connected to different serializer units
    pub fn tdm_master<CU: ClockUnit, SC: SerializerOrientation, F: Into<Hertz>>(
        hw: pac::I2S,
        pm: &mut pac::PM,
        master_clock_source: MCS,
        serial_freq: F,
        number_of_slots: u8,
        bits_per_slot: BitsPerSlot,
        serial_clock_pin: SC_P,
        frame_sync_pin: FS_P,
        data_in_pin: RX_P,
        data_out_pin: TX_P,
    ) -> Self
    where
        MCS: MasterClock<CU>,
        SC_P: SerialClock<CU>,
        FS_P: FrameSync<CU>,
        RX_P: SerializerRx<SC>,
        TX_P: SerializerTx<SC>,
    {
        // Turn on the APB clock to the I2S peripheral
        pm.apbcmask.modify(|_, w| w.i2s_().set_bit());

        Self::reset(&hw);

        // defmt::info!("Master clock running at {:?}", master_clock_source.freq().0);

        let master_clock_divisor = (master_clock_source.freq() / serial_freq.into() - 1) as u8;
        // defmt::info!("divisor is {:?}", master_clock_divisor);

        // unsafe is due to the bits() calls
        unsafe {
            hw.clkctrl[CU::ID as usize].write(|w| {
                w.mckdiv()
                    .bits(master_clock_divisor)
                    // .mcksel().mckpin() // Use MCK pin as master clock input
                    // .scksel().sckpin() // Uses SCK pin as input
                    .sckoutinv()
                    .set_bit() // Invert serial clock
                    .bitdelay()
                    .i2s() // 1-bit delay between fsync and data
                    .fswidth()
                    .bit_()
                    .nbslots()
                    .bits(number_of_slots - 1)
                    .slotsize()
                    .variant(bits_per_slot)
            });
        }

        hw.serctrl[SC::RX_ID as usize]
            .write(|w| w.clksel().variant(CU::ID));

        hw.serctrl[SC::TX_ID as usize]
            .write(|w| w.clksel().variant(CU::ID).sermode().tx());

        // Synchronization doesn't seem to happen until the peripheral is enabled

        match CU::ID {
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



    pub fn i2s_master<CU: ClockUnit, SC: SerializerOrientation, F: Into<Hertz>> (
        hw: pac::I2S,
        pm: &mut pac::PM,
        master_clock_source: MCS,
        sample_rate: F,
        bit_depth: BitsPerSlot,
        sck_pin: SC_P,
        fs_pin: FS_P,
        tx_pin: TX_P,
        rx_pin: RX_P,
    ) -> Self
    where
        MCS: MasterClock<CU>,
        SC_P: SerialClock<CU>,
        FS_P: FrameSync<CU>,
        TX_P: SerializerTx<SC>,
        RX_P: SerializerRx<SC>,
    {
        //pseudo rx_pin for I2S object
        //let rx_pin: RX_P = NoneT;

        // enable APB clock path via the power manager
        pm.apbcmask.modify(|_, w | w.i2s_().set_bit());

        Self::reset(&hw);

        // divisor is simple ratio between source clock and target frequency
        let sck_freq = sample_rate.into() * (((bit_depth as u32) + 1) * 8);
        let m_clk_div: u8 = (master_clock_source.freq() / sck_freq) as u8;

        // datasize is dependent on bitdepth
        let data_size: u8 = match bit_depth.into() {
            2 => 0x01,
            _ => 0x04,
        };

        // setting bits in clock control register for I2S mode
        unsafe {
            hw.clkctrl[CU::ID as usize].write(|w | {
                w
                    .mckdiv().bits(m_clk_div)
                    .slotsize().variant(bit_depth)
                    .nbslots().bits(0x01)
                    .bitdelay().i2s()
                    .fswidth().bits(0x01)
            });
        }

        //configure the serializer (only one for I2S tx)
        unsafe {
            hw.serctrl[SC::TX_ID as usize].write(|w| {
                w
                    .clksel().variant(CU::ID)
                    .sermode().tx()
                    .datasize().bits(data_size)
            });
        }

        // enabling serializer + clock units
        match SC::TX_ID {
            Serializer::M0 => {
                hw.ctrla.modify(|_, w | w.seren0().set_bit());
            }

            Serializer::M1 => {
                hw.ctrla.modify(|_, w | w.seren1().set_bit());
            }
        }

        match CU::ID {
            ClockUnitID::CLK0 => {
                hw.ctrla.modify(|_, w | w.cken0().set_bit());
            }

            ClockUnitID::CLK1 => {
                hw.ctrla.modify(|_, w | w.cken1().set_bit());
            }
        }

        Self {
            hw,
            serial_clock_pin: sck_pin.into(),
            frame_sync_pin: fs_pin.into(),
            data_in_pin: rx_pin.into(),
            data_out_pin: tx_pin.into(),
            master_clock_source,
        }
    }



    pub fn send<SerializerCfg: SerializerOrientation>(&self, v: u32) -> Result<()>
    where
        RX_P: SerializerRx<SerializerCfg>,
        TX_P: SerializerTx<SerializerCfg>,
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
        SC_P,
        FS_P,
        RX_P,
        TX_P,
        MCS,
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
        RX_P: SerializerRx<SerializerCfg>,
        TX_P: SerializerTx<SerializerCfg>,
    {
        unsafe {
            self.hw
                .intenset
                .write(|w| w.bits(SerializerCfg::RECEIVE_READY_MASK));
        }
    }

    pub fn enable_receive_overrun_interrupt<SerializerCfg: SerializerOrientation>(&self)
    where
        RX_P: SerializerRx<SerializerCfg>,
        TX_P: SerializerTx<SerializerCfg>,
    {
        unsafe {
            self.hw
                .intenset
                .write(|w| w.bits(SerializerCfg::RECEIVE_OVERRUN_MASK));
        }
    }

    pub fn enable_transmit_ready_interrupt<SerializerCfg: SerializerOrientation>(&self)
    where
        RX_P: SerializerRx<SerializerCfg>,
        TX_P: SerializerTx<SerializerCfg>,
    {
        unsafe {
            self.hw
                .intenset
                .write(|w| w.bits(SerializerCfg::TRANSMIT_READY_MASK));
        }
    }
    pub fn enable_transmit_underrun_interrupt<SerializerCfg: SerializerOrientation>(&self)
    where
        RX_P: SerializerRx<SerializerCfg>,
        TX_P: SerializerTx<SerializerCfg>,
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
        RX_P: SerializerRx<SerializerCfg>,
        TX_P: SerializerTx<SerializerCfg>,
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
        RX_P: SerializerRx<SerializerCfg>,
        TX_P: SerializerTx<SerializerCfg>,
    {
        I2sDmaBuffer(&self.hw.data[SerializerCfg::TX_ID as usize] as *const _ as *mut u32)
    }

    #[cfg(feature = "dma")]
    pub fn transmit_dma_trigger<SerializerCfg: SerializerOrientation>(&self) -> DmaTriggerSource
    where
        RX_P: SerializerRx<SerializerCfg>,
        TX_P: SerializerTx<SerializerCfg>,
    {
        match SerializerCfg::TX_ID {
            Serializer::M0 => DmaTriggerSource::I2S_TX_0,
            Serializer::M1 => DmaTriggerSource::I2S_TX_1,
        }
    }

    #[cfg(feature = "dma")]
    pub fn receive_dma_buffer<SerializerCfg: SerializerOrientation>(&self) -> I2sDmaBuffer
    where
        RX_P: SerializerRx<SerializerCfg>,
        TX_P: SerializerTx<SerializerCfg>,
    {
        I2sDmaBuffer(&self.hw.data[SerializerCfg::RX_ID as usize] as *const _ as *mut u32)
    }

    #[cfg(feature = "dma")]
    pub fn receive_dma_trigger<SerializerCfg: SerializerOrientation>(&self) -> DmaTriggerSource
    where
        RX_P: SerializerRx<SerializerCfg>,
        TX_P: SerializerTx<SerializerCfg>,
    {
        match SerializerCfg::RX_ID {
            Serializer::M0 => DmaTriggerSource::I2S_RX_0,
            Serializer::M1 => DmaTriggerSource::I2S_RX_1,
        }
    }
}
