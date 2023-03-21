#![no_std]

use embedded_hal as hal;
use embedded_hal::digital::v2::OutputPin;
use hal::blocking::spi;
use stm32f1xx_hal::{spi::*};

pub struct Si4032<SPI, CS> {
    spi: SPI,
    cs: CS
}

impl<SPI, CS, E, PinError> Si4032<SPI, CS>
where
    SPI: spi::Transfer<u8, Error=E> + spi::Write<u8, Error=E>,
    CS: OutputPin<Error = PinError>
{
    pub fn new(spi:SPI, cs:CS) -> u8 {
        let mut radio = Si4032  {
            spi,
            cs};
        0
    }
}



fn write_register(reg: u8, data: u8) {}
fn read_register(reg: u8) -> u8 {0}

pub fn set_freq() {}

pub fn set_tx_pwr() {}

pub fn write_fifo() {}



pub fn get_device_status () {}

pub fn set_tx () {}

pub fn set_sync_wrd() {}

pub fn set_tx_header() {}

pub fn set_tx_packet_leng() {}

pub fn set_modulation_mode() {}



