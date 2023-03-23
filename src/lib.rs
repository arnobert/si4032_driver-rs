#![no_std]

mod registers;
use embedded_hal as hal;
use embedded_hal::digital::v2::OutputPin;
use hal::blocking::spi;
use stm32f1xx_hal::{spi::*};
use crate::registers::Registers;

#[derive(Debug, Copy, Clone)]
pub struct Si4032<SPI, CS> {
    spi: SPI,
    cs: CS
}

impl<SPI, CS, E, PinError> Si4032<SPI, CS>
where
    SPI: spi::Transfer<u8, Error=E> + spi::Write<u8, Error=E>,
    CS: OutputPin<Error = PinError>
{
    pub fn new(spi:SPI, cs:CS) -> Si4032<SPI, CS> {
        let mut radio = Si4032  {
            spi,
            cs};
    radio
    }

    fn write_register(&mut self, reg: u8, data: u8) {
        self.cs.set_low();
        let wrdata = [reg | (0x01<<7), data];
        self.spi.write(&wrdata);
        self.cs.set_high();
    }

    fn read_register(&mut self, reg: u8) -> u8 {
        self.cs.set_low();
        let mut rx_buy= [reg, 0];
        self.spi.transfer(&mut rx_buy);
        self.cs.set_high();
        rx_buy[1]
        }

    pub fn enter_standby(&mut self) {
        self.write_register(Registers::OP_FUN_CTRL_1.addr(), 0x09);
    }

    pub fn set_freq(&mut self, f_upper: u8, f_lower: u8) {
        self.write_register(Registers::CAR_FREQ_1.addr(), f_upper);
        self.write_register(Registers::CAR_FREQ_0.addr(), f_lower);
    }


    pub fn get_freq(&mut self) -> [u8; 2] {
        let mut rx_buf: [u8; 2] = [0, 0];
        rx_buf[1] = self.read_register(Registers::CAR_FREQ_1.addr());
        rx_buf[0] = self.read_register(Registers::CAR_FREQ_0.addr());
        rx_buf
    }


    pub fn set_tx_pwr(&mut self, power: u8) {
        self.write_register(Registers::TX_PWR.addr(), power);
    }

    pub fn write_fifo() {}

    pub fn get_device_status () {}

    pub fn set_tx () {}

    pub fn set_sync_wrd() {}

    pub fn set_tx_header() {}

    pub fn set_tx_packet_leng() {}

    pub fn set_modulation_mode() {}
}




