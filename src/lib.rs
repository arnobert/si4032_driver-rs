#![no_std]

mod registers;

use embedded_hal as hal;
use embedded_hal::digital::v2::OutputPin;
use hal::blocking::spi;
use stm32f1xx_hal::{spi::*};
use crate::registers::Registers;

#[repr(u8)]
pub enum mod_type {
    umod_car = 0x00,
    ook = 0x01,
    fsk = 0x02,
    gfsk = 0x03,
}

#[repr(u8)]
pub enum mod_data_src {
    direct_gpio = 0x00,
    direct_sdi = 0x01,
    fifo = 0x02,
    pn9 = 0x03,
}


// TX POWER:
#[repr(u8)]
pub enum e_tx_power {
    p_1dBm = 0x0,
    p_2dBm = 0x1,
    p_5dBm = 0x2,
    p_8dBm = 0x3,
    p_11dBm = 0x4,
    p_14dBm = 0x5,
    p_17dBm = 0x6,
    p_20dBm = 0x7,
}


#[derive(Debug, Copy, Clone)]
pub struct Si4032<SPI, CS> {
    spi: SPI,
    cs: CS,
}

impl<SPI, CS, E, PinError> Si4032<SPI, CS>
    where
        SPI: spi::Transfer<u8, Error=E> + spi::Write<u8, Error=E>,
        CS: OutputPin<Error=PinError>
{
    pub fn new(spi: SPI, cs: CS) -> Si4032<SPI, CS> {
        let mut radio = Si4032 {
            spi,
            cs,
        };
        radio
    }

    fn write_register(&mut self, reg: Registers, data: u8) {
        self.cs.set_low();
        let wrdata = [reg as u8 | (0x01 << 7), data];
        self.spi.write(&wrdata);
        self.cs.set_high();
    }

    /*
    Burst write mode
    Example:
    let dt = [0, 1, 3, 5];
    self.burst_write_register(Registers::OP_FUN_CTRL_1.addr(), &dt);
     */
    fn burst_write_register(&mut self, reg: Registers, data: &[u8]) {
        self.cs.set_low();
        let x_reg = reg as u8 | (0x01 << 7);
        self.spi.write(&[x_reg]);
        self.spi.write(data);
        self.cs.set_high();
    }

    fn read_register(&mut self, reg: Registers) -> u8 {
        self.cs.set_low();
        let mut rx_buy = [reg as u8, 0];
        self.spi.transfer(&mut rx_buy);
        self.cs.set_high();
        rx_buy[1]
    }

    // Set operating modes -------------------------------------------------------------------------
    // SHUTDOWN is not available for pin 20 is hardwired to gnd.

    pub fn enter_standby(&mut self) {
        self.write_register(Registers::OP_FUN_CTRL_1, 0x00);
    }

    pub fn enter_sleep(&mut self) {
        let reg_07 = self.read_register(Registers::OP_FUN_CTRL_1);
        self.write_register(Registers::OP_FUN_CTRL_1, 0x40); //enlbd bit
    }

    pub fn enter_ready(&mut self) {
        let reg_07 = self.read_register(Registers::OP_FUN_CTRL_1);
        self.write_register(Registers::OP_FUN_CTRL_1, reg_07 | 1); //xton bit
    }

    pub fn enter_tune(&mut self) {
        let reg_07 = self.read_register(Registers::OP_FUN_CTRL_1);
        self.write_register(Registers::OP_FUN_CTRL_1, reg_07 | (1 << 1)); //pllon bit
    }

    pub fn enter_tx(&mut self) {
        let reg_07 = self.read_register(Registers::OP_FUN_CTRL_1);
        self.write_register(Registers::OP_FUN_CTRL_1, reg_07 | (1 << 3)); //txon bit
    }


    // Frequency ctrl ------------------------------------------------------------------------------
    pub fn set_freq_band(&mut self, band: u8) {
        let freq_reg = self.read_register(Registers::FREQ_BAND_SEL);
        self.write_register(Registers::FREQ_BAND_SEL, freq_reg & !(0x1F) | band & (0x1F));
    }

    // hbsel (high band select): doubles tx frequency
    pub fn set_hb_sel(&mut self, hbsel: bool) {
        let freq_reg = self.read_register(Registers::FREQ_BAND_SEL);
        self.write_register(Registers::FREQ_BAND_SEL,
                            freq_reg & !(1 << 5) | (hbsel as u8) << 5);
    }

    pub fn set_freq(&mut self, f_upper: u8, f_lower: u8) {
        self.write_register(Registers::CAR_FREQ_1, f_upper);
        self.write_register(Registers::CAR_FREQ_0, f_lower);
    }

    pub fn get_freq(&mut self) -> [u8; 2] {
        let mut rx_buf: [u8; 2] = [0, 0];
        rx_buf[1] = self.read_register(Registers::CAR_FREQ_1);
        rx_buf[0] = self.read_register(Registers::CAR_FREQ_0);
        rx_buf
    }

    // TX power ------------------------------------------------------------------------------------
    pub fn set_tx_pwr(&mut self, power: u8) {
        self.write_register(Registers::TX_PWR, power);
    }

    pub fn set_modulation_type(&mut self, mod_mode: u8) {
        let mod_reg_2 = self.read_register(Registers::MODULATION_MODE_CTRL_2);
        let bits = (1 << 1) | 1;
        self.write_register(Registers::MODULATION_MODE_CTRL_2,
                            (mod_reg_2 & !(bits)) | ((mod_mode) & bits));
    }

    pub fn set_modulation_source(&mut self, mod_src: u8) {
        let mod_reg_2 = self.read_register(Registers::MODULATION_MODE_CTRL_2);
        let bits = (1 << 1) | 1;
        self.write_register(Registers::MODULATION_MODE_CTRL_2,
                            (mod_reg_2 & !(bits)) | ((mod_src) & bits));
    }
    pub fn write_fifo() {}

    pub fn get_device_status(&mut self) -> u8 {
        self.read_register(Registers::DEVICE_STATUS)
    }

    pub fn set_sync_wrd(&mut self, syncword: u32) {
        self.write_register(Registers::SYNC_WRD_0,
                            (syncword & (0xFF)) as u8);
        self.write_register(Registers::SYNC_WRD_1,
                            ((syncword >> 8) & (0xFF)) as u8);
        self.write_register(Registers::SYNC_WRD_2,
                            ((syncword >> 16) & (0xFF)) as u8);
        self.write_register(Registers::SYNC_WRD_3,
                            ((syncword >> 24) & (0xFF)) as u8);
    }

    pub fn set_tx_header(&mut self, tx_header: u32) {
        self.write_register(Registers::TX_HEADER_0,
                            (tx_header & (0xFF)) as u8);
        self.write_register(Registers::TX_HEADER_1,
                            ((tx_header >> 8) & (0xFF)) as u8);
        self.write_register(Registers::TX_HEADER_2,
                            ((tx_header >> 16) & (0xFF)) as u8);
        self.write_register(Registers::TX_HEADER_3,
                            ((tx_header >> 24) & (0xFF)) as u8);
    }

    pub fn set_auto_packet_handler(&mut self, ena: bool) {
        let data_reg = self.read_register(Registers::DATA_ACCESS_CTRL);
        let bits = (ena as u8) << 3;
        self.write_register(Registers::DATA_ACCESS_CTRL,
                            (data_reg & !(bits)) | (bits));
    }

    pub fn set_tx_packet_leng() {}
}




