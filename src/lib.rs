#![no_std]

mod registers;

use embedded_hal as hal;
use embedded_hal::digital::v2::OutputPin;
use hal::blocking::spi;
use crate::registers::Registers;

#[repr(u8)]
pub enum ModType {
    UmodCar = 0x00,
    OOK = 0x01,
    FSK = 0x02,
    GFSK = 0x03,
}

#[repr(u8)]
pub enum ModDataSrc {
    DirectGpio = 0x00,
    DirectSdi = 0x10,
    Fifo = 0x20,
    Pn9 = 0x30,
}

#[repr(u8)]
pub enum TxDataClk {
    Async = 0x00,
    Gpio = 0x40,
    Sdo = 0x80,
    nIRQ = 0xC0,
}

#[repr(u8)]
pub enum ETxPower {
    P1dBm = 0x0,
    P2dBm = 0x1,
    P5dBm = 0x2,
    P8dBm = 0x3,
    P11dBm = 0x4,
    P14dBm = 0x5,
    P17dBm = 0x6,
    P20dBm = 0x7,
}

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

    fn write_register(&mut self, reg: Registers, data: u8)  {
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

    pub fn swreset(&mut self) {
        self.write_register(Registers::OP_FUN_CTRL_1, 0x80);
    }

    fn enter_standby(&mut self) {
        self.write_register(Registers::OP_FUN_CTRL_1, 0x00);
    }

    fn enter_sleep(&mut self) {
        let reg_07 = self.read_register(Registers::OP_FUN_CTRL_1);
        self.write_register(Registers::OP_FUN_CTRL_1, 0x40); //enlbd bit
    }

    fn enter_ready(&mut self) {
        let reg_07 = self.read_register(Registers::OP_FUN_CTRL_1);
        self.write_register(Registers::OP_FUN_CTRL_1, reg_07 | 1); //xton bit
    }

    fn enter_tune(&mut self) {
        let reg_07 = self.read_register(Registers::OP_FUN_CTRL_1);
        self.write_register(Registers::OP_FUN_CTRL_1, reg_07 | (1 << 1)); //pllon bit
    }

    pub fn enter_tx(&mut self) {
        let reg_07 = self.read_register(Registers::OP_FUN_CTRL_1);

        let register_set =
              (1 << 0)  //XTON
            | (1 << 1)  //PLLON
            | (1 << 3)  //TXON
            | (1 << 6);  //ENLBD
        self.write_register(Registers::OP_FUN_CTRL_1, reg_07 | register_set);
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
    pub fn set_tx_pwr(&mut self, power: ETxPower) {
        //TODO: read reg before writing
        self.write_register(Registers::TX_PWR, power as u8);
        let x = self.read_register(Registers::TX_PWR);
    }

    pub fn set_modulation_type(&mut self, mod_mode: ModType) {
        let mod_reg_2 = self.read_register(Registers::MODULATION_MODE_CTRL_2);
        let bits = (1 << 1) | 1;
        self.write_register(Registers::MODULATION_MODE_CTRL_2,
                            (mod_reg_2 & !(bits)) | ((mod_mode as u8) & bits));
    }

    pub fn set_modulation_source(&mut self, mod_src: ModDataSrc) {
        let mod_reg_2 = self.read_register(Registers::MODULATION_MODE_CTRL_2);
        let bits = ((1 << 5) | (1 << 4));
        self.write_register(Registers::MODULATION_MODE_CTRL_2,
                            (mod_reg_2 & !(bits)) | ((mod_src as u8) & bits));
    }

    pub fn set_tx_data_clk(&mut self, clk: TxDataClk) {
        let mod_reg_2 = self.read_register(Registers::MODULATION_MODE_CTRL_2);
        let bits = (1 << 7) | (1 << 6);
        self.write_register(Registers::MODULATION_MODE_CTRL_2,
                            (mod_reg_2 & !(bits)) | ((clk as u8) & bits));
    }

    // FIFO ACCESS ---------------------------------------------------------------------------------
    pub fn write_fifo(&mut self, data: &[u8]) {
        self.burst_write_register(Registers::FIFO_ACCESS, data);
    }

    // DEVICE STATUS -------------------------------------------------------------------------------
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

    pub fn set_packet_len(&mut self, len: u8) {
        self.write_register(Registers::TX_PACKET_LEN, len);
    }


    // GPIO ----------------------------------------------------------------------------------------
    // GPIO 0 is n/c | GPIO 1: HEAT REF | GPIO 2: RADIO PROBE POINT

    pub fn init_gpio_1(&mut self) {
        self.write_register(Registers::GPIO_1_CFG, 0x0A);
    }

    pub fn init_gpio_2(&mut self) {
        self.write_register(Registers::GPIO_2_CFG, 0x0A);
    }

    pub fn set_gpio_1(&mut self, dio: bool) {
        let io = self.read_register(Registers::IO_PORT_CFG);
        self.write_register(Registers::IO_PORT_CFG, io & !(1 << 1) | (dio as u8) << 1);
    }

    pub fn set_gpio_2(&mut self, dio: bool) {
        let io = self.read_register(Registers::IO_PORT_CFG);
        self.write_register(Registers::IO_PORT_CFG, io & !(1 << 2) | (dio as u8) << 2);
    }

    // DIAG ----------------------------------------------------------------------------------------
    pub fn read_bat_volt(&mut self) -> u8 {
        self.read_register(Registers::BAT_VOLT_LVL)
    }

    pub fn get_tx_pow(&mut self) -> u8 {
        self.read_register(Registers::TX_PWR)
    }

    // Generate CW (for testing purposes) ----------------------------------------------------------
    pub fn set_cw(&mut self) {

        // Set Mod Type
        self.set_modulation_type(ModType::OOK);

        // Set FIFO mode
        self.set_modulation_source(ModDataSrc::Fifo);

        self.set_auto_packet_handler(false);

        // Write ones into fifo
        let n_ones: u8 = 8;
        let mut c: u8 = 0;
        while c < n_ones {
            let write_data: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];
            self.write_fifo(&write_data);
            c = c+1;
        }
        self.set_packet_len(n_ones);

        let vbat = self.read_bat_volt();
        let stat = self.get_device_status();

        self.enter_tx();

        let pwr = self.get_tx_pow();
    }
}