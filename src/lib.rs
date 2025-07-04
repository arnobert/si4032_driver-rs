#![no_std]

// Driver for the Si4032 transmitter used on RS41 radio sonde.
mod registers;

use embedded_hal as hal;
use embedded_hal::digital::v2::OutputPin;
use hal::blocking::spi;
use crate::registers::Registers;


/// Si4032 supports four modulation types:
#[repr(u8)]
pub enum ModType {
    UmodCar = 0x00,
    OOK = 0x01,
    FSK = 0x02,
    GFSK = 0x03,
}

/// Four modulation data sources available.
/// On RS41 we use Fifo
#[repr(u8)]
pub enum ModDataSrc {
    DirectGpio = 0x00,
    DirectSdi = 0x10,
    Fifo = 0x20,
    Pn9 = 0x30,
}

/// TX data clock source
#[repr(u8)]
pub enum TxDataClk {
    Async = 0x00,
    Gpio = 0x40,
    Sdo = 0x80,
    nIRQ = 0xC0,
}

/// TX power
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

/// CRC polynomial
#[repr(u8)]
pub enum CrcPoly {
    Ccitt = 0x0,
    Crc16 = 0x1,
    Iec16 = 0x2,
    Biacheva = 0x3,
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

    /// Write SPI register
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
    /// Write to N SPI registers
    fn burst_write_register(&mut self, reg: Registers, data: &[u8]) {
        self.cs.set_low();
        let x_reg = reg as u8 | (0x01 << 7);
        self.spi.write(&[x_reg]);
        self.spi.write(data);
        self.cs.set_high();
    }

    /// Read SPI register
    fn read_register(&mut self, reg: Registers) -> u8 {
        self.cs.set_low();
        let mut rx_buy = [reg as u8, 0];
        self.spi.transfer(&mut rx_buy);
        self.cs.set_high();
        rx_buy[1]
    }

    // Set operating modes -------------------------------------------------------------------------
    // SHUTDOWN is not available for pin 20 is hardwired to gnd.

    /// Perform soft reset; all registers hold their default values afterwards.
    /// Poll interrupt register (chip_ready).
    pub fn swreset(&mut self) {
        self.write_register(Registers::OP_FUN_CTRL_1, 0x80);
    }

    /// Go into standby
    fn enter_standby(&mut self) {
        self.write_register(Registers::OP_FUN_CTRL_1, 0x00);
    }

    /// Go into sleep; set enlbd bit
    fn enter_sleep(&mut self) {
        let reg_07 = self.read_register(Registers::OP_FUN_CTRL_1);
        self.write_register(Registers::OP_FUN_CTRL_1, 0x40); //enlbd bit
    }

    /// Go into ready; set xton bit
    fn enter_ready(&mut self) {
        let reg_07 = self.read_register(Registers::OP_FUN_CTRL_1);
        self.write_register(Registers::OP_FUN_CTRL_1, reg_07 | 1); //xton bit
    }

    /// Go into tuning mode; set pllon bit
    fn enter_tune(&mut self) {
        let reg_07 = self.read_register(Registers::OP_FUN_CTRL_1);
        self.write_register(Registers::OP_FUN_CTRL_1, reg_07 | (1 << 1)); //pllon bit
    }

    /// Transmit
    pub fn tx_on(&mut self) {
        let reg_07 = self.read_register(Registers::OP_FUN_CTRL_1);
        self.write_register(Registers::OP_FUN_CTRL_1, reg_07 | (1 << 3)); //txon bit
    }

    pub fn is_tx_on(&mut self) -> bool {
        let reg_07 = self.read_register(Registers::OP_FUN_CTRL_1);
        reg_07 & (1 << 3) != 0
    }

    /// Set all bits necessary in order to transmit
    pub fn enter_tx(&mut self) {
        let reg_07 = self.read_register(Registers::OP_FUN_CTRL_1);

        let register_set =
            (1 << 0)  //XTON
                | (1 << 1)  //PLLON
                | (1 << 3)  //TXON
                | (1 << 6);  //ENLBD
        self.write_register(Registers::OP_FUN_CTRL_1, reg_07 | register_set);
    }

    /// Turn TX off
    pub fn tx_off(&mut self) {
        let reg_07 = self.read_register(Registers::OP_FUN_CTRL_1);

        let register_set =
            (1 << 0)  //XTON
                | (1 << 1)  //PLLON
                | (0 << 3)  //TXON
                | (1 << 6);  //ENLBD
        self.write_register(Registers::OP_FUN_CTRL_1, (reg_07 & 0xF7) | register_set);
    }


    /// Chip ready after reset
    pub fn chip_ready(&mut self) -> bool {
        let isr2 = self.read_register(Registers::INTERRUPT_STATUS_2);
        isr2 & (1 << 1) != 0
    }

    // Frequency ctrl ------------------------------------------------------------------------------
    /// Set frequency band (Register 0x75)
    pub fn set_freq_band(&mut self, band: u8) {
        let freq_reg = self.read_register(Registers::FREQ_BAND_SEL);
        self.write_register(Registers::FREQ_BAND_SEL, freq_reg & !(0x1F) | band & (0x1F));
    }

    /// Return frequency (Register 0x75)
    pub fn get_freq_band(&mut self) -> u8 {
        self.read_register(Registers::FREQ_BAND_SEL)
    }

    /// Set high band (Register 0x75)
    pub fn set_hb_sel(&mut self, hbsel: bool) {
        let freq_reg = self.read_register(Registers::FREQ_BAND_SEL);
        self.write_register(Registers::FREQ_BAND_SEL,
                            freq_reg & !(1 << 5) | (hbsel as u8) << 5);
    }

    /// Set frequency (Registers 0x76 / 0x77)
    pub fn set_freq(&mut self, f_upper: u8, f_lower: u8) {
        self.write_register(Registers::CAR_FREQ_1, f_upper);
        self.write_register(Registers::CAR_FREQ_0, f_lower);
    }

    /// Return frequency (Registers 0x76 / 0x77)
    pub fn get_freq(&mut self) -> [u8; 2] {
        let mut rx_buf: [u8; 2] = [0, 0];
        rx_buf[1] = self.read_register(Registers::CAR_FREQ_1);
        rx_buf[0] = self.read_register(Registers::CAR_FREQ_0);
        rx_buf
    }

    /// Set TX power (Register 0x6D)
    pub fn set_tx_pwr(&mut self, power: ETxPower) {
        let txpwr_reg = self.read_register(Registers::TX_PWR);
        self.write_register(Registers::TX_PWR,
                            txpwr_reg & !(0x07) | power as u8);
    }

    /// Return TX power
    pub fn get_tx_pow(&mut self) -> u8 {
        self.read_register(Registers::TX_PWR)
    }

    /// Set modulation type (Register 0x71)
    pub fn set_modulation_type(&mut self, mod_mode: ModType) {
        let mod_reg_2 = self.read_register(Registers::MODULATION_MODE_CTRL_2);
        let bits = (1 << 1) | 1;
        self.write_register(Registers::MODULATION_MODE_CTRL_2,
                            (mod_reg_2 & !(bits)) | ((mod_mode as u8) & bits));
    }

    /// Set modulation source (Register 0x71)
    pub fn set_modulation_source(&mut self, mod_src: ModDataSrc) {
        let mod_reg_2 = self.read_register(Registers::MODULATION_MODE_CTRL_2);
        let bits = ((1 << 5) | (1 << 4));
        self.write_register(Registers::MODULATION_MODE_CTRL_2,
                            (mod_reg_2 & !(bits)) | ((mod_src as u8) & bits));
    }

    /// Set TX data clock (Register 0x71)
    pub fn set_tx_data_clk(&mut self, clk: TxDataClk) {
        let mod_reg_2 = self.read_register(Registers::MODULATION_MODE_CTRL_2);
        let bits = (1 << 7) | (1 << 6);
        self.write_register(Registers::MODULATION_MODE_CTRL_2,
                            (mod_reg_2 & !(bits)) | ((clk as u8) & bits));
    }

    /// Set TX data rate (Registers 0x6E / 0x6F)
    pub fn set_data_rate(&mut self, rate: u16) {
        self.write_register(Registers::TX_DATA_RATE_1, ((rate & 0xFF00) >> 8) as u8);
        self.write_register(Registers::TX_DATA_RATE_0, (rate & 0xFF) as u8);
    }

    /// Write into FIFO (Register 0x77)
    pub fn write_fifo(&mut self, data: &[u8]) {
        self.burst_write_register(Registers::FIFO_ACCESS, data);
    }

    /// Clear Fifo (Register 0x08)
    pub fn clear_fifo(&mut self) {
        self.write_register(Registers::OP_FUN_CTRL_2, 0x01);
        self.write_register(Registers::OP_FUN_CTRL_2, 0x00);
    }

    /// Return device status (Register 0x02)
    pub fn get_device_status(&mut self) -> u8 {
        self.read_register(Registers::DEVICE_STATUS)
    }


    /// Set sync word (Register 0x36 .. 0x39)
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

    // Header CTRL ---------------------------------------------------------------------------------
    /// Set header length (Register 0x33)
    pub fn set_tx_header_len(&mut self, head_len: u8) {
        let hdrctrl = self.read_register(Registers::HEADER_CTRL);
        self.write_register(Registers::HEADER_CTRL,
                            hdrctrl & !(0x70) | (head_len & 0x7) << 4,
        );
    }

    /// Include packet length into header (Register 0x33)
    /// ! False -> Length is included in header !
    pub fn set_tx_fixplen(&mut self, fix_len: bool) {
        let hdrctrl = self.read_register(Registers::HEADER_CTRL);
        let mask: u8 = 1 << 3;
        let bit = (fix_len as u8) << 3;
        self.write_register(Registers::HEADER_CTRL,
                            hdrctrl & !(mask) | bit);
    }

    /// Set sync word length (Register 0x33)
    pub fn set_tx_sync_len(&mut self, sync_len: u8) {
        let hdrctrl = self.read_register(Registers::HEADER_CTRL);
        self.write_register(Registers::HEADER_CTRL,
                            hdrctrl & !(0x03 << 1) | (sync_len & 0x3) << 1,
        );
    }

    /// Set preamble length (Register 0x34)
    pub fn set_tx_prealen(&mut self, prea_len: u16) {
        let hdrctrl: u8 = self.read_register(Registers::HEADER_CTRL);
        let len_msb: u8 = ((prea_len & 0x100) >> 8) as u8;

        self.write_register(Registers::HEADER_CTRL, hdrctrl & !(0xFE) | len_msb);
        self.write_register(Registers::PREAMBLE_LEN, (prea_len & 0xFF) as u8);
    }

    /// Set TX header (Register 0x3A .. 0x3D)
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

    /// Set trxdrtscale (for data rates below 30 kbps, Register 0x70)
    pub fn set_trxdrtscale(&mut self, scale: bool) {
        let modmodectrl = self.read_register(Registers::MODULATION_MODE_CTRL_1);
        let mask: u8 = 1 << 5;
        let bit = (scale as u8) << 5;
        self.write_register(Registers::MODULATION_MODE_CTRL_1,
                            modmodectrl & !(mask) | bit);
    }

    /// Set Manchester Preamble Polarity (Register 0x70)
    pub fn set_man_preamble_pol(&mut self, preamble: bool) {
        let modmodectrl = self.read_register(Registers::MODULATION_MODE_CTRL_1);
        let mask: u8 = 1 << 3;
        let bit = (preamble as u8) << 3;
        self.write_register(Registers::MODULATION_MODE_CTRL_1,
                            modmodectrl & !(mask) | bit);
    }

    /// Set Manchester Data Inversion (Register 0x70)
    pub fn set_man_data_inv(&mut self, inv: bool) {
        let modmodectrl = self.read_register(Registers::MODULATION_MODE_CTRL_1);
        let mask: u8 = 1 << 2;
        let bit = u8::from(inv) << 2;
        self.write_register(Registers::MODULATION_MODE_CTRL_1,
                            modmodectrl & !(mask) | bit);
    }

    /// Set Manchester enable (Register 0x70)
    pub fn set_man_en(&mut self, en: bool) {
        let modmodectrl = self.read_register(Registers::MODULATION_MODE_CTRL_1);
        let mask: u8 = 1 << 1;
        let bit = u8::from(en) << 1;
        self.write_register(Registers::MODULATION_MODE_CTRL_1,
                            modmodectrl & !(mask) | bit);
    }

    /// Set Manchester Data Whiting (Register 0x70)
    pub fn set_man_data_whit(&mut self, en: bool) {
        let modmodectrl = self.read_register(Registers::MODULATION_MODE_CTRL_1);
        let mask: u8 = 1;
        let bit = en as u8;
        self.write_register(Registers::MODULATION_MODE_CTRL_1,
                            modmodectrl & !(mask) | bit);
    }

    /// Set automatic packet handler (Register 0x30)
    pub fn set_auto_packet_handler(&mut self, ena: bool) {
        let data_reg = self.read_register(Registers::DATA_ACCESS_CTRL);
        let mask: u8 = 1 << 3;
        let bits = (ena as u8) << 3;
        self.write_register(Registers::DATA_ACCESS_CTRL,
                            (data_reg & !(mask)) | (bits));
    }


    /// Set LSB transmitted first (Register 0x30)
    pub fn set_lsb_first(&mut self, ena: bool) {
        let data_reg = self.read_register(Registers::DATA_ACCESS_CTRL);
        let mask: u8 = 1 << 6;
        let bits = (ena as u8) << 6;
        self.write_register(Registers::DATA_ACCESS_CTRL,
                            (data_reg & !(mask)) | (bits));
    }

    /// Set CRC enable (Register 0x30)
    pub fn set_crc_en(&mut self, ena: bool) {
        let data_reg = self.read_register(Registers::DATA_ACCESS_CTRL);
        let mask: u8 = 1 << 2;
        let bits = (ena as u8) << 2;
        self.write_register(Registers::DATA_ACCESS_CTRL,
                            (data_reg & !(mask)) | (bits));
    }

    /// Set CRC Polynome (Register 0x30)
    pub fn set_crc_poly(&mut self, poly: CrcPoly) {
        let data_reg = self.read_register(Registers::DATA_ACCESS_CTRL);
        let bits = poly as u8;
        self.write_register(Registers::DATA_ACCESS_CTRL,
                            (data_reg & !(0x3)) | (bits));
    }

    /// CRC only on data (Register 0x30)
    pub fn set_crc_d_only(&mut self, crc: bool) {
        let data_acc = self.read_register(Registers::DATA_ACCESS_CTRL);
        let mask = 1 << 5;
        let bits = (crc as u8) << 5;
        self.write_register(Registers::DATA_ACCESS_CTRL,
                            (data_acc & !(mask)) | (bits));
    }

    /// ENPAC (Register 0x30)
    pub fn set_enpac(&mut self, enpac: bool) {
        let data_acc = self.read_register(Registers::DATA_ACCESS_CTRL);
        let mask = 1 << 3;
        let bits = (enpac as u8) << 3;
        self.write_register(Registers::DATA_ACCESS_CTRL,
                            (data_acc & !(mask)) | (bits));
    }


    /// Set packet length (Register 0x3E)
    pub fn set_packet_len(&mut self, len: u8) {
        self.write_register(Registers::TX_PACKET_LEN, len);
    }

    // FIFO
    /// Fifo almost full
    pub fn fifo_full(&mut self) -> bool {
        let int_reg_01 = self.read_register(Registers::INTERRUPT_STATUS_1);
        int_reg_01 & (1 << 6) != 0
    }

    /// Fifo almost empty
    pub fn fifo_empty(&mut self) -> bool {
        let int_reg_01 = self.read_register(Registers::INTERRUPT_STATUS_1);
        int_reg_01 & (1 << 5) != 0
    }

    // Frequency Deviation
    /// Set Frequency Offset
    pub fn set_freq_offset(&mut self, offset: u16) {
        let f_offset_low: u8  = (offset & 0xFF) as u8;
        let f_offset_high: u8 = ((offset & 0x300) >> 8) as u8;
        self.write_register(Registers::FREQ_OFFSET_1, f_offset_low);
        self.write_register(Registers::FREQ_OFFSET_2, f_offset_high);
    }

    /// Set Frequency Deviation
    pub fn set_freq_deviation(&mut self, deviation: u8) {
        self.write_register(Registers::FREQ_DEVIATION, deviation);
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
    /// RETURN battery voltage
    pub fn read_bat_volt(&mut self) -> u8 {
        self.read_register(Registers::BAT_VOLT_LVL)
    }

    // Generate CW (for testing purposes) ----------------------------------------------------------
    /// Put out unmodulated carrier
    pub fn set_cw(&mut self) {
        self.set_modulation_type(ModType::UmodCar);
        self.enter_tx();
    }
}
