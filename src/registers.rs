pub enum Registers {
    DEVICE_VERSION = 0x01,
    DEVICE_STATUS = 0x02,
    INTERRUPT_STATUS_1 = 0x03,
    INTERRUPT_STATUS_2 = 0x04,
    INTERRUPT_ENABLE_1 = 0x05,
    INTERRUPT_ENABLE_2 = 0x06,
    OP_FUN_CTRL_1 = 0x07,
    OP_FUN_CTRL_2 = 0x08,
    XTAL_OSC_LD_CAP = 0x09,
    UC_OUTPUT_CLK = 0x0A,
    GPIO_0_CFG = 0x0B,
    GPIO_1_CFG = 0x0C,
    GPIO_2_CFG = 0x0D,
    IO_PORT_CFG = 0x0E,
    ADC_CFG = 0x0F,
    ADC_AMP_OFFSET = 0x10,
    ADC_VALUE = 0x11,
    TEMP_SENS_CTRL = 0x12,
    TEMP_VAL_OFFSET = 0x13,
    WAKEUP_TIMER_PER_1 = 0x14,
    WAKEUP_TIMER_PER_2 = 0x15,
    WAKEUP_TIMER_PER_3 = 0x16,
    WAKEUP_TIMER_VAL_1 = 0x17,
    WAKEUP_TIMER_VAL_2 = 0x18,
    // 19 RESERVED
    LOW_BAT_DET_THR = 0x1A,
    BAT_VOLT_LVL = 0x1B,
    // 1C..2F RESERVED
    DATA_ACCESS_CTRL = 0x30,
    EZ_MAC_STATUS = 0x31,
    // 32 RESERVED
    HEADER_CTRL = 0x33,
    PREAMBLE_LEN = 0x34,
    // 35 RESERVED
    SYNC_WRD_3 = 0x36,
    SYNC_WRD_2 = 0x37,
    SYNC_WRD_1 = 0x38,
    SYNC_WRD_0 = 0x39,
    TX_HEADER_3 = 0x3A,
    TX_HEADER_2 = 0x3B,
    TX_HEADER_1 = 0x3C,
    TX_HEADER_0 = 0x3D,
    TX_PACKET_LEN = 0x3E,
    // 3F..4E RESERVED
    ADC8_CTRL = 0x4F,
    // 50..61 RESERVED
    XOSC_CTRL_TEST = 0x62,
    // 63..6C RESERVED
    TX_PWR = 0x6D,
    TX_DATA_RATE_1 = 0x6E,
    TX_DATA_RATE_0 = 0x6F,
    MODULATION_MODE_CTRL_1 = 0x70,
    MODULATION_MODE_CTRL_2 = 0x71,
    FREQ_DEVIATION = 0x72,
    FREQ_OFFSET_1 = 0x73,
    FREQ_OFFSET_2 = 0x74,
    FREQ_BAND_SEL = 0x75,
    CAR_FREQ_1 = 0x76,
    CAR_FREQ_0 = 0x77,
    // 78 RESERVED
    FREQ_HOPPING_CHAN_SEL = 0x79,
    FREQ_HOPPING_STEP_SIZE = 0x7A,
    // 7B RESERVED
    TX_FIFO_CTRL_1 = 0x7C,
    TX_FIFO_CTRL_0 = 0x7D,
    // 7E RESERVED
    FIFO_ACCESS = 0x7F
}