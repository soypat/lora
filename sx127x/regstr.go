package sx127x

type regstr uint8

const (
	// registers
	_FIFO                 regstr = 0x00
	_OP_MODE              regstr = 0x01
	_FRF_MSB              regstr = 0x06
	_FRF_MID              regstr = 0x07
	_FRF_LSB              regstr = 0x08
	_PA_CONFIG            regstr = 0x09
	_PA_RAMP              regstr = 0x0a
	_OCP                  regstr = 0x0b
	_LNA                  regstr = 0x0c
	_FIFO_ADDR_PTR        regstr = 0x0d
	_FIFO_TX_BASE_ADDR    regstr = 0x0e
	_FIFO_RX_BASE_ADDR    regstr = 0x0f
	_FIFO_RX_CURRENT_ADDR regstr = 0x10
	_IRQ_FLAGS_MASK       regstr = 0x11
	_IRQ_FLAGS            regstr = 0x12
	_RX_NB_BYTES          regstr = 0x13
	_PKT_SNR_VALUE        regstr = 0x19
	_PKT_RSSI_VALUE       regstr = 0x1a
	_RSSI_VALUE           regstr = 0x1b
	_MODEM_CONFIG_1       regstr = 0x1d
	_MODEM_CONFIG_2       regstr = 0x1e
	_SYMB_TIMEOUT_LSB     regstr = 0x1f
	_PREAMBLE_MSB         regstr = 0x20
	_PREAMBLE_LSB         regstr = 0x21
	_PAYLOAD_LENGTH       regstr = 0x22
	_MAX_PAYLOAD_LENGTH   regstr = 0x23
	_HOP_PERIOD           regstr = 0x24
	_MODEM_CONFIG_3       regstr = 0x26
	_FREQ_ERROR_MSB       regstr = 0x28
	_FREQ_ERROR_MID       regstr = 0x29
	_FREQ_ERROR_LSB       regstr = 0x2a
	_RSSI_WIDEBAND        regstr = 0x2c
	_DETECTION_OPTIMIZE   regstr = 0x31
	_INVERTIQ             regstr = 0x33
	_DETECTION_THRESHOLD  regstr = 0x37
	_SYNC_WORD            regstr = 0x39
	_INVERTIQ2            regstr = 0x3b
	_DIO_MAPPING_1        regstr = 0x40
	_DIO_MAPPING_2        regstr = 0x41
	_VERSION              regstr = 0x42
	_PA_DAC               regstr = 0x4d
)
