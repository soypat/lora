package sx127x

import "github.com/soypat/lora"

const (
	// 32MHz frequency for crystal oscillator is typical.
	fXOSC = 32_000_000
	// PLL uses a 19-bit sigma-delta modulator whose frequency resolution, constant over the whole frequency range, is given by
	fSTEP = fXOSC / (1 << 19)
)

const (
	// registers
	regFIFO                 = 0x00
	regOP_MODE              = 0x01
	regFRF_MSB              = 0x06
	regFRF_MID              = 0x07
	regFRF_LSB              = 0x08
	regPA_CONFIG            = 0x09
	regPA_RAMP              = 0x0a
	regOCP                  = 0x0b
	regLNA                  = 0x0c
	regFIFO_ADDR_PTR        = 0x0d
	regFIFO_TX_BASE_ADDR    = 0x0e
	regFIFO_RX_BASE_ADDR    = 0x0f
	regFIFO_RX_CURRENT_ADDR = 0x10
	regIRQ_FLAGS_MASK       = 0x11
	regIRQ_FLAGS            = 0x12
	regRX_NB_BYTES          = 0x13
	regPKT_SNR_VALUE        = 0x19
	regPKT_RSSI_VALUE       = 0x1a
	regRSSI_VALUE           = 0x1b
	regMODEM_CONFIG_1       = 0x1d
	regMODEM_CONFIG_2       = 0x1e
	regSYMB_TIMEOUT_LSB     = 0x1f
	regPREAMBLE_MSB         = 0x20
	regPREAMBLE_LSB         = 0x21
	regPAYLOAD_LENGTH       = 0x22
	regMAX_PAYLOAD_LENGTH   = 0x23
	regHOP_PERIOD           = 0x24
	regMODEM_CONFIG_3       = 0x26
	regFREQ_ERROR_MSB       = 0x28
	regFREQ_ERROR_MID       = 0x29
	regFREQ_ERROR_LSB       = 0x2a
	regRSSI_WIDEBAND        = 0x2c
	regDETECTION_OPTIMIZE   = 0x31
	regINVERTIQ             = 0x33
	regDETECTION_THRESHOLD  = 0x37
	regSYNC_WORD            = 0x39
	regINVERTIQ2            = 0x3b
	regDIO_MAPPING_1        = 0x40
	regDIO_MAPPING_2        = 0x41
	regVERSION              = 0x42
	regPA_DAC               = 0x4d
	// PA config
	paBOOST = 0x80

	expectedVersion = 0x12

	// Bits masking the corresponding IRQs from the radio
	irqRXTOUT_MASK uint8 = 0x80
	irqRXDONE_MASK uint8 = 0x40
	irqCRCERR_MASK uint8 = 0x20
	irqHEADER_MASK uint8 = 0x10
	irqTXDONE_MASK uint8 = 0x08
	irqCDDONE_MASK uint8 = 0x04
	irqFHSSCH_MASK uint8 = 0x02
	irqCDDETD_MASK uint8 = 0x01

	// DIO function mappings                D0D1D2D3
	mapDIO0_LORA_RXDONE uint8 = 0x00 // 00------
	mapDIO0_LORA_TXDONE uint8 = 0x40 // 01------
	mapDIO1_LORA_RXTOUT uint8 = 0x00 // --00----
	mapDIO1_LORA_NOP    uint8 = 0x30 // --11----
	mapDIO2_LORA_NOP    uint8 = 0xC0 // ----11--

	// SX127X_PAYLOAD_LENGTH uint8 = 0x40

	// Low Noise Amp
	lnaMAX_GAIN uint8 = 0b001
	lnaOFF_GAIN uint8 = 0b111
	lnaLOW_GAIN uint8 = 0b110

	// Bandwidth
	bw7_8   uint8 = 0x00
	bw10_4  uint8 = 0x01
	bw15_6  uint8 = 0x02
	bw20_8  uint8 = 0x03
	bw31_25 uint8 = 0x04
	bw41_7  uint8 = 0x05
	bw62_5  uint8 = 0x06
	bw125_0 uint8 = 0x07
	bw250_0 uint8 = 0x08
	bw500_0 uint8 = 0x09
	// Automatic gain control
	agcAUTO_OFF uint8 = 0x00
	agcAUTO_ON  uint8 = 0x01

	publicSyncword  = 0x34
	privateSyncword = 0x14
)

var bandwidths = [bw500_0 + 1]lora.Frequency{
	bw7_8:   7.8e3 * lora.Hertz,
	bw10_4:  10.4e3 * lora.Hertz,
	bw15_6:  15.6e3 * lora.Hertz,
	bw20_8:  20.8e3 * lora.Hertz,
	bw31_25: 31.25e3 * lora.Hertz,
	bw41_7:  41.7e3 * lora.Hertz,
	bw62_5:  62.5e3 * lora.Hertz,
	bw125_0: 125e3 * lora.Hertz,
	bw250_0: 250e3 * lora.Hertz,
	bw500_0: 500e3 * lora.Hertz,
}

func bwReg(bandwidth lora.Frequency) (value byte) {
	value = bw7_8
	for i := byte(len(bandwidths) - 1); i >= 0; i-- {
		if bandwidth >= bandwidths[i] {
			value = i
			break
		}
	}
	return value
}

func reg2Bw(bwByte byte) (value lora.Frequency) {
	if bwByte > byte(len(bandwidths)-1) {
		return 0
	}
	return bandwidths[bwByte]
}

// Operation modes.
const (
	opmMASK      uint8 = 0x07
	opmSLEEP     uint8 = 0x00
	opmSTANDBY   uint8 = 0x01
	opmFSTX      uint8 = 0x02
	opmTX        uint8 = 0x03
	opmFSRX      uint8 = 0x04
	opmRX        uint8 = 0x05
	opmRX_SINGLE uint8 = 0x06
	opmCAD       uint8 = 0x07
	opmLORA      uint8 = 0x80
)

// OpMode represents the available operation modes of the SX127x devices.
type OpMode uint8

// LoRa Operation modes.
const (
	// Sleep mode will sleep goroutine for 15ms on SetOpmode call.
	OpSleep OpMode = iota
	// Standby mode is the default mode when waiting to operate.
	OpStandby
	// Frequency synthesis TX (FSTX)
	OpFSTx
	// Transmit (TX)
	OpTx
	// Frequency synthesis RX (FSRX)
	OpFSRx
	// Receive continuous (RXCONTINUOUS)
	OpRx
	// receive single (RXSINGLE)
	OpRxSingle
	// Channel activity detection (CAD)
	OpCAD
	opLoRaBit = OpMode(1 << 7)
)

func (op OpMode) String() (s string) {
	op = op &^ opLoRaBit
	switch op {
	case OpSleep:
		s = "sleep"
	case OpStandby:
		s = "standby"
	case OpFSTx:
		s = "fstx"
	case OpTx:
		s = "tx"
	case OpFSRx:
		s = "fsrx"
	case OpRx:
		s = "rx"
	case OpRxSingle:
		s = "rx-single"
	case OpCAD:
		s = "cad"
	default:
		s = "unknown"
	}
	return s
}
