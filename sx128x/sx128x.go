package sx128x

import (
	"errors"
	"io"
	"runtime"
	"time"

	"github.com/soypat/lora"
)

type Output func(bool)
type Input func() bool

type SPI interface {
	Transfer(w byte) (byte, error)
	Tx(writeBuffer, readBuffer []byte) error
}

type DeviceLoRa struct {
	spi      SPI
	cs       Output
	rst      Output
	busy     Input
	dio1     Input
	spiTxBuf []byte
	spiRxBuf []byte
	config   lora.Config
}

func DefaultConfig(freq lora.Frequency) lora.Config {
	return lora.Config{
		Frequency:                freq,
		SpreadFactor:             lora.SF9,
		Bandwidth:                lora.BW1625k,
		CodingRate:               lora.CR4_5,
		PreambleLength:           12,
		HeaderType:               lora.HeaderExplicit,
		MaxImplicitPayloadLength: 0, // No need to be set when working with explicit headers.
		CRC:                      true,
		SyncWord:                 0x1424,
		TxPower:                  2,     // Low power by default.
		LDRO:                     false, // not available on sx128x
		IQInversion:              false,
	}
}

func NewLoRa(spi SPI, cs Output, rst Output, busy Input, dio1 Input) *DeviceLoRa {
	return &DeviceLoRa{
		spi:      spi,
		cs:       cs,
		rst:      rst,
		busy:     busy,
		dio1:     dio1,
		spiTxBuf: make([]byte, 256), // TODO: optimize buffer size
		spiRxBuf: make([]byte, 256),
	}
}

func (d *DeviceLoRa) Configure(config lora.Config) error {
	switch {
	case config.Frequency < 2400*lora.Megahertz:
		return errFrequencyTooLow
	case config.Frequency > 2500*lora.Megahertz:
		return errFrequencyTooHigh
	case config.TxPower < -18:
		return errPowerTooLow
	case config.TxPower > 13:
		return errPowerTooHigh
	case config.HeaderType != lora.HeaderExplicit && config.HeaderType != lora.HeaderImplicit:
		return errInvalidHeaderType
	case config.Bandwidth != lora.BW1625k:
		return errInvalidBandwidth
	case config.CodingRate != lora.CR4_5 && config.CodingRate != lora.CR4_6 && config.CodingRate != lora.CR4_7 && config.CodingRate != lora.CR4_8:
		return errInvalidCodingRate
	}
	d.Reset()
	d.config = config
	// Switch to standby prior to configuration changes
	err := d.setStandby(standbyRC)
	if err != nil {
		return err
	}
	// Clear errors, disable radio interrupts for the moment
	err = d.setPacketType(packetTypeLoRa)
	if err != nil {
		return err
	}
	err = d.setRfFrequency(config.Frequency)
	if err != nil {
		return err
	}
	err = d.setModulationParamsLoRa(config.SpreadFactor, config.Bandwidth, config.CodingRate)
	if err != nil {
		return err
	}

	// special register setting depending on spreading factor chosen
	switch config.SpreadFactor {
	case lora.SF5, lora.SF6:
		d.writeRegister(regSpreadingFactorAdditionalConfiguration, []byte{0x1E})
	case lora.SF7, lora.SF8:
		d.writeRegister(regSpreadingFactorAdditionalConfiguration, []byte{0x37})
	default:
		d.writeRegister(regSpreadingFactorAdditionalConfiguration, []byte{0x32})
	}
	d.writeRegister(regFrequencyErrorCorrection, []byte{0x01})

	// TODO(jwetzell): hardcoded radio ramp
	err = d.setTxParams(config.TxPower, radioRamp02us)
	if err != nil {
		return err
	}
	err = d.setPacketParamsLoRa(uint32(config.PreambleLength), config.HeaderType, 0xFF, config.CRC, config.IQInversion)
	if err != nil {
		return err
	}
	var syncWord [2]uint8
	syncWord[0] = uint8(config.SyncWord >> 8)
	syncWord[1] = uint8(config.SyncWord & 0x00FF)

	err = d.writeRegister(regLoRaSyncWordMSB, syncWord[:])
	if err != nil {
		return err
	}
	return nil
}

func (d *DeviceLoRa) Tx(data []byte) error {
	if len(data) > 255 {
		return errors.New("data length exceeds maximum of 255 bytes")
	}

	// TODO(jwetzell): check chip status prior to setting Rx mode and return error if not ready
	d.setStandby(standbyRC)
	d.setPacketParamsLoRa(uint32(d.config.PreambleLength), d.config.HeaderType, uint8(len(data)&0xFF), d.config.CRC, d.config.IQInversion)
	d.setBufferBaseAddress(0, 0)
	d.writeBuffer(0, data)
	d.setDioIrqParams(irqTxDone|irqTimeout, irqTxDone|irqTimeout, 0x00, 0x00)
	d.clearIrqStatus(irqAll)
	d.setTx(periodBase4Ms, 250) // fixed timeout for now
	for {
		if d.dio1() {
			irqStatus, err := d.getIrqStatus()
			if err != nil {
				return err
			}
			if irqStatus&irqTimeout != 0 {
				return errRxTimeout
			}
			if irqStatus&irqTxDone != 0 {
				return nil
			}
		}
	}
}

func (d *DeviceLoRa) RxSingle(dst []byte) (uint8, error) {
	if len(dst) < 255 {
		return 0, io.ErrShortBuffer
	}
	// TODO(jwetzell): check chip status prior to setting Rx mode and return error if not ready
	d.setStandby(standbyRC)
	d.setDioIrqParams(irqRxDone|irqTimeout, irqRxDone|irqTimeout, 0x00, 0x00)
	d.setBufferBaseAddress(0, 0)
	d.clearIrqStatus(irqAll)
	d.setRx(periodBase4Ms, 250) // fixed timeout for now
	for {
		if d.dio1() {
			irqStatus, err := d.getIrqStatus()
			if err != nil {
				return 0, err
			}
			if irqStatus&irqTimeout != 0 {
				return 0, errRxTimeout
			}
			if irqStatus&irqRxDone != 0 {
				payloadLength, offset, err := d.getRxBufferStatus()
				if err != nil {
					return 0, err
				}
				data, err := d.readBuffer(offset, payloadLength)
				if err != nil {
					return 0, err
				}
				copy(dst, data)
				return payloadLength, nil
			}
		}
	}
}

func (d *DeviceLoRa) Reset() {
	d.rst(true)
	time.Sleep(10 * time.Millisecond)
	d.rst(false)
	time.Sleep(10 * time.Millisecond)
	d.rst(true)
	time.Sleep(10 * time.Millisecond)
	d.cs(false)
}

func (d *DeviceLoRa) waitWhileBusy(timeout time.Duration) error {
	// largest busy period is on boot with around ~400ish this should be more than enough
	now := time.Now()
	for d.busy() {
		if time.Since(now) > timeout {
			return ErrBusyPinTimeout
		}
		runtime.Gosched()
	}
	return nil
}

func (d *DeviceLoRa) writeRegister(addr uint16, data []byte) error {
	err := d.waitWhileBusy(time.Second)
	if err != nil {
		return err
	}
	d.cs(false)
	d.spiTxBuf = d.spiTxBuf[:0]
	d.spiTxBuf = append(d.spiTxBuf, cmdWriteRegister, uint8((addr>>8)&0xFF), uint8(addr&0xFF))
	d.spiTxBuf = append(d.spiTxBuf, data...)
	err = d.spi.Tx(d.spiTxBuf, nil)
	d.cs(true)
	return err
}

func (d *DeviceLoRa) readRegister(addr uint16) (uint8, error) {
	err := d.waitWhileBusy(time.Second)
	if err != nil {
		return 0, err
	}
	d.cs(false)
	d.spiTxBuf = d.spiTxBuf[:0]
	d.spiTxBuf = append(d.spiTxBuf, cmdReadRegister, uint8((addr&0xFF00)>>8), uint8(addr&0x00FF), 0x00, 0x00)
	d.spiRxBuf = d.spiRxBuf[:5]
	err = d.spi.Tx(d.spiTxBuf, d.spiRxBuf)
	d.cs(true)
	if err != nil {
		return 0, err
	}
	return d.spiRxBuf[4], nil
}

func (d *DeviceLoRa) writeBuffer(offset uint8, data []byte) error {
	if len(data) > 256 {
		return errDataTooLong
	}
	err := d.waitWhileBusy(time.Second)
	if err != nil {
		return err
	}
	d.cs(false)
	d.spiTxBuf = d.spiTxBuf[:0]
	d.spiTxBuf = append(d.spiTxBuf, cmdWriteBuffer, offset)
	d.spiTxBuf = append(d.spiTxBuf, data...)
	err = d.spi.Tx(d.spiTxBuf, nil)
	d.cs(true)
	return err
}

// Read data from the payload buffer starting at the given offset with the given length
func (d *DeviceLoRa) readBuffer(offset uint8, length uint8) ([]byte, error) {
	err := d.waitWhileBusy(time.Second)
	if err != nil {
		return nil, err
	}
	d.cs(false)
	d.spiTxBuf = d.spiTxBuf[:0]
	d.spiTxBuf = append(d.spiTxBuf, cmdReadBuffer, offset, 0x00)
	for i := uint8(0); i < length; i++ {
		d.spiTxBuf = append(d.spiTxBuf, 0x00)
	}
	d.spiRxBuf = d.spiRxBuf[:len(d.spiTxBuf)]
	err = d.spi.Tx(d.spiTxBuf, d.spiRxBuf)
	d.cs(true)
	if err != nil {
		return nil, err
	}
	return d.spiRxBuf[3 : 3+length], nil
}

// Put device into standby mode, 0 (RC) or 1 (XOSC)
func (d *DeviceLoRa) setStandby(standbyConfig standbyConfig) error {
	if standbyConfig > standbyXOSC { // XOSC is the highest standby config anything higher is invalid
		return errInvalidStandbyConfig
	}
	err := d.waitWhileBusy(time.Second)
	if err != nil {
		return err
	}
	d.cs(false)
	d.spiTxBuf = d.spiTxBuf[:0]
	d.spiTxBuf = append(d.spiTxBuf, cmdSetStandby, uint8(standbyConfig))
	err = d.spi.Tx(d.spiTxBuf, nil)
	d.cs(true)
	return err
}

func checkPeriodBase(periodBase periodBase) error {
	if periodBase > periodBase4Ms { // 4ms is the highest period base anything higher is invalid
		return errInvalidPeriodBase
	}
	return nil
}

// Sets the device in transmit mode, the IRQ status should be cleared before using this command
// timout is determined by periodBase * periodBaseCount
func (d *DeviceLoRa) setTx(periodBase periodBase, periodBaseCount uint16) error {
	err := checkPeriodBase(periodBase)
	if err != nil {
		return err
	}
	err = d.waitWhileBusy(time.Second)
	if err != nil {
		return err
	}
	d.cs(false)
	d.spiTxBuf = d.spiTxBuf[:0]
	d.spiTxBuf = append(d.spiTxBuf, cmdSetTx, uint8(periodBase), uint8((periodBaseCount>>8)&0xFF), uint8(periodBaseCount&0xFF))
	err = d.spi.Tx(d.spiTxBuf, nil)
	d.cs(true)
	return err
}

// Sets the device in receive mode, the IRQ status should be cleared before using this command
// timeout is determined by periodBase * periodBaseCount
func (d *DeviceLoRa) setRx(periodBase periodBase, periodBaseCount uint16) error {
	err := checkPeriodBase(periodBase)
	if err != nil {
		return err
	}
	err = d.waitWhileBusy(time.Second)
	if err != nil {
		return err
	}
	d.cs(false)
	d.spiTxBuf = d.spiTxBuf[:0]
	d.spiTxBuf = append(d.spiTxBuf, cmdSetRx, uint8(periodBase), uint8((periodBaseCount>>8)&0xFF), uint8(periodBaseCount&0xFF))
	err = d.spi.Tx(d.spiTxBuf, nil)
	d.cs(true)
	return err
}

// Choose between GFSK, LoRa, Ranging, FLRC or BLE packet types, this will affect the available configuration parameters and the structure of the packet
func (d *DeviceLoRa) setPacketType(packetType packetType) error {
	if packetType > packetTypeBLE { // BLE is the highest packet type anything higher is invalid.
		return errInvalidPacketType
	}
	err := d.waitWhileBusy(time.Second)
	if err != nil {
		return err
	}
	d.cs(false)
	d.spiTxBuf = d.spiTxBuf[:0]
	d.spiTxBuf = append(d.spiTxBuf, cmdSetPacketType, uint8(packetType))
	err = d.spi.Tx(d.spiTxBuf, nil)
	d.cs(true)
	return err
}

// Set the RF frequency in Hz, must be between 2.4 GHz and 2.5 GHz
func (d *DeviceLoRa) setRfFrequency(frequencyHz lora.Frequency) error {
	if frequencyHz < 2400000000 {
		return errFrequencyTooLow
	}
	if frequencyHz > 2500000000 {
		return errFrequencyTooHigh
	}
	err := d.waitWhileBusy(time.Second)
	if err != nil {
		return err
	}
	d.cs(false)
	d.spiTxBuf = d.spiTxBuf[:0]
	rfFrequency := uint32((uint64(frequencyHz) << 18) / 52000000)
	d.spiTxBuf = append(d.spiTxBuf, cmdSetRFFrequency, uint8((rfFrequency>>16)&0xFF), uint8((rfFrequency>>8)&0xFF), uint8(rfFrequency&0xFF))
	err = d.spi.Tx(d.spiTxBuf, nil)
	d.cs(true)
	return err
}

// Set the output power in dBm, must be between -18 and 13 dBm, and the ramp time
func (d *DeviceLoRa) setTxParams(powerdBm int8, rampTime radioRampTime) error {
	if powerdBm < -18 {
		return errPowerTooLow
	}
	if powerdBm > 13 {
		return errPowerTooHigh
	}

	err := d.waitWhileBusy(time.Second)
	if err != nil {
		return err
	}
	d.cs(false)
	d.spiTxBuf = d.spiTxBuf[:0]
	adjustedPower := uint8(powerdBm + 18)
	d.spiTxBuf = append(d.spiTxBuf, cmdSetTxParams, adjustedPower, uint8(rampTime))
	err = d.spi.Tx(d.spiTxBuf, nil)
	d.cs(true)
	return err
}

// Set the base address for the internal buffer for Tx and Rx operations.
// When transmitting or receiving data is read from or written to the buffer starting at the given offset.
func (d *DeviceLoRa) setBufferBaseAddress(txBase uint8, rxBase uint8) error {
	err := d.waitWhileBusy(time.Second)
	if err != nil {
		return err
	}
	d.cs(false)
	d.spiTxBuf = d.spiTxBuf[:0]
	d.spiTxBuf = append(d.spiTxBuf, cmdSetBufferBaseAddress, txBase, rxBase)
	err = d.spi.Tx(d.spiTxBuf, nil)
	d.cs(true)
	return err
}

func (d *DeviceLoRa) setModulationParamsLoRa(spreadingFactor lora.SpreadFactor, bandwidth lora.Frequency, codingRate lora.CodingRate) error {
	err := d.waitWhileBusy(time.Second)
	if err != nil {
		return err
	}
	d.cs(false)
	d.spiTxBuf = d.spiTxBuf[:0]
	d.spiTxBuf = append(d.spiTxBuf, cmdSetModulationParams) //, uint8(spreadingFactor), uint8(bandwidth), uint8(codingRate))
	d.spiTxBuf = append(d.spiTxBuf, uint8(spreadingFactor<<4))
	switch bandwidth {
	case lora.BW1625k:
		d.spiTxBuf = append(d.spiTxBuf, loraBW1600)
	// TODO(jwetzell): support for other bandwidths
	default:
		return errInvalidBandwidth
	}
	switch codingRate {
	case lora.CR4_5:
		d.spiTxBuf = append(d.spiTxBuf, loraCR4_5)
	case lora.CR4_6:
		d.spiTxBuf = append(d.spiTxBuf, loraCR4_6)
	case lora.CR4_7:
		d.spiTxBuf = append(d.spiTxBuf, loraCR4_7)
	case lora.CR4_8:
		d.spiTxBuf = append(d.spiTxBuf, loraCR4_8)
	// TODO(jwetzell): support for LI coding rates
	default:
		return errInvalidCodingRate
	}
	err = d.spi.Tx(d.spiTxBuf, nil)
	d.cs(true)
	return err
}

// Set LoRa related packet parameters, this assumes the packet type is already set to LoRa.
// - payloadLength: range of 1-255
func (d *DeviceLoRa) setPacketParamsLoRa(preambleLength uint32, headerType lora.HeaderType, payloadLength uint8, crcEnabled bool, iqInversion bool) error {
	if payloadLength == 0 {
		return errPayloadLengthTooShort
	}
	exponent, mantissa := getExponentAndMantissa(preambleLength)
	err := d.waitWhileBusy(time.Second)
	if err != nil {
		return err
	}
	d.cs(false)
	d.spiTxBuf = d.spiTxBuf[:0]
	d.spiTxBuf = append(d.spiTxBuf, cmdSetPacketParams, uint8(exponent<<4)|mantissa)
	switch headerType {
	case lora.HeaderExplicit:
		d.spiTxBuf = append(d.spiTxBuf, loraHeaderExplicit)
	case lora.HeaderImplicit:
		d.spiTxBuf = append(d.spiTxBuf, loraHeaderImplicit)
	default:
		return errInvalidHeaderType
	}
	d.spiTxBuf = append(d.spiTxBuf, payloadLength)
	if crcEnabled {
		d.spiTxBuf = append(d.spiTxBuf, loraCRCEnable)
	} else {
		d.spiTxBuf = append(d.spiTxBuf, loraCRCDisable)
	}
	if iqInversion {
		d.spiTxBuf = append(d.spiTxBuf, loraIQInverted)
	} else {
		d.spiTxBuf = append(d.spiTxBuf, loraIQStandard)
	}
	d.spiTxBuf = append(d.spiTxBuf, 0, 0) // unused parameters
	err = d.spi.Tx(d.spiTxBuf, nil)
	d.cs(true)
	return err
}

func getExponentAndMantissa(value uint32) (uint8, uint8) {
	// pulled from RadioLib https://github.com/jgromes/RadioLib/blob/master/src/modules/SX128x/cpp
	e := uint8(1)
	m := uint8(1)
	len := uint32(0)
	for e = uint8(1); e <= 15; e++ {
		for m = uint8(1); m <= 15; m++ {
			len = uint32(m) * (uint32(1 << e))
			if len >= value {
				break
			}
		}
		if len >= value {
			break
		}
	}

	return e, m
}

// Get information about the most recent packet received.
// Return the payload length, the offset in the buffer where the payload starts.
func (d *DeviceLoRa) getRxBufferStatus() (uint8, uint8, error) {
	err := d.waitWhileBusy(time.Second)
	if err != nil {
		return 0, 0, err
	}
	d.cs(false)
	d.spiTxBuf = d.spiTxBuf[:0]
	d.spiTxBuf = append(d.spiTxBuf, cmdGetRxBufferStatus, 0x00, 0x00, 0x00)
	d.spiRxBuf = d.spiRxBuf[:4]
	err = d.spi.Tx(d.spiTxBuf, d.spiRxBuf)
	d.cs(true)
	if err != nil {
		return 0, 0, err
	}
	return d.spiRxBuf[2], d.spiRxBuf[3], nil
}

// Configure the overall IRQ mask and the mapping of individual IRQs to the DIO1, DIO2 and DIO3 pins
func (d *DeviceLoRa) setDioIrqParams(irqMask irqMask, dio1Mask irqMask, dio2Mask irqMask, dio3Mask irqMask) error {
	err := d.waitWhileBusy(time.Second)
	if err != nil {
		return err
	}
	d.cs(false)
	d.spiTxBuf = d.spiTxBuf[:0]
	d.spiTxBuf = append(d.spiTxBuf, cmdSetDIOIRQParams, uint8((irqMask&0xFF00)>>8), uint8(irqMask&0x00FF))
	d.spiTxBuf = append(d.spiTxBuf, uint8((dio1Mask&0xFF00)>>8), uint8(dio1Mask&0x00FF))
	d.spiTxBuf = append(d.spiTxBuf, uint8((dio2Mask&0xFF00)>>8), uint8(dio2Mask&0x00FF))
	d.spiTxBuf = append(d.spiTxBuf, uint8((dio3Mask&0xFF00)>>8), uint8(dio3Mask&0x00FF))
	err = d.spi.Tx(d.spiTxBuf, nil)
	d.cs(true)
	return err
}

// Get the current IRQ status.
func (d *DeviceLoRa) getIrqStatus() (irqMask, error) {
	err := d.waitWhileBusy(time.Second)
	if err != nil {
		return 0, err
	}
	d.cs(false)
	d.spiTxBuf = d.spiTxBuf[:0]
	d.spiTxBuf = append(d.spiTxBuf, cmdGetIRQStatus, 0x00, 0x00, 0x00)
	d.spiRxBuf = d.spiRxBuf[:4]
	err = d.spi.Tx(d.spiTxBuf, d.spiRxBuf)
	d.cs(true)
	if err != nil {
		return 0, err
	}
	return irqMask(uint16(d.spiRxBuf[2])<<8 | uint16(d.spiRxBuf[3])), err
}

// Clear the IRQ bits specified in the irqMask.
func (d *DeviceLoRa) clearIrqStatus(irqMask irqMask) error {
	err := d.waitWhileBusy(time.Second)
	if err != nil {
		return err
	}
	d.cs(false)
	d.spiTxBuf = d.spiTxBuf[:0]
	d.spiTxBuf = append(d.spiTxBuf, cmdClearIRQStatus, uint8((irqMask&0xFF00)>>8), uint8(irqMask&0x00FF))
	err = d.spi.Tx(d.spiTxBuf, nil)
	d.cs(true)
	return err
}
