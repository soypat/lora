package sx127x

import (
	"encoding/binary"
	"errors"
	"time"

	"github.com/soypat/lora"
)

// TODO: page 82: To minimize the current consumption of the SX1276/77/78/79, please ensure that the CLKOUT signal is disabled when not required.

type PinOutput func(level bool)

type SPI interface {
	Tx(writeBuffer, readBuffer []byte) error
}

type DeviceLoRa struct {
	rst PinOutput
	cs  PinOutput
	bus SPI
}

func NewLoRa(bus SPI, cs, reset PinOutput) *DeviceLoRa {
	d := DeviceLoRa{bus: bus, cs: cs, rst: reset}
	return &d
}

func (d *DeviceLoRa) Configure(cfg lora.Config) error {
	switch {
	case cfg.SpreadFactor < lora.SF6 || cfg.SpreadFactor > lora.SF12:
		return errors.New("bad spread factor")
	case cfg.SpreadFactor == lora.SF6 && cfg.HeaderType != lora.HeaderImplicit:
		return errors.New("SF6 can only be used with implicit header type") // Page 30: Implicit Header Mode.
	case cfg.PreambleLength < 6:
		return errors.New("preamble length too short")
	}
	d.Reset()
	// We need to be in sleep mode to set LoRa mode if in FSK/OOK.
	d.write8(regOP_MODE, opmSLEEP) // No need to check error, do it in SetOpmode.
	if !d.IsConnected() {
		return errors.New("sx127x not detected")
	}
	err := d.SetOpMode(OpSleep)
	if err != nil {
		return err
	}
	err = d.setTxPower(cfg.TxPower)
	if err != nil {
		return err
	}
	return nil
}

func (d *DeviceLoRa) SetOpMode(mode OpMode) error {
	// We always write the LoRa mode bit
	err := d.write8(regOP_MODE, byte(mode|opLoRaBit))
	if err != nil {
		return err
	}
	if mode == OpSleep {
		time.Sleep(15 * time.Millisecond) // TODO: do we need this sleep?
	}
	got, err := d.GetOpMode()
	if err != nil {
		return err
	}
	if got != mode {
		return errors.New("tried to set opmode " + mode.String() + ", got " + got.String())
	}
	return nil
}

func (d *DeviceLoRa) GetOpMode() (OpMode, error) {
	const invalidOpMode = 0xff
	got, err := d.read8(regOP_MODE)
	if err != nil {
		return invalidOpMode, err
	}
	if got&byte(opLoRaBit) == 0 {
		// if the LoRa mode bit is not set, which would mean the device is in
		// FSK/OOK mode or disconnected.
		return invalidOpMode, errors.New("device in FSK/OOK mode")
	}
	return OpMode(got & opmMASK), nil
}

// setPreambleLength defines number of preamble
func (d *DeviceLoRa) setPreambleLength(pLen uint16) error {
	var buf [2]byte
	binary.BigEndian.PutUint16(buf[:], pLen)
	d.write8(regPREAMBLE_MSB, buf[0])
	return d.write8(regPREAMBLE_LSB, buf[1])
}

// SetCrc Enable CRC generation and check on payload.
func (d *DeviceLoRa) enableCRC(enable bool) error {
	return d.writeMasked8(regMODEM_CONFIG_2, 1<<2, b2u8(enable)<<2)
}

// setOCP defines Overload Current Protection configuration. It receives
// the max current (Imax) in milliamperes.
func (d *DeviceLoRa) setOCP(mA uint8) error {
	const ocpEnabledMask = 1 << 5
	if mA < 45 {
		mA = 45 // Absolute minimum is 45mA.
	}
	var ocpTrim uint8
	switch {
	case mA <= 120: // Imax [mA] = 45 +5*OcpTrim
		ocpTrim = (mA - 45) / 5
	case mA <= 240: // Imax [mA] = -30 + 10*OcpTrim
		ocpTrim = (mA + 30) / 10
	default: // Imax = 240mA
		ocpTrim = 27
	}
	return d.write8(regOCP, ocpEnabledMask|(0x1F&ocpTrim))
}

// setTxPower sets the transmit power without using te PA_BOOST.
func (d *DeviceLoRa) setTxPower(txPow int8) error {
	if txPow >= 16 {
		return errors.New("requested tx power exceeds capabilities without PA_BOOST")
	}
	// Pout=Pmax-(15-OutputPower)
	// Pmax=10.8+0.6*MaxPower [dBm]
	const Pmax = 0b111 // Use Pmax ceiling.
	const PoutMask = 0b1111
	Pout := Pmax - (15 - txPow)
	if Pout < 0 {
		Pout = 0
	}
	// This unsets PaSelect bit which switches mode of operation to RFO pin (limited to 14dBm power).
	err := d.write8(regPA_CONFIG, (Pmax<<4)|(PoutMask&uint8(Pout)))
	if err != nil {
		return err
	}
	return d.write8(regOCP, 0) // TODO: Disable OCP?
}

// setFrequency sets the radio frequency.
func (d *DeviceLoRa) setFrequency(freq uint32) error {
	var freqReg [3]byte
	frf := freq / fSTEP // Page 82, 5.3.3 PLL.
	freqReg[0] = byte(frf >> 16)
	freqReg[1] = byte(frf >> 8)
	freqReg[2] = byte(frf >> 0)
	d.write8(regFRF_MSB, freqReg[0])
	d.write8(regFRF_MID, freqReg[1])
	// Note pg82: A change in the center frequency will only be taken into account when the
	// least significant byte FrfLsb in RegFrfLsb is written.
	return d.write8(regFRF_LSB, freqReg[2]) // Write LSB last!
}

func (d *DeviceLoRa) writeMasked8(addr uint8, mask, value byte) error {
	if value != 0 && value&^mask != 0 {
		panic("misuse of writeMasked8") // Bug in this package if hit.
	}
	existing, err := d.read8(addr)
	if err != nil {
		return err
	}
	existing &^= mask        // remove mask bits from register value.
	existing |= mask & value // add value's bits as masked.
	return d.write8(addr, existing)
}

func (d *DeviceLoRa) read8(addr byte) (byte, error) {
	readBuf := make([]byte, 2)
	writeBuf := []byte{addr &^ (1 << 7), 0} // unset write bit. Should be pretty much useless though we play it safe.
	d.csEnable(true)
	err := d.bus.Tx(writeBuf, readBuf)
	d.csEnable(false)
	return readBuf[1], err
}

func (d *DeviceLoRa) write8(addr, value byte) error {
	readBuf := make([]byte, 2)
	writeBuf := []byte{addr | (1 << 7), value} // set write bit.
	d.csEnable(true)
	err := d.bus.Tx(writeBuf, readBuf)
	d.csEnable(false)
	return err
}

func (d *DeviceLoRa) csEnable(b bool) {
	d.cs(!b)
}

func (d *DeviceLoRa) Reset() {
	d.rst(true)
	time.Sleep(200 * time.Millisecond)
	d.rst(false)
	time.Sleep(200 * time.Millisecond)
	d.rst(true)
	time.Sleep(200 * time.Millisecond)
}

func (d *DeviceLoRa) IsConnected() bool {
	version, err := d.read8(regVERSION)
	if version == expectedVersion && err == nil {
		return true
	}
	return false
}

func b2u8(b bool) uint8 {
	if b {
		return 1
	}
	return 0
}
