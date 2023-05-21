/*
package sx127x implements a driver for the SX127x LoRa tranceiver family.
It is a low-level driver that exposes the device's registers and their

The SX127x family includes the SX1276, SX1277, SX1278 and SX1279. The
differences between them are the supported frequency bands and the
maximum output power.

# FIFO and packet handling

The SX127x has a 256-byte FIFO buffer that is accesible via the SPI interface.
The FIFO is shared by the transmitter and the receiver but can be split via
the FifoTxBaseAddr and FifoRxBaseAddr registers.

What follows is a diagram of the FIFO buffer and the data pointers:

	+-------------------+
	|                   |
	|       Unused      |
	|                   |
	+-------------------+ <-- RxByteAddr (address of last byte received)
	|                   | ^
	|     Packet N      | |  FifoRxBytesUp (Only for )
	|                   | |
	+-------------------+ <-- RegFifoRxCurrentAddr (address of start of last packet received)
	|     Packet N-1    |
	+-------------------+
	|        ...        | <-- FifoAddrPtr (SPI read/write pointer)
	+-------------------+
	|     Packet 0      |
	+-------------------+ <-- FifoRxBaseAddr
	|       Unused      |
	+-------------------+
	|                   | ^
	|  Packet to        | |  PayloadLength
	|  Transmit         | |
	+-------------------+ <-- RegFifoTxCurrentAddr
	|      Unused       |
	+-------------------+

The FIFO is accessible through the SPI
*/
package sx127x

import (
	"encoding/binary"
	"errors"
	"io"
	"runtime"
	"time"

	"github.com/soypat/lora"
)

// TODO: page 82: To minimize the current consumption of the SX1276/77/78/79, please ensure that the CLKOUT signal is disabled when not required.

// PinOutput is a function that sets the logic-level of a pin to high (true)
// or low (false). It is used to abstract a GPIO pin interface.
type PinOutput func(level bool)

type SPI interface {
	Transfer(w byte) (byte, error)
	Tx(writeBuffer, readBuffer []byte) error
}

type DeviceLoRa struct {
	rst        PinOutput
	cs         PinOutput
	bus        SPI
	headerType lora.HeaderType
}

func NewLoRa(bus SPI, cs, reset PinOutput) *DeviceLoRa {
	d := DeviceLoRa{bus: bus, cs: cs, rst: reset}
	return &d
}

var (
	errBadSpread            = errors.New("bad spread factor")
	errSF6Implicit          = errors.New("SF6 can only be used with implicit header type") // Page 30: Implicit Header Mode.
	errPreambleTooShort     = errors.New("preamble length too short")
	ErrNotDetected          = errors.New("sx127x not detected")
	errBadMode              = errors.New("bad mode: sx127x in FSK/OOK mode, not LoRa or viceversa")
	errBadCodingRate        = errors.New("bad coding rate")
	errUnsupportedBandwidth = errors.New("bandwidth too high for frequency around 169MHz")
	errIRQNotCleared        = errors.New("IRQs not cleared")
)

func (d *DeviceLoRa) Configure(cfg lora.Config) (err error) {
	switch {
	case cfg.SpreadFactor < lora.SF6 || cfg.SpreadFactor > lora.SF12:
		err = errBadSpread
	case cfg.SpreadFactor == lora.SF6 && cfg.HeaderType != lora.HeaderImplicit:
		err = errSF6Implicit
	case cfg.PreambleLength < 6:
		err = errPreambleTooShort
	case cfg.CodingRate < lora.CR4_5 || cfg.CodingRate > lora.CR4_8:
		err = errBadCodingRate
	case cfg.Frequency < 175*lora.MegaHertz && cfg.Bandwidth > 125*lora.KiloHertz:
		err = errUnsupportedBandwidth
	case cfg.HeaderType != lora.HeaderImplicit && cfg.HeaderType != lora.HeaderExplicit:
		err = errors.New("bad header type")
	case cfg.TxPower > 20:
		err = errors.New("bad tx power")
	}
	if err != nil {
		return err
	}
	d.Reset()
	// We need to be in sleep mode to set LoRa mode if in FSK/OOK.
	d.write8(regOP_MODE, opmSLEEP) // No need to check error, do it in SetOpmode.
	if !d.IsConnected() {
		return ErrNotDetected
	}
	err = d.SetOpMode(OpSleep)
	if err != nil {
		return err
	}
	err = d.setFrequency(cfg.Frequency)
	if err != nil {
		return err
	}
	err = d.setBandwidth(cfg.Bandwidth)
	if err != nil {
		return err
	}
	err = d.enableCRC(cfg.CRC)
	if err != nil {
		return err
	}
	err = d.EnableAutoGainControl(true)
	if err != nil {
		return err
	}
	err = d.setPreambleLength(cfg.PreambleLength)
	if err != nil {
		return err
	}
	err = d.setTxPower(cfg.TxPower)
	if err != nil {
		return err
	}
	err = d.setSyncWord(cfg.SyncWord)
	if err != nil {
		return err
	}
	err = d.setCodingRate(cfg.CodingRate)
	if err != nil {
		return err
	}
	err = d.setSpreadFactorConsistent(cfg.SpreadFactor)
	if err != nil {
		return err
	}
	isImplicit := cfg.HeaderType == lora.HeaderImplicit
	err = d.enableImplicitHeaderMode(isImplicit)
	if err != nil {
		return err
	}
	err = d.enableIQInversion(cfg.IQInversion)
	if err != nil {
		return err
	}
	err = d.enableTxContinuousMode(false) // TODO: enable or disable?
	if err != nil {
		return err
	}
	err = d.setTimeoutInSymbols(1023) // Set timeout to max value.
	if err != nil {
		return err
	}
	const rxStart = 128
	d.write8(regFIFO_TX_BASE_ADDR, 0)
	d.write8(regFIFO_RX_BASE_ADDR, rxStart)
	d.setHopPeriod(0)
	d.headerType = cfg.HeaderType
	return d.SetOpMode(OpStandby)
}

func (d *DeviceLoRa) Reset() {
	d.rst(true)
	time.Sleep(200 * time.Millisecond)
	d.rst(false)
	time.Sleep(200 * time.Millisecond)
	d.rst(true)
	time.Sleep(200 * time.Millisecond)
}

// IsConnected reads the version register and checks if it matches the expected value.
func (d *DeviceLoRa) IsConnected() bool {
	version, err := d.read8(regVERSION)
	if version == expectedVersion && err == nil {
		return true
	}
	return false
}

// SetOpMode sets the operating mode of the SX127x to a LoRa mode.
func (d *DeviceLoRa) SetOpMode(mode OpMode) error {
	// We always write the LoRa mode bit
	err := d.write8(regOP_MODE, byte(mode|opLoRaBit))
	if err != nil {
		return err
	}
	if mode == OpSleep || true {
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

// GetOpMode returns the current operating mode of the SX127x. It returns an error
// if the device is not in LoRa mode.
func (d *DeviceLoRa) GetOpMode() (OpMode, error) {
	const invalidOpMode = 0xff
	got, err := d.read8(regOP_MODE)
	if err != nil {
		return invalidOpMode, err
	}
	if got&byte(opLoRaBit) == 0 || // LongRangeMode bit influences operation.
		got&(1<<6) != 0 { // AccessSharedReg bit allows access to FSK registers in LoRa mode, should not be set.
		// if the LoRa mode bit is not set, which would mean the device is in
		// FSK/OOK mode or disconnected.
		return invalidOpMode, errBadMode
	}
	return OpMode(got & opmMASK), nil
}

// SetLNAGain sets the Low Noise amplifier gain with a value between 0 and 6
// where 0 is Off and 6 is the maximum gain.
func (d *DeviceLoRa) SetLNAGain(gain uint8) error {
	if gain > 6 {
		return errors.New("gain must be between 0 and 6")
	}
	const lnaMask = 0b111 << 5
	gain = 0b111 - gain // invert gain value to reflect the fact that 0 is max gain.
	return d.writeMasked8(regLNA, lnaMask, gain<<5)
}

// EnableAutoGainControl enables/disables Automatic Gain Control. This means the value set
// by SetLNAGain will be ignored. Set to false to use the value set by SetLNAGain.
func (d *DeviceLoRa) EnableAutoGainControl(b bool) error {
	const agcMask = 1 << 2
	return d.writeMasked8(regMODEM_CONFIG_3, agcMask, b2u8(b)<<2)
}

func (d *DeviceLoRa) Tx(packet []byte) (err error) {
	if len(packet) > 255 {
		return errors.New("packet too long")
	}
	opmode, err := d.GetOpMode()
	if err != nil {
		return err
	}
	if opmode != OpStandby && opmode != OpSleep {
		println("unexpected opmode before Tx:", opmode.String())
		err = d.SetOpMode(OpSleep)
		if err != nil {
			return err
		}
	}
	err = d.write8(regPAYLOAD_LENGTH, uint8(len(packet)))
	if err != nil {
		return err
	}
	plen, _ := d.read8(regPAYLOAD_LENGTH)
	if plen != uint8(len(packet)) {
		return errors.New("payload length unable to be set correctly")
	}
	// FIFO registers only accesible in Standby mode.
	err = d.SetOpMode(OpStandby)
	if err != nil {
		return err
	}
	d.write8(regFIFO_TX_BASE_ADDR, 0)
	d.write8(regFIFO_ADDR_PTR, 0)
	for i := 0; i < len(packet); i++ {
		err := d.write8(regFIFO, packet[i])
		if err != nil {
			return err
		}
	}
	// Begin transmitting immediately.
	err = d.SetOpMode(OpTx)
	if err != nil {
		return err
	}
	counts := 0
	var reg uint8
	for {
		counts++
		reg, err = d.read8(regIRQ_FLAGS)
		if reg&irqTXDONE_MASK != 0 || err != nil {
			if err != nil {
				return err
			}
			break
		}
		runtime.Gosched() // Yield to scheduler.
	}
	err = d.clearIRQ(irqTXDONE_MASK)
	if err != nil {
		return err
	}
	return nil
}

// clearIRQ clears IRQ bits indicated by toClear:
//   - bit 0: CAD detected interrupt
//   - bit 1: FHSS change channel interrupt
//   - bit 2: CAD done interrupt
//   - bit 3: Tx done interrupt
//   - bit 4: Valid header received in Rx
//   - bit 5: Payload CRC error
//   - bit 6: Rx done interrupt
//   - bit 7: Rx timeout interrupt
func (d *DeviceLoRa) clearIRQ(toClear uint8) error {
	err := d.write8(regIRQ_FLAGS, toClear)
	if err != nil {
		return err
	}
	reg, _ := d.read8(regIRQ_FLAGS)
	if reg&toClear != 0 {
		return errIRQNotCleared
	}
	return nil
}

// RandomU32 returns a random uint32 generated by reading the RSSI during
// Rx OpMode. This method should not be used while the device is operating.
func (d *DeviceLoRa) RandomU32() (rnd uint32, err error) {
	var buf [4]byte
	err = d.RandomRead(buf[:], 10*time.Millisecond)
	if err != nil {
		return 0, err
	}
	return binary.LittleEndian.Uint32(buf[:]), d.SetOpMode(OpSleep)
}

// RandomRead reads random byte data to dst by reading the RSSI during Rx OpMode.
// The period between RSSI reads is readFromRSSIPeriod. A higher readFromRSSIPeriod
// will typically result in higher entropy in the random data. 10ms is a reasonable period.
// This method will take approximately readFromRSSIPeriod*len(dst)*8 + 50ms to complete.
func (d *DeviceLoRa) RandomRead(dst []byte, readFromRSSIPeriod time.Duration) error {
	// Disable ALL irqs
	err := d.clearIRQ(0xff)
	if err != nil {
		return err
	}
	err = d.SetOpMode(OpRx)
	if err != nil {
		return err
	}
	for i := 0; i < len(dst)*8; i++ {
		time.Sleep(readFromRSSIPeriod)
		val, err := d.read8(regRSSI_WIDEBAND)
		if err != nil {
			return err
		}
		// Unfiltered RSSI value reading. Only takes the LSB value
		dst[i/8] |= (val & 1) << (i % 8)
	}
	return d.SetOpMode(OpSleep)
}

// EstimateReadFromRSSIPeriod estimates the readFromRSSIPeriod required to read
// relatively random bits of data for testDuration time.
//
// It's recommended that one call this function several times with small
// durations and use a median value as a compromise between call duration
// and entropy. It is common to get values separated by orders of magnitude
// depending on whether there was a signal present during the test.
// See the [RandomRead] method.
func (d *DeviceLoRa) EstimateReadFromRSSIPeriod(testDuration time.Duration) (time.Duration, error) {
	err := d.clearIRQ(0xff)
	if err != nil {
		return 0, err
	}
	err = d.SetOpMode(OpRx)
	if err != nil {
		return 0, err
	}
	start := time.Now()
	val, _ := d.read8(regRSSI_WIDEBAND)
	lastBit := val&1 != 0
	maxBitHoldTime := time.Duration(0)
	lastBitChangeTime := start
	var readTime = time.Now()
	for readTime.Sub(start) < testDuration {
		val, err := d.read8(regRSSI_WIDEBAND)
		if err != nil {
			return 0, err
		}
		readTime = time.Now()
		bit0 := val&1 != 0
		if bit0 != lastBit {
			lastBit = bit0
			elapsedSinceBitChange := readTime.Sub(lastBitChangeTime)
			lastBitChangeTime = readTime
			if elapsedSinceBitChange > maxBitHoldTime {
				maxBitHoldTime = elapsedSinceBitChange
			}
		}
	}
	return maxBitHoldTime, d.SetOpMode(OpSleep)
}

// setBandwidth sets the bandwidth of the LoRa modulation.
func (d *DeviceLoRa) setBandwidth(bw lora.Frequency) error {
	const bwMask = 0b1111 << 4
	bwByte := bwReg(bw)
	return d.writeMasked8(regMODEM_CONFIG_1, bwMask, bwByte<<4)
}

// enableImplicitHeaderMode enables implicit header mode (instead of explicit).
func (d *DeviceLoRa) enableImplicitHeaderMode(enable bool) error {
	return d.writeMasked8(regMODEM_CONFIG_1, 1, b2u8(enable))
}

// enableIQInversion inverts LoRa I and Q signals when set to true.
func (d *DeviceLoRa) enableIQInversion(enable bool) error {
	const iqMask = 1 << 6
	return d.writeMasked8(regINVERTIQ, iqMask, b2u8(enable)<<6)
}

// ReadConfig reads the configuration parameters from the device and returns
// the corresponding lora.Config for the current device configuration.
// Some lora.Config parameters are not set such as IQ, LDR, and Tx power.
func (d *DeviceLoRa) ReadConfig() (cfg lora.Config, err error) {
	var buf [5]byte
	err = d.read(regMODEM_CONFIG_1, buf[:5])
	if err != nil {
		return cfg, err
	}
	cfg1 := buf[0]
	cfg.Bandwidth = reg2Bw(cfg1 >> 4)
	cfg.CodingRate = lora.CodingRate(cfg1>>1) & 0b111
	cfg.HeaderType = lora.HeaderType(cfg1 & 1)
	cfg2 := buf[1]
	cfg.SpreadFactor = lora.SpreadFactor(cfg1 >> 4)
	cfg.CRC = cfg2&0x4 != 0
	// continuousMode = cfg2&0x8 != 0
	cfg.PreambleLength = binary.BigEndian.Uint16(buf[3:])
	// Read sync word.
	sync, err := d.read8(regSYNC_WORD)
	if err != nil {
		return cfg, err
	}
	cfg.SyncWord = sync
	// Read Frequency.
	err = d.read(regFRF_MSB, buf[:3])
	freq := uint64(buf[0])<<16 | uint64(buf[1])<<8 | uint64(buf[2])
	cfg.Frequency = lora.Frequency((freq * 15625) >> 8)
	// Read IQ inversion.
	iq, err := d.read8(regINVERTIQ)
	if err != nil {
		return cfg, err
	}
	cfg.IQInversion = iq&(1<<6) != 0

	return cfg, nil
}

// setPreambleLength defines number of preamble
func (d *DeviceLoRa) setPreambleLength(pLen uint16) error {
	var buf [2]byte
	binary.BigEndian.PutUint16(buf[:], pLen)
	d.write8(regPREAMBLE_MSB, buf[0])
	return d.write8(regPREAMBLE_LSB, buf[1])
}

func (d *DeviceLoRa) enableTxContinuousMode(enable bool) error {
	return d.writeMasked8(regMODEM_CONFIG_2, 1<<3, b2u8(enable)<<3)
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
	// Set to minimal current.
	return d.setOCP(45)
	// return d.write8(regOCP, 0) // TODO: Disable OCP?
}

// setFrequency sets the center radio frequency.
func (d *DeviceLoRa) setFrequency(freq lora.Frequency) error {
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

// setSpreadFactorConsistent sets the spreading factor and closely related parameters
// including Low Data Rate Optimization, DetectionOptimize, and DetectionThreshold.
func (d *DeviceLoRa) setSpreadFactorConsistent(sf lora.SpreadFactor) (err error) {
	err = d.setSpreadingFactor(sf)
	if err != nil {
		return err
	}
	isSF6 := sf == lora.SF6
	err = d.enableLowDataRateOptimization(isSF6)
	if err != nil {
		return err
	}
	// Set DetectionOptimize to 0x05 for SF6 and to 0x03 otherwise (SF7 to SF12).
	err = d.writeMasked8(regDETECTION_OPTIMIZE, 0b111, 1|(0b10<<b2u8(isSF6)))
	if err != nil {
		return err
	}
	// Set DetectionThreshold to 0x0C for SF6 and to 0x0A otherwise (SF7 to SF12).
	return d.write8(regDETECTION_THRESHOLD, 0b1000|(0b10<<b2u8(isSF6)))
}

// setSpreadingFactor sets the spreading factor. The value must be between 6 and 12.
// It does not set parameters closely associated with the spreading factor such as
// the Low Data Optimization, the Detection threshold, Detection Optimize and the
// Symbol Timeout.
func (d *DeviceLoRa) setSpreadingFactor(sf lora.SpreadFactor) error {
	if sf < 6 || sf > 12 {
		return errBadSpread
	}
	const sfMask = 0b111 << 4
	return d.writeMasked8(regMODEM_CONFIG_2, sfMask, uint8(sf)<<4)
}

func (d *DeviceLoRa) setSyncWord(sync byte) error {
	return d.write8(regSYNC_WORD, sync)
}

// enableLowDataRateOptimization enables/disables Low Data Rate Optimization, a
// feature which is mandated when symbol length exceeds 16ms.
func (d *DeviceLoRa) enableLowDataRateOptimization(b bool) error {
	const ldoMask = 1 << 3
	return d.writeMasked8(regMODEM_CONFIG_3, ldoMask, b2u8(b)<<3)
}

// enableLowFrequencyMode enables/disables access to LowFrequencyMode registers.
func (d *DeviceLoRa) enableLowFrequencyMode(b bool) error {
	const lowFreqMask = 1 << 3
	return d.writeMasked8(regOP_MODE, lowFreqMask, b2u8(b)<<3)
}

// enableCRC enables/disables CRC generation and checking.
func (d *DeviceLoRa) enableCRC(b bool) error {
	const crcMask = 1 << 2
	return d.writeMasked8(regMODEM_CONFIG_2, crcMask, b2u8(b)<<2)
}

// setTimeoutInSymbols sets the timeout in symbols. The value must be between 0 and 1023.
// The timeout is used to stop reception automatically. The equation is:
//
//	Timeout = timeoutSymbols * Ts (where Ts is the symbol period)
func (d *DeviceLoRa) setTimeoutInSymbols(symbTimeout uint16) (err error) {
	if symbTimeout > 0x3FF {
		return errors.New("timeout value too large")
	}
	err = d.writeMasked8(regMODEM_CONFIG_2, 0b11, byte(symbTimeout>>8))
	if err != nil {
		return err
	}
	return d.write8(regSYMB_TIMEOUT_LSB, byte(symbTimeout))
}

// setCodingRate sets the error coding rate. The value must be between 4/5 and 4/8.
func (d *DeviceLoRa) setCodingRate(cr lora.CodingRate) error {
	if cr < lora.CR4_5 || cr > lora.CR4_8 {
		return errBadCodingRate
	}
	return d.writeMasked8(regMODEM_CONFIG_1, 0b111<<1, uint8(cr)<<1)
}

// setHopPeriod sets number of symbol periods between frequency hops. (0 = disabled).
func (d *DeviceLoRa) setHopPeriod(val uint8) error { return d.write8(regHOP_PERIOD, val) }

func (d *DeviceLoRa) writeMasked8(addr uint8, mask, value byte) error {
	if value != 0 && value&^mask != 0 {
		println("value", value, "mask", mask)
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

func b2u8(b bool) uint8 {
	if b {
		return 1
	}
	return 0
}

func (d *DeviceLoRa) read(addr uint8, buf []byte) error {
	if len(buf) <= 1 {
		return io.ErrShortBuffer
	}
	d.csEnable(true)
	_, err := d.bus.Transfer(addr)
	if err != nil {
		d.csEnable(false)
		return err
	}
	err = d.bus.Tx(nil, buf)
	d.csEnable(false)
	return err
}
