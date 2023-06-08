package sx127x

import (
	"errors"
	"time"

	"github.com/soypat/lora"
)

type DeviceLoRaBare struct {
	DL DeviceLoRa
}

func (d *DeviceLoRaBare) Configure(c lora.Config) (err error) {
	d.DL.Reset()
	// Seems to take just under ~6ms. Wait 20ms.
	err = d.DL.rxFSKChainCalibration(20 * time.Millisecond)
	if err != nil {
		return err
	}
	err = d.SetOpModeBare(OpSleep)
	// err = d.DL.InitLoRaMode()
	if err != nil {
		return err
	}

	for _, irv := range initRegValues {
		err = d.setModem(irv.modem)
		if err != nil {
			return err
		}
		err = d.DL.Write8(irv.addr, irv.val)
		if err != nil {
			return err
		}
	}
	return d.setModem(modeFSK)

	// Set up FIFO
	// We configure so that we can use the entire 256 byte FIFO for either receive
	// or transmit, but not both at the same time
	d.DL.Write8(regFIFO_TX_BASE_ADDR, 0)
	d.DL.Write8(regFIFO_RX_BASE_ADDR, 0)

	// Bw125Cr45Sf128
	d.DL.Write8(regMODEM_CONFIG_1, 0x72)
	d.DL.Write8(regMODEM_CONFIG_2, 0x74)
	d.DL.Write8(regMODEM_CONFIG_3, 0x00)

	d.DL.setPreambleLength(8)
	d.DL.setFrequency(lora.Freq169_4M)
	return d.crappySetTxPower(13, false)
}

func (d *DeviceLoRaBare) crappySetTxPower(power int8, useRFO bool) (err error) {
	if useRFO {
		if power > 14 {
			power = 14
		} else if power < -1 {
			power = -1
		}
		err = d.DL.Write8(regPA_CONFIG, 0x70|uint8(power+1))
	} else {
		if power > 23 {
			power = 23
		} else {
			power = 5
		}
		// For RH_RF95_PA_DAC_ENABLE, manual says '+20dBm on PA_BOOST when OutputPower=0xf'
		// RH_RF95_PA_DAC_ENABLE actually adds about 3dBm to all power levels. We will us it
		// for 21, 22 and 23dBm
		if power > 20 {
			d.DL.Write8(regPA_DAC, 0x07)
			power -= 3
		} else {
			d.DL.Write8(regPA_DAC, 0x04)
		}
		// RFM95/96/97/98 does not have RFO pins connected to anything. Only PA_BOOST
		// pin is connected, so must use PA_BOOST
		// Pout = 2 + OutputPower.
		// The documentation is pretty confusing on this topic: PaSelect says the max power is 20dBm,
		// but OutputPower claims it would be 17dBm.
		const paSelect = 0x80
		err = d.DL.Write8(regPA_CONFIG, paSelect|uint8(power-5))
	}
	return err
}

func (d *DeviceLoRaBare) SetOpModeBare(mode OpMode) (err error) {
	return d.DL.writeMasked8(regOP_MODE, uint8(mode), 0x07)
}

func (d *DeviceLoRaBare) Send(packet []byte) (err error) {
	if len(packet) > 251 {
		return errors.New("too long")
	}
	gotmode, err := d.DL.GetOpMode()
	if err != nil {
		return err
	}
	if gotmode != OpStandby && gotmode != OpSleep {
		return errors.New("not in standby or sleep before Tx")
	}
	err = d.DL.SetOpMode(OpStandby)
	if err != nil {
		return err
	}
	// Position at the beginning of the FIFO.
	d.DL.Write8(regFIFO_ADDR_PTR, 0)
	for i := range packet {
		d.DL.Write8(regFIFO, packet[i])
	}
	d.DL.Write8(regPAYLOAD_LENGTH, uint8(len(packet)))
	err = d.DL.SetOpMode(OpTx)
	if err != nil {
		return err
	}
	return d.DL.SetIRQTxDoneOnDIO0()
}

func (d *DeviceLoRaBare) HandleInterrupt() {
	flags, _ := d.DL.read8(regIRQ_FLAGS)
	if flags != 0 {
		println("got irqs:", irqFlagsString(flags))
	}
	// Clear flags
	d.DL.Write8(regIRQ_FLAGS, 0xff)
}

func (d *DeviceLoRaBare) Read(addr uint8) uint8 {
	val, _ := d.DL.read8(addr)
	return val
}

const modeFSK, modeLoRa = 0, 1

var initRegValues = []struct {
	modem uint8
	addr  uint8
	val   uint8
}{
	{modeFSK, regLNA, 0x23},
	{modeFSK, regRXConfig, 0x1E},
	{modeFSK, regRSSIConfig, 0xD2},
	{modeFSK, regAFCFEI, 0x01},
	{modeFSK, regPreambleDetect, 0xAA},
	{modeFSK, regOsc, 0x07},
	{modeFSK, regSyncConfig, 0x12},
	{modeFSK, regSyncValue1, 0xC1},
	{modeFSK, regSyncValue2, 0x94},
	{modeFSK, regSyncValue3, 0xC1},
	{modeFSK, regPacketConfig1, 0xD8},
	{modeFSK, regFifoThresh, 0x8F},
	{modeFSK, regImageCal, 0x02},
	{modeFSK, regDIO_MAPPING_1, 0x00},
	{modeFSK, regDIO_MAPPING_2, 0x30},
	{modeLoRa, regDETECTION_OPTIMIZE, 0x43},
	{modeLoRa, regMAX_PAYLOAD_LENGTH, 0x40},
}
