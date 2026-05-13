package sx128x

type standbyConfig uint8
type periodBase uint8
type packetType uint8
type radioRampTime uint8

// Misc
type irqMask uint16

const (

	// StandbyConfig
	standbyRC   = standbyConfig(0)
	standbyXOSC = standbyConfig(1)

	// PeriodBase
	periodBase4Ms = periodBase(3)

	// PacketType
	packetTypeLoRa = packetType(0x01)
	packetTypeBLE  = packetType(0x04)

	// RampTime
	radioRamp02us = radioRampTime(0x00)
	radioRamp04us = radioRampTime(0x20)
	radioRamp06us = radioRampTime(0x40)
	radioRamp08us = radioRampTime(0x60)
	radioRamp10us = radioRampTime(0x80)
	radioRamp12us = radioRampTime(0xA0)
	radioRamp16us = radioRampTime(0xC0)
	radioRamp20us = radioRampTime(0xE0)

	// LoRa Modulation Params
	loraBW1600 = byte(0x0A)
	loraBW800  = byte(0x18)
	loraBW400  = byte(0x26)
	loraBW200  = byte(0x34)

	// CodingRate
	loraCR4_5    = byte(0x01)
	loraCR4_6    = byte(0x02)
	loraCR4_7    = byte(0x03)
	loraCR4_8    = byte(0x04)
	loraCRLI_4_5 = byte(0x05)
	loraCRLI_4_6 = byte(0x06)
	loraCRLI_4_8 = byte(0x07)

	// LoraPacketParams
	// HeaderType
	loraHeaderExplicit = byte(0x00)
	loraHeaderImplicit = byte(0x80)

	// CRC Type
	loraCRCEnable  = byte(0x20)
	loraCRCDisable = byte(0x00)

	// IQ Type
	loraIQInverted = byte(0x00)
	loraIQStandard = byte(0x40)

	// IRQ masks
	irqAll     = irqMask(0xFFFF)
	irqTxDone  = irqMask(0b0000000000000001)
	irqRxDone  = irqMask(0b0000000000000010)
	irqTimeout = irqMask(0b0100000000000000)
)
