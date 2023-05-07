package main

import (
	"machine"
	"time"

	"github.com/soypat/lora"
	"github.com/soypat/lora/sx127x"
)

var (
	SX127X_SPI = machine.SPI0
)

const (
	SX127X_PIN_RST = machine.GP16
	// SPI definition for SX127x

	SX127X_PIN_SCK = machine.GP2
	SX127X_PIN_TX  = machine.GP3
	SX127X_PIN_RX  = machine.GP4
	SX127X_PIN_CS  = machine.GP5
)

func main() {
	time.Sleep(500 * time.Millisecond)
	defer println("program end")
	SX127X_PIN_RST.Configure(machine.PinConfig{Mode: machine.PinOutput})
	SX127X_PIN_CS.Configure(machine.PinConfig{Mode: machine.PinOutput})
	err := SX127X_SPI.Configure(machine.SPIConfig{
		Frequency: 100000,
		SCK:       SX127X_PIN_SCK,
		SDO:       SX127X_PIN_TX,
		SDI:       SX127X_PIN_RX,
	})
	if err != nil {
		panic(err.Error())
	}
	dev := sx127x.NewLoRa(SX127X_SPI, SX127X_PIN_CS.Set, SX127X_PIN_RST.Set)
	err = dev.Configure(lora.Config{
		Bandwidth:      lora.BW125k,
		Frequency:      lora.Freq433_0M,
		CodingRate:     lora.CR4_5,
		SpreadFactor:   lora.SF7,
		HeaderType:     lora.HeaderExplicit,
		TxPower:        0,
		CRC:            true,
		PreambleLength: 12,
	})
	if err != nil {
		panic(err.Error())
	}
	println("config success")
}
