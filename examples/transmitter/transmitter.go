//go:build tinygo

package main

import (
	"fmt"
	"machine"
	"time"

	"github.com/soypat/lora"
	"github.com/soypat/lora/sx127x"
)

var (
	SX127X_SPI = machine.SPI0
)

const (
	SX127X_PIN_RST = machine.GP6
	// SPI definition for SX127x

	SX127X_PIN_SCK = machine.GP2
	SX127X_PIN_TX  = machine.GP3
	SX127X_PIN_RX  = machine.GP4
	SX127X_PIN_CS  = machine.GP5
)

func main() {
	time.Sleep(500 * time.Millisecond)
	println("program start")
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
	cfg := sx127x.DefaultConfig(lora.Freq433_0M)
	err = dev.Configure(cfg)
	if err != nil {
		panic(err.Error())
	}

	randomU32, _ := dev.RandomU32()
	myName := fmt.Sprintf("user-%x", randomU32%0xffff)

	packet := []byte(myName + " says hello!")
	println("config success; sending messages as ", myName, "of length", len(packet))
	for {
		err = dev.Tx(packet)
		if err != nil {
			println("got error transmitting:", err.Error())
		} else {
			print(".")
		}
		time.Sleep(2 * time.Second)
		// err = dev.Configure(cfg)
		// if err != nil {
		// 	println("got error configuring:", err.Error())
		// }
	}
}
