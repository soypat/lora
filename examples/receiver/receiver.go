//go:build tinygo

package main

import (
	"context"
	"io"
	"machine"
	"strconv"
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
	// Replace with your frequency:
	err = dev.Configure(sx127x.DefaultConfig(lora.Freq433_0M))
	if err != nil {
		panic(err.Error())
	}
	println("config success; receiving messages")
	var rx [256]byte
	// This blocks forever reading messages received.
	ctx := context.Background()
	for {
		value, err := dev.ReadTemperature()
		println("temperature:", value, err)
		time.Sleep(10 * time.Second)
		err = dev.RxContinuous(ctx, func(r io.Reader) (_ error) {
			n, err := r.Read(rx[:])
			if err != nil {
				return err
			}
			println("received LoRa length", n, ":", strconv.Quote(string(rx[:n])))
			return nil
		})

		if err != nil {
			println(err.Error())
		} else {
			println("RX finished no error")
		}
	}
}
