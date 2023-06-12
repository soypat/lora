package main

import (
	"machine"
	"time"

	"github.com/soypat/lora"
	"github.com/soypat/lora/sx127x"
)

const (
	pinSCK  = machine.GPIO18
	pinMOSI = machine.GPIO19
	pinMISO = machine.GPIO16
	pinCS0  = machine.GPIO8
	pinRST0 = machine.GPIO9
	pinDIO0 = machine.GPIO7
)

var (
	dev0, dev1 *sx127x.DeviceLoRaBare
	cfg        lora.Config
)

func main() {
	time.Sleep(1500 * time.Millisecond)
	println("program start")
	var err error
	setup()
	defer println("program end")
	packet := []byte("hello LoRa")
	go func() {
		var last bool
		for {
			time.Sleep(time.Millisecond * 10)
			got := pinDIO0.Get()
			if got != last {
				println("DIO0 switch: ", got)
				last = got
			}
		}
	}()
	for {
		err = dev0.Configure(cfg)
		for err == nil { // Tx Loop
			println("transmitting packet")
			err = dev0.Send(packet)
			time.Sleep(10 * time.Second)
			dev0.HandleInterrupt()
		}
		if err != nil {
			println("got error: ", err.Error())
		}
		time.Sleep(time.Second)
	}
}

func setup() {
	var bus = machine.SPI0
	err := bus.Configure(machine.SPIConfig{
		Frequency: 10_000,
		SCK:       pinSCK,
		SDO:       pinMOSI,
		SDI:       pinMISO,
	})
	if err != nil {
		panic(err.Error())
	}
	pinCS0.Configure(machine.PinConfig{Mode: machine.PinOutput})
	pinRST0.Configure(machine.PinConfig{Mode: machine.PinOutput})
	pinDIO0.Configure(machine.PinConfig{Mode: machine.PinInputPulldown})

	dev0 = &sx127x.DeviceLoRaBare{DL: *sx127x.NewLoRa(bus, pinCS0.Set, pinRST0.Set)}

	// dev1 = sx127x.NewLoRa(bus, pinCS1.Set, pinRST1.Set)

	cfg = sx127x.DefaultConfig(lora.Freq433_0M)
	cfg.TxPower = 10
}
