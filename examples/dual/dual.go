package main

import (
	"fmt"
	"machine"
	"time"

	"github.com/soypat/lora"
	"github.com/soypat/lora/sx127x"
)

const (
	pinCS0  = machine.GPIO27
	pinCS1  = machine.GPIO28
	pinRST0 = machine.GPIO29
	pinRST1 = machine.GPIO29
	pinMOSI = machine.GPIO3
	pinMISO = machine.GPIO4
	pinSCK  = machine.GPIO6
)

var (
	dev0, dev1 *sx127x.DeviceLoRa
	cfg        lora.Config
)

func main() {
	time.Sleep(1500 * time.Millisecond)
	println("program start")
	var err error
	setup()
	defer println("program end")
	packet := []byte("hello LoRa")
	for {
		err = dev0.Configure(cfg)
		dev0.SetOCP(45)
		for err == nil { // Tx Loop
			println("transmitting packet")
			err = dev0.Tx(packet)
			time.Sleep(5 * time.Second)
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
	pinCS1.Configure(machine.PinConfig{Mode: machine.PinOutput})
	pinRST0.Configure(machine.PinConfig{Mode: machine.PinOutput})
	pinRST1.Configure(machine.PinConfig{Mode: machine.PinOutput})
	dev0 = sx127x.NewLoRa(bus, pinCS0.Set, pinRST0.Set)
	dev1 = sx127x.NewLoRa(bus, pinCS1.Set, pinRST1.Set)

	cfg = sx127x.DefaultConfig(lora.Freq433_0M)
	cfg.TxPower = 10
}

func _() {
	var err error
	var reg, fifo [256]byte
	dev0.Reset()
	if !dev0.IsConnected() {
		panic("not connected")
	}
	dev0.Write8(1, 0)
	time.Sleep(100 * time.Millisecond)
	// dev0.SetOpMode(sx127x.OpSleep)
	err = dev0.SetOpMode(sx127x.OpSleep)
	if err != nil {
		panic(err.Error())
	}
	time.Sleep(100 * time.Millisecond)
	err = dev0.FullStatus(fifo[:], reg[:])
	if err != nil {
		panic(err.Error())
	}
	fmt.Printf("precomreg: %v\n", reg[:0x7d+1])
	err = dev0.Configure(cfg)
	if err != nil {
		panic(err.Error())
	}
	err = dev0.FullStatus(fifo[:], reg[:])
	if err != nil {
		panic(err.Error())
	}
	fmt.Printf("poscomreg: %v\n", reg[:0x7d+1])
}
