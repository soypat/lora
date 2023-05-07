package lora_test

import (
	"math"
	"testing"
	"time"

	"github.com/soypat/lora"
)

func TestTimeOnAir(t *testing.T) {
	const (
		loraWANCodeRate       = lora.CR4_5
		loraWANPreambleLength = 12
	)
	testCases := []struct {
		desc     string
		expected time.Duration
		plen     int
		cfg      lora.Config
	}{
		{
			desc: "LoRaWAN 240bytes", // https://www.thethingsnetwork.org/airtime-calculator
			cfg: lora.Config{
				Bandwidth:      lora.BW125k,
				Frequency:      915e6,
				SpreadFactor:   lora.SF7,
				HeaderType:     lora.HeaderExplicit,
				CodingRate:     loraWANCodeRate,
				CRC:            true,
				PreambleLength: loraWANPreambleLength,
			},
			plen:     240,
			expected: 394.5e3 * time.Microsecond,
		},
		{
			desc: "LoRaWAN 5byte BW=500kHz", // https://www.thethingsnetwork.org/airtime-calculator
			cfg: lora.Config{
				Bandwidth:      lora.BW500k,
				Frequency:      915e6,
				SpreadFactor:   lora.SF7,
				HeaderType:     lora.HeaderExplicit,
				CodingRate:     loraWANCodeRate,
				CRC:            true,
				PreambleLength: loraWANPreambleLength,
			},
			plen:     10,
			expected: 15.4e3 * time.Microsecond,
		},
	}
	for _, tC := range testCases {
		t.Run(tC.desc, func(t *testing.T) {
			got := tC.cfg.TimeOnAir(tC.plen)
			expect := tC.expected.Seconds()
			if math.Abs(got.Seconds()-expect) > 10*expect/100 { // TODO: improve accuracy of TimeOnAir
				t.Errorf("%s: got %s, expected %s", tC.desc, got.String(), tC.expected.String())
			}
		})
	}
}
