package sx127x

import (
	"math"
	"testing"
)

func TestRFOPowReg(t *testing.T) {
	for txPow := int8(-4); txPow < 17; txPow++ {
		maxPow, outPow := rfoPowReg(txPow)
		Pmax := 10.8 + 0.6*float64(maxPow)
		Pout := Pmax - (15 - float64(outPow))
		diff := math.Abs(Pout - float64(txPow))
		if diff > .3 {
			t.Errorf("wanted %d, got %f", txPow, Pout)
		}
	}
}

func TestPAPowReg(t *testing.T) {
	for txPow := int8(2); txPow < 20; txPow++ {
		outPow := paBoostPowReg(txPow)
		Pout := 17.0 - (15 - float64(outPow))
		diff := math.Abs(Pout - float64(txPow))
		if diff > .01 {
			t.Errorf("wanted %d, got %f", txPow, Pout)
		}
	}
}

func TestOCPTrim(t *testing.T) {
	for imax := uint8(45); imax < 241; imax++ {
		var gotImax float64
		trim := ocpTrim(imax)
		switch {
		case trim <= 15:
			gotImax = 45.0 + 5.0*float64(trim)
		case trim <= 27:
			gotImax = -30.0 + 10.0*float64(trim)
		default:
			gotImax = 240.0
		}
		if trim > 27 {
			t.Error("trim > 27")
		}
		diff := math.Abs(gotImax - float64(imax))
		if diff > 11 {
			t.Errorf("wanted %d, got %f", imax, gotImax)
		}
	}
}
