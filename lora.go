package lora

import "time"

type Config struct {
	// Bandwidth relates to data rate of communication. Double the bandwidth means
	// double the data rate, which implies faster communications and less energy usage.
	Bandwidth Frequency
	Frequency Frequency
	// Length of the preamble preceding the header. Can be arbitrarily long.
	// Longer preambles ensure more robust communications but can lead to congestion.
	//
	// Note: The preamble length on a receiver should be configured equal-to or greater
	// than the expected packet preamble length on the SX1278.
	PreambleLength uint16
	HeaderType     HeaderType
	CodingRate     CodingRate
	SpreadFactor   SpreadFactor
	SyncWord       uint8
	TxPower        int8 // Tx Power in dBm.
	CRC            bool
	// Low data rate optimisation flag. The use of this flag is mandated when
	// the symbol duration exceeds 16ms. Increases reliability at high spreading factors.
	LDRO bool
	// IQInversion configures I and Q signal inversion.
	IQInversion bool
}

// SymbolPeriod returns the time it takes to transmit a single symbol given the
// current configuration parameters. It depends on Spread factor and Bandwidth.
func (cfg *Config) SymbolPeriod() time.Duration {
	T_s := time.Second * time.Duration(cfg.SpreadFactor.ChipsPerSymbol()) /
		time.Duration(cfg.Bandwidth.Hertz())
	return T_s
}

// TimeOnAir returns the time it takes to transmit a packet of the given payload
// length. It depends on the following config parameters:
//   - Bandwidth	(proportional)
//   - CRC presence	(presence == longer)
//   - Header type	(explicit == longer)
//   - Coding rate	(proportional)
//   - Spread factor	(inversely proportional)
//   - Preamble length	(proportional)
//   - Low data rate optimisation	(presence == longer)
func (cfg *Config) TimeOnAir(payloadLength int) time.Duration {
	if cfg.Bandwidth == 0 {
		return 0
	}
	crc := int64(b2u8(cfg.CRC))
	ih := int64(cfg.HeaderType)
	ldr := int64(b2u8(cfg.LDRO))
	cr := int64(cfg.CodingRate)
	spread := int64(cfg.SpreadFactor)
	// Page 31 SX1276IMLTRT SEMTECH | Alldatasheet.
	Npayload := 8*int64(payloadLength) - 4*spread + 28 + 16*crc - 20*ih
	div := 4 * (spread - 2*ldr)
	// Apply Ceil and max with minimal branching.
	if Npayload < 0 || div <= 0 {
		Npayload = 0
	} else if Npayload%div == 0 {
		Npayload /= div
		Npayload *= (cr + 4)
	} else {
		Npayload /= div
		Npayload++
		Npayload *= (cr + 4)
	}
	// Says 4.25 in manual but we round up to 5. This means we'll overestimate the
	// time calculated.
	Npayload += 8 + int64(cfg.PreambleLength) + 5
	// Calculate LoRa Transmission Parameter Relationship page 28.
	chipsPerSymbol := cfg.SpreadFactor.ChipsPerSymbol() // chips per symbol.
	return time.Second * time.Duration(Npayload*chipsPerSymbol) /
		time.Duration(cfg.Bandwidth.Hertz())
}

type HeaderType uint8

const (
	HeaderExplicit HeaderType = 0
	HeaderImplicit HeaderType = 1
)

type CodingRate uint8

const (
	// 4/5 coding rate.
	CR4_5 CodingRate = 1
	// 4/6 coding rate.
	CR4_6 CodingRate = 2
	// 4/7 coding rate.
	CR4_7 CodingRate = 3
	// 4/8 coding rate.
	CR4_8 CodingRate = 4
)

type SpreadFactor uint8

const (
	SF6 SpreadFactor = iota + 6
	SF7
	SF8
	SF9
	SF10
	SF11
	SF12
)

func (sf SpreadFactor) ChipsPerSymbol() int64 {
	return 1 << sf
}

type Frequency int64

func (f Frequency) Hertz() int64 { return int64(f) }

const (
	Hertz     Frequency = 1
	KiloHertz Frequency = 1000 * Hertz
	MegaHertz Frequency = 1000 * KiloHertz
)

// Common LoRa bandwidths
const (
	BW125k = 125 * KiloHertz
	BW250k = 250 * KiloHertz
	BW500k = 500 * KiloHertz
)

// Common LoRa frequencies
const (
	// 433.05MHz  Low limit medical, scientific and industrial band.
	Freq433_0M = 433050000 * Hertz
	// 434.8MHz High limit medical, scientific and industrial band.
	Freq434_8M = 434790000 * Hertz
	Freq868_1M = 868100000 * Hertz
	Freq868_5M = 868500000 * Hertz
	Freq916_8M = 916800000 * Hertz
	Freq923_3M = 923300000 * Hertz
)

// 169.4MHz radio band ([Wize]), formerly known as ERMES band. Historically used by pagers.
//
// [Wize]: https://en.wikipedia.org/wiki/Wize_technology
const (
	Freq169_4M = 169400000 * Hertz
	Freq169_8M = 169812500 * Hertz
)

func b2u8(b bool) uint8 {
	if b {
		return 1
	}
	return 0
}
