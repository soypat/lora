package sx128x

import "errors"

var (
	ErrBusyPinTimeout        = errors.New("busy pin timeout")
	errDataTooLong           = errors.New("data over 256 bytes")
	errInvalidStandbyConfig  = errors.New("invalid standby config")
	errFrequencyTooLow       = errors.New("frequency below 2.4Ghz")
	errFrequencyTooHigh      = errors.New("frequency above 2.5Ghz")
	errPowerTooLow           = errors.New("power level below -18dBm")
	errPowerTooHigh          = errors.New("power level above 13dBm")
	errInvalidPeriodBase     = errors.New("invalid period base")
	errInvalidPacketType     = errors.New("invalid packet type")
	errInvalidHeaderType     = errors.New("invalid header type")
	errInvalidBandwidth      = errors.New("invalid bandwidth")
	errInvalidCodingRate     = errors.New("invalid coding rate")
	errPayloadLengthTooShort = errors.New("payload length too short")
	errRxTimeout             = errors.New("rx timeout")
)
