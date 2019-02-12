#pragma once

const double	GSM_RATE	= 1625000.0 / 6.0;		// bit rate of GMSK modulated symbols (bits / second)

const double	BURST_LEN	= 156.25;			// length in bits of a burst
const double	FRAME_LEN	= 8 * BURST_LEN;		// length in bits of a frame
const int	GUARD_LEN	= 8;				// length in bits of guard period in a normal burst
const int	DATA_LEN	= 148;				// length in bits of data in a normal burst

const double	FCCH_FREQ	= GSM_RATE / 4.0;		// frequency of peak in a FCCH burst (Hz)

const double	QN_TIME		= 12.0l / 13.0l / 1000000.0l;	// time in a quarter-bit (seconds)
const double	BN_TIME		= 4.0l * QN_TIME;		// time in a bit (seconds)
const double	TN_TIME		= BURST_LEN * BN_TIME;		// time in a burst (seconds)
const double	FN_TIME		= 8.0l * TN_TIME;		// time in a frame (seconds)

const int	MAX_FN		= 26 * 51 * 2048;		// maximum frame number

const double	T		= 1.0l / GSM_RATE;		// time for 1 bit
const double	B		= 0.3l / T;			// normalized bandwidth (BT = 0.3)
