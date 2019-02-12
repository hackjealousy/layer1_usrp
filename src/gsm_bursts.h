#pragma once

/*
 * Standard Tail Bits
 */
static const int TB_LEN	= 3;
static const int TB_OS1	= 0;
static const int TB_OS2	= 145;
static const unsigned char tail_bits[TB_LEN] = {0, 0, 0};


/*
 * The frequency correction burst is used for frequency synchronization
 * of the mobile.  This is broadcast in TS0 together with the SCH and
 * BCCH.
 *
 * Modulating the bits below causes a spike at 1625 / 24 (67.708333) kHz above
 * the center frequency.  One can use this spike to accurately determine the
 * center of the channel.
 *
 * Since each bit, after differential encoding, is a 1, the phase is increased
 * by pi / 2 each bit.  The bit speed is (1625000 / 6) and so, as after 4 bits
 * a full cycle occurs, there is a tone at (1625000 / 6) / 4 Hz.
 */
static const int FC_CODE_LEN	= 142;
static const int FC_OS		= 3;
static const unsigned char fc_fb[FC_CODE_LEN] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static const unsigned char fc_fb_tb[TB_LEN + FC_CODE_LEN + TB_LEN] = {
	0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0
};


/*
 * The synchronization burst is used for time synchronization of the
 * mobile.  The bits given below were chosen for their correlation
 * properties.  The synchronization channel (SCH) contains a long
 * training sequence (given below) and carries the TDMA frame number and
 * base station identity code.  It is broadcast in TS0 in the frame
 * following the frequency correction burst.
 */
static const int SB_CODE_LEN	= 64;			// length of training sequence code
static const int SB_ETS_OS	= 42;			// offset from start of burst to training sequence code
static const int SB_EDATA_LEN_1	= 39;			// length of first data part
static const int SB_EDATA_OS_1	= 3;			// offset from start of burst to first data part
static const int SB_EDATA_LEN_2	= 39;			// length of second data part
static const int SB_EDATA_OS_2	= 106;			// offset from start of burst to second data part
static const unsigned char sb_etsc[SB_CODE_LEN] = {
	1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 0,
	0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
	0, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1,
	0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1
};


/*
 * The normal burst is used to carry information on traffic and control
 * channels.
 */
static const int N_TSC_NUM	= 8;	// number of training sequence codes
static const int N_TSC_CODE_LEN	= 26;	// length of tsc
static const int N_TSC_OS	= 61;	// tsc offset
static const int N_EDATA_LEN_1	= 58;	// length of first data section
static const int N_EDATA_OS_1	= 3;	// offset of first data section
static const int N_EDATA_LEN_2	= 58;	// length of second data section
static const int N_EDATA_OS_2	= 87;	// offset of second data section
static const unsigned char n_tsc[N_TSC_NUM][N_TSC_CODE_LEN] = {
/* 0 */	{
		0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0,
		0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1
	},
/* 1 */	{
		0, 0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1,
		1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1 
	},
/* 2 */	{
		0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1,
		0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0
	},
/* 3 */	{
		0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0,
		1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0
	},
/* 4 */	{
		0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0,
		1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1
	},
/* 5 */	{
		0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0,
		0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0
	},
/* 6 */	{
		1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1,
		0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1
	},
/* 7 */	{
		1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0,
		0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0
	}
};


/*
 * A base tranceiver station must transmit a burst in every timeslot of
 * every TDMA frame in channel C0.  The dummy burst will be transmitted
 * on all timeslots of all TDMA frames for which no other channel
 * requires a burst to be transmitted.
 */
static const int D_CODE_LEN	= 142;
static const int D_MB_OS	= 3;
static const unsigned char d_mb[D_CODE_LEN] = {
	1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0,
	0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0,
	0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0,
	0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0,
	0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0,
	0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 0,
	0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1,
	1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1,
	0, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0
};


/*
 * The access burst is used for random access from a mobile.
 */
static const int AB_ETB_CODE_LEN	= 8;
static const int AB_ETB_OS		= 0;
static const unsigned char ab_etb[AB_ETB_CODE_LEN] = {
	0, 0, 1, 1, 1, 0, 1, 0
};

static const int AB_SSB_CODE_LEN	= 41;
static const int AB_SSB_OS		= 8;
static const unsigned char ab_ssb[AB_SSB_CODE_LEN] = {
	0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1,
	1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0,
	0, 0, 1, 1, 1, 1, 0, 0, 0
};

static const unsigned char ab_ts1_ssb[AB_SSB_CODE_LEN] = {
	0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0,
	1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1,
	0, 0, 1, 0, 0, 1, 1, 0, 1
};

static const unsigned char ab_ts2_ssb[AB_SSB_CODE_LEN] = {
	1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1,
	0, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1,
	1, 0, 1, 1, 1, 0, 1, 1, 1
};
