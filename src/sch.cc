#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gsm.h"
#include "usrp_source.h"
#include "fcch_detector.h"
#include "gsm_bursts.h"
#include "gsm_demod.h"


/*
 * Synchronization channel.
 *
 * Timeslot	Repeat length		Frame Number (mod repeat length)
 * 0		51			1, 11, 21, 31, 41
 */

/*
 * Parity for the GSM SCH.
 *
 * 	g(x) = x^10 + x^8 + x^6 + x^5 + x^4 + x^2 + 1
 *
 * Note: The SCH parity is not a Fire code.
 */
static const unsigned int DATA_BLOCK_SIZE	= 25;
static const unsigned int PARITY_SIZE		= 10;
static const unsigned int TAIL_BITS_SIZE	= 4;
static const unsigned int PARITY_OUTPUT_SIZE	= (DATA_BLOCK_SIZE + PARITY_SIZE + TAIL_BITS_SIZE);

static const unsigned char parity_polynomial[PARITY_SIZE + 1] = {
	1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1
};

static const unsigned char parity_remainder[PARITY_SIZE] = {
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1
};


static inline void parity_encode(unsigned char *d, unsigned char *p) {

	unsigned int i;
	unsigned char buf[DATA_BLOCK_SIZE + PARITY_SIZE], *q;

	memcpy(buf, d, DATA_BLOCK_SIZE);
	memset(buf + DATA_BLOCK_SIZE, 0, PARITY_SIZE);

	for(q = buf; q < buf + DATA_BLOCK_SIZE; q++)
		if(*q)
			for(i = 0; i < PARITY_SIZE + 1; i++)
				q[i] ^= parity_polynomial[i];
	for(i = 0; i < PARITY_SIZE; i++)
		p[i] = !buf[DATA_BLOCK_SIZE + i];
}


static inline int parity_check(unsigned char *d) {

	unsigned int i;
	unsigned char buf[DATA_BLOCK_SIZE + PARITY_SIZE], *q;

	memcpy(buf, d, DATA_BLOCK_SIZE + PARITY_SIZE);

	for(q = buf; q < buf + DATA_BLOCK_SIZE; q++)
		if(*q)
			for(i = 0; i < PARITY_SIZE + 1; i++)
				q[i] ^= parity_polynomial[i];
	return memcmp(buf + DATA_BLOCK_SIZE, parity_remainder, PARITY_SIZE);
}


/*
 * Convolutional encoding and Viterbi decoding for the GSM SCH.
 *
 * Rate 1/2, order 4; (Equivalent to the GSM SACCH.)
 *
 * 	G_0 = 1 + x^3 + x^4
 * 	G_1 = 1 + x + x^3 + x^4
 *
 * i.e.,
 *
 * 	c_{2k} = u_k + u_{k - 3} + u_{k - 4}
 * 	c_{2k + 1} = u_k + u_{k - 1} + u_{k - 3} + u_{k - 4}
 */
static const unsigned int CONV_INPUT_SIZE	= PARITY_OUTPUT_SIZE;
static const unsigned int CONV_SIZE		= (2 * CONV_INPUT_SIZE);
static const unsigned int K			= 5;
static const unsigned int MAX_ERROR		= (2 * CONV_INPUT_SIZE + 1);


/*
 * Given the current state and input bit, what are the output bits?
 *
 * 	encode[current_state][input_bit]
 */
static const unsigned int encode[1 << (K - 1)][2] = {
	{0, 3}, {3, 0}, {3, 0}, {0, 3},
	{0, 3}, {3, 0}, {3, 0}, {0, 3},
	{1, 2}, {2, 1}, {2, 1}, {1, 2},
	{1, 2}, {2, 1}, {2, 1}, {1, 2}
};


/*
 * Given the current state and input bit, what is the next state?
 * 
 * 	next_state[current_state][input_bit]
 */
static const unsigned int next_state[1 << (K - 1)][2] = {
	{0, 8}, {0, 8}, {1, 9}, {1, 9},
	{2, 10}, {2, 10}, {3, 11}, {3, 11},
	{4, 12}, {4, 12}, {5, 13}, {5, 13},
	{6, 14}, {6, 14}, {7, 15}, {7, 15}
};


/*
 * Given the previous state and the current state, what input bit caused
 * the transition?  If it is impossible to transition between the two
 * states, the value is 2.
 *
 * 	prev_next_state[previous_state][current_state]
 */
static const unsigned int prev_next_state[1 << (K - 1)][1 << (K - 1)] = {
        {0,  2,  2,  2,  2,  2,  2,  2,  1,  2,  2,  2,  2,  2,  2,  2},
        {0,  2,  2,  2,  2,  2,  2,  2,  1,  2,  2,  2,  2,  2,  2,  2},
        {2,  0,  2,  2,  2,  2,  2,  2,  2,  1,  2,  2,  2,  2,  2,  2},
        {2,  0,  2,  2,  2,  2,  2,  2,  2,  1,  2,  2,  2,  2,  2,  2},
        {2,  2,  0,  2,  2,  2,  2,  2,  2,  2,  1,  2,  2,  2,  2,  2},
        {2,  2,  0,  2,  2,  2,  2,  2,  2,  2,  1,  2,  2,  2,  2,  2},
        {2,  2,  2,  0,  2,  2,  2,  2,  2,  2,  2,  1,  2,  2,  2,  2},
        {2,  2,  2,  0,  2,  2,  2,  2,  2,  2,  2,  1,  2,  2,  2,  2},
        {2,  2,  2,  2,  0,  2,  2,  2,  2,  2,  2,  2,  1,  2,  2,  2},
        {2,  2,  2,  2,  0,  2,  2,  2,  2,  2,  2,  2,  1,  2,  2,  2},
        {2,  2,  2,  2,  2,  0,  2,  2,  2,  2,  2,  2,  2,  1,  2,  2},
        {2,  2,  2,  2,  2,  0,  2,  2,  2,  2,  2,  2,  2,  1,  2,  2},
        {2,  2,  2,  2,  2,  2,  0,  2,  2,  2,  2,  2,  2,  2,  1,  2},
        {2,  2,  2,  2,  2,  2,  0,  2,  2,  2,  2,  2,  2,  2,  1,  2},
        {2,  2,  2,  2,  2,  2,  2,  0,  2,  2,  2,  2,  2,  2,  2,  1},
        {2,  2,  2,  2,  2,  2,  2,  0,  2,  2,  2,  2,  2,  2,  2,  1}
};


static inline unsigned int hamming_distance2(unsigned int w) {

	return (w & 1) + !!(w & 2);
}


static inline void conv_encode(unsigned char *data, unsigned char *output) {

	unsigned int i, state = 0, o;

	// encode data
	for(i = 0; i < CONV_INPUT_SIZE; i++) {
		o = encode[state][data[i]];
		state = next_state[state][data[i]];
		*output++ = !!(o & 2);
		*output++ = o & 1;
	}
}


static inline int conv_decode(unsigned char *data, unsigned char *output) {

	unsigned int i, t, rdata, state, nstate, b, o, distance, accumulated_error, min_state, min_error, cur_state;

	unsigned int ae[1 << (K - 1)];
	unsigned int nae[1 << (K - 1)]; // next accumulated error
	unsigned int state_history[1 << (K - 1)][CONV_INPUT_SIZE + 1];

	// initialize accumulated error, assume starting state is 0
	for(i = 0; i < (1 << (K - 1)); i++)
		ae[i] = nae[i] = MAX_ERROR;
	ae[0] = 0;

	// build trellis
	for(t = 0; t < CONV_INPUT_SIZE; t++) {

		// get received data symbol
		rdata = (data[2 * t] << 1) | data[2 * t + 1];

		// for each state
		for(state = 0; state < (1 << (K - 1)); state++) {

			// make sure this state is possible
			if(ae[state] >= MAX_ERROR)
				continue;

			// find all states we lead to
			for(b = 0; b < 2; b++) {

				// get next state given input bit b
				nstate = next_state[state][b];

				// find output for this transition
				o = encode[state][b];

				// calculate distance from received data
				distance = hamming_distance2(rdata ^ o);

				// choose surviving path
				accumulated_error = ae[state] + distance;
				if(accumulated_error < nae[nstate]) {

					// save error for surviving state
					nae[nstate] = accumulated_error;

					// update state history
					state_history[nstate][t + 1] = state;
				}
			}
		}
		
		// get accumulated error ready for next time slice
		for(i = 0; i < (1 << (K - 1)); i++) {
			ae[i] = nae[i];
			nae[i] = MAX_ERROR;
		}
	}

	// the final state is the state with the fewest errors
	min_state = (unsigned int)-1;
	min_error = MAX_ERROR;
	for(i = 0; i < (1 << (K - 1)); i++) {
		if(ae[i] < min_error) {
			min_state = i;
			min_error = ae[i];
		}
	}

	// trace the path
	cur_state = min_state;
	for(t = CONV_INPUT_SIZE; t >= 1; t--) {
		min_state = cur_state;
		cur_state = state_history[cur_state][t]; // get previous
		output[t - 1] = prev_next_state[cur_state][min_state];
	}

	// return the number of errors detected (hard-decision)
	return min_error;
}


static inline float conv_decode_soft(float *data, unsigned char *output) {

	unsigned int i, t, state, nstate, b, min_state, cur_state, o;
	unsigned int state_history[1 << (K - 1)][CONV_INPUT_SIZE + 1];
	double ae[1 << (K - 1)];
	double nae[1 << (K - 1)]; // next accumulated error
	double rd1, rd2, distance, accumulated_error, min_error;
	unsigned int o1, o2;

	// initialize accumulated error, assume starting state is 0
	for(i = 0; i < (1 << (K - 1)); i++)
		ae[i] = nae[i] = MAX_ERROR;
	ae[0] = 0;

	// build trellis
	for(t = 0; t < CONV_INPUT_SIZE; t++) {

		// get received data symbol
		rd1 = data[2 * t];
		rd2 = data[2 * t + 1];

		// for each state
		for(state = 0; state < (1 << (K - 1)); state++) {

			// make sure this state is possible
			if(ae[state] >= MAX_ERROR)
				continue;

			// find all states we lead to
			for(b = 0; b < 2; b++) {

				// get next state given input bit b
				nstate = next_state[state][b];

				// find output for this transition
				o = encode[state][b];

				o1 = (o & 2) >> 1;
				o2 = (o & 1);

				// calculate distance from received data
				distance = fabs((double)o1 - rd1) + fabs((double)o2 - rd2);

				// choose surviving path
				accumulated_error = ae[state] + distance;
				if(accumulated_error < nae[nstate]) {

					// save error for surviving state
					nae[nstate] = accumulated_error;

					// update state history
					state_history[nstate][t + 1] = state;
				}
			}
		}
		
		// get accumulated error ready for next time slice
		for(i = 0; i < (1 << (K - 1)); i++) {
			ae[i] = nae[i];
			nae[i] = MAX_ERROR;
		}
	}

	// the final state is the state with the fewest errors
	min_state = (unsigned int)-1;
	min_error = MAX_ERROR;
	for(i = 0; i < (1 << (K - 1)); i++) {
		if(ae[i] < min_error) {
			min_state = i;
			min_error = ae[i];
		}
	}

	// trace the path
	cur_state = min_state;
	for(t = CONV_INPUT_SIZE; t >= 1; t--) {
		min_state = cur_state;
		cur_state = state_history[cur_state][t]; // get previous
		output[t - 1] = prev_next_state[cur_state][min_state];
	}

	// return the magnitude of errors detected
	return min_error;
}


int decode_sch(const unsigned char *buf, int *fn_o, int *bsic_o) {

	int errors, bsic, t1, t2, t3p, t3, fn, tt;
	unsigned char data[CONV_SIZE], decoded_data[PARITY_OUTPUT_SIZE];

	// extract encoded data from synchronization burst
	memcpy(data, buf + SB_EDATA_OS_1, SB_EDATA_LEN_1);
	memcpy(data + SB_EDATA_LEN_1, buf + SB_EDATA_OS_2, SB_EDATA_LEN_2);

	// Viterbi decode
	if((errors = conv_decode(data, decoded_data))) {
		// fprintf(stderr, "error: sch: conv_decode (%d)\n", errors);
		return errors;
	}

	// check parity
	if(parity_check(decoded_data)) {
		// fprintf(stderr, "error: sch: parity failed\n");
		return 1;
	}

	// Synchronization channel information, 44.018 page 171. (V7.2.0)
	bsic =
	   (decoded_data[ 7] << 5)  |
	   (decoded_data[ 6] << 4)  |
	   (decoded_data[ 5] << 3)  |
	   (decoded_data[ 4] << 2)  |
	   (decoded_data[ 3] << 1)  |
	   (decoded_data[ 2] << 0);
	t1 =
	   (decoded_data[ 1] << 10) |
	   (decoded_data[ 0] << 9)  |
	   (decoded_data[15] << 8)  |
	   (decoded_data[14] << 7)  |
	   (decoded_data[13] << 6)  |
	   (decoded_data[12] << 5)  |
	   (decoded_data[11] << 4)  |
	   (decoded_data[10] << 3)  |
	   (decoded_data[ 9] << 2)  |
	   (decoded_data[ 8] << 1)  |
	   (decoded_data[23] << 0);
	t2 = 
	   (decoded_data[22] << 4)  |
	   (decoded_data[21] << 3)  |
	   (decoded_data[20] << 2)  |
	   (decoded_data[19] << 1)  |
	   (decoded_data[18] << 0);
	t3p =
	   (decoded_data[17] << 2)  |
	   (decoded_data[16] << 1)  |
	   (decoded_data[24] << 0);

	t3 = 10 * t3p + 1;

	// modulo arithmetic
        tt = t3;
        while(tt < t2)
                tt += 26;
        tt = (tt - t2) % 26;
	fn = (51 * 26 * t1) + (51 * tt) + t3;

	/*
	 * BSIC: Base Station Identification Code
	 * 	BCC: Base station Color Code
	 * 	NCC: Network Color Code
	 *
	 * FN: Frame Number
	 */
        /*
	printf("bsic: %x (bcc: %u; ncc: %u)\tFN: %u\n", bsic, bsic & 7,
	   (bsic >> 3) & 7, fn);
         */

	if(fn_o)
		*fn_o = fn;
	if(bsic_o)
		*bsic_o = bsic;

	return 0;
}


int decode_sch_soft(const float *buf, int *fn_o, int *bsic_o) {

	int i, bsic, t1, t2, t3p, t3, fn, tt;
	unsigned char decoded_data[PARITY_OUTPUT_SIZE];
	float data[CONV_SIZE], errors;

	// extract encoded data from synchronization burst
	for(i = 0; i < SB_EDATA_LEN_1; i++)
		data[i] = buf[SB_EDATA_OS_1 + i];
	for(i = 0; i < SB_EDATA_LEN_2; i++)
		data[SB_EDATA_LEN_1 + i] = buf[SB_EDATA_OS_2 + i];

	// Viterbi decode
	errors = conv_decode_soft(data, decoded_data);

	// check parity
	if(parity_check(decoded_data)) {
		// fprintf(stderr, "error: decode_sch_soft: parity failed (viterbi errors = %f)\n", errors);
		return -1;
	}

	// Synchronization channel information, 44.018 page 171. (V7.2.0)
	bsic =
	   (decoded_data[ 7] << 5)  |
	   (decoded_data[ 6] << 4)  |
	   (decoded_data[ 5] << 3)  |
	   (decoded_data[ 4] << 2)  |
	   (decoded_data[ 3] << 1)  |
	   (decoded_data[ 2] << 0);
	t1 =
	   (decoded_data[ 1] << 10) |
	   (decoded_data[ 0] << 9)  |
	   (decoded_data[15] << 8)  |
	   (decoded_data[14] << 7)  |
	   (decoded_data[13] << 6)  |
	   (decoded_data[12] << 5)  |
	   (decoded_data[11] << 4)  |
	   (decoded_data[10] << 3)  |
	   (decoded_data[ 9] << 2)  |
	   (decoded_data[ 8] << 1)  |
	   (decoded_data[23] << 0);
	t2 = 
	   (decoded_data[22] << 4)  |
	   (decoded_data[21] << 3)  |
	   (decoded_data[20] << 2)  |
	   (decoded_data[19] << 1)  |
	   (decoded_data[18] << 0);
	t3p =
	   (decoded_data[17] << 2)  |
	   (decoded_data[16] << 1)  |
	   (decoded_data[24] << 0);

	t3 = 10 * t3p + 1;

	// modulo arithmetic
        tt = t3;
        while(tt < t2)
                tt += 26;
        tt = (tt - t2) % 26;
	fn = (51 * 26 * t1) + (51 * tt) + t3;

	/*
	 * BSIC: Base Station Identification Code
	 * 	BCC: Base station Color Code
	 * 	NCC: Network Color Code
	 *
	 * FN: Frame Number
	 */
        /*
	printf("bsic: %x (bcc: %u; ncc: %u)\tFN: %u\n", bsic, bsic & 7,
	   (bsic >> 3) & 7, fn);
         */

	if(fn_o)
		*fn_o = fn;
	if(bsic_o)
		*bsic_o = bsic;

	return 0;
}
