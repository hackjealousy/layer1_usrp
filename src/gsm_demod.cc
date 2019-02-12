#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "usrp_complex.h"
#include "dsp.h"
#include "gsm.h"
#include "gsm_bursts.h"
#include "gsm_demod.h"
#include "fcch_detector.h"


/*
 * Given a training sequence code, this function generates a modulated version
 * suitable for correlating against incoming signals.
 *
 * 	sps		samples per symbol
 * 	tsc		training sequence in bits
 *	tsc_len		number of bits in training sequence
 *	tsc_offset	offset from start of burst to tsc
 *	toa		time of arrival -- index of peak in correlation
 *	gain		value of peak in correlation -- used in channel response
 *	mtsc_len	length of modulated traning sequence
 *
 *	returns		modulated training sequence
 */
complex *generate_modulated_tsc(const float sps, const unsigned char *tsc,
   const unsigned int tsc_len, const unsigned int tsc_offset,
   float *toa_o, complex *gain_o, unsigned int *mtsc_len_o) {

	unsigned int mtsc_len;
	float toa;
	complex *mtsc, gain;

	// modulate tsc
	if(!(mtsc = modulate(tsc, tsc_len, 0, sps, &mtsc_len)))
		return 0;

	// rotate to match transmit in actual burst
	scale(mtsc, mtsc_len, exp(complex(0, (M_PI / 2.0) * (tsc_offset % 4))));

	toa = ((float)tsc_len / 2.0) + tsc_offset;
	gain = complex(tsc_len, 0);

	if(toa_o)
		*toa_o = toa;
	if(gain_o)
		*gain_o = gain;
	if(mtsc_len_o)
		*mtsc_len_o = mtsc_len;
	return mtsc;
}


int generate_modulated_tsc(const float sps, const unsigned char *tsc,
   const unsigned int tsc_len, const unsigned int tsc_offset, mtsc_s **mtsc) {

	mtsc_s *m;

	if(!mtsc) {
		fprintf(stderr, "error: generate_modulated_tsc: no space for mtsc given\n");
		return -1;
	}
	if(!*mtsc) {
		*mtsc = new mtsc_s;
		if(!*mtsc) {
			fprintf(stderr, "error: generate_modulated_tsc: new failed\n");
			return -1;
		}
	}
	m = *mtsc;
	m->tsc = generate_modulated_tsc(sps, tsc, tsc_len, tsc_offset, &m->toa, &m->gain, &m->len);
	if(!m->tsc) {
		delete m;
		return -1;
	}

	return 0;
}


/*
 * demod_burst
 *
 * Given a traning sequence for a burst, demodulate the burst into soft samples.
 *
 * If a DFE filter is given, use that.  Otherwise, create one.
 */
float *demod_burst(const float sps, unsigned int *burst_len,
   const complex * const s, const unsigned int s_len,
   const mtsc_s *mtsc,
   dfe_filter_s **d,
   unsigned int cr_len, unsigned int dfe_len) {

	static const float SNR_THRESHOLD = 3.0;

	unsigned int c_len, v_len, b_len;
	float toa, adjusted_toa, SNR, *b;
	complex *c, peak, *cr, *v;
	dfe_filter_s *dfe_new, *dfe; 

	if(s_len < sps * DATA_LEN) {
		fprintf(stderr, "error: demod_burst: not enough samples\n");
		return 0;
	}
	if(!mtsc) {
		fprintf(stderr, "error: demod_burst: no training sequence given\n");
		return 0;
	}

	// correlate burst with TSC
	c = correlate_nodelay(s, s_len, mtsc->tsc, mtsc->len, &c_len);
	if(!c)
		return 0;

	// find point of maximum correlation
	toa = peak_detect(c, c_len, &peak, 0);

	// calculate approximate SNR
	if(peak2mean(c, c_len, peak, (unsigned int)nearbyintf(toa), 4, &SNR)) {
		delete[] c;
		return 0;
	}

	// does this look like a peak?
	if(SNR < SNR_THRESHOLD) {
		delete[] c;
		return 0;
	}

	// adjust for offsets
	adjusted_toa = toa - mtsc->toa;

	/*
	 * If toa is negative, we're missing the first part of the burst data.
	 * The standard guard period of 3 bits should help a bit.
	 */
	if(adjusted_toa < -2) {
		delete[] c;
		return 0;
	}

	/*
	 * Make sure there are enough samples to get all the data even when we
	 * adjust for toa.
	 */
	if(s_len < DATA_LEN * sps + adjusted_toa + 2) {
		delete[] c;
		return 0;
	}

	/*
	 * Do we need to build the DFE?
	 */
	if((!d) || (!*d)) {

		dfe_new = new dfe_filter_s;
		if(!dfe_new) {
			fprintf(stderr, "error: demod_burst: new failed\n");
			delete[] c;
			return 0;
		}

		// build channel response
		cr = generate_channel_response(c, c_len, cr_len, toa, mtsc->gain);
		if(!cr) {
			delete dfe_new;
			delete[] c;
			return 0;
		}

		// design DFE
		if(design_DFE(cr, cr_len, SNR, dfe_len, &dfe_new->ff, &dfe_new->ff_len, &dfe_new->fb, &dfe_new->fb_len)) {
			delete[] cr;
			delete[] dfe_new;
			delete[] c;
			return 0;
		}
		delete[] cr;

		if(d)
			*d = dfe_new;
		dfe = dfe_new;
	} else
		dfe = *d;
	delete[] c;

	// center burst for equalization
	v_len = (unsigned int)(ceil(DATA_LEN * sps + adjusted_toa + 2));
	v = new complex[v_len];
	if(!v) {
		fprintf(stderr, "error: demod_burst: new failed\n");
		return 0;
	}
	memcpy(v, s, v_len * sizeof(complex));
	delay(v, v_len, -adjusted_toa);

	// equalize burst
	b = equalize(v, v_len, dfe->ff, dfe->ff_len, dfe->fb, dfe->fb_len, &b_len);

	delete[] v;

	if(burst_len)
		*burst_len = b_len;

	return b;
}


/*
 * get_burst_sch
 *
 * This function returns the next buffer containing a synchronization burst.
 * 
 * Unlike the other get_burst_ functions, we don't require a frame number or a
 * time slot.  Moreover, the returned buffer may be larger (or smaller) than a
 * burst and may not actually contain the synchronization burst.
 */
complex *get_burst_sch(usrp_source *u, unsigned int *buf_len) {

	static const unsigned int MAX_SEARCH = 20;

	unsigned int fb_mframe_len, frame_len, burst_len, c_len, consumed, overruns = 0, offset_found = 0, offset_search_count = 0;
	float offset, sps;
	complex *c;
	fcch_detector *l;
	circular_buffer *cb;

	sps = u->sample_rate() / GSM_RATE;
	fb_mframe_len = (unsigned int)ceil((12 * FRAME_LEN + BURST_LEN) * sps);
	frame_len = (unsigned int)ceil(FRAME_LEN * sps);
	burst_len = (unsigned int)ceil(BURST_LEN * sps);

	l = new fcch_detector(u->sample_rate());
	if(!l) {
		fprintf(stderr, "error: get_burst_sch: bad new\n");
		return 0;
	}

	cb = u->get_buffer();

	/*
	 * Ensure at least fb_mframe_len contiguous samples are read from usrp.
	 * This should ensure that we can find a FCH burst.
	 *
	 * Since we aren't sync'ed, we flush() first.
	 */
	u->flush();
	while((!offset_found) && (offset_search_count < MAX_SEARCH)) {
		offset_search_count += 1;
		do {
			if(u->fill(fb_mframe_len, &overruns))
				return 0;
			if(overruns) {
				u->flush();
			}
		} while(overruns);

		// get a pointer to the next samples
		c = (complex *)cb->peek(&c_len);

		// search the buffer for a pure tone
		offset_found = l->scan(c, c_len, &offset, &consumed);

		// consume samples to the end of the frequency burst data part
		cb->purge(consumed);
	}

	delete l;

	if(!offset_found) {
		return 0;
	}

	/*
	 * should we actually do something with the offset?
	 */

	/*
	 * The sync burst should be one frame after the frequency burst in TN =
	 * 0.  We've consumed to the end of the data in the frequency burst
	 * above, so if we get one frame of samples, we should have the sync
	 * burst in our buffer.  We'll add a whole burst length of buffer to
	 * make sure we get it.
	 *
	 * However, we actually can consume quite a bit after the frequency
	 * burst.
	 */
	if(u->fill(frame_len + burst_len, &overruns))
		return 0;
	if(overruns) {
		/*
		 * If we overflow between getting the offset and getting the
		 * next frame, something is wrong.
		 */
		return 0;
	}

	/*
	 * We'll purge the bursts we don't need.  Since we may have gone fairly
	 * far into the next time slot, we'll leave a lot of room.
	 */
	cb->purge(frame_len - 2 * burst_len);
	c = (complex *)cb->peek(&c_len);

	if(buf_len)
		*buf_len = c_len;
	return c;
}


/*
complex *get_burst(usrp_source *u, unsigned int *burst_len, const unsigned int fn, const unsigned int ts) {

	return 0;
}
 */
