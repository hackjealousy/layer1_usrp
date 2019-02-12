/*
 * Copyright (c) 2011, Joshua Lackey
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     *  Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *
 *     *  Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "usrp_source.h"
#include "circular_buffer.h"
#include "fcch_detector.h"
#include "arfcn_freq.h"
#include "util.h"
#include "gsm.h"
#include "dsp.h"


static const unsigned int	AVG_COUNT		= 100;
static const unsigned int	AVG_THRESHOLD		= (AVG_COUNT / 10);
static const float		ERROR_DETECT_OFFSET_MAX	= 40e3;
static const unsigned int	NOTFOUND_MAX		= 10;


int offset_detect(usrp_source *u, fcch_detector *l, float *p_avg_offset, float *p_min, float *p_max, float *p_stddev) {

	int r = -1, l_in = 1;
	unsigned int s_len, b_len, consumed, count, new_overruns = 0, overruns = 0, notfound_count = 0;
	float offset = 0.0, min = 0.0, max = 0.0, avg_offset = 0.0, stddev = 0.0, sps, offsets[AVG_COUNT];
	complex *cbuf;
	circular_buffer *cb;

	if(!l) {
		l_in = 0;
		l = new fcch_detector(u->sample_rate());
		if(!l) {
			fprintf(stderr, "error: new\n");
			return -1;
		}
	}

	/*
	 * We deliberately grab 12 frames and 1 burst.  We are guaranteed to
	 * find at least one FCCH burst in this much data.
	 */
	sps = u->sample_rate() / GSM_RATE;
	s_len = (unsigned int)ceil((12 * FRAME_LEN + BURST_LEN) * sps);
	cb = u->get_buffer();

	count = 0;
	while(count < AVG_COUNT) {

		// ensure at least s_len contiguous samples are read from usrp
		do {
			u->flush();
			if(u->fill(s_len, &new_overruns)) {
				goto jump_leaving;
			}
			if(new_overruns) {
				overruns += new_overruns;
			}
		} while(new_overruns);

		// get a pointer to the next samples
		cbuf = (complex *)cb->peek(&b_len);

		// search the buffer for a pure tone
		if(l->scan(cbuf, b_len, &offset, &consumed)) {

			// FCH is a sine wave at GSM_RATE / 4
			offset = offset - FCCH_FREQ;

			// sanity check offset
			if(fabs(offset) < ERROR_DETECT_OFFSET_MAX) {
				offsets[count] = offset;
				count += 1;
				notfound_count = 0;
			} else {
				notfound_count += 1;
			}
		} else {
			notfound_count += 1;
		}

		// consume used samples
		cb->purge(consumed);

		if(notfound_count >= NOTFOUND_MAX)
			goto jump_leaving;
	}
	r = 0;

jump_leaving:
	if(!l_in) {
		delete l;
	}

	if(!r) {
		// construct stats
		sort(offsets, AVG_COUNT);
		avg_offset = avg(offsets + AVG_THRESHOLD, AVG_COUNT - 2 * AVG_THRESHOLD, &stddev);
		min = offsets[AVG_THRESHOLD];
		max = offsets[AVG_COUNT - AVG_THRESHOLD - 1];

		if(p_avg_offset)
			*p_avg_offset = avg_offset;
		if(p_min)
			*p_min = min;
		if(p_max)
			*p_max = max;
		if(p_stddev)
			*p_stddev = stddev;
	}

	return r;
}


int c0_detect(usrp_source *u, int bi, int strict) {

	int i, chan_count, ret = -1;
	unsigned int overruns, b_len, frames_len, found_count, notfound_count, r;
	float offset, spower[BUFSIZ], min, max, stddev;
	double freq, sps, n, power[BUFSIZ], sum = 0, a;
	complex *b;
	circular_buffer *ub;
	fcch_detector *l;


	if(bi == BI_NOT_DEFINED) {
		fprintf(stderr, "error: c0_detect: band not defined\n");
		return -1;
	}
	/*
	if(u->tune_if(band_center(bi))) {
		fprintf(stderr, "error: usrp_source::tune_if\n");
		return -1;
	}
	 */

	l = new fcch_detector(u->sample_rate());
	if(!l) {
		fprintf(stderr, "error: new\n");
		return -1;
	}

	sps = u->sample_rate() / GSM_RATE;
	frames_len = (unsigned int)ceil((12 * FRAME_LEN + BURST_LEN) * sps);
	ub = u->get_buffer();

	// first, we calculate the power in each channel
	// XXX should filter to 200kHz
	u->start();
	u->flush();
	for(i = first_chan(bi); i > 0; i = next_chan(i, bi)) {
		freq = arfcn_to_freq(i, &bi);
		/*
		if(u->fast_tune(freq)) {
			fprintf(stderr, "error: usrp_source::fast_tune\n");
			goto jump_leaving;
		}
		 */

		if(u->tune(freq)) {
			fprintf(stderr, "error: usrp_source::fast_tune\n");
			goto jump_leaving;
		}
		do {
			u->flush();
			if(u->fill(frames_len, &overruns)) {
				fprintf(stderr, "error: usrp_source::fill\n");
				goto jump_leaving;
			}
		} while(overruns);

		b = (complex *)ub->peek(&b_len);
		n = sqrt(vectornorm2(b, frames_len));
		power[i] = n;
	}

	/*
	 * We want to use the average to determine which channels have
	 * power, and hence a possibility of being channel 0 on a BTS.
	 * However, some channels in the band can be extremely noisy.  (E.g.,
	 * CDMA traffic in GSM-850.)  Hence we won't consider the noisiest
	 * channels when we construct the average.
	 */
	chan_count = 0;
	for(i = first_chan(bi); i > 0; i = next_chan(i, bi)) {
		spower[chan_count++] = power[i];
	}
	sort(spower, chan_count);

	// average the lowest %60
	a = avg(spower, chan_count - 4 * chan_count / 10, 0);

	// then we look for fcch bursts
	printf("%s:\n", bi_to_str(bi));
	found_count = 0;
	notfound_count = 0;
	sum = 0;
	i = first_chan(bi);
	do {
		if(power[i] <= a) {
			i = next_chan(i, bi);
			continue;
		}

		freq = arfcn_to_freq(i, &bi);
		/*
		if(u->fast_tune(freq)) {
			fprintf(stderr, "error: usrp_source::fast_tune\n");
			goto jump_leaving;
		}
		 */
		if(u->tune(freq)) {
			fprintf(stderr, "error: usrp_source::fast_tune\n");
			goto jump_leaving;
		}

		do {
			u->flush();
			if(u->fill(frames_len, &overruns)) {
				fprintf(stderr, "error: usrp_source::fill\n");
				goto jump_leaving;
			}
		} while(overruns);

		b = (complex *)ub->peek(&b_len);
		r = l->scan(b, b_len, &offset, 0);
		offset -= FCCH_FREQ;
		if(r && (fabsf(offset) < ERROR_DETECT_OFFSET_MAX)) {
			// found
			if(strict) {
				if(!offset_detect(u, l, &offset, &min, &max, &stddev)) {
					printf("\tchan: %d (%.1fMHz ", i, freq / 1e6);
					display_freq(offset);
					printf(")\tpower: %6.2lf\t[min, max, range]: [%d, %d, %d]\tstddev: %f\n", power[i], (int)round(min), (int)round(max), (int)round(max - min), stddev);
				}
			} else {
				printf("\tchan: %d (%.1fMHz ", i, freq / 1e6);
				display_freq(offset);
				printf(")\tpower: %6.2lf\n", power[i]);
			}
			notfound_count = 0;
			i = next_chan(i, bi);
		} else {
			// not found
			notfound_count += 1;
			if(notfound_count >= NOTFOUND_MAX) {
				notfound_count = 0;
				i = next_chan(i, bi);
			}
		}
	} while(i > 0);

	ret = 0;
jump_leaving:

	u->stop();
	delete l;

	return ret;
}
