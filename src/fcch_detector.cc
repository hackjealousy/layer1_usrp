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

/*
 * This is based on the algorithm found in the paper,
 *
 *	Varma, G. Narendra, Usha Sahu, and G. Prabhu Charan.  "Robust
 *	Frequency Burst Detection Algorithm for GSM / GPRS."
 *
 * The algorithm uses an adaptive filter to calculate the error difference from
 * a pure tone.  When the error goes low, the tone is detected.  When it goes
 * back high, the scan function returns and indicates the number of samples the
 * error was low.
 *
 * The following code is an original work and the above BSD-license should
 * apply.  However, the algorithm itself may be patented and any use of this
 * code should take that into consideration.
 */

#include <stdio.h>	// for debug
#include <stdlib.h>

#include <stdexcept>
#include <string.h>
#include "gsm.h"
#include "fcch_detector.h"
#include "dsp.h"


static const char * const fftw_plan_name = ".layer1_usrp_fftw_plan";


fcch_detector::fcch_detector(const float sample_rate, const unsigned int D,
   const float p, const float G) {

	FILE *plan_fp;
	char plan_name[BUFSIZ];
	const char *home;

	m_D = D;
	m_p = p;
	m_G = G;
	m_e = 0.0;

	m_sample_rate = sample_rate;
	m_fcch_burst_len =
	   (unsigned int)(DATA_LEN * (m_sample_rate / GSM_RATE));

	m_filter_delay = 8;
	m_w_len = 2 * m_filter_delay + 1;
	m_w = new complex[m_w_len];
	memset(m_w, 0, sizeof(complex) * m_w_len);

	m_x_cb = new circular_buffer(1024, sizeof(complex), 0);
	m_e_cb = new circular_buffer(1000000, sizeof(float), 0);

	m_in = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * FFT_SIZE);
	m_out = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * FFT_SIZE);
	if((!m_in) || (!m_out))
		throw std::runtime_error("fcch_detector: fftw_malloc failed!");

	home = getenv("HOME");
	if(strlen(home) + strlen(fftw_plan_name) + 2 < sizeof(plan_name)) {
		strcpy(plan_name, home);
		strcat(plan_name, "/");
		strcat(plan_name, fftw_plan_name);
		if((plan_fp = fopen(plan_name, "r"))) {
			fftw_import_wisdom_from_file(plan_fp);
			fclose(plan_fp);
		}
		m_plan = fftw_plan_dft_1d(FFT_SIZE, m_in, m_out, FFTW_FORWARD, FFTW_MEASURE);
		if((plan_fp = fopen(plan_name, "w"))) {
			fftw_export_wisdom_to_file(plan_fp);
			fclose(plan_fp);
		}
	} else
		m_plan = fftw_plan_dft_1d(FFT_SIZE, m_in, m_out, FFTW_FORWARD,
		   FFTW_ESTIMATE);
	if(!m_plan)
		throw std::runtime_error("fcch_detector: fftw plan failed!");
}


fcch_detector::~fcch_detector() {

	if(m_w) {
		delete[] m_w;
		m_w = 0;
	}
	if(m_x_cb) {
		delete m_x_cb;
		m_x_cb = 0;
	}
	if(m_e_cb) {
		delete m_e_cb;
		m_e_cb = 0;
	}

	if(m_in)
		fftw_free(m_in);
	if(m_out)
		fftw_free(m_out);

	/*
	 * When you create a plan, create the same plan, delete the second plan
	 * and then delete the first plan, fftw crashes.  At least when it is
	 * the same plan.
	 */
	if(m_plan)
		fftw_destroy_plan(m_plan);
	fftw_cleanup();
}


static inline float itof(float index, float sample_rate, unsigned int fft_size) {

	return (double)(index * (sample_rate / (double)fft_size) - (sample_rate / 2));
}


#ifndef MIN
#define MIN(a, b) (a)<(b)?(a):(b)
#endif /* !MIN */


float fcch_detector::freq_detect(const complex *s, const unsigned int s_len, float *pm) {

	unsigned int i, len;
	float max_i, avg_power;
	complex fft[FFT_SIZE], peak;

	len = MIN(s_len, FFT_SIZE);
	for(i = 0; i < len; i++) {
		m_in[i][0] = s[i].real();
		m_in[i][1] = s[i].imag();
	}
	for(; i < FFT_SIZE; i++) {
		m_in[i][0] = 0;
		m_in[i][1] = 0;
	}

	fftw_execute(m_plan);

	// center for correct peak detection
	for(i = 0; i < (FFT_SIZE / 2); i++) {
		fft[i + (FFT_SIZE / 2)].real() = m_out[i][0];
		fft[i + (FFT_SIZE / 2)].imag() = m_out[i][1];
	}
	for(; i < FFT_SIZE; i++) {
		fft[i - (FFT_SIZE / 2)].real() = m_out[i][0];
		fft[i - (FFT_SIZE / 2)].imag() = m_out[i][1];
	}

	max_i = peak_detect(fft, FFT_SIZE, &peak, &avg_power);
	if(pm)
		*pm = norm(peak) / avg_power;
	return itof(max_i, m_sample_rate, FFT_SIZE);
}


static unsigned int g_count = 0;
static int g_sign = 1;
static float g_threshold = 0.0;


static inline void low_to_high_init(float threshold) {

	g_count = 0;
	g_sign = 1;
	g_threshold = threshold;
}


static inline unsigned int low_to_high(float s) {

	unsigned int r = 0;

	if(s >= g_threshold) {
		if(g_sign == -1) {
			r = g_count;
			g_sign = 1;
			g_count = 0;
		}
		g_count += 1;
	} else {
		if(g_sign == 1) {
			g_sign = -1;
			g_count = 0;
		}
		g_count += 1;
	}

	return r;
}


/*
 * scan:
 * 	1.  calculate average error
 * 	2.  find neighborhoods with low error that satisfy minimum length
 * 	3.  for each such neighborhood, take fft and calculate peak/mean
 * 	4.  if peak/mean > 50, then this is a valid finding.
 *
 * Upon return, consumed will be the number of samples consumed to the end of the frequency burst.
 */
unsigned int fcch_detector::scan(const complex *s, const unsigned int s_len, float *offset, unsigned int *consumed) {

	static const float sps = m_sample_rate / GSM_RATE;
	static const unsigned int MIN_FB_LEN = 100 * sps;
	static const unsigned int MIN_PM = 50; // XXX arbitrary, depends on decimation

	unsigned int len, t, e_count, i, l_count, y_offset = 0, y_len = 0;
	float e, *a, loff = 0, pm;
	double sum = 0.0, avg, limit;
	const complex *y;

	/*
	 * If we don't find a pure tone in the buffer, we've consumed the whole
	 * buffer.
	 */
	if(consumed)
		*consumed = s_len;

	m_e_cb->flush();
	m_x_cb->flush();

	// calculate the error for each sample
	len = 0;
	while(len < s_len) {
		t = m_x_cb->write(s + len, 1);
		len += t; 
		if(!next_norm_error(&e)) {
			m_e_cb->write(&e, 1);
			sum += e;
		}
	}

	// calculate average error over entire buffer
	a = (float *)m_e_cb->peek(&e_count);
	avg = sum / (double)e_count;
	limit = 0.7 * avg;

	// find neighborhoods where the error is smaller than the limit
	low_to_high_init(limit);
	for(i = 0; i < e_count; i++) {
		l_count = low_to_high(a[i]);

		// see if p/m indicates a pure tone
		pm = 0;
		if(l_count >= MIN_FB_LEN) {
			y_offset = i - l_count;
			y_len = (l_count < m_fcch_burst_len)? l_count : m_fcch_burst_len;
			y = s + y_offset;
			loff = freq_detect(y, y_len, &pm);

			if(pm > MIN_PM)
				break;
		}
	}

	if(pm <= MIN_PM)
		return 0;

	if(offset)
		*offset = loff;

	if(consumed)
		*consumed = y_offset + y_len + get_delay();

	return 1;
}


unsigned int fcch_detector::get_delay() {

	return m_w_len - 1 + m_D;
}


unsigned int fcch_detector::filter_len() {

	return m_w_len;
}


/*
 * First y value comes out at sample x[n + m_D] = x[w_len - 1 + m_D].
 *
 * 	y[0] = X(x[0], ..., x[w_len - 1 + m_D])
 *
 * So y and e are delayed by w_len - 1 + m_D.
 */
int fcch_detector::next_norm_error(float *error) {

	unsigned int i, n, max;
	float E;
	complex *x, y, e;

	// n is "current" sample
	n = m_w_len - 1;

	// ensure there are enough samples in the buffer
	x = (complex *)m_x_cb->peek(&max);
	if(n + m_D >= max)
		return n + m_D - max + 1;

	// update G
	E = vectornorm2(x, m_w_len);
	if(m_G >= 2.0 / E)
		m_G = 1.0 / E;

	// calculate filtered value
	y = 0.0;
	for(i = 0; i < m_w_len; i++)
		y += std::conj(m_w[i]) * x[n - i];

	// calculate error from desired signal
	e = x[n + m_D] - y;

	// update filters with opposite gradient
	for(i = 0; i < m_w_len; i++)
		m_w[i] += m_G * std::conj(e) * x[n - i];

	// update error average power
	E /= m_w_len;
	m_e = (1.0 - m_p) * m_e + m_p * norm(e);

	// return error ratio
	if(error)
		*error = m_e / E;

	// remove the processed sample from the buffer
	m_x_cb->purge(1);

	return 0;
}

