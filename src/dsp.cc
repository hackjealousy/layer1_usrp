/*
 * Note: Many of the functions below are based on DSP routines found in OpenBTS.
 * They have been rewritten but OpenBTS was used as a reference.  Again, this means
 * that other licenses may apply.
 */


#include <stdio.h>
#include <string.h>
#include <math.h>
#include "usrp_complex.h"
#include "dsp.h"

#ifndef MIN
#define MIN(a, b) ((a)<(b)?(a):(b))
#endif /* !MIN */

static const unsigned int COMMON_FILTER_LEN	= 21;
static const unsigned int ROTATOR_LEN		= 1024;


static complex *	m_gmsk_rotator = 0;
static complex *	m_gmsk_rrotator = 0;
static complex *	m_gaussian_pulse = 0;
static unsigned int	m_gaussian_pulse_len = 0;


int build_rotators() {

	unsigned int i;

	if(!(m_gmsk_rotator = new complex[ROTATOR_LEN])) {
		return -1;
	}
	if(!(m_gmsk_rrotator = new complex[ROTATOR_LEN])) {
		delete[] m_gmsk_rotator;
		return -1;
	}
	for(i = 0; i < ROTATOR_LEN; i++) {
		m_gmsk_rotator[i] = exp(complex(0, (M_PI / 2.0) * (i % 4)));
		m_gmsk_rrotator[i] = exp(complex(0, -(M_PI / 2.0) * (i % 4)));
	}

	return 0;
}


float vectornorm2(const complex *v, const unsigned int len) {

	unsigned int i;
	float e = 0.0;

	for(i = 0; i < len; i++)
		e += norm(v[i]);

	return e;
}


float sinc(const float x) {

	if((x <= -0.0001) || (0.0001 <= x))
		return sinf(x) / x;
	return 1.0;
}


complex interpolate_point(const complex *s, const unsigned int s_len, const float s_i) {

	int start, end, i;
	unsigned int d;
	complex point;

	d = (COMMON_FILTER_LEN - 1) / 2;
	start = (int)(floor(s_i) - d);
	end = (int)(floor(s_i) + d + 1);
	if(start < 0)
		start = 0;
	if(end > (int)(s_len - 1))
		end = s_len - 1;
	for(point = 0.0, i = start; i <= end; i++)
		point += s[i] * sinc(M_PI * (i - s_i));
	return point;
}


float peak_detect(const complex *s, const unsigned int s_len, complex *peak, float *avg_power) {

	unsigned int i;
	float max = -1.0, max_i = -1.0, sample_power, sum_power, early_i, late_i, incr;
	complex early_p, late_p, cmax;

	sum_power = 0;
	for(i = 0; i < s_len; i++) {
		sample_power = norm(s[i]);
		sum_power += sample_power;
		if(sample_power > max) {
			max = sample_power;
			max_i = i;
		}
	}
	early_i = (1 <= max_i)? (max_i - 1) : 0;
	late_i = (max_i + 1 < s_len)? (max_i + 1) : s_len - 1;

	incr = 0.5;
	while(incr > 1.0 / 1024.0) {
		early_p = interpolate_point(s, s_len, early_i);
		late_p = interpolate_point(s, s_len, late_i);
		if(norm(early_p) < norm(late_p))
			early_i += incr;
		else if(norm(early_p) > norm(late_p))
			early_i -= incr;
		else
			break;
		incr /= 2.0;
		late_i = early_i + 2.0;
	}
	max_i = early_i + 1.0;
	if(max_i < 0)
		max_i = 0;
	if(max_i > s_len - 1)
		max_i = s_len - 1;
	cmax = interpolate_point(s, s_len, max_i);

	if(peak)
		*peak = cmax;

	if(avg_power)
		*avg_power = (sum_power - norm(cmax)) / (s_len - 1);

	return max_i;
}


/*
 * The theory is that there should be almost no match in the correlation of an
 * offset training sequence code.  Hence any signal strength near the peak of
 * the correlation is actually due to noise.
 *
 * Hence we can use p2m as an approximation of SNR.
 */
int peak2mean(complex *c, unsigned int c_len, complex peak, unsigned int peak_i, unsigned int width, float *SNR) {

	float valley = 0.0;
	unsigned int i, valley_count = 0;

	// these constants aren't the best for all burst types
	for(i = 2; i < 2 + width; i++) {
		if(i <= peak_i) {
			valley += norm(c[peak_i - i]);
			valley_count += 1;
		}
		if(peak_i + i < c_len) {
			valley += norm(c[peak_i + i]);
			valley_count += 1;
		}
	}

	if(valley_count < 2) {
		fprintf(stderr, "error: bad valley_count\n");
		return -1;
	}
	valley = sqrtf(valley / (float)valley_count) + 0.00001;

	if(SNR)
		*SNR = sqrtf(norm(peak)) / valley;

	return 0;
}


/*
 * rotate in place
 */
int gmsk_rotate(complex *v, const unsigned int len, const unsigned int offset) {

	unsigned int i;
	complex *c, *r;

	if(len > ROTATOR_LEN)
		return -1;
	if(!m_gmsk_rotator)
		if(build_rotators())
			return -1;

	for(i = 0, c = v, r = m_gmsk_rotator + offset; i < len; i++, c++, r++)
		*c = *r * *c;

	return 0;
}


int gmsk_rotate(complex *v, const unsigned int len) {

	return gmsk_rotate(v, len, 0);
}


/*
 * rotate in place
 */
int gmsk_rrotate(complex *v, const unsigned int len) {

	unsigned int i;
	complex *c, *r;

	if(len > ROTATOR_LEN)
		return -1;
	if(!m_gmsk_rrotator)
		if(!build_rotators())
			return -1;

	for(i = 0, c = v, r = m_gmsk_rrotator; i < len; i++, c++, r++)
		*c = *r * *c;

	return 0;
}


/*
 * scale in place
 */
void scale(complex *v, const unsigned int v_len, const complex s) {

	unsigned int i;

	for(i = 0; i < v_len; i++)
		v[i] = s * v[i];
}


void scale(complex *v, const unsigned int v_len, const float s) {

	unsigned int i;

	for(i = 0; i < v_len; i++)
		v[i] = s * v[i];
}


void scale(complex *u, const complex *v, const unsigned int v_len,
   const complex s) {

	unsigned int i;

	for(i = 0; i < v_len; i++)
		u[i] = s * v[i];
}


/*
 * x = x + y
 */
void add(complex *x, const unsigned int x_len, const complex *y,
   const unsigned int y_len) {

	unsigned int i;

	for(i = 0; (i < x_len) && (i < y_len); i++)
		x[i] = x[i] + y[i];
}


// in place
void conjugate_vector(complex *v, const unsigned int v_len) {

	unsigned int i;

	for(i = 0; i < v_len; i++)
		v[i] = conj(v[i]);
}


float *slice_soft(const complex *v, const unsigned int v_len, unsigned int *len_o) {

	unsigned int i;
	float *s;

	s = new float[v_len];
	if(!s) {
		fprintf(stderr, "error: new\n");
		return 0;
	}
	for(i = 0; i < v_len; i++) {
		s[i] = (1.0 - v[i].real()) / 2.0;
		if(s[i] > 1.0)
			s[i] = 1.0;
		else if(s[i] < 0.0)
			s[i] = 0.0;
	}

	if(len_o)
		*len_o = v_len;

	return s;
}


void slice_soft(float *s, const complex *v, const unsigned int v_len) {

	unsigned int i;

	for(i = 0; i < v_len; i++) {
		s[i] = (1.0 - v[i].real()) / 2.0;
		if(s[i] > 1.0)
			s[i] = 1.0;
		else if(s[i] < 0.0)
			s[i] = 0.0;
	}
}


// slice-hard
// assumes values between [0, 1]
unsigned char *slice(complex *s, unsigned int s_len) {

	unsigned int i;
	unsigned char *b;

	b = new unsigned char[s_len];

	for(i = 0; i < s_len; i++) {
		if(s[i].real() > 0.5)
			b[i] = 1;
		else
			b[i] = 0;
	}

	return b;
}


// slice-hard
// assumes values between [0, 1]
unsigned char *slice(float *s, unsigned int s_len) {

	unsigned int i;
	unsigned char *b;

	b = new unsigned char[s_len];

	for(i = 0; i < s_len; i++) {
		if(s[i] > 0.5)
			b[i] = 1;
		else
			b[i] = 0;
	}

	return b;
}


complex *convolve(const complex *s, const unsigned int s_len, const complex *h, const unsigned int h_len, unsigned int *len_o) {

	unsigned int i, n, len = s_len + h_len - 1;
	complex *y = new complex[len];

	if(!y) {
		if(len_o)
			*len_o = 0;
		return 0;
	}

	for(n = 0; n < len; n++) {
		y[n] = 0.0;
		for(i = 0; i < s_len; i++) {
			if(i > n)
				break;
			if(n < h_len + i)
				y[n] += s[i] * h[n - i];
		}
	}

	if(len_o)
		*len_o = len;
	return y;
}


void convolve_nodelay(complex *y, const complex *s, const unsigned int s_len, const complex *h, const unsigned int h_len) {

	unsigned int d, n, i;

	d = (h_len - 1) / 2;
	for(n = 0; n < s_len; n++) {
		y[n] = 0.0;
		for(i = 0; i < s_len; i++) {
			if(i > n + d)
				break;
			if(n + d < h_len + i)
				y[n] += s[i] * h[n + d - i];
		}
	}
}


complex *convolve_nodelay(const complex *s, const unsigned int s_len, const complex *h, const unsigned int h_len, unsigned int *len_o) {

	unsigned int n, d, i;
	complex *y = new complex[s_len];

	if(!y) {
		fprintf(stderr, "error: convolve_nodelay: new failed\n");
		if(len_o)
			*len_o = 0;
		return 0;
	}

	d = (h_len - 1) / 2;
	for(n = 0; n < s_len; n++) {
		y[n] = 0.0;
		for(i = 0; i < s_len; i++) {
			if(i > n + d)
				break;
			if(n + d < h_len + i)
				y[n] += s[i] * h[n + d - i];
		}
	}

	if(len_o)
		*len_o = s_len;

	return y;
}


complex *correlate(const complex *s1, const unsigned int s1_len, const complex *s2, const unsigned int s2_len, unsigned int *len_o) {

	unsigned int n, i, s_len;
	complex *y;

	s_len = s1_len + s2_len - 1;

	y = new complex[s_len];
	if(!y) {
		fprintf(stderr, "error: correlate: new failed\n");
		if(len_o)
			*len_o = 0;
		return 0;
	}

	for(n = 0; n < s_len; n++) {
		y[n] = 0.0;
		for(i = 0; i < s1_len; i++) {
			if(i > n)
				break;
			if(n < s2_len + i)
				y[n] += s1[i] * std::conj(s2[s2_len - 1 + i - n]);
		}
	}

	if(len_o)
		*len_o = s_len;

	return y;
}


complex *correlate_nodelay(const complex *s1, const unsigned int s1_len, const complex *s2, const unsigned int s2_len, unsigned int *len_o) {

	unsigned int n, d, i;
	complex *y = new complex[s1_len];

	if(!y) {
		fprintf(stderr, "error: correlate_nodelay: new failed\n");
		if(len_o)
			*len_o = 0;
		return 0;
	}

	d = (s2_len - 1) / 2;
	for(n = 0; n < s1_len; n++) {
		y[n] = 0.0;
		for(i = 0; i < s1_len; i++) {
			if(i > n + d)
				break;
			if(n + d < s2_len + i)
				y[n] += s1[i] * std::conj(s2[s2_len - 1 + i - n - d]);
		}
	}

	if(len_o)
		*len_o = s1_len;

	return y;
}


/*
 * Positive toa moves the signal to the future.
 *
 * The signal is assumed to have a guard period sufficient to absorb the toa if
 * toa is negative.  If toa is positive, we assume the guard time has
 * sufficient data to complete the burst.
 *
 * The signal is modified in place.
 */
int delay(complex *v, const unsigned int v_len, const float toa) {

	int ids, i;
	unsigned int h_len = COMMON_FILTER_LEN;
	float fds, center;
	complex h[h_len], *u = 0, *p;

	ids = (int)floor(toa);
	fds = toa - ids;

	// generate delay filter for reasonable fractional offset
	if(fds >= 0.01) {
		center = (h_len - 1) / 2;
		for(i = 0; i < (int)h_len; i++)
			h[i] = (complex)sinc(M_PI * (i - center - fds));

		// delay with filter
		u = convolve_nodelay(v, v_len, h, h_len, 0);
		if(!u)
			return -1;
		p = u;
	} else
		p = v;

	// delay for the integer offset
	if(ids < 0) {
		for(i = 0; (unsigned int)i < v_len + ids; i++)
			v[i] = p[i - ids];
		for(i = v_len + ids; (unsigned int)i < v_len; i++)
			v[i] = 0.0;
	} else if(ids > 0) {
		for(i = v_len - 1; i >= ids; i--)
			v[i] = p[i - ids];
		for(i = ids - 1; i >= 0; i--)
			v[i] = 0.0;
	}

	if(u)
		delete[] u;

	return 0;
}


// interp L, decim M
complex *polyphase_resample(const complex *s, const unsigned int s_len,
   const unsigned int L, const unsigned int M, const complex *h,
   const unsigned int h_len, unsigned int *len_o) {

	unsigned int v_len, d, i, j;
	complex *v;

	v_len = (unsigned int)ceil(s_len * (float)L / (float)M);
	v = new complex[v_len];

	d = (h_len - 1) / 2;
	for(i = 0; i < v_len; i++) {
		v[i] = 0;
		for(j = 0; j < s_len; j++) {
			if(L * j > M * i + d)
				break;
			if(M * i + d < h_len + L * j)
				v[i] += s[j] * h[M * i + d - L * j];
		}
	}

	if(len_o)
		*len_o = v_len;

	return v;
}


complex *generate_gaussian_pulse(float sps, unsigned int *len_o) {

	unsigned int i, num_samples, center_point;
	float arg, avg_abs_val;
	complex *x, *cx;

	num_samples = (unsigned int)ceil(2 * sps + 1);
	x = new complex[num_samples];
	if(!x) {
		fprintf(stderr, "error: generate_gaussian_pulse: new failed\n");
		if(len_o)
			*len_o = 0;
		return 0;
	}
	cx = x;

	center_point = (num_samples - 1) / 2;
	for(i = 0; i < num_samples; i++) {
		arg = ((float)i - (float)center_point) / (float)sps;
		*cx++ = 0.96 * exp(-1.1380 * arg * arg - 0.527 * arg * arg * arg * arg);
	}

	avg_abs_val = sqrtf(vectornorm2(x, num_samples) / sps);

	cx = x;
	for(i = 0; i < num_samples; i++) 
		*cx++ /= avg_abs_val;

	if(len_o)
		*len_o = num_samples;

	return x;
}


complex *modulate(const unsigned char *bv, const unsigned int bv_len, const unsigned int guard_len, float sps, unsigned int *len_o) {

	unsigned int i, len, m_len;
	complex *c, *m;

	len = (unsigned int)ceil(sps * (bv_len + guard_len));
	complex bv_p[len];
	memset(bv_p, 0, sizeof(complex) * len);

	// polarize bv
	c = bv_p;
	for(i = 0; i < bv_len; i++) {
		*c = 1.0 - 2.0 * bv[i];
		c += (unsigned int)floor(sps);
	}

	// rotate
	if(gmsk_rotate(bv_p, len)) {
		if(len_o)
			*len_o = 0;
		return 0;
	}

	if(!m_gaussian_pulse) {
		m_gaussian_pulse = generate_gaussian_pulse(1.0, &m_gaussian_pulse_len);
		if(!m_gaussian_pulse) {
			if(len_o)
				*len_o = 0;
			return 0;
		}
	}

	// convolve with gaussian pulse
	m = convolve_nodelay(bv_p, len, m_gaussian_pulse, m_gaussian_pulse_len, &m_len);

	if(len_o)
		*len_o = m_len;

	return m;
}


/*
 * a		signal correlated with tsc
 * a_len	length of the correlation
 * c_len	length of desired channel response
 * toa		index of peak
 * peak		peak of training sequence
 *
 * returns channel response
 */
complex *generate_channel_response(complex *a, unsigned int a_len, unsigned int c_len, float toa, complex peak) {

	unsigned int i, max_i, u_toa;
	float max_energy, energy;
	complex *c;

	c = new complex[c_len];
	if(!c) {
		fprintf(stderr, "error: generate_channel_response: new failed\n");
		return 0;
	}

	// find a c_len window around peak that has the most energy
	u_toa = (unsigned int)nearbyintf(toa);
	max_energy = -1.0;
	max_i = 0;
	for(i = 0; i < c_len; i++) {
		if((u_toa + i < c_len - 1) || (u_toa + i > a_len - 1))
			continue;
		energy = vectornorm2(a + u_toa + i - c_len + 1, c_len);
		if(energy > 0.95 * max_energy) {
			max_i = i;
			max_energy = energy;
		}
	}
	if(max_energy < 0) {
		fprintf(stderr, "error: could not generate a %d-tap channel response\n", c_len);
		return 0;
	}

	// copy channel response window from correlated signal
	for(i = 0; i < c_len; i++)
		c[i] = a[u_toa + max_i - c_len + 1 + i];
	scale(c, c_len, complex(1.0, 0.0) / peak);

	return c;
}


/*
 *	Fast Computation of Channel-Estimate Based Equalizers in
 *	Packet Data Transmission.
 *
 *	Naofal M. W. Al-Dhahir and John M. Cioffi.
 *
 * Symbol-spaced sampling case.
 *
 * 	h	channel response
 * 	h_len	length of channel response, (channel memory + 1)
 * 	SNR	estimate of SNR
 * 	Nf	number of feedforward taps
 */
int design_DFE(
   const complex *h, const unsigned int h_len,
   const float SNR,
   const unsigned int Nf,
   complex **feedforward_o, unsigned int *feedforward_len,
   complex **feedback_o, unsigned int *feedback_len
) {

	// sanity check
	if(Nf < 2) {
		fprintf(stderr, "error: design_DFE: taps must be >= 2\n");
		return -1;
	}

	float d = 1.0;
	complex v_k, w_i;
	complex *feedforward, *feedback;

	// channel memory
	unsigned int nu = h_len - 1;

	complex Gl[nu + 1], Gr[nu + 1], tGl[nu + 1], tGr[nu + 1], v[Nf];
	complex L[Nf][Nf + nu]; // L is transposed, i.e., columns x rows

	memset(Gl, 0, sizeof(Gl));
	memset(Gr, 0, sizeof(Gr));
	memset(L, 0, sizeof(L));

	// G_0(D) = G(D) = [ 1/sqrt(SNR) & h^*(D*) ] = [ Gl & Gr ]
	Gl[0] = 1.0 / sqrtf(SNR);
	for(unsigned int i = 0; i <= nu; i++)
		Gr[i] = conj(h[i]);

	/*
	 * Iterate to compute the N_f column of L, l_{N_f -1}(D), needed to
	 * compute the feedback filter
	 */
	for(unsigned int i = 0; i < Nf; i++) {

		// d_i = norm(G_i(0))^2
		d = norm(Gl[0]) + norm(Gr[0]);

		// l_i(D) = D^i G_i(D) G_i^*(0) d_i^-1
		for(unsigned int j = 0; j <= nu; j++)
			L[i][i + j] =
			   (Gl[j] * conj(Gl[0]) + Gr[j] * conj(Gr[0])) / d;

		// [ \alpha_i & \beta_i ] = d_i^{-1/2}G_i(0)
		// k_i = \beta_i / \alpha_i
		complex k = Gr[0] / Gl[0];

		// D G_{i + 1}(D) = G_i(D) [ D & -k_i \\ k_i^* D & 1 ]
		if(i != Nf - 1) {

			// tGl = Gl + Gr k^*
			scale(tGl, Gr, nu + 1, conj(k));
			add(tGl, nu + 1, Gl, nu + 1);

			// tGr = -k Gl + Gr
			scale(tGr, Gl, nu + 1, -k);
			add(tGr, nu + 1, Gr, nu + 1);

			// factor out D
			for(unsigned int t = 0; t < nu; t++)
				tGr[t] = tGr[t + 1];
			tGr[nu] = 0;

			// G = tG / sqrtf(1.0 + norm(k))
			scale(Gl, tGl, nu + 1, 1.0 / sqrtf(1.0 + norm(k)));
			scale(Gr, tGr, nu + 1, 1.0 / sqrtf(1.0 + norm(k)));
		}
	}

	// D^i b = [ 0 ... 0 1 b_1 ... b_nu ] is the N_f column of L
	feedback = new complex[nu];
	if(!feedback) {
		fprintf(stderr, "error: design_DFE: new failed\n");
		return -1;
	}

	// filter is \delta - b, i.e., don't copy leading 1 and make negative
	memcpy(feedback, &(L[Nf - 1][Nf]), nu * sizeof(complex));
	scale(feedback, nu, (complex)-1.0);

	// we want b^*
	conjugate_vector(feedback, nu);

	// back substitute to find w^*
	v[Nf - 1] = 1.0;
	for(int k = Nf - 2; k >= 0; k--) {
		v_k = 0.0;
		for(unsigned int j = k + 1; j <= Nf - 1; j++)
			v_k -= L[k][j] * v[j]; // remember, our L is transposed
		v[k] = v_k;
	}

	// w^* = d_{N_f - 1}^{-1} [ v_{N_f - 1}^* & 0_{1 x \nu} ] H^*
	feedforward = new complex[Nf];
	if(!feedforward) {
		fprintf(stderr, "error: design_DFE: new failed\n");
		delete[] feedback;
		return -1;
	}
	for(unsigned int i = 0; i < Nf; i++) {
		w_i = 0.0;
		unsigned int j = MIN(nu, Nf - 1 - i);
		for(unsigned int k = 0; k <= j; k++) {
			w_i += v[k + i] * conj(h[k]);
		}
		feedforward[i] = w_i / d;
	}

	*feedback_o = feedback;
	*feedback_len = nu;
	*feedforward_o = feedforward;
	*feedforward_len = Nf;

	return 0;
}


/*
 * Equalizer
 *
 * v		complex symbol-spaced samples
 * v_len	the number of samples in input
 * feedforward	the feedforward filter from design_DFE
 * feedback	the feedback filter from design_DFE
 */
float *equalize(complex *v, const unsigned int v_len,
   const complex *feedforward, const unsigned int feedforward_len,
   const complex *feedback, const unsigned int feedback_len,
   unsigned int *len_o) {

	unsigned int i, j;
	complex *post_forward_full, post_forward[v_len], DFE_output[v_len];

	// apply feedforward filter
	post_forward_full = convolve(v, v_len, feedforward, feedforward_len, 0);
	if(!post_forward_full) {
		fprintf(stderr, "error: equalize: new failed\n");
		return 0;
	}

	// remove delay XXX should just convolve_nodelay ???
	for(unsigned int i = 0; i < v_len; i++)
		post_forward[i] = post_forward_full[feedforward_len - 1 + i];
	delete[] post_forward_full;

	// apply feedback filter
	for(i = 0; i < v_len; i++) {

		// TODO knowing tail bits == 0 could increase accuracy

		// the current value is affected by earlier decisions
		for(j = 0; (j < feedback_len) && (i >= j + 1); j++) {
			post_forward[i] = post_forward[i] + feedback[j] * post_forward[i - j - 1];
		}

		// reverse rotate data for output
		post_forward[i] *= m_gmsk_rrotator[i];

		// save output
		DFE_output[i] = post_forward[i];

		// hard-slice decision
		post_forward[i] = (post_forward[i].real() > 0.0)? 1.0 : -1.0;

		// rotate back to be aligned with previous data
		post_forward[i] *= m_gmsk_rotator[i];
	}

	// return a soft-slice of values
	return slice_soft(DFE_output, v_len, len_o);
}
