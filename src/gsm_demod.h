#pragma once
#include "usrp_complex.h"
#include "usrp_source.h"

typedef struct {
	complex *	tsc;		// modulated training sequence code
	unsigned int	len;		// length of modulated tsc
	float		toa;		// time of arrival for midamble into tsc
	complex		gain;		// peak of correlation between midamble and tsc
} mtsc_s;


typedef struct {
	complex *	ff;		// feedforward filter
	unsigned int	ff_len;		// feedforward filter len
	complex *	fb;		// feedback filter
	unsigned int	fb_len;		// feedback filter len
} dfe_filter_s;


complex *generate_modulated_tsc(const float sps, const unsigned char *tsc,
   const unsigned int tsc_len, const unsigned int tsc_offset,
   float *toa_o, complex *gain_o, unsigned int *mtsc_len_o);

int generate_modulated_tsc(const float sps, const unsigned char *tsc,
   const unsigned int tsc_len, const unsigned int tsc_offset, mtsc_s **mtsc);

complex *get_burst_sch(usrp_source *u, unsigned int *buf_len);

complex *get_burst(usrp_source *u, unsigned int *burst_len,
   const unsigned int fn, const unsigned int ts);

float *demod_burst(const float sps, unsigned int *burst_len,
   const complex * const s, const unsigned int s_len,
   const mtsc_s *mtsc,
   dfe_filter_s **d,
   unsigned int cr_len = 6, unsigned int dfe_len = 5);
