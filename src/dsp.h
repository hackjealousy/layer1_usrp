#pragma once
#include "usrp_complex.h"

int build_rotators();
float vectornorm2(const complex *v, const unsigned int len);
float sinc(const float x);
complex interpolate_point(const complex *s, const unsigned int s_len, const float s_i);
float peak_detect(const complex *s, const unsigned int s_len, complex *peak, float *avg_power);
int peak2mean(complex *c, unsigned int c_len, complex peak, unsigned int peak_i, unsigned int width, float *p2m);
int gmsk_rotate(complex *v, const unsigned int len, const unsigned int offset);
int gmsk_rotate(complex *v, const unsigned int len);
int gmsk_rrotate(complex *v, const unsigned int len);
void scale(complex *v, const unsigned int v_len, const complex s);
void scale(complex *v, const unsigned int v_len, const float s);
void scale(complex *u, const complex *v, const unsigned int v_len, const complex s);
void add(complex *x, const unsigned int x_len, const complex *y, const unsigned int y_len);
void conjugate_vector(complex *v, const unsigned int v_len);
float *slice_soft(const complex *v, const unsigned int v_len, unsigned int *len_o);
void slice_soft(float *s, const complex *v, const unsigned int v_len);
unsigned char *slice(complex *s, unsigned int s_len);
unsigned char *slice(float *s, unsigned int s_len);
complex *convolve(const complex *s, const unsigned int s_len, const complex *h, const unsigned int h_len, unsigned int *len_o);
void convolve_nodelay(complex *y, const complex *s, const unsigned int s_len, const complex *h, const unsigned int h_len);
complex *convolve_nodelay(const complex *s, const unsigned int s_len, const complex *h, const unsigned int h_len, unsigned int *len_o);
complex *correlate(const complex *s1, const unsigned int s1_len, const complex *s2, const unsigned int s2_len, unsigned int *len_o);
complex *correlate_nodelay(const complex *s1, const unsigned int s1_len, const complex *s2, const unsigned int s2_len, unsigned int *len_o);
int delay(complex *v, const unsigned int v_len, const float toa);
complex *modulate(const unsigned char *bv, const unsigned int bv_len, const unsigned int guard_len, float sps, unsigned int *len_o);
complex *polyphase_resample(const complex *s, const unsigned int s_len, const unsigned int L, const unsigned int M, const complex *h, const unsigned int h_len, unsigned int *len_o);
complex *generate_channel_response(complex *a, unsigned int a_len, unsigned int c_len, float toa, complex peak);
int design_DFE(const complex *h, const unsigned int h_len, const float SNR, const unsigned int Nf, complex **ff_o, unsigned int *ff_len, complex **fb_o, unsigned int *fb_len);
float *equalize(complex *v, const unsigned int v_len, const complex *ff, const unsigned int ff_len, const complex *fb, const unsigned int fb_len, unsigned int *len_o);
