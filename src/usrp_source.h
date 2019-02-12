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
#pragma once

#include <uhd/usrp/single_usrp.hpp>
#include <uhd/types/time_spec.hpp>

#include "usrp_complex.h"
#include "circular_buffer.h"


class usrp_source {
public:
	usrp_source(double sample_rate, char *device_address = 0, long int fpga_master_clock_freq = 0, bool external_ref = false);
	~usrp_source();

	int open();
	int read(complex *buf, unsigned int num_samples, unsigned int *samples_read);
	int fill(unsigned int num_samples, unsigned int *overrun);
	int tune(double freq);
	/*
	int tune_if(double base);
	int fast_tune(double freq);
	 */
	void set_antenna(int antenna);
	void set_antenna(const std::string antenna);
	const char *get_antenna_name();
	bool set_gain(float gain);
	void start();
	void stop();
	int flush();

	void set_subdev(int ss);
	char *get_subdev_name();

	double sample_rate();
	double band_center();

	void set_usrp2();

	circular_buffer *get_buffer();

	double get_packet_time();
	void get_fn_ts(int *fn, int *ts);

	static const unsigned int side_A = 0;
	static const unsigned int side_B = 1;

private:
	void lock();
	void unlock();
	void set_antenna_nolock(const std::string antenna);

	uhd::usrp::single_usrp::sptr	m_u;
	uhd::device::sptr		m_dev;

	char *			m_device_address;

	double			m_sample_rate;
	double			m_desired_sample_rate;

	long int		m_fpga_master_clock_freq;
	bool			m_external_ref;

	double			m_freq_band_center;

	circular_buffer *	m_cb;

	unsigned int		m_recv_samples_per_packet;

	int			m_two_series;

	uhd::time_spec_t	m_packet_time;
	int			m_fn;
	int			m_ts;

	/*
	 * This mutex protects access to the USRP and daughterboards but not
	 * necessarily to any fields in this class.
	 */
	pthread_mutex_t		m_u_mutex;

	static const unsigned int	CB_LEN		= (1 << 20);
};
