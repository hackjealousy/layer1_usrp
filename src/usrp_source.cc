/*
 * Copyright (c) 2010, Joshua Lackey
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
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <math.h>
#include <complex>
#include <stdexcept>

#include "usrp_source.h"


usrp_source::usrp_source(double sample_rate, char *device_address, long int fpga_master_clock_freq, bool external_ref) {

	m_desired_sample_rate = sample_rate;
	m_device_address = device_address;
	m_fpga_master_clock_freq = fpga_master_clock_freq;
	m_external_ref = external_ref;
	m_sample_rate = 0.0;
	m_freq_band_center = -1.0;

	m_two_series = 0;

	m_u.reset();
	m_dev.reset();

	m_cb = new circular_buffer(CB_LEN, sizeof(complex), 0);

	pthread_mutex_init(&m_u_mutex, 0);

}


usrp_source::~usrp_source() {

	stop();
	pthread_mutex_destroy(&m_u_mutex);
	delete m_cb;
}


void usrp_source::stop() {

	lock();
	if(m_u) {
		uhd::stream_cmd_t cmd = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
		m_u->issue_stream_cmd(cmd);
	}
	unlock();
}


void usrp_source::start() {

	lock();
	if(m_u) {
		uhd::stream_cmd_t cmd = uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS;
		m_u->issue_stream_cmd(cmd);
	}
	unlock();
}


double usrp_source::sample_rate() {

	return m_sample_rate;

}


double usrp_source::band_center() {

	return m_freq_band_center;
}


/*
 * XXX implement with tune_request_t policy
int usrp_source::tune_if(double base) {

	struct freq_result_t fr;

	lock();
	fr = m_db_rx->set_freq(base);
	unlock();
	if(!fr.ok) {
		fprintf(stderr, "error: tune_if: set_freq: failed\n");
		return -1;
	}
	m_freq_band_center = fr.baseband_freq;

	return 0;
}


int usrp_source::fast_tune(double freq) {

	bool inverted = false;
	double ddc_freq;

	if(m_freq_band_center < 0.0) {
		fprintf(stderr, "warning: cannot fast_tune without band center, falling back to tune\n");
		return tune(freq);
	}

	lock();
	usrp_standard_common::calc_dxc_freq(freq, m_freq_band_center, m_u_rx->converter_rate(), &ddc_freq, &inverted);
	if(m_db_rx->spectrum_inverted())
		inverted = !inverted;
	if(inverted && !m_db_rx->is_quadrature()) {
		ddc_freq = -ddc_freq;
		inverted = !inverted;
	}
	if(!m_u_rx->set_rx_freq(0, ddc_freq)) {
		unlock();
		fprintf(stderr, "error: tune: set_rx_freq: failed\n");
		return -1;
	}
	unlock();

	return 0;
}
 */


int usrp_source::tune(double freq) {

	static const double MAX_ALLOWED_ERROR = 1.0; // Hz

	uhd::tune_result_t tr;

	lock();
	tr = m_u->set_rx_freq(freq);
	unlock();

	if(std::abs(tr.target_dsp_freq - tr.actual_dsp_freq) > MAX_ALLOWED_ERROR)
		return 1;
	return 0;
}


void usrp_source::set_antenna(const std::string antenna) {

	lock();
	std::vector<std::string>::iterator it;
	std::vector<std::string> antennas = m_u->get_rx_antennas();
	if((it = find(antennas.begin(), antennas.end(), antenna)) == antennas.end()) {
		fprintf(stderr, "warning; no such antenna: %s\n", antenna.c_str());
	} else
		set_antenna_nolock(antenna);
	unlock();
}


void usrp_source::set_antenna(int antenna) {

	lock();
	std::vector<std::string> antennas = m_u->get_rx_antennas();
	if(antenna < (int)antennas.size())
		set_antenna_nolock(antennas[antenna]);
	unlock();
}


void usrp_source::set_antenna_nolock(const std::string antenna) {

	m_u->set_rx_antenna(antenna);
}


const char *usrp_source::get_antenna_name() {

	const char *r;

	lock();
	r = m_u->get_rx_antenna().c_str();
	unlock();

	return r;
}


void usrp_source::set_subdev(int subdev) {

	lock();
	if(subdev)
		m_u->set_rx_subdev_spec(uhd::usrp::subdev_spec_t("B:0"));
	else
		m_u->set_rx_subdev_spec(uhd::usrp::subdev_spec_t("A:0"));
	unlock();
}


char *usrp_source::get_subdev_name() {

	static char r[BUFSIZ];

	lock();
	snprintf(r, sizeof(r), "%s %s", m_u->get_rx_subdev_spec().to_string().c_str(), m_u->get_rx_subdev_name().c_str());
	unlock();

	return r;
}


bool usrp_source::set_gain(float gain) {

	float min, max;
	uhd::gain_range_t gain_range;

	if((gain < 0.0) || (1.0 < gain))
		return false;

	lock();
	gain_range = m_u->get_rx_gain_range();
	min = gain_range.start();
	max = gain_range.stop();
	m_u->set_rx_gain(min + gain * (max - min));
	unlock();

	return true;
}


void usrp_source::set_usrp2() {

	m_two_series = 1;
}


/*
 * recv_frame_size
 * num_recv_frames
 * send_frame_size
 * num_send_frames
 */
int usrp_source::open() {

	uhd::device_addr_t device_address;
	if(m_device_address)
		device_address = uhd::device_addr_t(m_device_address);
	device_address["recv_frame_size"] = "4096";
	device_address["num_recv_frames"] = "64";

	lock();
	if(!m_u) {
		if(!(m_u = uhd::usrp::single_usrp::make(uhd::device_addr_t(device_address)))) {
			unlock();
			fprintf(stderr, "error: single_usrp::make failed\n");
			return -1;
		}
		if(m_fpga_master_clock_freq)
			m_u->set_master_clock_rate(m_fpga_master_clock_freq);
		m_u->set_rx_rate(m_desired_sample_rate);
		m_sample_rate = m_u->get_rx_rate();

		if(m_two_series) {
			uhd::clock_config_t clock_config;

			clock_config.pps_source = uhd::clock_config_t::PPS_SMA;
			clock_config.pps_polarity = uhd::clock_config_t::PPS_NEG;
			if(m_external_ref)
				clock_config.ref_source = uhd::clock_config_t::REF_SMA;
			else
				clock_config.ref_source = uhd::clock_config_t::REF_INT;
			m_u->set_clock_config(clock_config);
		}
		m_dev = m_u->get_device();
		m_recv_samples_per_packet = m_dev->get_max_recv_samps_per_packet();
	}
	unlock();

	return 0;
}


/*
 * TODO  Overflows are not such a big deal with timestamped packets.
 */
int usrp_source::fill(unsigned int num_samples, unsigned int *overrun_o) {

	unsigned int overruns = 0;
	size_t r;
	complex *c;
	uhd::rx_metadata_t metadata;

	while((m_cb->data_available() < num_samples) && (m_cb->space_available() >= m_recv_samples_per_packet)) {

		// get complex<float> buffer
		c = (complex *)m_cb->poke(0);

		// read one packet from the usrp
		lock();
		r = m_dev->recv(c, m_recv_samples_per_packet, metadata, uhd::io_type_t::COMPLEX_FLOAT32, uhd::device::RECV_MODE_ONE_PACKET);
		unlock();

		/*
		 * XXX 
		 *
		 * offset_detect doesn't work with complex<float> between -1
		 * and 1.  I'm not sure why as there aren't any strange
		 * constants there.  The rest of the code works just fine with
		 * values between -1 and 1.
		 *
		 * I'd go back to the native USRP1 complex<short> translation
		 * but I'm not sure what the native USRP2 implementation uses so
		 * I'm just going to scale the signal for now.
		 */

		for(unsigned int i = 0; i < r; i++)
			c[i] *= 32767.0;

		if((m_cb->data_available() == 0) && (metadata.has_time_spec))
			m_packet_time = metadata.time_spec;

		// update cb
		m_cb->wrote(r);

		// doesn't work on USRP1
		if(metadata.error_code & uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
			printf("overflow\n");
			overruns += 1;
		}
		// metadata.time_spec.get_real_secs();
	}

	// if the cb is full, we left behind data from the usb packet
	if(m_cb->space_available() == 0) {
		fprintf(stderr, "warning: local overrun\n");
		overruns++;
	}

	if(overrun_o)
		*overrun_o = overruns;

	return 0;
}


int usrp_source::read(complex *buf, unsigned int num_samples, unsigned int *samples_read) {

	unsigned int n;

	if(fill(num_samples, 0))
		return -1;

	n = m_cb->read(buf, num_samples);

	if(samples_read)
		*samples_read = n;

	return 0;
}


circular_buffer *usrp_source::get_buffer() {

	return m_cb;
}


int usrp_source::flush() {

	unsigned int space;
	complex *c;
	uhd::rx_metadata_t metadata;

	m_cb->flush();
	m_packet_time = 0.0;

	// get complex<float> buffer just to put samples somewhere
	c = (complex *)m_cb->poke(&space);
	if(m_recv_samples_per_packet < space)
		space = m_recv_samples_per_packet;

	// read all full buffers
	do {
		lock();
		m_dev->recv(c, space, metadata, uhd::io_type_t::COMPLEX_FLOAT32, uhd::device::RECV_MODE_ONE_PACKET, 1.0 / m_sample_rate);
		unlock();
	} while(!(metadata.error_code & uhd::rx_metadata_t::ERROR_CODE_TIMEOUT));

	return 0;
}


void usrp_source::lock() {

	pthread_mutex_lock(&m_u_mutex);
}


void usrp_source::unlock() {

	pthread_mutex_unlock(&m_u_mutex);
}


double usrp_source::get_packet_time() {
	
	return m_packet_time.get_real_secs();
}

void usrp_source::get_fn_ts(int *fn, int *ts) {

	*fn = m_fn;
	*ts = m_ts;
}
