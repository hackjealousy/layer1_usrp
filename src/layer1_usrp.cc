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

#ifdef HAVE_CONFIG_H
#include "config.h"
#else
#define PACKAGE_VERSION "custom build"
#endif /* HAVE_CONFIG_H */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#ifdef D_HOST_OSX
#include <libgen.h>
#endif /* D_HOST_OSX */
#include <string.h>
#include <sys/time.h>
#include <errno.h>

#include <usrp/usrp_dbid.h>

#include "usrp_source.h"
#include "fcch_detector.h"
#include "arfcn_freq.h"
#include "offset.h"
#include "version.h"
#include "gsm.h"
#include "util.h"
#include "sch.h"
#include "gsm_demod.h"
#include "gsm_bursts.h"
#include "sch.h"

static const float default_gain = 0.45;


void usage(char *prog) {

	printf("layer1_usrp v%s, Copyright (c) 2011, Joshua Lackey\n", layer1_usrp_version_string);
	printf("\nUsage:\n");
	printf("\t%s <-f frequency | -c channel> [options]\n", basename(prog));
	printf("\n");
	printf("Where options are:\n");
	printf("\t-a <addr>\tUHD device address\n");
	printf("\t-f <freq>\tfrequency of nearby GSM base station\n");
	printf("\t-c <chan>\tchannel of nearby GSM base station\n");
	printf("\t-b <band>\tband indicator (GSM850, GSM900, EGSM, DCS, PCS)\n");
	printf("\t-g <gain>\tgain as %% of range, defaults to %.0f%%\n", 100 * default_gain);
	printf("\t-R <side>\tside A (0) or B (1), defaults to B\n");
	printf("\t-A <ant>\tantenna TX/RX (0) or RX2 (1), defaults to RX2\n");
	printf("\t-F <freq>\tFPGA master clock frequency\n");
	printf("\t-2\t\tuse USRP2 series\n");
	printf("\t-x\t\tuse external reference clock\n");
	printf("\t-h\t\thelp\n");
	exit(-1);
}


int main(int argc, char **argv) {

	char *device_address = 0, *endptr;
	int c, bi = BI_NOT_DEFINED, chan = -1, two_series = 0, subdev = -1, antenna = -1;
	long int fpga_master_clock_freq = 0;
	float gain = default_gain;
	double freq = -1.0;
	usrp_source *u;

	while((c = getopt(argc, argv, "a:f:c:b:g:R:A:F:x2h?")) != EOF) {
		switch(c) {
			case 'a':
				device_address = optarg;
				break;

			case 'f':
				freq = strtod(optarg, 0);
				break;

			case 'c':
				chan = strtoul(optarg, 0, 0);
				break;

			case 'b':
				if((bi = str_to_bi(optarg)) == -1) {
					fprintf(stderr, "error: bad band indicator: ``%s''\n", optarg);
					usage(argv[0]);
				}
				break;

			case 'g':
				gain = strtod(optarg, 0);
				if((gain > 1.0) && (gain <= 100.0))
					gain /= 100.0;
				if((gain < 0.0) || (1.0 < gain))
					usage(argv[0]);
				break;

			case 'R':
				errno = 0;
				subdev = strtol(optarg, &endptr, 0);
				if((!errno) && (endptr != optarg))
					break;
				if(tolower(*optarg) == 'a') {
					subdev = 0;
				} else if(tolower(*optarg) == 'b') {
					subdev = 1;
				} else {
					fprintf(stderr, "error: bad side: ``%s''\n", optarg);
					usage(argv[0]);
				}
				break;

			case 'A':
				if(!strcmp(optarg, "RX2")) {
					antenna = 1;
				} else if(!strcmp(optarg, "TX/RX")) {
					antenna = 0;
				} else {
					errno = 0;
					antenna = strtol(optarg, &endptr, 0);
					if(errno || (endptr == optarg)) {
						fprintf(stderr, "error: bad antenna: ``%s''\n", optarg);
						usage(argv[0]);
					}
				}
				break;

			case 'F':
				fpga_master_clock_freq = strtol(optarg, 0, 0);
				if(!fpga_master_clock_freq)
					fpga_master_clock_freq = (long int)strtod(optarg, 0); 

				// was answer in MHz?
				if(fpga_master_clock_freq < 1000) {
					fpga_master_clock_freq *= 1000000;
				}

				if(!fpga_master_clock_freq)
					usage(argv[0]);
				break;

			case 'h':
			case '?':
			default:
				usage(argv[0]);
				break;
		}

	}

	if(freq < 0.0) {
		if(chan < 0) {
			fprintf(stderr, "error: must enter channel or frequency\n");
			usage(argv[0]);
		}
		if((freq = arfcn_to_freq(chan, &bi)) < 869e6)
			usage(argv[0]);
	}
	if((freq < 869e6) || (2e9 < freq)) {
		fprintf(stderr, "error: bad frequency: %lf\n", freq);
		usage(argv[0]);
	}
	if((chan = freq_to_arfcn(freq, &bi)) < 0) {
		fprintf(stderr, "error: not a GSM frequency: %lf\n", freq);
		return -1;
	}
	u = new usrp_source(GSM_RATE, device_address, fpga_master_clock_freq);
	if(!u) {
		fprintf(stderr, "error: usrp_source\n");
		return -1;
	}
	if(two_series)
		u->set_usrp2();
	if(u->open() == -1) {
		fprintf(stderr, "error: usrp_source::open\n");
		return -1;
	}
	if(subdev >= 0) {
		u->set_subdev(subdev);
	}
	if(antenna >= 0) {
		u->set_antenna(antenna);
	}
	if(!u->set_gain(gain)) {
		fprintf(stderr, "error: usrp_source::set_gain\n");
		return -1;
	}
	/*
	if((bcf = band_center(bi)) < 0.0) {
		fprintf(stderr, "error: bad band center\n");
		return -1;
	}
	if(u->tune_if(bcf)) {
		fprintf(stderr, "error: usrp_source::tune_if\n");
		return -1;
	}
	if(u->fast_tune(freq)) {
		fprintf(stderr, "error: usrp_source::fast_tune\n");
		return -1;
	}
	 */
	if(u->tune(freq)) {
		fprintf(stderr, "error: usrp_source::tune\n");
		return -1;
	}
	fprintf(stderr, "Daughterboard %s (antenna %s)\n", u->get_subdev_name(), u->get_antenna_name());
	fprintf(stderr, "Using %s channel %d (%.1fMHz)\n", bi_to_str(bi), chan, freq / 1e6);

	u->start();
	u->flush();

	complex *buf;
	unsigned int buf_len;

	mtsc_s *m = 0;
	// dfe_filter_s *d = 0;

	if(generate_modulated_tsc(1.0, sb_etsc, SB_CODE_LEN, SB_ETS_OS, &m) == -1) {
		return -1;
	}
	if(!(buf = get_burst_sch(u, &buf_len))) {
		printf("get_burst_sch: fail\n");
		return -1;
	}

	/*
	if((f = demod_burst(1.0, &f_len, sch_buf, sch_buf_len, m, 0))) {
		if(!decode_sch_soft(f, &fn, &bsic)) {
			success += 1;
			printf("%d %d\n", fn, bsic);
		} else
			printf("failed\n");
		delete[] f;
	} else
		printf("failed\n");
	 */

	u->stop();
	return 0;
}
