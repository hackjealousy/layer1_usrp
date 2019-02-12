bin_PROGRAMS = layer1_usrp

layer1_usrp_SOURCES = \
   arfcn_freq.cc \
   circular_buffer.cc \
   dsp.cc \
   fcch_detector.cc \
   gsm_demod.cc \
   layer1_usrp.cc \
   offset.cc \
   sch.cc \
   usrp_source.cc \
   util.cc \
   arfcn_freq.h \
   circular_buffer.h \
   dsp.h \
   fcch_detector.h \
   gsm_bursts.h \
   offset.h \
   sch.h \
   usrp_complex.h \
   usrp_source.h \
   util.h\
   version.h

layer1_usrp_CXXFLAGS = $(FFTW3_CFLAGS) $(UHD_CFLAGS)
layer1_usrp_LDADD = $(FFTW3_LIBS) $(UHD_LIBS)
