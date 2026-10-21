OBJS :=	src/main.cpp

CFLAGS = -g `pkg-config --cflags jack libusb-1.0 liblo`

# boost_system is a linkable library only on boost < 1.69; since then
# it is header-only and the library (and its pkg-config file) no longer
# exist, so -lboost_system fails.  Detect which case we are on:
#  1. pkg-config module libboost_system (Debian/Ubuntu ship one for
#     every boost lib), or
#  2. the .a/.so present in a standard lib directory.
BOOST_SYSTEM_DIR := $(shell pkg-config --variable=libdir libboost_system 2>/dev/null)
ifeq ($(BOOST_SYSTEM_DIR),)
	BOOST_SYSTEM_DIR := $(firstword $(foreach d,/usr/lib/x86_64-linux-gnu /usr/lib64 /usr/lib,$(if $(wildcard $(d)/libboost_system.so $(d)/libboost_system.a),$(d),)))
endif
ifneq ($(BOOST_SYSTEM_DIR),)
	BOOST_SYSTEM_LIB = -L$(BOOST_SYSTEM_DIR) -lboost_system
else
	BOOST_SYSTEM_LIB =
endif

LIBS = `pkg-config --libs jack libusb-1.0 liblo` -lrt $(BOOST_SYSTEM_LIB)

ultranova4linux: $(OBJS)
		 g++ $(CFLAGS) -o $@ $(OBJS) $(LIBS)

%.o:	%.cpp
	g++ $(CFLAGS) -g -Wall -c -o $@ $<

clean:;	rm -f src/*.o ultranova4linux

.PHONY: print-boost
print-boost:
	@echo "BOOST_SYSTEM_DIR = $(BOOST_SYSTEM_DIR)"
	@echo "BOOST_SYSTEM_LIB = $(BOOST_SYSTEM_LIB)"
