# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++

# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
LCMDIR=../../LCM/
LCMLIB=../../LCM/lib/libtypes.a

MAVLCMLIB=../../../Fixie/build/lib/liblcmtypes_mav-lcmtypes.a

MAVLIB=../../../mav/mavconn/build/lib

CFLAGS=-c -std=c++0x -Wall -O3 `pkg-config --cflags lcm bot2-core bot2-param-client bot2-lcmgl-client bot2-frames` -I/$(LCMDIR)

LIBS=`pkg-config --libs lcm bot2-core bot2-param-client bot2-lcmgl-client bot2-frames` $(LCMLIB)




all: stereo-monitor

stereo-monitor: stereo-monitor.o
	$(CC) stereo-monitor.o -o stereo-monitor $(LIBS)

stereo-monitor.o: stereo-monitor.cpp
	$(CC) $(CFLAGS) stereo-monitor.cpp

clean:
	rm -rf *o stereo-monitor



