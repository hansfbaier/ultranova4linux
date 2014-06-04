OBJS :=	src/main.cpp

CFLAGS = -g `pkg-config --cflags jack libusb-1.0 glib-2.0`
LIBS = `pkg-config --libs jack libusb-1.0 glib-2.0` -lrt

ultranova4linux: $(OBJS)
		 g++ $(CFLAGS) -o $@ $(OBJS) $(LIBS)

%.o:	%.cpp
	g++ $(CFLAGS) -g -Wall -c -o $@ $<

clean:;	rm -f src/*.o ultranova4linux
