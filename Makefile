OBJS :=	src/main.c

CFLAGS = -g `pkg-config --cflags libusb-1.0`
LIBS = `pkg-config --libs libusb-1.0`

ultranova4linux:	$(OBJS)
		gcc $(CFLAGS) -o $@ $(OBJS) $(LIBS)

%.o:	%.c
	gcc $(CFLAGS) -g -Wall -c -o $@ $<

clean:;	rm -f src/*.o ultranova4linux
