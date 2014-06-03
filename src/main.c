/*
 * ultranova driver for linux
 */

#include <signal.h>
#include <stdio.h>
#include <stdbool.h>
#include <libusb-1.0/libusb.h>
#include <jack/jack.h>
#include <jack/midiport.h>

#include "automap_protocol.h"

#define USB_VENDOR_ID            0x1235      // USB vendor ID used by the device
#define USB_PRODUCT_ID           0x0011      // USB product ID used by the device
#define CONTROLLER_ENDPOINT_IN   (LIBUSB_ENDPOINT_IN  | 5)   /* endpoint address */
#define CONTROLLER_ENDPOINT_OUT  (LIBUSB_ENDPOINT_OUT | 5)   /* endpoint address */
#define MIDI_ENDPOINT_IN         (LIBUSB_ENDPOINT_IN  | 3)   /* endpoint address */
#define MIDI_ENDPOINT_OUT        (LIBUSB_ENDPOINT_OUT | 3)   /* endpoint address */

// JACK stuff
jack_client_t *client;
jack_port_t *control_out;
jack_port_t *midi_out;
jack_nframes_t nframes;

struct timespec diff(struct timespec start, struct timespec end);
struct timespec last_cycle;
struct timespec cycle_period;

struct timespec control_in_t;
struct timespec midi_in_t;


// USB to MIDI
struct midi_event {
    struct timespec time;
    uint8_t buf[3];
};

#define MAX_EVENTS 1000
volatile bool control_buf_locked;
volatile bool midi_buf_locked;
volatile int control_byte_count;
volatile int midi_byte_count;
struct midi_event control_buf[MAX_EVENTS];
struct midi_event midi_buf[MAX_EVENTS];

int process(jack_nframes_t nframes, void *arg)
{
    struct timespec prev_cycle = last_cycle;
    clock_gettime(CLOCK_REALTIME, &last_cycle);
    cycle_period = diff(prev_cycle, last_cycle);
    if(cycle_period.tv_nsec <= 0) {
        return 0;
    }

    int i;
    void* control_buf_jack = jack_port_get_buffer(control_out, nframes);
    jack_midi_clear_buffer(control_buf_jack);
    void* midi_buf_jack = jack_port_get_buffer(midi_out, nframes);
    jack_midi_clear_buffer(midi_buf_jack);

    uint8_t* buffer = NULL;
    int control_buf_position = 0;
    int buf_byte_pos = 0;
    int current_event_count = control_byte_count - (control_byte_count % 3);

    if(!control_buf_locked) {
        control_buf_locked = true;
        for(i=0; i < current_event_count; i++) {
            control_buf_position = i / 3;
            buf_byte_pos         = i % 3;
            struct midi_event *ev = &control_buf[control_buf_position];

            if(buf_byte_pos == 0) {
                long nsec_since_start = diff(prev_cycle, ev->time).tv_nsec;
                long framepos = (nsec_since_start * nframes) / cycle_period.tv_nsec;
                buffer = jack_midi_event_reserve(control_buf_jack, framepos, 3);
            }
            if(buffer) {
                buffer[buf_byte_pos] = ev->buf[buf_byte_pos];
            }
        }

        if(i > 0 && (control_byte_count % 3) > 0) {
            struct midi_event *current_event = &control_buf[current_event_count / 3];
            control_buf[0].time = current_event->time;
            control_buf[0].buf[0] = current_event->buf[0];
            control_buf[0].buf[1] = current_event->buf[1];
            control_buf[0].buf[2] = current_event->buf[2];
        }

        control_byte_count %= 3;
        control_buf_locked = false;
    }

    buffer = NULL;
    int midi_buf_position = 0;
    buf_byte_pos = 0;
    current_event_count = midi_byte_count - (midi_byte_count % 3);

    if(!midi_buf_locked) {
        midi_buf_locked = true;
        for(i=0; i < current_event_count; i++) {
            midi_buf_position = i / 3;
            buf_byte_pos         = i % 3;
            struct midi_event *ev = &midi_buf[midi_buf_position];

            if(buf_byte_pos == 0) {
                long nsec_since_start = diff(prev_cycle, ev->time).tv_nsec;
                long framepos = (nsec_since_start * nframes) / cycle_period.tv_nsec;
                buffer = jack_midi_event_reserve(midi_buf_jack, framepos, 3);
            }
            if(buffer) {
                buffer[buf_byte_pos] = ev->buf[buf_byte_pos];
            }
        }

        if(i > 0 && (midi_byte_count % 3) > 0) {
            struct midi_event *current_event = &midi_buf[current_event_count / 3];
            midi_buf[0].time = current_event->time;
            midi_buf[0].buf[0] = current_event->buf[0];
            midi_buf[0].buf[1] = current_event->buf[1];
            midi_buf[0].buf[2] = current_event->buf[2];
        }

        midi_byte_count %= 3;
        midi_buf_locked = false;
    }

    return 0;
}

// USB
struct libusb_device_handle *devh = NULL;
#define LEN_IN_BUFFER 32
static uint8_t in_buffer_control[LEN_IN_BUFFER];
static uint8_t in_buffer_midi[LEN_IN_BUFFER];

#define CONTROLLER_MAXLENGTH 0x18

// OUT-going transfers (OUT from host PC to USB-device)
struct libusb_transfer *control_transfer_out = NULL;
struct libusb_transfer *midi_transfer_out    = NULL;

// IN-coming transfers (IN to host PC from USB-device)
struct libusb_transfer *control_transfer_in = NULL;
struct libusb_transfer *midi_transfer_in    = NULL;

static libusb_context *ctx = NULL;

bool do_exit = false;

// Function Prototypes:
void sighandler(int signum);
void print_libusb_transfer(struct libusb_transfer *p_t);

enum {
    OUT_DEINIT,
    OUT_RELEASE,
    OUT
} exitflag;

// state machine
enum {
    STARTUP,
    WAIT_FOR_AUTOMAP,
    AUTOMAP_PRESSED,
    LISTEN,
} state = STARTUP;

char *state_names[] = {
    "STARTUP",
    "WAIT_FOR_AUTOMAP",
    "AUTOMAP_PRESSED",
    "LISTEN",
};

bool buffer_equal(uint8_t *expected, uint8_t *actual, int length)
{
    int i;

    for(i = 0; i < length; i++) {
        if(expected[i] != actual[i]) {
            return false;
        }
    }

    return true;
}



// Out Callback
//   - This is called after the Out transfer has been received by libusb
void cb_control_out(struct libusb_transfer *transfer)
{
    fprintf(stderr, "cb_control_out: ");
    print_libusb_transfer(transfer);
}


void cb_control_in(struct libusb_transfer *transfer)
{
    clock_gettime(CLOCK_REALTIME, &control_in_t);
    fprintf(stderr, "cb_control_in: ");
    print_libusb_transfer(transfer);

    if(transfer->actual_length == sizeof(automap_button_press_in) &&
       buffer_equal(automap_ok, transfer->buffer, sizeof(automap_button_press_in))) {
        state = AUTOMAP_PRESSED;
        fprintf(stderr, "AUTOMAP PRESSED\n");
    }

    switch(state) {
    case STARTUP:
        if(transfer->actual_length == sizeof(automap_ok) &&
           buffer_equal(automap_ok, transfer->buffer, sizeof(automap_ok))) {
            state = LISTEN;
        } else if(transfer->actual_length == sizeof(automap_off) &&
                  buffer_equal(automap_off, transfer->buffer, sizeof(automap_off))) {
            state = WAIT_FOR_AUTOMAP;
        } else {
            fprintf(stderr, "state STARTUP, got unexpected reply\n");
            fflush(stderr);
        }
        break;

    case WAIT_FOR_AUTOMAP:
        if(transfer->actual_length == sizeof(automap_ok) &&
           buffer_equal(automap_ok, transfer->buffer, sizeof(automap_ok))) {
            libusb_fill_interrupt_transfer(control_transfer_out, devh, CONTROLLER_ENDPOINT_OUT,
                                           automap_ok, sizeof(automap_ok),
                                           cb_control_out, NULL, 0);
            libusb_submit_transfer(control_transfer_out);
            
            state = LISTEN;
        }
        break;

    case AUTOMAP_PRESSED:
        state = LISTEN;
        break;

    case LISTEN:
        if(transfer->actual_length == sizeof(automap_off) &&
           buffer_equal(automap_off, transfer->buffer, sizeof(automap_off))) {
            state = WAIT_FOR_AUTOMAP;
        } else {
            while(control_buf_locked) {
                fprintf(stderr, "control_in busy waiting for control_buf\n");
            }
            int i;
            control_buf_locked = true;
            for(i = 0; i < transfer->actual_length; i++) {
                int control_buf_position = control_byte_count / 3;
                int buf_byte_pos         = control_byte_count % 3;
                struct midi_event *ev = &control_buf[control_buf_position];
                if(buf_byte_pos == 2) { ev->time = control_in_t; }
                ev->buf[buf_byte_pos] = transfer->buffer[i];
                control_byte_count++;
                if(control_byte_count >= MAX_EVENTS * 3) {
                    break;
                }
            }
            control_buf_locked = false;
        }
        break;

    default:
        break;
    }

    libusb_submit_transfer(control_transfer_in);
}

void cb_midi_out(struct libusb_transfer *transfer)
{
    fprintf(stderr, "cb_midi_out: ");
    print_libusb_transfer(transfer);
}

void cb_midi_in(struct libusb_transfer *transfer)
{
    clock_gettime(CLOCK_REALTIME, &midi_in_t);
    fprintf(stderr, "cb_midi_in: ");
    print_libusb_transfer(transfer);
    
    while(midi_buf_locked) {
        fprintf(stderr, "midi_in busy waiting for midi_buf\n");
    }
    int i;
    midi_buf_locked = true;
    for(i = 0; i < transfer->actual_length; i++) {
        int midi_buf_position = midi_byte_count / 3;
        int buf_byte_pos      = midi_byte_count % 3;
        struct midi_event *ev = &midi_buf[midi_buf_position];
        if(buf_byte_pos == 2) { ev->time = midi_in_t; }
        ev->buf[buf_byte_pos] = transfer->buffer[i];
        midi_byte_count++;
        if(midi_byte_count >= MAX_EVENTS * 3) {
            break;
        }
    }
    midi_buf_locked = false;

    libusb_submit_transfer(midi_transfer_in);
}

int main(void)
{
    struct sigaction sigact;

    int r = 1;  // result
    int i;

    //init libUSB
    r = libusb_init(NULL);
    if (r < 0) {
        fprintf(stderr, "Failed to initialise libusb\n");
        return 1;
    }

    //open the device
    devh = libusb_open_device_with_vid_pid(ctx, USB_VENDOR_ID, USB_PRODUCT_ID);
     if (!devh) {
         perror("device not found");
         return 1;
     }

     //claim the interface
     r = libusb_claim_interface(devh, 0);
     if (r < 0) {
         fprintf(stderr, "usb_claim_interface error %d\n", r);
         exitflag = OUT;
         do_exit = true;
     } else  {
         fprintf(stderr, "Claimed interface\n");

         // init jack
         fprintf(stderr, "initializing jack\n");
         if((client = jack_client_open ("ultranova", JackNullOption, NULL)) == 0) {
             fprintf (stderr, "jack server not running?\n");
             do_exit = true;
         }
         jack_set_process_callback (client, process, 0);
         control_out = jack_port_register (client, "control_out", JACK_DEFAULT_MIDI_TYPE, JackPortIsOutput, 0);
         midi_out = jack_port_register (client, "midi_out", JACK_DEFAULT_MIDI_TYPE, JackPortIsOutput, 0);
         nframes = jack_get_buffer_size(client);
         if (jack_activate(client)) {
             fprintf (stderr, "cannot activate client");
             do_exit = true;
         }

         // allocate transfers
         control_transfer_in  = libusb_alloc_transfer(0);
         control_transfer_out = libusb_alloc_transfer(0);
         midi_transfer_in     = libusb_alloc_transfer(0);
         midi_transfer_out    = libusb_alloc_transfer(0);

         libusb_fill_interrupt_transfer(control_transfer_in, devh, CONTROLLER_ENDPOINT_IN,
                                        in_buffer_control, CONTROLLER_MAXLENGTH,
                                        cb_control_in, NULL, 0);
         libusb_fill_interrupt_transfer(midi_transfer_in, devh, MIDI_ENDPOINT_IN,
                                        in_buffer_midi, CONTROLLER_MAXLENGTH,
                                        cb_midi_in, NULL, 0);

         //submit the transfer, all following transfers are initiated from the CB
         libusb_submit_transfer(control_transfer_in);
         libusb_submit_transfer(midi_transfer_in);

         libusb_fill_interrupt_transfer(control_transfer_out, devh, CONTROLLER_ENDPOINT_OUT,
                                        automap_ok, sizeof(automap_ok),
                                        cb_control_out, NULL, 0);
         libusb_submit_transfer(control_transfer_out);

         // Define signal handler to catch system generated signals
         // (If user hits CTRL+C, this will deal with it.)
         sigact.sa_handler = sighandler;  // sighandler is defined below. It just sets do_exit.
         sigemptyset(&sigact.sa_mask);
         sigact.sa_flags = 0;
         sigaction(SIGINT, &sigact, NULL);
         sigaction(SIGTERM, &sigact, NULL);
         sigaction(SIGQUIT, &sigact, NULL);

         printf("Entering loop to process callbacks...\n");
    }

    /* The implementation of the following while loop makes a huge difference.
     * Since libUSB asynchronous mode doesn't create a background thread,
     * libUSB can't create a callback out of nowhere. This loop calls the event handler.
     * In real applications you might want to create a background thread or call the event
     * handler from your main event hanlder.
     * For a proper description see:
     * http://libusbx.sourceforge.net/api-1.0/group__asyncio.html#asyncevent
     * http://libusbx.sourceforge.net/api-1.0/group__poll.html
     * http://libusbx.sourceforge.net/api-1.0/mtasync.html
     */
     if(1) {
         // This implementation uses a blocking call
         while (!do_exit) {
             r = libusb_handle_events_completed(ctx, NULL);
             if (r < 0){   // negative values are errors
                 exitflag = OUT_DEINIT;
                 break;
             }
         }
     } else {
         // This implementation uses a blocking call and aquires a lock to the event handler
         struct timeval timeout;
         timeout.tv_sec  = 0;       // seconds
         timeout.tv_usec = 100000;  // ( .1 sec)
         libusb_lock_events(ctx);
         while (!do_exit) {
             r = libusb_handle_events_locked(ctx, &timeout);
             if (r < 0){   // negative values are errors
                 exitflag = OUT_DEINIT;
                 break;
             }
         }
         libusb_unlock_events(ctx);
     }

     // If these transfers did not complete then we cancel them.
     // Unsure if this is correct...
     if (control_transfer_out) {
         r = libusb_cancel_transfer(control_transfer_out);
         if (0 == r) {
             printf("control_transfer_out successfully cancelled\n");
         }
         if (r < 0) {
             exitflag = OUT_DEINIT;
         }

     }
     if (control_transfer_in) {
         r = libusb_cancel_transfer(control_transfer_in);
         if (0 == r) {
             printf("control_transfer_in successfully cancelled\n");
         }
         if (r < 0) {
             exitflag = OUT_DEINIT;
         }
     }

     switch(exitflag) {
     case OUT_DEINIT:
         printf("at OUT_DEINIT\n");
         libusb_free_transfer(control_transfer_out);
         libusb_free_transfer(control_transfer_in);
         jack_client_close(client);

     case OUT_RELEASE:
         libusb_release_interface(devh, 0);

     case OUT:
         libusb_close(devh);
         libusb_exit(NULL);
     }
     return 0;
}


// This will catch user initiated CTRL+C type events and allow the program to exit
void sighandler(int signum)
{
    printf("sighandler\n");
    do_exit = true;
}


// debugging function to display libusb_transfer
void print_libusb_transfer(struct libusb_transfer *p_t)
{   
    int i;
    if (NULL == p_t) {
        printf("No libusb_transfer...\n");
    }
    else {
        printf("state: %s\n", state_names[state]);

        printf("libusb_transfer structure:\n");
        printf("status  = %x \n", p_t->status);
        printf("flags   = %x \n", p_t->flags);
        printf("endpoint= %x \n", p_t->endpoint);
        printf("type    = %x \n", p_t->type);
        printf("timeout = %d \n", p_t->timeout);
        // length, and buffer are commands sent to the device
        printf("length        = %d \n", p_t->length);
        printf("actual_length = %d \n", p_t->actual_length);

        for (i=0; i < p_t->actual_length; i++){
            printf(" 0x%02x,", p_t->buffer[i]);
        }
        puts("\n\n");
    }

    fflush(stdout);
    return;
}

inline struct timespec diff(struct timespec start, struct timespec end)
{
	struct timespec temp;
	if((end.tv_nsec - start.tv_nsec) < 0) {
		temp.tv_sec  = end.tv_sec - start.tv_sec - 1;
		temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
	} else {
		temp.tv_sec  = end.tv_sec  - start.tv_sec;
		temp.tv_nsec = end.tv_nsec - start.tv_nsec;
	}
	return temp;
}

