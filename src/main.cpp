/*
 * ultranova driver for linux
 */

#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <vector>
#include <queue>
#include <glib.h>
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

using namespace std;

// JACK stuff
jack_client_t *client;
jack_port_t *controller_out;
jack_port_t *controller_in;
jack_port_t *midi_out;
jack_nframes_t nframes;

struct timespec diff(struct timespec start, struct timespec end);
struct timespec last_cycle;
struct timespec cycle_period;

struct timespec controller_in_t;
struct timespec midi_in_t;

#define SMALL_BUF_SIZE 4
typedef struct {
    struct timespec time;
    vector<uint8_t> buffer;
} midi_message_t;


// USB to MIDI
G_LOCK_DEFINE(midi_queue);
queue<midi_message_t> midi_queue;

G_LOCK_DEFINE(controller_queue);
queue<midi_message_t> controller_queue;

// USB
struct libusb_device_handle *devh = NULL;
#define LEN_IN_BUFFER 32
static uint8_t in_buffer_control[LEN_IN_BUFFER];
static uint8_t in_buffer_midi[LEN_IN_BUFFER];

#define CONTROLLER_MAXLENGTH 0x18

// OUT-going transfers (OUT from host PC to USB-device)
struct libusb_transfer *midi_transfer_out    = NULL;

// IN-coming transfers (IN to host PC from USB-device)
struct libusb_transfer *controller_transfer_in = NULL;
struct libusb_transfer *midi_transfer_in    = NULL;

static libusb_context *ctx = NULL;

bool do_exit = false;

// Function Prototypes:
void sighandler(int signum);
void print_libusb_transfer(struct libusb_transfer *p_t);
void cb_controller_out(struct libusb_transfer *transfer);

enum exitflag_t {
    OUT_DEINIT,
    OUT_RELEASE,
    OUT
} exitflag;

// state machine
enum state_t {
    STARTUP,
    WAIT_FOR_AUTOMAP,
    AUTOMAP_PRESSED,
    LISTEN,
} state = STARTUP;

const char* state_names[] = {
    "STARTUP",
    "WAIT_FOR_AUTOMAP",
    "AUTOMAP_PRESSED",
    "LISTEN",
};

size_t midi_event_size(uint8_t firstByte)
{
    size_t result = 3;

    uint8_t firstNibble = firstByte & 0xf0;
    if(firstNibble == 0xc0 ||
       firstNibble == 0xd0 ||
       firstByte == 0xf3) {
        result = 2;
    }

    uint8_t secondNibble = 0x0f & firstByte;
    if(firstNibble == 0xf0 &&
       secondNibble != 0 &&
       secondNibble != 2 &&
       secondNibble != 3) {
        result = 1;
    }

    if(firstByte == 0xf0) {
        return 0;
    }

    return result;
}

void pickup_from_queue(queue<midi_message_t>& queue,
                       void *jack_midi_buffer,
                       struct timespec& prev_cycle,
                       struct timespec& cycle_period,
                       jack_nframes_t nframes
                       )
{
    jack_nframes_t last_framepos = 0;

    while(!queue.empty()) {
        midi_message_t msg = queue.front();
        long nsec_since_start = diff(prev_cycle, msg.time).tv_nsec;
        long framepos = (nsec_since_start * nframes) / cycle_period.tv_nsec;
        if(framepos <= last_framepos) {
            framepos = last_framepos + 1;
        }

        if(framepos >= nframes) {
            framepos = nframes - 1;
        }

        uint8_t *buffer = jack_midi_event_reserve(jack_midi_buffer, framepos, msg.buffer.size());
        if(buffer) {
            memcpy(buffer, msg.buffer.data(), msg.buffer.size());
        } else {
            fprintf(stderr, "failed to allocate %d bytes midi buffer at framepos %ld (nframes = %d)",
                    msg.buffer.size(), framepos, nframes);
        }

        queue.pop();
    }
}

int process(jack_nframes_t nframes, void *arg)
{
    struct timespec prev_cycle = last_cycle;
    clock_gettime(CLOCK_REALTIME, &last_cycle);
    cycle_period = diff(prev_cycle, last_cycle);
    if(cycle_period.tv_nsec <= 0) {
        return 0;
    }

    int i;
    void* controller_buf_out_jack = jack_port_get_buffer(controller_out, nframes);
    jack_midi_clear_buffer(controller_buf_out_jack);

    void* midi_buf_out_jack = jack_port_get_buffer(midi_out, nframes);
    jack_midi_clear_buffer(midi_buf_out_jack);

    void* controller_buf_in_jack = jack_port_get_buffer(controller_in, nframes);
 
    jack_midi_event_t in_event;
    jack_nframes_t event_index = 0;
    jack_nframes_t event_count = jack_midi_get_event_count(controller_buf_in_jack);

    for(event_index = 0; event_index < event_count; event_index++) {
        jack_midi_event_get(&in_event, controller_buf_in_jack, event_index);

        uint8_t* outbuf = (uint8_t*) malloc(in_event.size);
        memcpy(outbuf, in_event.buffer, in_event.size);
        struct libusb_transfer *transfer = libusb_alloc_transfer(0);
        libusb_fill_interrupt_transfer(transfer, devh, CONTROLLER_ENDPOINT_OUT,
                                       outbuf, in_event.size,
                                       cb_controller_out, outbuf, 0);
        libusb_submit_transfer(transfer);
    }

    G_LOCK(controller_queue);
    pickup_from_queue(controller_queue, controller_buf_out_jack, prev_cycle, cycle_period, nframes);
    G_UNLOCK(controller_queue);

    G_LOCK(midi_queue);
    pickup_from_queue(midi_queue, midi_buf_out_jack, prev_cycle, cycle_period, nframes);
    G_UNLOCK(midi_queue);

    return 0;
}


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

void process_incoming(struct libusb_transfer *transfer, struct timespec time, midi_message_t& msg, queue<midi_message_t>& queue)
{
    int transfer_size = transfer->actual_length;
    int pos = 0;

    while(pos < transfer_size) {
        int event_size = 0;
        if(msg.buffer.empty()) {
            event_size = midi_event_size(transfer->buffer[pos]);
        } else {
            event_size = midi_event_size(msg.buffer[0]);
        }

        if(event_size) {
            int i;
            int remaining_size = event_size - msg.buffer.size();

            if(pos + remaining_size > transfer_size) {
                int i;
                for(i = pos; i < transfer_size; i++) {
                    msg.buffer.push_back(transfer->buffer[i]);
                }
                pos = i;
                // we're done
                break;
            } else if (0 < remaining_size && pos + remaining_size <= transfer_size) {
                for(i = pos; i < pos + remaining_size; i++) {
                    msg.buffer.push_back(transfer->buffer[i]);
                }
                pos = i;
                g_assert(event_size == msg.buffer.size());
                msg.time = time;
                queue.push(msg);
                msg.buffer.clear();
                continue;
            } else {
                fprintf(stderr, "ERROR, invalid remaining size\n");
            }
        } else {
            // sysex
            int i;
            for(i = pos; i < transfer_size; i++) {
                if(transfer->buffer[i] != 0xf7) {
                    msg.buffer.push_back(transfer->buffer[i]);
                } else {
                    msg.buffer.push_back(0xf7);
                    msg.time = time;
                    queue.push(msg);
                    msg.buffer.clear();
                    pos = i;
                    continue;
                }
            }
            pos = i;
        }
    }
}

// Out Callback
//   - This is called after the Out transfer has been received by libusb
void cb_controller_out(struct libusb_transfer *transfer)
{
    fprintf(stderr, "cb_controller_out: ");
    print_libusb_transfer(transfer);
    free(transfer->user_data);
    libusb_free_transfer(transfer);
}

void cb_controller_in(struct libusb_transfer *transfer)
{
    clock_gettime(CLOCK_REALTIME, &controller_in_t);
    fprintf(stderr, "cb_controller_in: ");
    print_libusb_transfer(transfer);

    static midi_message_t msg;
    if(msg.buffer.size()) {
        fprintf(stderr, "pending controller message size: %d\n", msg.buffer.size());
    }

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
            state = LISTEN;

            struct libusb_transfer *transfer = libusb_alloc_transfer(0);
            libusb_fill_interrupt_transfer(transfer, devh, CONTROLLER_ENDPOINT_OUT,
                                           automap_ok, sizeof(automap_ok),
                                           cb_controller_out, NULL, 0);
            libusb_submit_transfer(transfer);
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
            G_LOCK(controller_queue);
            process_incoming(transfer, controller_in_t, msg, controller_queue);
            G_UNLOCK(controller_queue);
        }
        break;

    default:
        break;
    }

    libusb_submit_transfer(controller_transfer_in);
}

void cb_midi_out(struct libusb_transfer *transfer)
{
    fprintf(stderr, "cb_midi_out: ");
    print_libusb_transfer(transfer);
}

volatile bool aftertouch_seen = false;

#define IS_AFTERTOUCH(a) (((a) & 0xf0) == 0xd0)
void cb_midi_in(struct libusb_transfer *transfer)
{
    clock_gettime(CLOCK_REALTIME, &midi_in_t);
    fprintf(stderr, "cb_midi_in: ");
    print_libusb_transfer(transfer);

    static midi_message_t msg;
    if(msg.buffer.size()) {
        fprintf(stderr, "pending midi message size: %d\n", msg.buffer.size());
    }

    G_LOCK(midi_queue);
    process_incoming(transfer, midi_in_t, msg, midi_queue);
    G_UNLOCK(midi_queue);

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
         controller_out = jack_port_register (client, "controller_out", JACK_DEFAULT_MIDI_TYPE, JackPortIsOutput, 0);
         controller_in  = jack_port_register (client, "controller_in",  JACK_DEFAULT_MIDI_TYPE, JackPortIsInput,  0);
         midi_out    = jack_port_register (client, "midi_out",    JACK_DEFAULT_MIDI_TYPE, JackPortIsOutput, 0);

         nframes = jack_get_buffer_size(client);
         if (jack_activate(client)) {
             fprintf (stderr, "cannot activate client");
             do_exit = true;
         }

         // allocate transfers
         controller_transfer_in  = libusb_alloc_transfer(0);
         midi_transfer_in        = libusb_alloc_transfer(0);
         midi_transfer_out       = libusb_alloc_transfer(0);

         libusb_fill_interrupt_transfer(controller_transfer_in, devh, CONTROLLER_ENDPOINT_IN,
                                        in_buffer_control, CONTROLLER_MAXLENGTH,
                                        cb_controller_in, NULL, 0);
         libusb_fill_interrupt_transfer(midi_transfer_in, devh, MIDI_ENDPOINT_IN,
                                        in_buffer_midi, CONTROLLER_MAXLENGTH,
                                        cb_midi_in, NULL, 0);

         //submit the transfer, all following transfers are initiated from the CB
         libusb_submit_transfer(controller_transfer_in);
         libusb_submit_transfer(midi_transfer_in);

         struct libusb_transfer *transfer = libusb_alloc_transfer(0);
         libusb_fill_interrupt_transfer(transfer, devh, CONTROLLER_ENDPOINT_OUT,
                                        automap_ok, sizeof(automap_ok),
                                        cb_controller_out, NULL, 0);
         libusb_submit_transfer(transfer);

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

     switch(exitflag) {
     case OUT_DEINIT:
         printf("at OUT_DEINIT\n");
         libusb_free_transfer(controller_transfer_in);
         libusb_free_transfer(midi_transfer_in);
         libusb_free_transfer(midi_transfer_out);
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

