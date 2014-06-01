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

// JACK stuff
jack_client_t *client;
jack_port_t *output_port;
jack_nframes_t nframes;

int process(jack_nframes_t nframes, void *arg)
{
    int i;
    void* port_buf = jack_port_get_buffer(output_port, nframes);
    jack_midi_clear_buffer(port_buf);

    unsigned char* buffer;

    for(i=0; i<nframes; i++)
	{
            /*
            buffer = jack_midi_event_reserve(port_buf, i, 3);
            buffer[2] = 64;		// velocity 
            buffer[1] = 0;              //pitch;
            buffer[0] = 0x90;	        // note on 
            */
	}
    return 0;
}

//USB STUFF
struct libusb_device_handle *devh = NULL;
#define LEN_IN_BUFFER 1024*8
static uint8_t in_buffer[LEN_IN_BUFFER];

#define CONTROLLER_MAXLENGTH 0x18

// OUT-going transfers (OUT from host PC to USB-device)
struct libusb_transfer *transfer_out = NULL;

// IN-coming transfers (IN to host PC from USB-device)
struct libusb_transfer *transfer_in = NULL;

static libusb_context *ctx = NULL;

bool do_exit = false;

// Function Prototypes:
void sighandler(int signum);
void print_libusb_transfer(struct libusb_transfer *p_t);


struct timespec t1, t2;

enum {
    OUT_DEINIT,
    OUT_RELEASE,
    OUT
} exitflag;

// state machine
enum {
    STARTUP,
    STARTUP1,
    STARTUP2,
    AUTOMAP_PRESSED,
    LISTEN,
} state = STARTUP;

char *state_names[] = {
    "STARTUP",
    "STARTUP1",
    "STARTUP2",
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
void cb_out(struct libusb_transfer *transfer)
{
    fprintf(stderr, "cb_out: ");
    print_libusb_transfer(transfer);

    if(state == STARTUP1) {
            libusb_fill_interrupt_transfer(transfer_out, devh, CONTROLLER_ENDPOINT_OUT,
                                           automap_server_start_2_out, sizeof(automap_server_start_2_out), cb_out, NULL, 0);
            libusb_submit_transfer(transfer_out);
            state = LISTEN;
    }
}

// In Callback
//   - This is called after the command for version is processed.
//     That is, the data for in_buffer IS AVAILABLE.
void cb_in(struct libusb_transfer *transfer)
{
    //measure the time
    clock_gettime(CLOCK_REALTIME, &t2);
    fprintf(stderr, "cb_in: ");
  
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
        } else {
            fprintf(stderr, "state STARTUP, got unexpected reply\n");
            fflush(stderr);
        }
        break;

    case AUTOMAP_PRESSED:
        state = LISTEN;
        break;

    case LISTEN:
        break;

    default:
        break;
    }

    libusb_submit_transfer(transfer_in);
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
         output_port = jack_port_register (client, "out", JACK_DEFAULT_MIDI_TYPE, JackPortIsOutput, 0);
         nframes = jack_get_buffer_size(client);
         if (jack_activate(client)) {
             fprintf (stderr, "cannot activate client");
             do_exit = true;
         }

         // allocate transfers
         transfer_in = libusb_alloc_transfer(0);
         transfer_out = libusb_alloc_transfer(0);

         libusb_fill_interrupt_transfer(transfer_in, devh, CONTROLLER_ENDPOINT_IN,
                                        in_buffer, CONTROLLER_MAXLENGTH,  // Note: in_buffer is where input data written.
                                        cb_in, NULL, 0); // no user data

         //take the initial time measurement
         clock_gettime(CLOCK_REALTIME, &t1);
         //submit the transfer, all following transfers are initiated from the CB
         r = libusb_submit_transfer(transfer_in);

         libusb_fill_interrupt_transfer(transfer_out, devh, CONTROLLER_ENDPOINT_OUT,
                                        automap_ok, sizeof(automap_ok),
                                        cb_out, NULL, 0);
         libusb_submit_transfer(transfer_out);

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
     if (transfer_out) {
         r = libusb_cancel_transfer(transfer_out);
         if (0 == r) {
             printf("transfer_out successfully cancelled\n");
         }
         if (r < 0) {
             exitflag = OUT_DEINIT;
         }

     }
     if (transfer_in) {
         r = libusb_cancel_transfer(transfer_in);
         if (0 == r) {
             printf("transfer_in successfully cancelled\n");
         }
         if (r < 0) {
             exitflag = OUT_DEINIT;
         }
     }

     switch(exitflag) {
     case OUT_DEINIT:
         printf("at OUT_DEINIT\n");
         libusb_free_transfer(transfer_out);
         libusb_free_transfer(transfer_in);

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

