/*
 * ultranova driver for linux
 */

#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <vector>
#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/assert.hpp>
#include <libusb-1.0/libusb.h>
#include <jack/jack.h>
#include <jack/midiport.h>
#include <lo/lo.h>

#include "automap_protocol.h"

#define USB_VENDOR_ID                0x1235
#define ULTRANOVA_PRODUCT_ID         0x0011
#define MININOVA_PRODUCT_ID          0x001e

#define CONTROLLER_ENDPOINT_IN       (LIBUSB_ENDPOINT_IN  | 5)
#define CONTROLLER_ENDPOINT_OUT      (LIBUSB_ENDPOINT_OUT | 5)
#define ULTRANOVA_MIDI_ENDPOINT_IN   (LIBUSB_ENDPOINT_IN  | 3)
#define ULTRANOVA_MIDI_ENDPOINT_OUT  (LIBUSB_ENDPOINT_OUT | 3)
#define MININOVA_MIDI_ENDPOINT_IN    (LIBUSB_ENDPOINT_IN  | 1)
#define MININOVA_MIDI_ENDPOINT_OUT   (LIBUSB_ENDPOINT_OUT | 2)

int midi_endpoint_in  = ULTRANOVA_MIDI_ENDPOINT_IN;
int midi_endpoint_out = ULTRANOVA_MIDI_ENDPOINT_OUT;

using namespace std;

static bool debug     = false;
static bool ultranova = true;

// controller state
char encoder_states[10];

// JACK stuff
jack_client_t *client;
jack_port_t *controller_out;
jack_port_t *controller_in;
jack_port_t *midi_out;
jack_port_t *midi_in;
jack_nframes_t nframes;

struct timespec diff(struct timespec start, struct timespec end);
struct timespec last_cycle;
struct timespec cycle_period;

struct timespec controller_in_t;
struct timespec midi_in_t;

#define SMALL_BUF_SIZE 4
#define IS_AFTERTOUCH(a) (((a) & 0xf0) == 0xd0)
#define IS_NOTE_ON(a)    (((a) & 0xf0) == 0x90)
#define IS_NOTE_OFF(a)   (((a) & 0xf0) == 0x80)
typedef struct {
    struct timespec time;
    vector<uint8_t> buffer;
} midi_message_t;

bool is(midi_message_t& msg, uint8_t *buf)
{
    for (int i = 0; i < 3; i++) {
        if (msg.buffer[i] != buf[i]) return false;
    }

    return true;
}

// USB to MIDI
boost::mutex midi_mutex;
queue<midi_message_t> midi_queue;

boost::mutex controller_mutex;
queue<midi_message_t> controller_queue;

// USB
struct libusb_device_handle *devh = NULL;
#define LEN_IN_BUFFER 32
static uint8_t in_buffer_control[LEN_IN_BUFFER];
static uint8_t in_buffer_midi[LEN_IN_BUFFER];

#define CONTROLLER_MAXLENGTH 0x18

// IN-coming transfers (IN to host PC from USB-device)
struct libusb_transfer *controller_transfer_in = NULL;
struct libusb_transfer *midi_transfer_in       = NULL;

static libusb_context *ctx = NULL;

// OSC
lo_address ardour;
uint8_t ardour_mute_states;
uint8_t ardour_recen_states;

bool do_exit = false;

// Function Prototypes:
void sighandler(int signum);
void print_libusb_transfer(struct libusb_transfer *p_t);
void cb_controller_out(struct libusb_transfer *transfer);
void cb_midi_out(struct libusb_transfer *transfer);

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

int automap_octave = 0;

void set_automap_led(uint8_t led, uint8_t value)
{
    static struct libusb_transfer *transfer = NULL;
    transfer = libusb_alloc_transfer(0);
    uint8_t buf[] = { 0xb0, led, value };

    libusb_fill_interrupt_transfer(transfer, devh, CONTROLLER_ENDPOINT_OUT,
                                   buf,
                                   sizeof(buf),
                                   cb_controller_out, NULL, 0);
    libusb_submit_transfer(transfer);
}

size_t midi_event_size(uint8_t firstByte)
{
    size_t result = 3;

    uint8_t firstNibble = firstByte & 0xf0;
    if (firstNibble == 0xc0 ||
        firstNibble == 0xd0 ||
        firstByte == 0xf3) {
        result = 2;
    }

    uint8_t secondNibble = 0x0f & firstByte;
    if (firstNibble == 0xf0 &&
        secondNibble != 0 &&
        secondNibble != 2 &&
        secondNibble != 3) {
        result = 1;
    }

    if (firstByte == 0xf0) {
        return 0;
    }

    return result;
}

inline int clamp_to(int value, int from, int to)
{
    if (value > to) {
        value = to;
    }
    if (value < from) {
        value = from;
    }
    return value;
}

void manipulate_automap(midi_message_t& msg, queue<midi_message_t>& queue)
{
    static std::map<uint8_t, uint8_t> dangling_notes;

    if (state == LISTEN) {
        uint8_t orig_note = msg.buffer[1];
        uint8_t mangled_note = clamp_to((int)(orig_note + automap_octave * 12), 0, 127);

        if (IS_NOTE_ON(msg.buffer[0])) {
            dangling_notes[orig_note] = mangled_note;
            msg.buffer[1] = mangled_note;
        } else if (IS_NOTE_OFF(msg.buffer[0])) {
            std::map<uint8_t, uint8_t>::iterator it = dangling_notes.find(orig_note);
            if (it != dangling_notes.end()) {
                msg.buffer[1] = it->second;
                dangling_notes.erase(it);
            }
        } else if (&queue == &controller_queue &&
                   msg.buffer[0] == 0xb0 &&
                   msg.buffer[1] >= 0    &&
                   msg.buffer[1] <= 9) {
            // 8 rotary touch encoders
            // add 0x10 so that the second does not conflict
            // with modwheel
            msg.buffer[1] += 0x10;
        }
    }
}


#define AUTOMAP_ENCODERS 0xb0
#define AUTOMAP_BUTTONS 0xb2
void process_controller_out_message(midi_message_t& msg)
{
    if (msg.buffer[0] == AUTOMAP_ENCODERS && msg.buffer[1] >= 0x10 && msg.buffer[1] <= 0x19) {
        int encoder_number = msg.buffer[1] - 0x10;
        int value = msg.buffer[2];
        if (64 <= value && value <= 127) {
            value = value - 128;
        }
        encoder_states[encoder_number] = clamp_to((int)encoder_states[encoder_number] + value, 0, 127);
        msg.buffer[2] = encoder_states[encoder_number];

        if (ardour && encoder_number <= 8) {
            int target_id = encoder_number == 8 ? 318 : encoder_number + 1;
            lo_send(ardour, "/ardour/routes/gainabs", "if", target_id, 2.0 * ((float)msg.buffer[2])/127.0);
        }
    }

    if (msg.buffer[0] == AUTOMAP_BUTTONS) {
        msg.buffer[2] = msg.buffer[2] ? 127 : 0;
        uint8_t value = msg.buffer[2];
        uint8_t button = msg.buffer[1];

        if (ardour) {
            if (button <= 7 && value) {
               if (value) {
                 ardour_mute_states ^= 1 << button;
                 lo_send(ardour, "/ardour/routes/mute", "ii", button + 1, (ardour_mute_states & (1 << button)) ? 1 : 0);
               }
            }
            button == 0x1d && lo_send(ardour, "/ardour/transport_stop", "");
            button == 0x1e && lo_send(ardour, "/ardour/transport_play", "");

            if (value) {
                button == 0x20 && lo_send(ardour, "/ardour/loop_toggle", "");
                button == 0x22 && lo_send(ardour, "/ardour/rec_enable_toggle", "");
                if (button == 0x13) {
                    ardour_recen_states ^= 1 << 0;
                    lo_send(ardour, "/ardour/routes/recenable", "ii", 1, (ardour_recen_states & (1 << 0)) ? 1 : 0);
                }
                if (button == 0x15) {
                    ardour_recen_states ^= 1 << 1;
                    lo_send(ardour, "/ardour/routes/recenable", "ii", 2, (ardour_recen_states & (1 << 1)) ? 1 : 0);
                }
                if (button == 0x17) {
                    ardour_recen_states ^= 1 << 2;
                    lo_send(ardour, "/ardour/routes/recenable", "ii", 3, (ardour_recen_states & (1 << 2)) ? 1 : 0);
                }
                if (button == 0x19) {
                    ardour_recen_states ^= 1 << 3;
                    lo_send(ardour, "/ardour/routes/recenable", "ii", 4, (ardour_recen_states & (1 << 3)) ? 1 : 0);
                }
                if (button == 0x1a) {
                    ardour_recen_states ^= 1 << 4;
                    lo_send(ardour, "/ardour/routes/recenable", "ii", 5, (ardour_recen_states & (1 << 4)) ? 1 : 0);
                }
                if (button == 0x1c) {
                    ardour_recen_states ^= 1 << 5;
                    lo_send(ardour, "/ardour/routes/recenable", "ii", 6, (ardour_recen_states & (1 << 5)) ? 1 : 0);
                }
                if (button == 0x1f) {
                    ardour_recen_states ^= 1 << 6;
                    lo_send(ardour, "/ardour/routes/recenable", "ii", 7, (ardour_recen_states & (1 << 6)) ? 1 : 0);
                }
                if (button == 0x21) {
                    ardour_recen_states ^= 1 << 7;
                    lo_send(ardour, "/ardour/routes/recenable", "ii", 8, (ardour_recen_states & (1 << 7)) ? 1 : 0);
                }
            }
        }
    }
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
        if (framepos <= last_framepos) {
            framepos = last_framepos + 1;
        }

        if (framepos >= nframes) {
            framepos = nframes - 1;
        }

        if (state == LISTEN && &queue == &controller_queue) {
            process_controller_out_message(msg);
        }

        uint8_t *buffer = jack_midi_event_reserve(jack_midi_buffer, framepos, msg.buffer.size());
        if (buffer) {
            memcpy(buffer, msg.buffer.data(), msg.buffer.size());
        } else {
            fprintf(stderr, "failed to allocate %d bytes midi buffer at framepos %ld (nframes = %d)",
                    msg.buffer.size(), framepos, nframes);
        }

        queue.pop();
    }
}

void jack_to_usb(void *jack_midi_buffer, jack_port_t *jack_port, int endpoint, libusb_transfer_cb_fn callback)
{
    jack_midi_event_t in_event;
    jack_nframes_t event_index = 0;
    jack_nframes_t event_count = jack_midi_get_event_count(jack_midi_buffer);

    for (event_index = 0; event_index < event_count; event_index++) {
        jack_midi_event_get(&in_event, jack_midi_buffer, event_index);

        uint8_t* outbuf = (uint8_t*) malloc(in_event.size);
        memcpy(outbuf, in_event.buffer, in_event.size);
        struct libusb_transfer *transfer = libusb_alloc_transfer(0);
        libusb_fill_interrupt_transfer(transfer, devh, endpoint,
                                       outbuf, in_event.size,
                                       callback, outbuf, 0);
        libusb_submit_transfer(transfer);
    }
}

int process(jack_nframes_t nframes, void *arg)
{
    struct timespec prev_cycle = last_cycle;
    clock_gettime(CLOCK_REALTIME, &last_cycle);
    cycle_period = diff(prev_cycle, last_cycle);
    if (cycle_period.tv_nsec <= 0) {
        return 0;
    }

    int i;

    void* controller_buf_out_jack;
    if (ultranova) {
        controller_buf_out_jack = jack_port_get_buffer(controller_out, nframes);
        jack_midi_clear_buffer(controller_buf_out_jack);
    }

    void* midi_buf_out_jack = jack_port_get_buffer(midi_out, nframes);
    jack_midi_clear_buffer(midi_buf_out_jack);

    if (ultranova) {
        void* controller_buf_in_jack = jack_port_get_buffer(controller_in, nframes);
        jack_to_usb(controller_buf_in_jack, controller_in, CONTROLLER_ENDPOINT_OUT, cb_controller_out);
    }

    void* midi_buf_in_jack = jack_port_get_buffer(midi_in, nframes);
    jack_to_usb(midi_buf_in_jack, midi_in, midi_endpoint_out, cb_midi_out);

    if (ultranova) {
        controller_mutex.lock();
        pickup_from_queue(controller_queue, controller_buf_out_jack, prev_cycle, cycle_period, nframes);
        controller_mutex.unlock();
    }

    midi_mutex.lock();
    pickup_from_queue(midi_queue, midi_buf_out_jack, prev_cycle, cycle_period, nframes);
    midi_mutex.unlock();

    return 0;
}


bool buffer_equal(uint8_t *expected, uint8_t *actual, int length)
{
    int i;

    for (i = 0; i < length; i++) {
        if (expected[i] != actual[i]) {
            return false;
        }
    }

    return true;
}

void process_incoming(struct libusb_transfer *transfer, struct timespec time, midi_message_t& msg, queue<midi_message_t>& queue)
{
    int transfer_size = transfer->actual_length;
    // byte position inside the incoming transfer buffer
    int input_pos = 0;

    while(input_pos < transfer_size) {
        int event_size = 0;
        if (msg.buffer.empty()) {
            event_size = midi_event_size(transfer->buffer[input_pos]);
        } else {
            event_size = midi_event_size(msg.buffer[0]);
        }

        if (event_size > 0 && event_size <= msg.buffer.size()) {
                fprintf(stderr, "ERROR: already complete message contained, but not submitted, event_size: %d, message buffer size: %d\n", event_size, msg.buffer.size());
                fprintf(stderr, "message buffer: \n");
                for (int i=0; i < msg.buffer.size(); i++){
                    fprintf(stderr, " 0x%02x,", msg.buffer[i]);
                }
                fputs("\n", stderr);
                print_libusb_transfer(transfer);
        }

        if (event_size > 0) {
            // how many bytes we still need to get in order
            // for the current midi message to be complete
            int remaining_size = event_size - msg.buffer.size();

            if (remaining_size == 0) {
                // complete event, submit the message
                msg.time = time;
                manipulate_automap(msg, queue);
                queue.push(msg);
                msg.buffer.clear();
            } else  if (input_pos + remaining_size > transfer_size) {
                // in this case we received some more bytes for the
                // current message, but the message is not complete yet
                // so then append the incoming bytes to the message
                int i = 0;
                for (i = input_pos; i < transfer_size; i++) {
                    msg.buffer.push_back(transfer->buffer[i]);
                }
                input_pos = i;
            } else if (0 <= remaining_size && input_pos + remaining_size <= transfer_size) {
                // in this case we have received a complete event,
                // so copy the data over to the message buffer
                int i = 0;
                for (i = input_pos; i < input_pos + remaining_size; i++) {
                    msg.buffer.push_back(transfer->buffer[i]);
                }
                input_pos = i;
                BOOST_ASSERT(event_size == msg.buffer.size());
                msg.time = time;
                manipulate_automap(msg, queue);
                // and submit the message
                queue.push(msg);
                msg.buffer.clear();
                // and continue to read the next message from the remaining
                // input transfer bytes
            } else {
                fprintf(stderr, "ERROR, invalid remaining size %d (input_pos: %d, event_size: %d, message buffer size: %d)\n", remaining_size, input_pos, (int)event_size, msg.buffer.size());
                fprintf(stderr, "message buffer: \n");
                for (int i=0; i < msg.buffer.size(); i++){
                    fprintf(stderr, " 0x%02x,", msg.buffer[i]);
                }
                fputs("\n", stderr);
                print_libusb_transfer(transfer);
                msg.buffer.clear();
            }
        } else {
            // sysex
            int i = 0;
            for (i = input_pos; i < transfer_size; i++) {
                if (transfer->buffer[i] != 0xf7) {
                    msg.buffer.push_back(transfer->buffer[i]);
                } else {
                    msg.buffer.push_back(0xf7);
                    msg.time = time;
                    queue.push(msg);
                    msg.buffer.clear();
                    // account for last byte
                    i++;
                    // message complete, break out of for loop
                    break;
                }
            }
            input_pos = i;
        }
    }
}

void cb_controller_out(struct libusb_transfer *transfer)
{
    if (debug) {
        fprintf(stderr, "cb_controller_out: ");
        print_libusb_transfer(transfer);
    }
    libusb_free_transfer(transfer);
}

void cb_midi_out(struct libusb_transfer *transfer)
{
    if (debug) {
        fprintf(stderr, "cb_midi_out: ");
        print_libusb_transfer(transfer);
    }
    libusb_free_transfer(transfer);
}

void cb_controller_in(struct libusb_transfer *transfer)
{
    clock_gettime(CLOCK_REALTIME, &controller_in_t);

    if (debug) {
        fprintf(stderr, "cb_controller_in: ");
        print_libusb_transfer(transfer);
    }

    static midi_message_t msg;

    if (transfer->actual_length == sizeof(automap_button_press_in) &&
       buffer_equal(automap_ok, transfer->buffer, sizeof(automap_button_press_in))) {
        state = AUTOMAP_PRESSED;
        fprintf(stderr, "AUTOMAP PRESSED\n");
    }

    switch(state) {
    case STARTUP:
        if (transfer->actual_length == sizeof(automap_ok) &&
           buffer_equal(automap_ok, transfer->buffer, sizeof(automap_ok))) {
            state = LISTEN;
        } else if (transfer->actual_length == sizeof(automap_off) &&
                  buffer_equal(automap_off, transfer->buffer, sizeof(automap_off))) {
            state = WAIT_FOR_AUTOMAP;
        } else {
            fprintf(stderr, "state STARTUP, got unexpected reply\n");
            fflush(stderr);
        }
        break;

    case WAIT_FOR_AUTOMAP:
        if (transfer->actual_length == sizeof(automap_ok) &&
           buffer_equal(automap_ok, transfer->buffer, sizeof(automap_ok))) {
            state = LISTEN;

            struct libusb_transfer *transfer = libusb_alloc_transfer(0);
            libusb_fill_interrupt_transfer(transfer, devh, CONTROLLER_ENDPOINT_OUT,
                                           automap_ok, sizeof(automap_ok),
                                           cb_controller_out, NULL, 0);
            libusb_submit_transfer(transfer);

            transfer = libusb_alloc_transfer(0);
            libusb_fill_interrupt_transfer(transfer, devh, CONTROLLER_ENDPOINT_OUT,
                                           ultranova4linux_greeting,
                                           sizeof(ultranova4linux_greeting),
                                           cb_controller_out, NULL, 0);
            libusb_submit_transfer(transfer);
        }
        break;

    case AUTOMAP_PRESSED:
        state = LISTEN;
        break;

    case LISTEN:
        if (transfer->actual_length == sizeof(automap_off) &&
           buffer_equal(automap_off, transfer->buffer, sizeof(automap_off))) {
            state = WAIT_FOR_AUTOMAP;
        } else {
            controller_mutex.lock();
            process_incoming(transfer, controller_in_t, msg, controller_queue);
            if (is(msg, button_octave_minus)) automap_octave -= 1;
            if (is(msg, button_octave_plus))  automap_octave += 1;
            automap_octave = clamp_to(automap_octave, -4, +4);
            if (automap_octave  > 0)   set_automap_led(led_octave_plus, 1);
            if (automap_octave == 0) { set_automap_led(led_octave_plus, 0); set_automap_led(led_octave_minus, 0); }
            if (automap_octave  < 0)   set_automap_led(led_octave_minus, 1);
            controller_mutex.unlock();
        }
        break;

    default:
        break;
    }

    if (msg.buffer.size()) {
        fprintf(stderr, "pending controller message size: %d\n\n", msg.buffer.size());
    }

    libusb_submit_transfer(controller_transfer_in);
}

volatile bool aftertouch_seen = false;

void cb_midi_in(struct libusb_transfer *transfer)
{
    clock_gettime(CLOCK_REALTIME, &midi_in_t);

    if (debug) {
        fprintf(stderr, "cb_midi_in: ");
        print_libusb_transfer(transfer);
    }

    static midi_message_t msg;

    midi_mutex.lock();
    process_incoming(transfer, midi_in_t, msg, midi_queue);
    midi_mutex.unlock();

    if (msg.buffer.size() && debug) {
        fprintf(stderr, "pending midi message size: %d\n\n", msg.buffer.size());
    }

    libusb_submit_transfer(midi_transfer_in);
}

int main(int argc, char *argv[])
{
    bool control_ardour = false;

    for (int i = 0; i < argc; i++){
        if (strcmp(argv[i], "--debug") == 0) {
            debug = true;
        } else if (strcmp(argv[i], "--ardour-osc") == 0) {
            control_ardour = true;
        }
    }

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
    devh = libusb_open_device_with_vid_pid(ctx, USB_VENDOR_ID, ULTRANOVA_PRODUCT_ID);

    if (!devh) {
        devh = libusb_open_device_with_vid_pid(ctx, USB_VENDOR_ID, MININOVA_PRODUCT_ID);
        if (!devh) {
            perror("neither Novation Ultranova nor Novation Mininova found");
            return 1;
        }

        // we have a mininova
        ultranova = false;
        midi_endpoint_in  = MININOVA_MIDI_ENDPOINT_IN;
        midi_endpoint_out = MININOVA_MIDI_ENDPOINT_OUT;
    }

    //claim the interface
    bool success = ultranova ?
       (libusb_claim_interface(devh, 0) >= 0 &&
        libusb_claim_interface(devh, 1) >= 0 &&
        libusb_claim_interface(devh, 3) >= 0)
       :
       (libusb_claim_interface(devh, 0) >= 0);

    if (!success) {
        fprintf(stderr, "usb_claim_interface error\n");
        exitflag = OUT;
        do_exit = true;
    } else  {
        fprintf(stderr, "Claimed interface\n");

        // init OSC
        if (ultranova && control_ardour) {
            ardour = lo_address_new_from_url("osc.udp://localhost:3819/");
        }

        // init jack
        fprintf(stderr, "initializing jack\n");
        if ((client = jack_client_open (ultranova ? "ultranova" : "mininova", JackNullOption, NULL)) == 0) {
            fprintf (stderr, "jack server not running?\n");
            do_exit = true;
        }

        jack_set_process_callback (client, process, 0);

        if (ultranova) {
            controller_out = jack_port_register (client, "controller_out", JACK_DEFAULT_MIDI_TYPE, JackPortIsOutput, 0);
            controller_in  = jack_port_register (client, "controller_in",  JACK_DEFAULT_MIDI_TYPE, JackPortIsInput,  0);
        }

        midi_out    = jack_port_register (client, "midi_out",    JACK_DEFAULT_MIDI_TYPE, JackPortIsOutput, 0);
        midi_in     = jack_port_register (client, "midi_in",     JACK_DEFAULT_MIDI_TYPE, JackPortIsInput,  0);

        nframes = jack_get_buffer_size(client);
        if (jack_activate(client)) {
            fprintf (stderr, "cannot activate client");
            do_exit = true;
        }

        // allocate transfers
        if (ultranova) {
            controller_transfer_in = libusb_alloc_transfer(0);
        }
        midi_transfer_in = libusb_alloc_transfer(0);

        if (ultranova) {
            libusb_fill_interrupt_transfer(controller_transfer_in, devh, CONTROLLER_ENDPOINT_IN,
                                           in_buffer_control, CONTROLLER_MAXLENGTH,
                                           cb_controller_in, NULL, 0);
        }
        libusb_fill_interrupt_transfer(midi_transfer_in, devh, midi_endpoint_in,
                                       in_buffer_midi, LEN_IN_BUFFER,
                                       cb_midi_in, NULL, 0);

        //submit the transfer, all following transfers are initiated from the CB
        if (ultranova) {
            libusb_submit_transfer(controller_transfer_in);
        }
        libusb_submit_transfer(midi_transfer_in);

        if (ultranova) {
            struct libusb_transfer *transfer = libusb_alloc_transfer(0);
            libusb_fill_interrupt_transfer(transfer, devh, CONTROLLER_ENDPOINT_OUT,
                                           automap_ok, sizeof(automap_ok),
                                           cb_controller_out, NULL, 0);
            libusb_submit_transfer(transfer);
        }

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
    if (1) {
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
        jack_client_close(client);

    case OUT_RELEASE:
        libusb_release_interface(devh, 0);
        if (ultranova) {
            libusb_release_interface(devh, 1);
            libusb_release_interface(devh, 3);
        }

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
inline void print_libusb_transfer(struct libusb_transfer *p_t)
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
	if ((end.tv_nsec - start.tv_nsec) < 0) {
		temp.tv_sec  = end.tv_sec - start.tv_sec - 1;
		temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
	} else {
		temp.tv_sec  = end.tv_sec  - start.tv_sec;
		temp.tv_nsec = end.tv_nsec - start.tv_nsec;
	}
	return temp;
}

