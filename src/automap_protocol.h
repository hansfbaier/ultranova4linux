uint8_t automap_ok[] = {
    0xf0, 0x00, 0x01, 0xf7
};

uint8_t automap_off[] = {
    0xf0, 0x00, 0x00, 0xf7
};

uint8_t automap_button_press_in[] = {
    0xb0, 0x63, 0x3e, 0xb0, 0x62, 0x00, 0xb0, 0x06, 0x00
};

uint8_t ultranova4linux_greeting[] = {
    0xf0, 0x02, 0, 30, 'u', 'l', 't', 'r', 'a', 'n', 'o', 'v', 'a', '4', 'l', 'i', 'n', 'u', 'x', 0xf7
};

uint8_t button_octave_minus[] = { 0xb2, 0x09, 0x1 }; uint8_t led_octave_minus = 9;
uint8_t button_octave_plus [] = { 0xb2, 0x0b, 0x1 }; uint8_t led_octave_plus  = 11;
