ultranova4linux
===============

Open Source Drivers for the Ultranova and Mininova Synthesizers'
MIDI and control surface functionality (latter only for Ultranova).
Please note that Mininova support is completely
untested yet, since I don't own a Mininova (hardware donations welcome).

This is a userspace program for use with the
JACK audio connection kit. Before starting
the program make sure JACK is running.

Also to access the device, the permissions
on the USB device have to be right.
To do this properly, create the file
`/etc/udev/rules.d/92-novation.rules`

```bash
# Novation Ultranova
SUBSYSTEM=="usb", ATTRS{idVendor}=="1235", ATTRS{idProduct}=="0011", MODE="0664", GROUP="audio"
# Novation Mininova
SUBSYSTEM=="usb", ATTRS{idVendor}=="1235", ATTRS{idProduct}=="001e", MODE="0664", GROUP="audio"
```

And the user you intend to use this with has to be in the `audio` group.

Then just fire up
```bash
$ ./ultranova4linux
```

