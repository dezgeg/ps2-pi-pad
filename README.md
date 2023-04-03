# ps2-pi-pad
This project allows plugging an arbitrary USB (or Bluetooth) controller to PlayStation 2 using a Raspberry Pi as a converter.

The protocol between the console and controller is SPI with slight modifications, with the controller acting as the slave.
That means emulating a controller requires tight and stable timings that are not possible to achieve via normal Linux application programming.
(The required timing precision are in the microseconds range wheras normal Linux programs may get scheduled out or interrupted for _milli_seconds!).

To accomplish that on a Raspberry Pi Model B, the timing sensitive part is done inside a FIQ interrupt handler hooked up to the GPIO falling edge interrupt.
Reading from the USB controller is done in a normal Linux application which stashes the controller state in a shared memory page that the FIQ handler reads from.

## /boot/cmdline.txt
By default the USB driver uses the FIQ interrupt, so that must be prevented that via `dwc_otg.fiq_enable=0 dwc_otg.fiq_fsm_enable=0 dwc_otg.speed=1`.

## /boot/config.txt
Reportedly, `disable_pvt=1` helps with having more consistent timings.
(I don't remember if I measured the effect of that).
I'm also using `force_turbo=1` for good measure.
