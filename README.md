# xv11lidar-arduino

Arduino/Teensy library for communication with XV11 lidar.

## Hardware 

Library needs:
- XV11 lidar
- pwm motor controller 

## State

Work in progress
- motor PID works
- decoding packets works

Currently distance/flags/signal_strength is not decoded (left as 4 byte int).

(library used for motor control, precise timestamping and data passed to higher controller).

## Dependencies 

xv11lidar-arduino uses modified [Arduino-PID-Library](https://github.com/br3ttb/Arduino-PID-Library/).

Modification uses float instead of double implementation of PID library.

TO DO - add modified library on github (I am using simple `#define double float`)

## Installation

Copy:
- `xv11lidar-arduino` directory to your sketchbook `libraries` directory
- float Arduino-PID-Library implementation to your sketchbook `libraries` directory

## Testing

## Using

```C++
#include <xv11lidar.h>

const int PWM_PIN = 35;
const int RPM=250;

//which serial to use (Teensy), which pin for PWM
XV11Lidar lidar(Serial5, PWM_PIN );
XV11Packet packet;

void setup()
{
  lidar.setup(RPM);
}

void loop()
{
  bool got_packet;
  
  got_packet=lidar.processAvailable(&packet);
  lidar.applyMotorPID();

  if(got_packet)
  {
    //do something with the packet
  }
}
```


## License

Library is licensed under Mozilla Public License, v. 2.0

This is similiar to LGPL but more permissive:

- you can use it as LGPL in prioprietrary software
- unlike LGPL you may compile it statically with your code

Like in LGPL, if you modify this library, you have to make your changes available. Making a github fork of the library with your changes satisfies those requirements perfectly.
