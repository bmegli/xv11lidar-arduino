# xv11lidar-arduino

Arduino/Teensy library for non-blocking communication with XV11 lidar.

## Hardware 

Library needs:
- XV11 lidar (hardware)
- pwm motor controller 

## Purpose

- speed motor control
- non-blocking communication
- precise timestamps

## State

Functional:
- motor PID
- decoding packets
- precise timestamping on first packet byte

## Dependencies 

xv11lidar-arduino uses [Arduino-PID-Library](https://github.com/br3ttb/Arduino-PID-Library.git).

## Installation

- in Arduino IDE Sketch -> Include Library -> Manage Libraries... -> PID -> Install
- copy or clone `xv11lidar-arduino` directory to your sketchbook `libraries` directory

## Using

For real use see [cave-crawler-mcu](https://github.com/bmegli/cave-crawler-mcu/blob/master/cave-crawler-mcu.ino#L72)

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
