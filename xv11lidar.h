/*
 * xv11lidar-arduino library header
 *
 * Copyright 2018 (C) Bartosz Meglicki <meglickib@gmail.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. 
 *
 */

#ifndef XV11LIDAR_H
#define XV11LIDAR_H

#include "Arduino.h"

#include <stdint.h>
#include <PID_v1.h> //uses modified PID library (float instead of double implementation)


struct XV11Packet
{	
	uint32_t timestamp_us;//timestamp in microseconds
	uint8_t angle_quad;	//0-89 for readings 0-4 356-359
	uint16_t speed64;		//divide by 64 for speed in rpm	
	uint16_t distances[4]; //flags and distance or error code
	uint16_t signals[4]; //signal strengths
};

class XV11Lidar
{
	enum State {AwaitingStartByte, CollectingPacket};
	enum {PacketBytes=22};
	const float SpeedMultiplier=1.0f/64.0f;
public:
	/* Setup */
	XV11Lidar(HardwareSerial &serial, int pwm_pin);
	void setup(int motor_rpm);	
	
	/* Cyclic */
	bool processAvailable(XV11Packet *packet);
	bool applyMotorPID();
private:
	void decodePacket(const uint8_t *data, XV11Packet *packet) const;
	uint16_t checksum(const uint8_t data[PacketBytes]) const;
	inline uint16_t decode_u16(const uint8_t *data) const
	{
		return (uint16_t)data[1] << 8 | data[0];
	}
private:
	/* Hardware */
	HardwareSerial &m_serial;
	int m_pwm_pin;
	
	/* Packet & Decoding */ 
	State m_state;
	uint8_t m_packet[PacketBytes];
	int m_packet_bytes;
	uint32_t m_packet_timestamp_us;
	
	/* Motor control */
	PID m_motor_pid;
	float m_motor_actual_rpm;
	float m_motor_setpoint_rpm;
	float m_motor_pwm;
};

#endif
