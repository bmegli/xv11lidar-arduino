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

#include "xv11lidar.h"

enum {StartByte = 0xFA, OffsetIndex=1, OffsetSpeed=2, OffsetReadings=4,
 OffsetCRC = 20, PacketWithoutCRCBytes=20, ReadingSize=4, DistanceSize=2};

XV11Lidar::XV11Lidar(HardwareSerial &serial, int pwm_pin):
 m_serial(serial),
 m_pwm_pin(pwm_pin),
 m_state(AwaitingStartByte),
 m_packet_bytes(0),
 m_motor_pid(&m_motor_actual_rpm, &m_motor_pwm, &m_motor_setpoint_rpm, 1.0f, 0.5f, 0.0f, DIRECT	),
 m_motor_actual_rpm(250),
 m_motor_setpoint_rpm(250),
 m_motor_pwm(100) 
 
{
	m_packet[0]=StartByte;
}

void XV11Lidar::setup(int motor_rpm)
{
  pinMode(m_pwm_pin, OUTPUT);
  analogWriteFrequency(m_pwm_pin, 100);
  m_serial.begin(115200, SERIAL_8N1);
  m_motor_setpoint_rpm=motor_rpm;
  m_motor_actual_rpm=m_motor_setpoint_rpm;
  m_motor_pid.SetMode(AUTOMATIC);
  m_motor_pid.SetSampleTime(100);
}

bool XV11Lidar::processAvailable(XV11Packet *packet)
{
	int byte;

	while(m_serial.available())
	{
		byte=m_serial.read();
		
		if(m_state == AwaitingStartByte)
		{
			if(byte == StartByte)
			{
				m_state = CollectingPacket;
				m_packet_bytes=1;
				m_packet_timestamp_us=micros();
			}
			continue;
		}
	
		m_packet[m_packet_bytes++]=byte;
		
		if(m_packet_bytes == PacketBytes)
		{
			m_packet_bytes=0;
			m_state = AwaitingStartByte;

			uint16_t crc_calc = checksum(m_packet);
			uint16_t crc_rcv = decode_u16(m_packet+OffsetCRC);

			if(crc_calc == crc_rcv)
			{
				decodePacket(m_packet, packet);
				m_motor_actual_rpm=(float)packet->speed64 * SpeedMultiplier;
				packet->timestamp_us=m_packet_timestamp_us;
								
				return true;
			}
		}
	}

	return false;
}

void XV11Lidar::decodePacket(const uint8_t *data, XV11Packet *packet) const
{
	packet->angle_quad = data[OffsetIndex] - 0xA0;
	packet->speed64 = decode_u16(data + OffsetSpeed);
	for(int i=0;i<4;++i)
	{
		packet->distances[i] = decode_u16(data+OffsetReadings+i*ReadingSize);
		packet->signals[i] = decode_u16(data+OffsetReadings+i*ReadingSize+DistanceSize);
	}
}


uint16_t XV11Lidar::checksum(const uint8_t data[PacketBytes]) const
{
	uint32_t chk32=0;
	uint16_t word;
	int i;
	
	for(i=0;i<10;++i)
	{
		word=data[2*i] + (data[2*i+1] << 8);
		chk32 = (chk32 << 1) + word;
	}
	
	uint32_t checksum=(chk32 & 0x7FFF) + (chk32 >> 15);
	return checksum & 0x7FFF;
}

bool XV11Lidar::applyMotorPID()
{
	bool computed=m_motor_pid.Compute();
	if(computed)
		analogWrite(m_pwm_pin, m_motor_pwm);
	return computed;
}