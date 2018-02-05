// Copyright (C) 2018 kaz Kojima
//
// This file is part of PMLPS program.  This program is free
// software; you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the
// Free Software Foundation; either version 3, or (at your option)
// any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program;
// see the files COPYING and EXCEPTION respectively.

#include <cstdio>

#include <pthread.h>

#include <netdb.h>
#include <unistd.h>
#include <stdlib.h>
#include <strings.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// TODO: should move to config*
#define TLM_ADDR "192.168.11.1"
#define TLM_PORT 5900

// for MAVLink
#include <mavlink_types.h>
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS 1
#define MAVLINK_SEND_UART_BYTES send_tcp_bytes
static void send_tcp_bytes(mavlink_channel_t chan, const uint8_t *buf, uint16_t len);
extern mavlink_system_t mavlink_system;
#include <common/mavlink.h>
#include <ardupilotmega/mavlink_msg_vision_position_delta.h>

mavlink_system_t mavlink_system = { 20, 98 };

extern bool update_pos;
extern uint64_t timestamp_pos;
extern float estimated_px, estimated_py, estimated_pz;

static int tlmfd = -1;
static void
send_tcp_bytes(mavlink_channel_t chan, const uint8_t *buf, uint16_t len)
{
  int n = write(tlmfd, buf, len);
  if (n < 0)
    {
    }
}

inline static uint64_t utimestamp(void)
{
  struct timeval tv;
  gettimeofday(&tv,NULL);
  return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

pthread_mutex_t mavmutex;

void *
mavlink_thread(void *p)
{
  // Open a TCP socket for telemetry
  if ((tlmfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
      fprintf (stderr, "client: can't open stream socket for telemetry");
      exit (1);
    }

  struct sockaddr_in serv_addr;
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = inet_addr(TLM_ADDR);
  serv_addr.sin_port = htons (TLM_PORT);

  while (connect(tlmfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
    {
      //printf("waiting connecting telemetry\n");
      sleep(2);
      continue;
    }
  printf("telemetry connected\n");
  
  int option = 1;
  ioctl(tlmfd, FIONBIO, &option);

  uint8_t target_sysid;
  uint8_t target_compid;
  bool request_sent = false;

  float prev_pos[3];
  float prev_ang[3];
  uint64_t prev_timestamp;

  float roll_angle, pitch_angle, yaw_angle;
  bool update_attitude = false;

  prev_ang[0] = prev_ang[1] = prev_ang[2] = 0.0;

  // mavlink loop
  while(1)
    {
      uint8_t c;
      int n = read(tlmfd, &c, 1);
      if (n > 0)
	{
	  mavlink_message_t msg;
	  mavlink_status_t status;
	  if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
	    {
	      switch(msg.msgid)
		{
		case MAVLINK_MSG_ID_HEARTBEAT:
		  target_sysid = msg.sysid;
		  target_compid = msg.compid;
		  if (!request_sent)
		    {
		      const uint8_t stream_id = MAV_DATA_STREAM_EXTRA1;
		      mavlink_msg_request_data_stream_send(MAVLINK_COMM_0,
							   target_sysid,
							   target_compid,
							   stream_id,
							   4,
							   1);
		      // send SET_GPS_GLOBAL_ORIGIN with fake data
		      mavlink_set_gps_global_origin_t origin;
		      origin.latitude = 37.2343 * 1E7;
		      origin.longitude = -115.8067 * 1E7;
		      origin.altitude = 61.0 * 1E3;
		      origin.target_system = target_sysid;
		      origin.time_usec = prev_timestamp = utimestamp();
		      mavlink_msg_set_gps_global_origin_send_struct(MAVLINK_COMM_0,
								    &origin);
		      printf("send SET_GPS_GLOBAL_ORIGIN\n");
		      request_sent = true;
		    }
 		  break;
		case MAVLINK_MSG_ID_ATTITUDE:
		  mavlink_attitude_t attitude;
		  mavlink_msg_attitude_decode(&msg, &attitude);
		  pthread_mutex_lock (&mavmutex);
		  roll_angle = attitude.roll;
		  pitch_angle = attitude.pitch;
		  yaw_angle = attitude.yaw;
		  update_attitude = true;
		  pthread_mutex_unlock (&mavmutex);
 		  break;
		default:
		  break;
		}
	    }
	}
      pthread_mutex_lock (&mavmutex);
      // if position was updated, send VISION_POSITION_DELTA
      if (update_pos)
	{
	  // Fill delta with updated position and current attitude
	  mavlink_vision_position_delta_t delta;
	  delta.time_usec = timestamp_pos;
	  delta.time_delta_usec = delta.time_usec - prev_timestamp;
#if 0
	  if (update_attitude)
	    {
	      delta.angle_delta[0] = roll_angle - prev_ang[0];
	      delta.angle_delta[1] = pitch_angle - prev_ang[1];
	      delta.angle_delta[2] = yaw_angle - prev_ang[2];
	      update_attitude = false;
	    }
	  else
	    {
	      delta.angle_delta[0] = 0.0;
	      delta.angle_delta[1] = 0.0;
	      delta.angle_delta[2] = 0.0;
	    }
#else
	  delta.angle_delta[0] = 0.0;
	  delta.angle_delta[1] = 0.0;
	  delta.angle_delta[2] = 0.0;
#endif
	  delta.position_delta[0] = estimated_px - prev_pos[0];
	  delta.position_delta[1] = estimated_py - prev_pos[1];
	  delta.position_delta[2] = estimated_pz - prev_pos[2];
	  delta.confidence = 90.0;
	  mavlink_msg_vision_position_delta_send_struct(MAVLINK_COMM_0, &delta);
	  prev_timestamp = delta.time_usec;
#if 0
	  prev_ang[0] = roll_angle;
	  prev_ang[1] = pitch_angle;
	  prev_ang[2] = yaw_angle;
#endif
	  prev_pos[0] = estimated_px;
	  prev_pos[1] = estimated_py;
	  prev_pos[2] = estimated_pz;
	  //printf("send VISION_POSITION_DELTA\n");
	  update_pos = false;
	}
      pthread_mutex_unlock (&mavmutex);
     }

  return 0;
}
