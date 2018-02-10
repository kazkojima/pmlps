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
#include <errno.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "config.h"

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
extern float estimated_yaw;

// MAVLink attitude
float roll_angle, pitch_angle, yaw_angle;
bool update_attitude = false;

static int tlmfd = -1;
static bool connected = false;

static void
send_tcp_bytes(mavlink_channel_t chan, const uint8_t *buf, uint16_t len)
{
  int n = write(tlmfd, buf, len);
  if (n < 0)
    {
      if (errno == ECONNRESET)
	{
	  connected = false;
	  close(tlmfd);
	  //printf("Disconnected from telemetry\n");
	  if ((tlmfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	    {
	      fprintf (stderr, "can't open stream socket for telemetry");
	      exit (1);
	    }
	}
    }
}

inline static uint64_t utimestamp(void)
{
  struct timeval tv;
  gettimeofday(&tv,NULL);
  return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

inline static void
frame_delta(float a, float b, float alpha, float& x, float&y)
{
  x = cosf(alpha)*a - sinf(alpha)*b;
  y = -(sinf(alpha)*a + cosf(alpha)*b);
}

inline static float
angle_mod(float alpha)
{
  if (alpha < -M_PI)
    alpha += 2*M_PI;
  else if (alpha > M_PI)
    alpha -= 2*M_PI;
  return alpha;
}

pthread_mutex_t mavmutex;

void *
mavlink_thread(void *p)
{
  // Open a TCP socket for telemetry
  if ((tlmfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
      fprintf (stderr, "can't open stream socket for telemetry");
      exit (1);
    }

  struct sockaddr_in serv_addr;
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = inet_addr(TLM_ADDR);
  serv_addr.sin_port = htons (TLM_PORT);

  uint8_t target_sysid;
  uint8_t target_compid;
  bool request_sent = false;

  float prev_pos[3];
  bool prev_set = false;
  float prev_yaw;
  uint64_t prev_timestamp;

  //static float sx = 0.0, sy = 0.0, sz = 0.0;

  struct pollfd fds[1];

  // mavlink loop
  while(1)
    {
      if (!connected)
	{
	  if (0 == connect(tlmfd, (struct sockaddr *) &serv_addr,
			   sizeof(serv_addr)))
	    {
	      connected = true;
	      request_sent = false;
	      //printf("telemetry connected\n");
	      int option = 1;
	      ioctl(tlmfd, FIONBIO, &option);
	      fds[0].fd = tlmfd;
	      fds[0].events = POLLIN | POLLRDHUP;
	    }
	  else
	    {
	      //printf("waiting connecting telemetry\n");
	      sleep(1);
	      continue;
	    }
	}

      int rtn = poll(fds, 1, -1);
      if (rtn < 0)
	{
	  fprintf (stderr, "Failed to poll - %s\n", strerror (errno));
	  exit(1);
	}
      if ((fds[0].revents & POLLRDHUP) != 0)
	{
	  connected = false;
	  close(tlmfd);
	  //printf("Disconnected from telemetry\n");
	  if ((tlmfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	    {
	      fprintf (stderr, "can't open stream socket for telemetry");
	      exit (1);
	    }
	  continue;
	}

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
							   20,
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
		      //printf("send SET_GPS_GLOBAL_ORIGIN\n");
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
      if (!prev_set)
	{
	  prev_timestamp = timestamp_pos;
	  prev_yaw = estimated_yaw;
	  prev_pos[0] = estimated_px;
	  prev_pos[1] = estimated_py;
	  prev_pos[2] = estimated_pz;
	  prev_set = true;
	}
      else if (update_pos)
	{
	  // Fill delta with updated position and current attitude
	  mavlink_vision_position_delta_t delta;
	  delta.time_usec = timestamp_pos;
	  delta.time_delta_usec = delta.time_usec - prev_timestamp;
	  delta.angle_delta[0] = roll_angle;
	  delta.angle_delta[1] = pitch_angle;
	  delta.angle_delta[2] = angle_mod(estimated_yaw - prev_yaw);
	  float fx, fy, fz;
	  frame_delta(estimated_px - prev_pos[0], estimated_py - prev_pos[1],
		      estimated_yaw + CAM_DIRECTION, fx, fy);
	  fz = estimated_pz - prev_pos[2];
	  delta.position_delta[0] = fx;
	  delta.position_delta[1] = fy;
	  delta.position_delta[2] = fz;
	  //sx += fx; sy += fy; sz += fz;
	  delta.confidence = 50.0;
	  mavlink_msg_vision_position_delta_send_struct(MAVLINK_COMM_0, &delta);
	  prev_timestamp = delta.time_usec;
	  //printf("VISION_POSITION_DELTA %f %f %f %f\n", fx, fy, fz, estimated_yaw);
	  prev_yaw = estimated_yaw;
	  prev_pos[0] = estimated_px;
	  prev_pos[1] = estimated_py;
	  prev_pos[2] = estimated_pz;
	  update_pos = false;
	}
      pthread_mutex_unlock (&mavmutex);
      //printf("sum dxyz %5.3f %5.3f %5.3f\n", sx, sy, sz);
     }

  return 0;
}
