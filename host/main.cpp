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

#include "config.h"
#include "pmlps.h"

// for Kalman filter
#include "opencv2/video/tracking_c.h"

#include <netdb.h>
#include <unistd.h>
#include <stdlib.h>
#include <strings.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <pthread.h>
#include <getopt.h>

//#define DEBUG

static int show_flags;

#define SHOW_POS         (1<<0)
#define SHOW_YAW         (1<<1)

struct pmlps_config config =
  {
    // addr and port
    .lps_port = LPS_PORT,
    .tlm_addr = TLM_ADDR,
    .tlm_port = TLM_PORT,
    // cam
    .cam_image_width = CAM_IMAGE_WIDTH,
    .cam_image_height = CAM_IMAGE_HEIGHT,
    .cam_lens_ratio = CAM_LENS_RATIO,
    .cam_lens_fisheye = true,
    .cam_height = CAM_HEIGHT,
    .cam_direction = CAM_DIRECTION,
    // marker
    .marker_type = MARKER_TYPE_I,
    .marker_sqsize = SQ_SIZEOF_SURROUND,
  };

extern pthread_mutex_t mavmutex;
extern bool update_attitude;
extern void *mavlink_thread (void *);
bool update_pos;
uint64_t timestamp_pos;
float estimated_px, estimated_py, estimated_pz;
float estimated_yaw;

inline static uint64_t utimestamp(void)
{
  struct timeval tv;
  gettimeofday(&tv,NULL);
  return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

inline static int kdelta(int i, int j) { return (i == j) ? 1 : 0;}

static struct __attribute__((packed)) pkt {
  unsigned short fcount;
  unsigned short bcount;
  unsigned short cx;
  unsigned short cy;
  unsigned short pixels;
  unsigned short code;
} pkt;

#define COUNT_TO_STABILIZE (50*4)

// Main loop
void
loop(int sockfd)
{
  int n;
  socklen_t clilen;
  struct sockaddr_in cli_addr;
  std::vector<ImageSensorPoint> m;
  Point3D frame_marker[num_frame_markers];
  VisualYawEstimater yest;

  float hint = CAM_HEIGHT/2;
  float h = hint;

  int count = 0;

  // Kalman filter for position estimation
  CvKalman *kalman = cvCreateKalman(6, 3);

  cvSetIdentity(kalman->measurement_matrix, cvRealScalar(1.0));
  cvSetIdentity(kalman->process_noise_cov, cvRealScalar(1e-5));
  cvSetIdentity(kalman->measurement_noise_cov, cvRealScalar(0.1));
  cvSetIdentity(kalman->error_cov_post, cvRealScalar(1.0));

  for(int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
      kalman->DynamMatr[i*6 + j] = kdelta(i, j) + kdelta(i, j - 3);
  //kalman->DynamMatr[2*6 + 5] = 0.2;

  bool found = false;
  for(;;)
    {
      clilen = sizeof(cli_addr);
      bzero (&cli_addr, clilen);
      n = recvfrom(sockfd, &pkt, sizeof(pkt), 0,
		   (struct sockaddr *) &cli_addr, &clilen);
      if (n < 0)
	{
	  fprintf (stderr, "server: recvfrom error");
	  exit (1);
	}

      if (pkt.fcount != 0xa5a5)
	{
#ifdef DEBUG
	  printf ("%d %d: ", pkt.fcount, pkt.bcount);
	  printf ("(%03d, %03d) %d %04x\n", pkt.cx, pkt.cy, pkt.pixels,
		  pkt.code);
#endif
	  ImageSensorPoint p(pkt.cx, pkt.cy);
	  m.push_back(p);
	}
      else
	{
#ifdef DEBUG
	  printf ("end of frame\n");
#endif
	  float herr;
	  if (count < COUNT_TO_STABILIZE)
	    {
	      if (h < hint)
		h = hint;
	    }
	  // 1st Try without attitude info.
	  int np = find_frame(m, h, frame_marker, found, herr);
	  if (np == 0)
	    np = find_frame(m, hint, frame_marker, found, herr);
	  //printf ("np %d estimated height %f err %f\n", np, h, herr);
	  if (np)
	    {
	      found = true;
	      float sx = 0, sy = 0, sz = 0;
	      for(int i = 0; i < np; i++)
		{
		  float x = frame_marker[i].ex();
		  float y = frame_marker[i].ey();
		  float z = frame_marker[i].ez();
		  if (np != 3 || i != 0)
		    {
		      sx += x; sy += y; sz += z;
		    }
#ifdef DEBUG
		  printf("1st fp %d: (%3.1f, %3.1f, %3.1f)\n", i, x, y, z);
#endif
		}
	      if (np != 3)
		{
		  sx /= np; sy /= np; sz /= np;
		}
	      else
		{
		  sx /= 2; sy /= 2; sz /= 2;
		}
	      sx = sx*herr; sy = sy*herr; sz = sz*herr;

	      // Estimite yaw with makers
	      float yaw = yest.estimate_visual_yaw(frame_marker);
	      if (show_flags & SHOW_YAW)
		printf("%3.3f\n", yaw);

	      // 2nd Try with attitude info if available.
	      adjust_frame_center(frame_marker, sx, sy, sz, yaw);

	      // update and predict position
	      float meas[3];
	      meas[0] = sx;
	      meas[1] = sy;
	      meas[2] = sz;
	      CvMat measurement = cvMat(3, 1, CV_32FC1, meas);
	      const CvMat *correction = cvKalmanCorrect(kalman, &measurement);
	      const CvMat *prediction = cvKalmanPredict(kalman);
	      sx = prediction->data.fl[0];
	      sy = prediction->data.fl[1];
	      sz = prediction->data.fl[2];
	      h = sz;
	      // print center
	      if (show_flags & SHOW_POS)
		printf("%3.1f, %3.1f, %3.1f\n", sx, sy, CAM_HEIGHT - sz);

	      pthread_mutex_lock(&mavmutex);
	      // estimated position in meter
	      estimated_px = sx/100;
	      estimated_py = sy/100;
	      estimated_pz = sz/100;
	      estimated_yaw = yaw;
	      timestamp_pos = utimestamp();
	      if (count > COUNT_TO_STABILIZE)
		update_pos = true;
	      update_attitude = false;
	      pthread_mutex_unlock(&mavmutex);
	    }
	  m.clear();
	}

#if 0
      if (sendto(sockfd, &pkt, n, 0, (struct sockaddr *)
		 &cli_addr, clilen) != n)
	{
	  fprintf (stderr, "sendto error");
	  exit (1);
	}
#endif
      count++;
#if 0
      // for profiling
      if (count > 10000)
	exit(0);
#endif
    }
}

static void
err_quit (const char *msg)
{
  fprintf (stderr, "%s\n", msg);
  exit (1);
}

int
main(int argc, char *argv[])
{
  int sockfd;
  int option;
  char *s;
  struct sockaddr_in serv_addr;

  const struct option longopts[] =
    {
      { "help", no_argument, NULL, 'h' },
      { "yaw-offset", required_argument, NULL, 'y' },
      { "show-position",  no_argument, NULL, 'P' },
      { "show-yaw",  no_argument, NULL, 'Y' },
      { "telemetry-address", required_argument, NULL, 'a' },
      { "telemetry-port", required_argument, NULL, 't' },
      { "lps-port", required_argument, NULL, 'u' },
      { "lens-ratio", required_argument, NULL, 'r' },
      { "cam-height", required_argument, NULL, 'z' },
      { "marker-square-size", required_argument, NULL, 's' },
      { 0, 0, 0, 0 },
    };
  int opt;
  int index;
  while ((opt = getopt_long(argc, argv, "PYhy:", longopts, &index)) != -1)
    {
      switch (opt)
        {
        case 'h':
          printf ("Usage:\n"
                  "  pmlps [OPTION...]\n"
                  "\nHelp Options:\n"
                  "  -h (--help) Show help options\n"
                  "\nReport options:\n"
                  "  -P (--show-position) Show position in CAM frame\n"
		  "  -Y (--show-yaw) Show computed yaw (rad. from north)\n"
                  "\nSetting options:\n"
                  "  -y (--yaw-offset) FLOAT_VALUE  Set yaw direction offset\n"
		  );
          exit (1);
	case 'y': // next arg is yaw direction offset
          config.cam_direction = atof(optarg);
          break;
	case 'a':
          config.tlm_addr = optarg;
          break;
	case 't':
          config.tlm_port = atoi(optarg);
          break;
	case 'u':
          config.lps_port = atoi(optarg);
          break;
	case 'r':
          config.cam_lens_ratio = atof(optarg);
          break;
	case 'z':
          config.cam_height = atof(optarg);
          break;
	case 's':
          config.marker_sqsize = atof(optarg);
          break;
	case 'P':
          show_flags |= SHOW_POS;
          break;
	case 'Y':
          show_flags |= SHOW_YAW;
          break;
	default:
	  err_quit("illegal option");
	}
    }

  // Open a UDP socket for cam
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    err_quit("server: can't open datagram socket for cam");

  option = 1;
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));

  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = htonl (INADDR_ANY);
  serv_addr.sin_port = htons (config.lps_port);

  if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
    err_quit("can't bind local address");

  pthread_t pthread;
  pthread_mutex_init (&mavmutex, NULL);
  pthread_create (&pthread, NULL, mavlink_thread, NULL);

  loop(sockfd);
  /* NOTREACHED */
}
