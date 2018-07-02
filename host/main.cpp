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
    .marker_type = MARKER_TYPE_I3,
    .marker_sqsize = SQ_SIZEOF_SURROUND,
    .marker_sqratio = SQ_RATIO_I3,
    // mavlink
    .use_position_delta = true,
  };

extern pthread_mutex_t mavmutex;
extern bool update_attitude;
extern void *mavlink_thread (void *);
std::queue<EstimatedPosition> pos_queue;

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
  unsigned short w;
  unsigned short h;
} pkt;

static struct  __attribute__((packed)) {
  int16_t id;
  int16_t ix, iy, iz;
  int16_t a0;
  int16_t a1;
} spkt;

#define COUNT_TO_STABILIZE (50*4)

// Main loop
void
loop(int sockfd)
{
  int n;
  socklen_t clilen;
  struct sockaddr_in cli_addr;
  std::vector<ImageSensorBlob> m;
  Point3D frame_marker[num_frame_markers];
  VisualYawEstimater yest;

  float hint = config.cam_height/2;
  float h = hint;

  int count = 0;

  // Kalman filter for position estimation
  CvKalman *kalman = cvCreateKalman(6, 3);

  cvSetIdentity(kalman->measurement_matrix, cvRealScalar(1.0));
  cvSetIdentity(kalman->process_noise_cov, cvRealScalar(1e-5));
  cvSetIdentity(kalman->measurement_noise_cov, cvRealScalar(0.1));
  //cvmSet(kalman->measurement_noise_cov, 2, 2, 0.3);
  cvSetIdentity(kalman->error_cov_post, cvRealScalar(1.0));

  for(int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
      kalman->DynamMatr[i*6 + j] = kdelta(i, j) + kdelta(i, j - 3);
  kalman->DynamMatr[2*6 + 5] = 0.5;

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
	  printf ("(%03d, %03d) [%03d %03d]\n", pkt.cx, pkt.cy, pkt.w, pkt.h);
#endif
	  ImageSensorBlob p(pkt.cx, pkt.cy, pkt.w, pkt.h);
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
	  int np = 0;
	  if (config.marker_type == MARKER_TYPE_I)
	    {
	      // 1st Try without attitude info.
	      np = find_frame(m, h, frame_marker, found, herr);
	      if (np == 0)
		np = find_frame(m, hint, frame_marker, found, herr);
	    }
	  else if  (config.marker_type == MARKER_TYPE_I3)
	    {
	      // 1st Try without attitude info.
	      np = find_frame_i3(m, h, frame_marker, found, herr);
	      if (np == 0)
		np = find_frame_i3(m, hint, frame_marker, found, herr);
	    }
	  else
	    {
	      fprintf (stderr, "sendto error");
	    }

	  //printf ("np %d estimated height %f err %f\n", np, h, herr);

	  if (np)
	    {
	      found = true;
	      float sx = 0, sy = 0, sz = 0;
	      for(int i = 0; i < 2; i++)
		{
		  float x = frame_marker[i].ex();
		  float y = frame_marker[i].ey();
		  float z = frame_marker[i].ez();
		  sx += x; sy += y; sz += z;
#ifdef DEBUG
		  printf("1st fp %d: (%3.1f, %3.1f, %3.1f)\n", i, x, y, z);
#endif
		}
	      sx /= 2; sy /= 2; sz /= 2;
	      sx = sx*herr; sy = sy*herr; sz = sz*herr;
	      //printf("C (%3.1f, %3.1f, %3.1f)\n", sx, sy, sz);

	      // Estimite yaw with makers
	      bool estimated;
	      float yaw = yest.estimate_visual_yaw(frame_marker, estimated);
	      if (show_flags & SHOW_YAW)
		printf("%3.3f\n", yaw);

	      // 2nd Try with attitude info and yaw if available.
	      if (estimated)
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
		printf("%3.1f, %3.1f, %3.1f\n", sx, sy, config.cam_height - sz);
	      pthread_mutex_lock(&mavmutex);
	      // estimated position in meter
	      EstimatedPosition pos(sx/100, sy/100, sz/100, yaw, utimestamp());
	      if (count > COUNT_TO_STABILIZE)
		pos_queue.push(pos);
	      //printf("up %ud %ld\n", pos_queue.size(), utimestamp());
	      update_attitude = false;
	      pthread_mutex_unlock(&mavmutex);

	      int hx, hy, rho;
	      memset(&spkt, 0, sizeof(pkt));
	      if (fish(sx, sy, sz, hx, hy, rho))
		{
		  spkt.ix = (int16_t)hx;
		  spkt.iy = (int16_t)hy;
		  spkt.a0 = (int16_t)rho;
		}
	      else
		{
		  spkt.ix = spkt.iy = -1;
		}
	      spkt.iz = (int16_t)sz;
	      spkt.a1 = (count > COUNT_TO_STABILIZE) ? -1 : 0;
	      n = sizeof(spkt);
	      if (sendto(sockfd, &spkt, n, 0, (struct sockaddr *)
			 &cli_addr, clilen) != n)
		{
		  fprintf (stderr, "sendto error");
		}
	    }
	  else
	    {
	      memset(&spkt, 0, sizeof(pkt));
	      spkt.ix = spkt.iy = -1;
	      spkt.a1 = (count > COUNT_TO_STABILIZE) ? -1 : 0;
	      n = sizeof(spkt);
	      if (sendto(sockfd, &spkt, n, 0, (struct sockaddr *)
			 &cli_addr, clilen) != n)
		{
		  fprintf (stderr, "sendto error");
		}
	    }
	  m.clear();
	}

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
      { "no-fisheye", no_argument, NULL, 'n' },
      { "cam-height", required_argument, NULL, 'z' },
      { "marker-type", required_argument, NULL, 'm' },
      { "marker-square-size", required_argument, NULL, 's' },
      { "vicon-position-estimate", no_argument, NULL, 'p' },
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
		  "\nLong only options:\n"
		  "  --telemetry-address aaa.bbb.ccc.ddd  Set telemetry tcp addr\n"
		  "  --telemetry-port nnnn  Set telemetry port\n"
		  "  --lps-port uuuu  Set UDP port of LPS\n"
		  "  --lens-ratio FLOAT_VALUE  Set lens ratio\n"
		  "  --no-fisheye  No fisheye lens\n"
		  "  --cam-height FLOAT_VALUE  Set height of CAM in cm\n"
		  "  --marker-type TYPE_NAME  Specify marker type (I,I3,...)\n"
		  "  --marker-square-size FLOAT_VALUE  Set square size in cm^2\n"
		  "  --vicon-position-estimate  Use vicon_position_estimate message\n"
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
	case 'n':
          config.cam_lens_fisheye = false;
          break;
	case 'r':
          config.cam_lens_ratio = atof(optarg);
          break;
	case 'z':
          config.cam_height = atof(optarg);
          break;
	case 'm':
	  if (0 == strcmp(optarg, "I"))
	    config.marker_type = MARKER_TYPE_I;
	  else if (0 == strcmp(optarg, "I3"))
	    config.marker_type = MARKER_TYPE_I3;
	  else
	    err_quit("unsupportd maker type");
          break;
	case 's':
          config.marker_sqsize = atof(optarg);
          break;
	case 'p':
          config.use_position_delta = false;
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
