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

#if OPENCV
// for Kalman filter
#include "opencv2/video/tracking_c.h"
#endif

#include <netdb.h>
#include <unistd.h>
#include <stdlib.h>
#include <strings.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <pthread.h>

//#define DEBUG

extern pthread_mutex_t mavmutex;
extern void *mavlink_thread (void *);
bool update_pos;
uint64_t timestamp_pos;
float estimated_px, estimated_py, estimated_pz;
float estimated_yaw;

extern float estimate_visual_yaw(Point3D fm[]);

inline static uint64_t utimestamp(void)
{
  struct timeval tv;
  gettimeofday(&tv,NULL);
  return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

static struct __attribute__((packed)) pkt {
  unsigned short fcount;
  unsigned short bcount;
  unsigned short cx;
  unsigned short cy;
  unsigned short pixels;
  unsigned short code;
} pkt;

// Main loop
void
loop(int sockfd)
{
  int n;
  socklen_t clilen;
  struct sockaddr_in cli_addr;
  std::vector<ImageSensorPoint> m;
  Point3D frame_marker[num_frame_markers];

  float hint = 100.0;
  float h = hint;

  // Kalman filter for height estimation
#if OPENCV
  CvKalman *kalman = cvCreateKalman(2, 1);

  cvSetIdentity(kalman->measurement_matrix, cvRealScalar(1.0));
  cvSetIdentity(kalman->process_noise_cov, cvRealScalar(1e-5));
  cvSetIdentity(kalman->measurement_noise_cov, cvRealScalar(0.1));
  cvSetIdentity(kalman->error_cov_post, cvRealScalar(1.0));

  kalman->DynamMatr[0] = 1.0; kalman->DynamMatr[1] = 1.0;
  kalman->DynamMatr[2] = 0.0; kalman->DynamMatr[3] = 1.0;
#else
  const float Q = 0.0001;
  const float R = 0.1;
  const float H = 1.0;
  const float B = 0.5;
  float xhat = hint;
  float p = Q;
  float u = 0;
#endif

  for(;;) {
#if 1
    clilen = sizeof(cli_addr);
    bzero (&cli_addr, clilen);
    n = recvfrom(sockfd, &pkt, sizeof(pkt), 0,
		 (struct sockaddr *) &cli_addr, &clilen);
    if (n < 0) {
      fprintf (stderr, "server: recvfrom error");
      exit (1);
    }
#else
    //#define DEBUG
    static int count = 0;
    static struct pkt pkts[] = {
      { 1, 0, 165, 65, 4, 1},
      { 1, 0, 185, 65, 4, 1},
      { 1, 0, 174, 66, 4, 1},
      { 1, 0, 185, 165, 4, 1},
      { 1, 0, 175, 74, 4, 1},
      { 1, 0, 164, 74, 4, 1},
      { 0xa5a5, 0, 0, 0, 0, 0},
    };
    if (count >= sizeof(pkts)/sizeof(pkt))
      exit(1);
    bcopy(&pkts[count++], &pkt, sizeof(pkt));
#endif

    if (pkt.fcount != 0xa5a5) {
#ifdef DEBUG
      printf ("%d %d: ", pkt.fcount, pkt.bcount);
      printf ("(%03d, %03d) %d %04x\n", pkt.cx, pkt.cy, pkt.pixels, pkt.code);
#endif
      ImageSensorPoint p(pkt.cx, pkt.cy);
      m.push_back(p);
    } else {
#ifdef DEBUG
      printf ("end of frame\n");
#endif
      float herr;
      int np = find_frame(m, h, frame_marker, herr);
      if (np == 0)
	np = find_frame(m, hint, frame_marker, herr);
      //printf ("np %d estimated height %f err %f\n", np, h, herr);
      if (np) {
	float sx = 0, sy = 0, sz = 0;
	for(int i = 0; i < np; i++) {
	  float x = frame_marker[i].ex();
	  float y = frame_marker[i].ey();
	  float z = frame_marker[i].ez();
	  if (np != 3 || i != 0) {
	    sx += x; sy += y; sz += z;
	  }
#ifdef DEBUG
	  printf("fp %d: (%3.1f, %3.1f, %3.1f)\n", i, x, y, z);
#endif
	}
	if (np != 3) {
	  sx /= np; sy /= np; sz /= np;
	} else {
	  sx /= 2; sy /= 2; sz /= 2;
	}

	// update and predict height
#if OPENCV
	float mat[1];
	mat[0] = sz*herr;
	CvMat measurement = cvMat(1, 1, CV_32FC1, mat);
	const CvMat *correction = cvKalmanCorrect(kalman, &measurement);
	const CvMat *prediction = cvKalmanPredict(kalman);
	h = prediction->data.fl[0];
#else
	float xhat_prev = xhat;
	// predctor step
	xhat = xhat + B*u;
	p = p + Q;
	// corrector step
	float K = H*p/(H*H*p + R);
	xhat = xhat + K*(sz*herr - H*xhat);
	p = p*(1 - H*K);
	u = xhat - xhat_prev;
	h = xhat;
#endif
#ifdef DEBUG
	// print center
	printf("%3.1f, %3.1f, %3.1f\n", sx, sy, CAM_HEIGHT - sz);
#endif
	float yaw = estimate_visual_yaw(frame_marker);
	//printf("yaw %3.1f\n", yaw);
	pthread_mutex_lock(&mavmutex);
	// estimated position in meter
	estimated_px = sx/100;
	estimated_py = sy/100;
	estimated_pz = sz/100;
	estimated_yaw = yaw;
	timestamp_pos = utimestamp();
	update_pos = true;
	pthread_mutex_unlock(&mavmutex);
      }
      m.clear();
    }

#if 0
    if (sendto(sockfd, &pkt, n, 0, (struct sockaddr *)
	       &cli_addr, clilen) != n) {
      fprintf (stderr, "server: sendto error");
      exit (1);
    }
#endif
  }
}

void
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

  while (--argc > 0 && (*++argv)[0] == '-')
    for (s = argv[0]+1; *s != '\0'; s++)
      switch (*s) {
      default:
	err_quit("illegal option");
      }

  // Open a UDP socket for cam
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
      fprintf (stderr, "server: can't open datagram socket for cam");
      exit (1);
    }

  option = 1;
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));

  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = htonl (INADDR_ANY);
  serv_addr.sin_port = htons (LPS_PORT);

  if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
    {
      fprintf (stderr, "server: can't bind local address");
      exit (1);
    }

  pthread_t pthread;
  pthread_mutex_init (&mavmutex, NULL);
  pthread_create (&pthread, NULL, mavlink_thread, NULL);

  loop(sockfd);
  /* NOTREACHED */
}
