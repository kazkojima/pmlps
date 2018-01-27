#include <cstdio>

#include "pmlps.h"

#include <netdb.h>
#include <unistd.h>
#include <stdlib.h>
#include <strings.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define LPS_PORT 5770

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
 
  for(;;) {
#if 0
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
      float hint = 150.0;
      int np = find_frame(m, hint, frame_marker);
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
	// print center
	printf("%3.1f, %3.1f\n", sx, sy);
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

  /*
   * Open a UDP socket (an Internet datagram socket).
   */

  if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    fprintf (stderr, "server: can't open datagram socket");
    exit (1);
  }

  option = 1;
  setsockopt(sockfd,SOL_SOCKET,SO_REUSEADDR,&option,sizeof(option));

  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = htonl (INADDR_ANY);
  serv_addr.sin_port = htons (LPS_PORT);

  if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
    fprintf (stderr, "server: can't bind local address");
    exit (1);
  }

  loop(sockfd);
  /* NOTREACHED */
}
