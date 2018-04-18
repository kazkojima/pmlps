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
#include "space/vsr_cga3D_op.h"

using namespace vsr;
using namespace vsr::cga;

// Map 3-D to 2-D with Fish-eye.
// Input: (x, y, z) normalized position
// Output: (ix, iy) sensor image cordinate
// Output: rho      estimated bounded box size

bool
fish(const float x, const float y, const float z, int& ix, int& iy, int& rho)
{
  Vec v;
  if (config.cam_lens_fisheye)
    {
      float nv = sqrtf(x*x+y*y+z*z);
      if (nv < 1)
	return false;
      v = Vec(x/nv, y/nv, -z/nv - 1);
    }
  else
    {
      v = Vec(x, y, -z - 1);
    }

  //v.print();
  auto pointN = Construct::point(0,0,1);
  auto line = pointN ^ v ^ Inf(1);
  auto pos = (Vec(0,0,1) <= line);
  //pos.print();

  float hx, hy;
  hx = pos[0]/pos[3];
  hy = pos[1]/pos[3];

  unsigned short cx = config.cam_image_width/2;
  unsigned short cy = config.cam_image_height/2;
  float lens_ratio = config.cam_lens_ratio;
  ix = (int)(cx + hx/lens_ratio);
  iy = (int)(cy - hy/lens_ratio);
  //printf("[%d, %d]\n", ix, iy);

  // Estimate bounded box size roughly.
  // Assume that the marker will move N(=2) times of its size between frames
  // and use it for the size of bounded sphere. Scale that sphere with
  // the distance from the origin and use the size of scaled sphere for
  // the bounded box size on the image plane. This isn't acculate at all
  // but will be ok for a very rough estimation.
  float marker_sq = config.marker_sqsize;
  // Add marker_sq to divisor so as to avoid the floting division issue.
  float rk = 2*sqrtf(marker_sq/(marker_sq+x*x+y*y+z*z));
  rho = (int)(rk/lens_ratio);
  //printf("bounded box size %d\n", rho);
  return true;
}
