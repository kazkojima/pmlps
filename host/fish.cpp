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

bool
fish(const float x, const float y, const float z, int& ix, int& iy)
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
  return true;
}
