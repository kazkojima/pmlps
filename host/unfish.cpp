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

// 3-D reconstruction from Fish-eye image and height info.
// Input: (x, y) normalized position on image sensor
// Input: h the vertical distance from sensor
// Output: pos[0], pos[1], pos[2] estimated position (X, Y, -h)

bool unfish(float x, float y, float h, float& hx, float& hy)
{
#if 1
  // v = -(u-e3) e3 / (u-e3) i.e. the stereographic projection from
  // x-y plane to unit sphere.
  auto v = - Vec(x,y,-1) * Vec(0,0,1) / Vec(x,y,-1);
#else
  float u2 = x*x + y*y;
  auto v = Vec(2*x/(u2+1), 2*y/(u2+1), (u2-1)/(u2+1));
#endif
  auto pointO = Construct::point(0,0,0);
  auto pointP = Construct::point(0,0,-h);
  auto line = pointO ^ v ^ Inf(1);
  auto dlp = (pointP <= Drv(0,0,1));
  // flat point as meet
  auto pos = (dlp <= line);
  hx = pos[0]/pos[3];
  hy = pos[1]/pos[3];
  return true;
}
