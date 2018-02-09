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
#include <cmath>

#include "config.h"
#include "pmlps.h"
#include "space/vsr_cga3D_op.h"

using namespace vsr;
using namespace vsr::cga;

#define MARKER_TYPE_I 1

// (frame size)^2 * 0.3 ???
static const float frame_eplsilon = 30.0;
#if MARKER_TYPE_H
static const float marker_sq = 128.0;
static const float size_sq_min = 128.0/2 * 0.25;
static const float size_sq_max = 128.0/2 * 2.25;
#elif MARKER_TYPE_I
static const float marker_sq = SQ_SIZEOF_SUPPORT;
static const float size_sq_min = SQ_SIZEOF_SUPPORT * 0.25;
static const float size_sq_max = SQ_SIZEOF_SUPPORT * 2.25;
#endif

int
find_frame(std::vector<ImageSensorPoint>& m, float h, Point3D fm[], float& herr)
{
  size_t n = m.size();
  std::vector<Point3D> um(n);
  int np = 0;
  float sq = 0;

  // Find frame with hint height h
  // unfish all
  for(int i = 0; i < n; i++)
    {
      float x, y;
      unfish(m[i].ex(), m[i].ey(), h, x, y);
      um[i] = Point3D(x, y, h);
    }
#if MARKER_TYPE_H
  // triple loop
  for(int i = 0; i < n; i++)
    for(int j = i+1; j < n; j++)
      for(int k = j+1; k < n; k++)
	{
	  float dij = Point3D::dsq(um[i], um[j]);
	  float djk = Point3D::dsq(um[j], um[k]);
	  float dki = Point3D::dsq(um[k], um[i]);
	  if (dij >= djk && dij >= dki)
	    {
	      if (djk < size_sq_min || djk > size_sq_max
		  || dki < size_sq_min || dki > size_sq_max)
		continue;
	      if (fabsf(djk-dki) < frame_eplsilon
		  && fabsf(dij-djk-dki) < 2*frame_eplsilon)
		{
		  fm[0] = um[k];
		  fm[1] = um[i];
		  fm[2] = um[j];
		  sq = dij;
		  np = 3;
		  break;
		}
	    }
	  else if (djk >= dij && djk >= dki)
	    {
	      if (dij < size_sq_min || dij > size_sq_max
		  || dki < size_sq_min || dki > size_sq_max)
		continue;
	      if (fabsf(dij-dki) < frame_eplsilon
		  && fabsf(djk-dki-dij) < 2*frame_eplsilon)
		{
		  fm[0] = um[i];
		  fm[1] = um[j];
		  fm[2] = um[k];
		  sq = djk;
		  np = 3;
		  break;
		}
	    }
	  else // dki maximum
	    {
	      if (djk < size_sq_min || djk > size_sq_max
		  || dij < size_sq_min || dij > size_sq_max)
		continue;
	      if (fabsf(dij-djk) < frame_eplsilon
		  && fabsf(dki-dij-djk) < 2*frame_eplsilon)
		{
		  fm[0] = um[j];
		  fm[1] = um[k];
		  fm[2] = um[i];
		  sq = dki;
		  np = 3;
		  break;
		}
	    }
	}

  if (np == 3)
    {
      // Not yet
    }
#elif MARKER_TYPE_I
  // double loop
  for(int i = 0; i < n; i++)
    for(int j = i+1; j < n; j++)
	{
	  float dij = Point3D::dsq(um[i], um[j]);
	  //printf("dsq %3.1f\n", dij);
	  if (dij < size_sq_min || dij > size_sq_max)
	    continue;
	  fm[0] = um[i];
	  fm[1] = um[j];
	  sq = dij;
	  np = 2;
	  break;
	}
#endif

  // No frame marker found.
  if (np == 0)
    return np;

  // Estimate the height error ratio from sq ratio.
  herr = sqrtf(marker_sq / sq);
  
  return np;
}

extern pthread_mutex_t mavmutex;
extern float roll_angle, pitch_angle, yaw_angle;
extern bool update_attitude;

// Adjust center 3D position with frame attitude.
void
adjust_frame_center(Point3D fm[], float& sx, float& sy, float& sz,
		    float estimated_yaw)
{
  float alpha, beta, gamma;

  // Assume MARKER_TYPE_I. TODO for H.
  pthread_mutex_lock(&mavmutex);
  // FIXME Don't use old ATTITUDE.
  if (1||update_attitude)
    {
      // Use visually estimated yaw instead of yaw_angle.
      alpha = estimated_yaw;
      beta = pitch_angle;
      gamma = roll_angle;
      pthread_mutex_unlock(&mavmutex);
      auto rox = Gen::rot(Biv::yz*(-gamma/2));
      auto roy = Gen::rot(Biv::xz*(-beta/2));
      auto roz = Gen::rot(Biv::xy*((alpha + CAM_DIRECTION)/2));
      // Body 3-2-1 sequence: first yaw, next pitch and last roll/cam.
      Vec top = Vec::z.sp(rox).sp(roy).sp(roz);
      //printf("top: %f ", yaw);
      //top.print();
      auto v0 = Vec(fm[0].ex(), fm[0].ey(), -fm[0].ez());
      auto v1 = Vec(fm[1].ex(), fm[1].ey(), -fm[1].ez());
      auto pointO = Construct::point(0, 0, 0);
      auto pointP = Construct::point(sx, sy, -sz);
      auto line0 = pointO ^ v0 ^ Inf(1);
      auto line1 = pointO ^ v1 ^ Inf(1);
      auto dlp = (pointP <= Drv(top[0], top[1], top[2]));
      auto pos0 = (dlp <= line0);
      auto pos1 = (dlp <= line1);
      float hx0 = pos0[0]/pos0[3];
      float hy0 = pos0[1]/pos0[3];
      float hz0 = pos0[2]/pos0[3];
      float hx1 = pos1[0]/pos1[3];
      float hy1 = pos1[1]/pos1[3];
      float hz1 = pos1[2]/pos1[3];
      float sq = (hx0-hx1)*(hx0-hx1)+(hy0-hy1)*(hy0-hy1)+(hz0-hz1)*(hz0-hz1);
      // Estimate the height error ratio from sq ratio.
      float herr = sqrtf(marker_sq / sq);
      //printf("%3.3f %3.3f\n", herr, sz*herr);
      sx *= herr; sy *= herr; sz *= herr;
    }
  else
    pthread_mutex_unlock(&mavmutex);

  return;
}
