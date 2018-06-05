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
#include <cfloat>

#include "config.h"
#include "pmlps.h"
#include "space/vsr_cga3D_op.h"

//#define DEBUG

using namespace vsr;
using namespace vsr::cga;

// (frame size)^2 * 0.3 ???
static const float frame_epsilon = 30.0;
static const float position_sq_epsilon = 120.0;

int
find_frame(std::vector<ImageSensorBlob>& m, float h, Point3D fm[],
	   bool prev, float& herr)
{
  size_t n = m.size();
  std::vector<Point3D> um(n);
  int np = 0;
  float sq = 0;
  float marker_sq = config.marker_sqsize;
  float size_sq_min = config.marker_sqsize * 0.0625;
  float size_sq_max = config.marker_sqsize * 4.0;

  // Find frame with hint height h
  // unfish all
  for(int i = 0; i < n; i++)
    {
      float x, y;
      unfish(m[i].ex(), m[i].ey(), h, x, y);
      um[i] = Point3D(x, y, h);
    }

  float errmin = FLT_MAX;
  Point3D pm[2];
  if (prev)
    {
      pm[0] = fm[0];
      pm[1] = fm[1];
      // double loop. match with previous result
      for(int i = 0; i < n; i++)
	for(int j = i+1; j < n; j++)
	  {
	    float d01 = Point3D::dsq(pm[0], um[i]) + Point3D::dsq(pm[1], um[j]);
	    float d10 = Point3D::dsq(pm[1], um[i]) + Point3D::dsq(pm[0], um[j]);
	    float d = fminf(d01, d10);
	    if (d > errmin)
	      continue;
	    //printf("cand pv (%3.1f, %3.1f, %3.1f) (%3.1f, %3.1f, %3.1f) err=%3.1f\n", um[i].ex(), um[i].ey(), um[i].ez(), um[j].ex(), um[j].ey(), um[j].ez(), d);
	    errmin = d;
	    fm[0] = um[i];
	    fm[1] = um[j];
	  }

      sq = Point3D::dsq(fm[0], fm[1]);
      //printf("pv err %3.1f sq %3.1f\n", errmin, sq);
      if (errmin < position_sq_epsilon && sq > size_sq_min && sq < size_sq_max)
	{
	  np = 2;
	  //printf("err %3.1f sq %3.1f\n", errmin, sq);
	}
    }

  if (np == 0)
    {
      // double loop. match with square size
      errmin = FLT_MAX;
      for(int i = 0; i < n; i++)
	for(int j = i+1; j < n; j++)
	  {
	    float dij = Point3D::dsq(um[i], um[j]);
	    //printf("dsq %3.1f\n", dij);
	    float sqdiff = fabsf(marker_sq - dij);
	    if (sqdiff > errmin)
	      continue;
	    //printf("cand sq (%3.1f, %3.1f, %3.1f) (%3.1f, %3.1f, %3.1f) err=%3.1f\n", um[i].ex(), um[i].ey(), um[i].ez(), um[j].ex(), um[j].ey(), um[j].ez(), sqdiff);
	    errmin = sqdiff;
	    fm[0] = um[i];
	    fm[1] = um[j];
	    sq = dij;
	  }

      if (sq > size_sq_min && sq < size_sq_max)
	{
	  //printf("sq %3.1f\n", sq);
	  np = 2;
	}
    }

  // No frame marker found.
  if (np == 0)
    return np;

  // Estimate the height error ratio from sq ratio.
  herr = sqrtf(marker_sq / sq);

  return np;
}

static const float ratio_epsilon = 30.0;

static bool
linear(Point3D p, Point3D q, Point3D r)
{
  // ez are same ATM.
  float ax = p.ex() - q.ex();
  float ay = p.ey() - q.ey();
  float bx = r.ex() - q.ex();
  float by = r.ey() - q.ey();
  float sqa = ax*ax + ay*ay;
  float sqb = bx*bx + by*by;
  if (sqa < 0.1 || sqb < 0.1)
    return false;
  float sq = fabsf(ax*by - ay*bx);
  sq = sq*sq;
  //printf("sq %3.3f sqa*sqb*0.01 %3.3f\n", sq, sqa*sqb*0.01);
  return (sq < sqa*sqb*0.01);
}

static Point3D
merge(Point3D p, Point3D q)
{
  Point3D r((p.ex() + q.ex())/2, (p.ey() + q.ey())/2, (p.ez() + q.ez())/2);
  return r;
}

// Find frame and return head and tail.
// Take into account that head and neck markers might be merged into one blob.
int
find_frame_i3(std::vector<ImageSensorBlob>& m, float h, Point3D fm[],
	      bool prev, float& herr)
{

  size_t n = m.size();
  std::vector<Point3D> um(n);
  int np = 0;
  float sq = 0;
  float marker_sq = config.marker_sqsize;
  float size_sq_min;
  float size_sq_max = config.marker_sqsize * 4.0;
  float hc = h/config.cam_height;
  size_sq_min = config.marker_sqsize * hc * hc * 0.6;

  // Find frame with hint height h
  // unfish all
  for(int i = 0; i < n; i++)
    {
      float x, y;
      unfish(m[i].ex(), m[i].ey(), h, x, y);
      um[i] = Point3D(x, y, h);
    }

  float errmin = FLT_MAX;
  Point3D pm[3];
  int hidx = -1, nidx = -1, tidx = -1;
  if (prev)
    {
      pm[0] = fm[0];
      pm[1] = fm[1];
      // double loop. match with previous result
      for(int i = 0; i < n; i++)
	for(int j = i+1; j < n; j++)
	  {
	    float d01 = Point3D::dsq(pm[0], um[i]) + Point3D::dsq(pm[1], um[j]);
	    float d10 = Point3D::dsq(pm[1], um[i]) + Point3D::dsq(pm[0], um[j]);
	    float d = fminf(d01, d10);
	    if (d > errmin)
	      continue;
	    //printf("cand pv (%3.1f, %3.1f, %3.1f) (%3.1f, %3.1f, %3.1f) err=%3.1f\n", um[i].ex(), um[i].ey(), um[i].ez(), um[j].ex(), um[j].ey(), um[j].ez(), d);
	    errmin = d;
	    if (d01 < d10)
	      {
		fm[0] = um[i];
		fm[1] = um[j];
		hidx = i; tidx = j;
	      }
	    else
	      {
		fm[0] = um[j];
		fm[1] = um[i];
		hidx = j; tidx = i;
	      }
	  }

      float nerr = FLT_MAX;
      for (int k = 0; k < n; k++)
	{
	  if (!linear(fm[0], um[k], fm[1]))
	    continue;
	  float nd = Point3D::dsq(fm[0], um[k]);
	  if (nd > nerr)
	    continue;
	  nerr = nd;
	  fm[2] = um[k];
	  nidx = k;
	}

      // Too far or too big
      if (nerr > position_sq_epsilon
	  || Point3D::dsq(fm[0], fm[2]) > SQ_RATIO_I3*size_sq_max)
	nidx = -1;

      if (nidx < 0)
	sq = Point3D::dsq(fm[0], fm[1]);
      else
	{
	  // merge head and neck
	  fm[0] = merge(fm[0], fm[2]);
	  sq = Point3D::dsq(fm[0], fm[1]);
	}
      //printf("pv err %3.1f sq %3.1f\n", errmin, sq);

      if (errmin < position_sq_epsilon && sq > size_sq_min && sq < size_sq_max)
	{
	  np = 2;
#ifdef DEBUG
	  printf("match prev. err %3.1f sq %3.1f\n", errmin, sq);
#endif
	}
    }

  if (np == 0)
    {
      // double loop. match with square size
      errmin = FLT_MAX;
      for(int i = 0; i < n; i++)
	for(int j = i+1; j < n; j++)
	  {
	    float dij = Point3D::dsq(um[i], um[j]);
	    //printf("dsq %3.1f\n", dij);
	    float sqdiff = fabsf(marker_sq - dij);
	    if (sqdiff > errmin)
	      continue;
	    //printf("cand sq (%3.1f, %3.1f, %3.1f) (%3.1f, %3.1f, %3.1f) err=%3.1f\n", um[i].ex(), um[i].ey(), um[i].ez(), um[j].ex(), um[j].ey(), um[j].ez(), sqdiff);
	    errmin = sqdiff;
	    fm[0] = um[i];
	    fm[1] = um[j];
	    hidx = i; tidx = j; // assume anyway
	    sq = dij;
	  }

      float nerr = FLT_MAX;
      nidx = -1;
      for (int k = 0; k < n; k++)
	{
	  if (!linear(fm[0], um[k], fm[1]))
	    continue;
	  float d0 = Point3D::dsq(fm[0], um[k]);
	  float d1 = Point3D::dsq(fm[1], um[k]);
	  float nd = fminf(d0, d1);
	  if (nd > nerr)
	    continue;
	  nerr = nd;
	  if (d0 > d1)
	    {
	      std::swap(hidx, tidx);
	      fm[0] = um[hidx];
	      fm[1] = um[tidx];
	    }
	  fm[2] = um[k];
	  nidx = k;
	}

      // Too far or too big
      if (nerr > position_sq_epsilon
	  || Point3D::dsq(fm[0], fm[2]) > SQ_RATIO_I3*size_sq_max)
	nidx = -1;

      if (nidx >= 0)
	{
	  // merge head and neck
	  fm[0] = merge(fm[0], fm[2]);
	  sq = Point3D::dsq(fm[0], fm[1]);
	}

       if (sq > size_sq_min && sq < size_sq_max)
	{
#ifdef DEBUG
	  if (nidx > 0)
	    printf("match 3 pts. err %3.1f sq %3.1f\n", errmin, sq);
	  else
	    printf("match 2 pts. err %3.1f sq %3.1f\n", errmin, sq);
#endif
	  np = 2;
	}
    }

  if (np == 2 && nidx < 0)
    {
#ifdef DEBUG
      printf("h area %3.6f t area %3.6f\n", m[hidx].sw()*m[hidx].sh(),
	     m[tidx].sw()*m[tidx].sh());
#endif
      // Last resort. merged blobs would be fat
      if (1.2*m[hidx].sw()*m[hidx].sh() < m[tidx].sw()*m[tidx].sh())
	{
	  fm[0] = um[tidx];
	  fm[1] = um[hidx];
	}
    }

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
  if (update_attitude)
    {
      float yaw_direction_offset = config.cam_direction;
      // Use visually estimated yaw instead of yaw_angle.
      alpha = estimated_yaw;
      beta = pitch_angle;
      gamma = roll_angle;
      pthread_mutex_unlock(&mavmutex);
      auto rox = Gen::rot(Biv::yz*(-gamma/2));
      auto roy = Gen::rot(Biv::xz*(-beta/2));
      auto roz = Gen::rot(Biv::xy*((alpha + yaw_direction_offset)/2));
      // Body 3-2-1 sequence: first yaw, next pitch and last roll.
      // Reverse it.
      Vec top = Vec::z.sp(rox).sp(roy).sp(roz);
      //printf("top: %f ", yaw);
      //top.print();
      auto v0 = Vec(fm[0].ex(), fm[0].ey(), -fm[0].ez());
      auto v1 = Vec(fm[1].ex(), fm[1].ey(), -fm[1].ez());
      auto pointO = Construct::point(0, 0, 0);
      auto pointP = Construct::point(sx, sy, -sz);
      auto line0 = pointO ^ v0 ^ Inf(1);
      auto line1 = pointO ^ v1 ^ Inf(1);
      //auto dlp = (pointP <= Drv(top[0], top[1], top[2]));
      auto dlp = (pointP <= (top ^ Inf(1)));
      // flat points as meet
      auto pos0 = (dlp <= line0);
      auto pos1 = (dlp <= line1);
      // sq distance
      // sq = -2 * ((pointO <= (pos0/pos0[3])).null()
      //            <= (pointO <= (pos1/pos1[3])).null());
#if 1
      Vec u0 = Vec(pos0[0]/pos0[3], pos0[1]/pos0[3], pos0[2]/pos0[3]);
      Vec u1 = Vec(pos1[0]/pos0[3], pos1[1]/pos0[3], pos1[2]/pos0[3]);
      float sq = -2 * (u0.null() <= u1.null())[0];
#else
      Point3D q0(pos0[0]/pos0[3], pos0[1]/pos0[3], pos0[2]/pos0[3]);
      Point3D q1(pos1[0]/pos0[3], pos1[1]/pos0[3], pos1[2]/pos0[3]);
      float sq = Point3D::dsq(q0, q1);
#endif
      // Estimate the height error ratio from sq ratio.
      float marker_sq = config.marker_sqsize;
      float herr = sqrtf(marker_sq / sq);
      sx *= herr; sy *= herr; sz *= herr;
    }
  else
    pthread_mutex_unlock(&mavmutex);

  return;
}
