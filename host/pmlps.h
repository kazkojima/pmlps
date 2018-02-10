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

#ifndef _PMLPS_H
#define _PMLPS_H
#include <vector>

// Constants for QVGA
const unsigned short ImageSensorCentorX = (CAM_IMAGE_WIDTH/2);
const unsigned short ImageSensorCentorY = (CAM_IMAGE_HEIGHT/2);
// Constants for lens
const float lens_ratio_x = (CAM_LENS_RATIO*CAM_IMAGE_WIDTH);
const float lens_ratio_y = (CAM_LENS_RATIO*CAM_IMAGE_HEIGHT);

class Point3D
{
 public:
  Point3D() { _x = 0; _y = 0; _z = 0;}
  Point3D(float x, float y, float z) : _x(x), _y(y), _z(z) {}
  Point3D(const Point3D& p) : _x(p._x), _y(p._y), _z(p._z) {}
  virtual ~Point3D() {}
  const float ex() { return _x;}
  const float ey() { return _y;}
  const float ez() { return _z;}
  static const float dsq(const Point3D p, const Point3D q) {
    return ((p._x-q._x)*(p._x-q._x)
	    + (p._y-q._y)*(p._y-q._y)
	    + (p._z-q._z)*(p._z-q._z));
  }
 private:
  float _x, _y, _z;
};

class ImageSensorPoint
{
 public:
  ImageSensorPoint() : _ix(0), _iy(0) {}
  ImageSensorPoint(const unsigned short x, const unsigned short y)
    {
      _ix = (x-ImageSensorCentorX)/(float)ImageSensorCentorX * lens_ratio_x;
      _iy = (ImageSensorCentorY-y)/(float)ImageSensorCentorY * lens_ratio_y;
    }
  ImageSensorPoint(const ImageSensorPoint& p) : _ix(p._ix), _iy(p._iy) {}
  virtual ~ImageSensorPoint() {}
  const float ex() { return _ix;}
  const float ey() { return _iy;}
 private:
  float _ix, _iy;
};

class VisualYawEstimater
{
 public:
  VisualYawEstimater() : _xhat(0), _p(Q), _u(0), _initialized(false) {}
  virtual ~VisualYawEstimater() {}
  // predictor step
  void predict()
  {
    _xhat_prev = _xhat;
    _xhat = _xhat + B*_u;
    _p = _p + Q;
  }
  // corrector step
  float correct(float yaw_meas)
  {
    float K = H*_p/(H*H*_p + R);
    _xhat = _xhat + K*(yaw_meas - H*_xhat);
    _p = _p*(1 - H*K);
    _u = _xhat - _xhat_prev;
    return _xhat;
  }
  float estimate_visual_yaw(Point3D fm[]);

 private:
  const float Q = 0.0001;
  const float R = 0.1;
  const float H = 1.0;
  const float B = 0.5;
  float _xhat;
  float _xhat_prev;
  float _p;
  float _u;
  bool _initialized;
};

const int num_frame_markers = 4;

bool unfish(const float ix, const float iy, const float height,
	    float& hx, float& hy);

int find_frame(std::vector<ImageSensorPoint>& m, float h, Point3D fm[],
	       bool prev, float& herr);

void
adjust_frame_center(Point3D fm[], float& sx, float& sy, float& sz,
		    float estimated_yaw);

#endif
