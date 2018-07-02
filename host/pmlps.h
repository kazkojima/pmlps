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
#include <queue>
#include <cstdint>

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

class ImageSensorBlob
{
 public:
  ImageSensorBlob() : _ix(0), _iy(0), _iw(0), _ih(0) {}
  ImageSensorBlob(const unsigned short x, const unsigned short y,
		  const unsigned short w, const unsigned short h)
    {
      unsigned short cx = config.cam_image_width/2;
      unsigned short cy = config.cam_image_height/2;
      float lens_ratio = config.cam_lens_ratio;

      _ix = (float)(x-cx) * lens_ratio;
      _iy = (float)(cy-y) * lens_ratio;
      _iw = (float)w * lens_ratio;
      _ih = (float)h * lens_ratio;
    }
  ImageSensorBlob(const ImageSensorBlob& p) :
  _ix(p._ix), _iy(p._iy), _iw(p._iw), _ih(p._ih) {}
  virtual ~ImageSensorBlob() {}
  const float ex() { return _ix;}
  const float ey() { return _iy;}
  const float sw() { return _iw;}
  const float sh() { return _ih;}
 private:
  float _ix, _iy, _iw, _ih;
};

class EstimatedPosition
{
 public:
  EstimatedPosition() : _px(0), _py(0), _pz(0), _yaw(0), _t(0) {}
  EstimatedPosition(float x, float y, float z, float yaw, uint64_t time) :
   _px(x), _py(y), _pz(z), _yaw(yaw), _t(time) {}
  virtual ~EstimatedPosition() {}
  const float px() { return _px;}
  const float py() { return _py;}
  const float pz() { return _pz;}
  const float yaw() { return _yaw;}
  const uint64_t time() { return _t;}
 private:
  float _px, _py, _pz;
  float _yaw;
  uint64_t _t;
};

class VisualYawEstimater
{
 public:
  VisualYawEstimater() : _xhat(0), _p(Q), _u(0), _ct(0), _initialized(false) {}
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
  float estimate_visual_yaw(Point3D fm[], bool& suc);
  void dump(float meas);

 private:
  const float Q = 0.0001;
  const float R = 0.1;
  const float H = 1.0;
  const float B = 0.8;
  float _xhat;
  float _xhat_prev;
  float _p;
  float _u;
  int _ct;
  bool _initialized;
};

const int num_frame_markers = 4;

bool unfish(const float ix, const float iy, const float height,
	    float& hx, float& hy);

bool fish(const float x, const float y, const float z, int& ix, int& iy,
	  int& rho);

int find_frame(std::vector<ImageSensorBlob>& m, float h, Point3D fm[],
	       bool prev, float& herr);

int find_frame_i3(std::vector<ImageSensorBlob>& m, float h, Point3D fm[],
		  bool prev, float& herr);

void
adjust_frame_center(Point3D fm[], float& sx, float& sy, float& sz,
		    float estimated_yaw);
#endif
