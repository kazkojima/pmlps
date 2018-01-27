#ifndef _PMLPS_H
#define _PMLPS_H
#include <vector>

// Constants for QVGA
const unsigned short ImageSensorCentorX = 160;
const unsigned short ImageSensorCentorY = 120;
// Constants for lens
const float lens_ratio_x = 0.48;
const float lens_ratio_y = 0.36;

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

const int num_frame_markers = 4;

bool unfish(const float ix, const float iy, const float height,
	    float& hx, float& hy);

int find_frame(std::vector<ImageSensorPoint>& m, float h, Point3D fm[]);

#endif
