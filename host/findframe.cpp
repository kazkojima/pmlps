#include <cstdio>
#include <cmath>

#include "pmlps.h"

// (frame size)^2 * 0.3 ???
static const float frame_eplsilon = 30.0;
static const float size_sq_min = 64.0 * 0.6;
static const float size_sq_max = 64.0 * 1.4;

int
find_frame(std::vector<ImageSensorPoint>& m, float h, Point3D fm[])
{
  size_t n = m.size();
  std::vector<Point3D> um(n);
  int np = 0;

  // unfish all
  for(int i = 0; i < n; i++)
    {
      float x, y;
      unfish(m[i].ex(), m[i].ey(), h, x, y);
      um[i] = Point3D(x, y, h);
    }
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
		  np = 3;
		  break;
		}
	    }
	}

  if (np == 3)
    {
      // Not yet
    }

  return np;
}
