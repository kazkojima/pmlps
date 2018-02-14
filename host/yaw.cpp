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

#include <pthread.h>

using namespace vsr;
using namespace vsr::cga;

extern pthread_mutex_t mavmutex;
extern float roll_angle, pitch_angle, yaw_angle;
extern bool update_attitude;

#define YAW_INITIALIZE_COUNT 40

// Estimate yaw from markers and mavlink yaw.
float
VisualYawEstimater::estimate_visual_yaw(Point3D fm[])
{
  static float prev_yaw = 0.0;
  static float prev_yaw_angle = 10.0; // Invalid angle
  // Assume MARKER_TYPE_I. TODO for H.
  float x = fm[0].ex() - fm[1].ex();
  float y = fm[0].ey() - fm[1].ey();
  float alpha;

  //printf("x y %3.3f %3.3f\n", x, y);
  if (!_initialized)
    {
      pthread_mutex_lock(&mavmutex);
      if (update_attitude)
	{
	  alpha = -(yaw_angle + CAM_DIRECTION);
	  //printf("angle %3.3f alpha %3.3f\n", yaw_angle, alpha);
	  pthread_mutex_unlock(&mavmutex);
	  prev_yaw = alpha;
	  // Return yaw_angle if it looks unstable.
	  if (fabsf(yaw_angle - prev_yaw_angle) > 0.05)
	    {
	      prev_yaw_angle = yaw_angle;
	      return yaw_angle;
	    }
	  if (++_ct > YAW_INITIALIZE_COUNT)
	    _initialized = true;
	}
      else
	{
	  pthread_mutex_unlock(&mavmutex);
	  // Not right
	  return 0.0;
	}
    }
  else
    alpha = prev_yaw;

  float hx = cosf(alpha);
  float hy = sinf(alpha);
  float yaw_meas;
#ifdef DEBUG
  printf("visual_yaw: hx, hy %3.3f %3.3f x, y %3.3f %3.3f\n", hx, hy, x, y);
#endif

  if (x * hx + y * hy <= 0)
    {
      x = -x;
      y = -y;
    }

  // Adjust with attitude
  // TODO: Don't old pitch_angle.
  float cx = (fm[0].ex() + fm[1].ex())/2;
  float cy = (fm[0].ey() + fm[1].ey())/2;
  float cz = (fm[0].ez() + fm[1].ez())/2;
#if 0
  // Spherical triangle version.
  float s = cz/sqrtf(cx*cx+cy*cy+cz*cz);
  float c = sqrtf(1-s*s);

  float tau = 0.0;
  if (cosf(pitch_angle) > c && s > 0.2 && fabsf(cx*y - cy*x) > 100.0)
    {
      float salpha = c/cosf(pitch_angle);
      float calpha = sqrtf(1-salpha*salpha);
      float ctau = calpha/s;
      tau = acosf(ctau);
      if (pitch_angle < 0)
	tau = -tau;
      if (cx*y - cy*x < 0)
	tau = -tau;
      //printf("tau %3.3f p %3.3f i %6.1f\n", tau, pitch_angle, cx*y - cy*x);
    }
  yaw_meas = atan2f(y, x);
  yaw_meas += tau;
#else
  // GA version.
  float nv = sqrtf(x*x+y*y);
  Vec v = Vec(x/nv, y/nv, 0);
  Vec w = Vec(y/nv, -x/nv, 0);
  auto bi = v ^ Vec(0, 0, 1);
  auto rop = Gen::rot(bi*(-pitch_angle/2));
  //v.sp(rop).print();
  float nu = sqrtf(cx*cx+cy*cy+cz*cz);
  Vec u = Vec(cx/nu, cy/nu, -cz/nu);
  auto PointO = PT(0, 0, 0);
  auto dlp0 = (PointO ^ (v ^ u) ^ Inf(1)).dual();
  //  One less line. Not exact but might be enough.
  //  auto dlp1 = (PointO ^ (w ^ (v.sp(rop))) ^ Inf(1)).dual();
  //  auto direc = (dlp0 ^ dlp1);
  //  yaw_meas = atan2f(-direc[1], direc[2]);
  auto cir = v.sp(rop).null() ^ Biv::xy;
  auto pp = dlp0 <= cir;
  auto pp0 = Round::split(pp, true);
  yaw_meas = atan2f(pp0[1], pp0[0]);
  //printf("adjusted yaw_meas %3.3f\n", yaw_meas);
#endif

  // Uncover
  float fn = roundf((prev_yaw - yaw_meas)/(2*M_PI));
  yaw_meas = yaw_meas + 2*M_PI*fn;
#ifdef DEBUG
  printf("visual_yaw: prev: %3.3f meas: %3.3f\n", prev_yaw, yaw_meas);
#endif
  // Predict and Update
  predict();
  float yaw = correct(yaw_meas);
  if (_initialized)
    prev_yaw = yaw;
  yaw = yaw + CAM_DIRECTION;
  //printf("est yaw: %3.3f\n", yaw);
  // Proj to [-pi,pi]
  fn = roundf(yaw / (2*M_PI));
  yaw = -(yaw - 2*M_PI*fn);
  if (yaw > M_PI)
    yaw -= 2*M_PI;
  if (yaw < -M_PI)
    yaw += 2*M_PI;
  return yaw;
}
