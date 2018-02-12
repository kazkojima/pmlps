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
  yaw_meas = atan2f(y, x);

  // Adjust with attitude
  // Spherical triangle version. Rewrite with versor!
  // TODO: Don't old pitch_angle.
  float sx = (fm[0].ex() + fm[1].ex())/2;
  float sy = (fm[0].ey() + fm[1].ey())/2;
  float sz = (fm[0].ez() + fm[1].ez())/2;
  float s = sz/sqrtf(sx*sx+sy*sy+sz*sz);
  float c = sqrtf(1-s*s);

  if (cosf(pitch_angle) > c && s > 0.2 && fabsf(sx*y - sy*x) > 100.0)
    {
      float salpha = c/cosf(pitch_angle);
      float calpha = sqrtf(1-salpha*salpha);
      float ctau = calpha/s;
      float tau = acosf(ctau);
      if (pitch_angle < 0)
	tau = -tau;
      if (sx*y - sy*x < 0)
	tau = -tau;
      //printf("tau %3.3f p %3.3f i %6.1f\n", tau, pitch_angle, sx*y - sy*x);
      yaw_meas += tau;
    }

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
