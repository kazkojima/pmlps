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
	  alpha = yaw_angle;
	  pthread_mutex_unlock(&mavmutex);
	  // Return yaw_angle if it looks unstable.
	  if (fabsf(yaw_angle - prev_yaw_angle) > 0.05)
	    {
	      prev_yaw_angle = yaw_angle;
	      return yaw_angle;
	    }
	  _initialized = true;
	  alpha = alpha + CAM_DIRECTION;
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
  if (x * hx + y * hy > 0)
    yaw_meas = atan2f(y, x);
  else
    yaw_meas = atan2f(-y, -x);
  // Uncover
  float fn = roundf((prev_yaw - yaw_meas)/(2*M_PI));
  yaw_meas = yaw_meas + 2*M_PI*fn;
  // Predict and Update
  predict();
  float yaw = correct(yaw_meas);
  prev_yaw = yaw;
  yaw = -(yaw + CAM_DIRECTION);
  // Proj to [-pi,pi]
  fn = floorf(yaw / (2*M_PI));
  yaw = yaw - 2*M_PI*fn - M_PI;

  return yaw;
}
