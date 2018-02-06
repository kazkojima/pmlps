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
estimate_visual_yaw(Point3D fm[])
{
  static float prev_yaw = 0.0;
  // Assume MARKER_TYPE_I. TODO for H.
  float x = fm[0].ex() - fm[1].ex();
  float y = fm[0].ey() - fm[1].ey();
  pthread_mutex_lock(&mavmutex);
  if (update_attitude)
    {
      float alpha = yaw_angle;
      pthread_mutex_unlock(&mavmutex);
      alpha += CAM_DIRECTION;
      float hx = cosf(alpha);
      float hy = -sinf(alpha);
      float yaw;
      if (x * hx + y * hy > 0)
	yaw = atan2f(y, x);
      else
	yaw = atan2f(-y, -x);
      yaw = -(yaw + CAM_DIRECTION);
      if (yaw < -M_PI)
	yaw += 2*M_PI;
      prev_yaw = yaw;
      update_attitude = false;
      return yaw;
    }
  else
    pthread_mutex_unlock(&mavmutex);
  // Not quite right
  return prev_yaw;
}
