#include <algorithm>  // for std::min/max
#include <math.h>


#include "navigation/NavUtils.h"

namespace OutdoorBot
{
namespace Navigation
{

double wrapAngle(double angle)
{
  while (angle < -M_PI)
  {
    angle += 2.0 * M_PI;
  }
  while (angle > M_PI)
  {
    angle -= 2.0 * M_PI;
  }
  return angle;
}

double wrapAngle2Pi(double angle)
{
  while (angle < 0)
  {
    angle += 2.0 * M_PI;
  }
  while (angle > 2.0 * M_PI)
  {
    angle -= 2.0 * M_PI;
  }
  return angle;
}

double angularDistance(double angle1, double angle2)
{
  angle1 = wrapAngle2Pi(angle1);
  angle2 = wrapAngle2Pi(angle2);

  return std::min(fabs(wrapAngle2Pi(angle1 - angle2)), fabs(wrapAngle2Pi(angle2 - angle1)));
}

}  // namespace Navigation
}  // namespace OutdoorBot
