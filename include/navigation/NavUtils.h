#ifndef __OUTDOOR_BOT_NAV_UTILS_H__
#define __OUTDOOR_BOT_NAV_UTILS_H__

namespace OutdoorBot
{
namespace Navigation
{

double wrapAngle(double angle);
double wrapAngle2Pi(double angle);
double angularDistance(double angle1, double angle2);

}  // namespace Navigation
}  // namespace OutdoorBot

#endif  // __OUTDOOR_BOT_NAV_UTILS_H__
