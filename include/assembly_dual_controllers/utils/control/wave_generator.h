#include <Eigen/Dense>
#include <assembly_dual_controllers/utils/dyros_math.h>

/**
 * t: current time
 * s: 1 cycle time
 * usgae: a = magnitutde * cubicWaveGen(t,s);
*/
static double cubicWaveGen(double t, double s)
{
  if(t < s / 4)
    return dyros_math::cubic(t, 0, s/4, 0, 1.0, 0, 0);
  return sin(2 * M_PI * t / s);
}