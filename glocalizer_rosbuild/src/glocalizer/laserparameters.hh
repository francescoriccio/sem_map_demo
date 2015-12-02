#ifndef LASERPARAMETERS_HH
#define LASERPARAMETERS_HH

#include "localizemap.hh"
#include <vector>

typedef Pose2<double> DPose2;
typedef Transformation2<double> DTransformation2;

struct LaserParameters{
  LaserParameters(int beams, double firstBeamAngle, double angularStep, double maxRange);
  DTransformation2 laserPose;
  std::vector<DTransformation2> beams;
  double maxRange;
  double minRange;
};

#endif
