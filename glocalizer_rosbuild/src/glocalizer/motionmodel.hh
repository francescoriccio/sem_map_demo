#ifndef MOTIONMODEL_HH
#define MOTIONMODEL_HH

#include "localizemap.hh"
typedef Pose2<double> DPose2;
typedef Transformation2<double> DTransformation2;

struct MotionModel{
  /*noise parameters:
    forward forward;
    forward sideward;
    forward rotational;
    sideward sideward;
    sideward rotational;
    rotational rotational;
  */
  double ff, fs, fr, ss, sr, rr;
  
  /*non motion dependent noise,
    used for static convergence*/

  double sx, sy, sth;

  /*width of the triangular distribution*/
  double wx, wy, wth;

  void prepareSampling(const DPose2& motion);
  DPose2 sampleMotion(const DPose2& motion) const;
};


#endif
