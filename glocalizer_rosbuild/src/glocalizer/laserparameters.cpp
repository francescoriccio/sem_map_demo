#include "laserparameters.hh"

LaserParameters::LaserParameters(int nbeams, double firstBeamAngle, double angularStep, double maxRang){
  laserPose=DTransformation2(0.,0.,0.);
  maxRange=maxRang;
  minRange=0.;
  if (nbeams>0){
    beams.resize(nbeams);
    double alpha=firstBeamAngle;
    for (int i=0; i<nbeams; i++){
      beams[i]=DTransformation2(0,0,alpha);
      alpha+=angularStep;
    }
  }
}
