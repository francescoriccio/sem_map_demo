#include <iostream>
#include "transformation3.hh"

using namespace std;

typedef Operations3D<double>::PoseType           Pose;
typedef Operations3D<double>::RotationType       Rotation;
typedef Operations3D<double>::TranslationType    Translation;
typedef Operations3D<double>::TransformationType Transformation;
typedef Operations3D<double>::CovarianceType     CovarianceMatrix;
typedef Operations3D<double>::InformationType    InformationMatrix;


int main (int argc, const char** argv){
  
  Translation t1(1,2,3);
  Translation r1(0.4,0.5,0.6);

  Pose        p1(t1, r1);

  cerr << "initial configuration" << endl;
  Pose pp=p1;
  cerr << pp.x() << " " << pp.y() << " " <<  pp.z() << " " << pp.roll() << " " << pp.pitch() << " " << pp.yaw() << endl;

  
  Transformation tr1(p1);
  cerr << "recovering RPY angles " << endl;
  
  pp=tr1.toPoseType();
  cerr << pp.x() << " " << pp.y() << " " <<  pp.z() << " " << pp.roll() << " " << pp.pitch() << " " << pp.yaw() << endl;

  Transformation i_tr1=tr1.inv();
  cerr << "inverse" << endl;
  pp=i_tr1.toPoseType();
  cerr << pp.x() << " " << pp.y() << " " << pp.z() << " " << pp.roll() << " " << pp.pitch() << " " << pp.yaw() << endl;

  
  Transformation zero=tr1*i_tr1;
  pp=zero.toPoseType();


  cerr << "zero" << endl;
  pp=zero.toPoseType();
  cerr << pp.x() << " " << pp.y() << " " << pp.z() << " " << pp.roll() << " " << pp.pitch() << " " << pp.yaw() << endl;


  Transformation tr1_aa=tr1;
  Rotation r=tr1.rotation();
  tr1_aa.setRotation(r);
  pp=tr1_aa.toPoseType();
  cerr << pp.x() << " " << pp.y() << " " << pp.z() << " " << pp.roll() << " " << pp.pitch() << " " << pp.yaw() << endl;

  cerr << "chaining" << endl;
  Transformation tcum(0,0,0,0,0,0);
  Transformation itcum(0,0,0,0,0,0);

  for (int i=0; i<10; i++){
   tcum=tcum*tr1;
   itcum=i_tr1*itcum;
  }
  
  zero=tcum*itcum;
  pp=zero.toPoseType();
  cerr << pp.x() << " " << pp.y() << " " << pp.z() << " " << pp.roll() << " " << pp.pitch() << " " << pp.yaw() << endl;
  
  cerr << "translationChaining" << endl;
  Transformation tc=tr1;
  tc.setRotation(Rotation());
  tcum=Transformation(0,0,0,0,0,0);
  for (int i=0; i<10; i++){
   tcum=tcum*tc;
  }
  pp=tcum.toPoseType();
  cerr << pp.x() << " " << pp.y() << " " << pp.z() << " " << pp.roll() << " " << pp.pitch() << " " << pp.yaw() << endl;


 }

