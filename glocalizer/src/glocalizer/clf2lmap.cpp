#include "localizemap.hh"
#include "gridlinetraversal.hh"
#include "laserparameters.hh"
#include "localizer.hh"
#include <cstring>
#include <istream>
#include <fstream>
#include <sstream>
#include <list>

using namespace std;

const char *message[]={
  "pgm2lmap: builds a localization map out of a carmen log file",
  "usage clf2lmap [options] <pgm_file> <lmap_file>",
  "options:",
  " -debug                               : dumps images \"omap.pgm\" and \"dmap.pgm\" showing the generated",
  "                                        gridmap and the distance map",
  " -res    <meters>                     : the resolution of a grid cell",
  "                                        default [0.1m]",
  " -maxrange    <meters>                : the maximum range of the scanner",
  " -usablerange <meters>                : the maximum usable range of the scanner",
  " -border      <meters>                : add a border of +- x meters to the generated map",
  " -offset <x_meters> <y_meters> <theta_radians>",
  "                                       : the offset of the robot when taking the initial scan",
  "                                        default (0, 0, 0)",
  " -full <occval>                       : if the occupancy of a cell is above this value",
  "                                        it will be considered full in the generation of the distance map",
  " -dmapDist <meters>                   : distance up to which propagate the distances in the distance map",
  0
};

enum Type{RobotLaser=0, Imu=1};

struct Data{
  Type type;
  double timestamp;
  virtual ~Data(){}
};

struct LaserScan: public Data{
  std::vector<double> ranges;
  DPose2 odomPose, laserPose;
  LaserParameters laserParams;
  double timestamp;
  LaserScan():laserParams(180, -M_PI/2, M_PI/180., 50.){
    type=RobotLaser;
  }
};

struct IMUMeasurement: public Data{
  IMUMeasurement(){type=RobotLaser;}
  double ax, ay, az; //accelerations 
  double q0, q1, q2, q3; // //overall heading
  double mx, my, mz; // magnetic field
  double gx, gy, gz; //gyros
  double yaw;
};

Data* readLogLine(istream& log){
#define BUFSIZE 20000
  char line[BUFSIZE];
  log.getline(line, BUFSIZE);
  istringstream ss(line);
  string tag;
  ss>> tag;

  int type;
  double angle, fov, res, maxrange, acc, remission_mode;
  bool newlaser=false;
  
  bool laserFound=false;
  bool imuFound=false;
  if (tag=="ROBOTLASER1"){
    newlaser=true;
    ss >> type >> angle >> fov >> res >> maxrange >> acc >> remission_mode;
      
    laserFound=true;
  } 
  if (tag=="IMU")
    imuFound=true;

  if (tag=="FLASER"){
    laserFound=true;
  }
  

  if (laserFound) {
    LaserScan *pscan=new LaserScan;
    LaserScan& scan(*pscan);
    int beams;
    ss >> beams;
  
    if (newlaser){
      scan.laserParams=LaserParameters(beams, angle, res, maxrange);      
    } else {
      if (beams==180 ||  beams==181){
	scan.laserParams=LaserParameters(beams, -M_PI/2, M_PI/180., 50.);      
      } else if (beams==360 || beams ==361){
	scan.laserParams=LaserParameters(beams, -M_PI/2, M_PI/360., 50.);      
      } else {
	cerr << "unknown scanner model" << endl;
	return false;
      }
    }
  
  
    scan.ranges.resize(beams);
    for (int i=0; i<beams; i++){
      ss >> scan.ranges[i];
    }
    
    if (tag=="ROBOTLASER1"){
      int rems;
      ss >> rems;
      for (int i=0; i<rems; i++){
	double z;
	ss >> z;
      }
    }
    
    double x,y,theta;
    ss >> x >> y >> theta;
    scan.odomPose=DPose2(x,y,theta);
    ss >> x >> y >> theta;
    scan.laserPose=DPose2(x,y,theta);
    ss >> scan.timestamp;
    return pscan;
  } 
  if (imuFound){
    IMUMeasurement *pimu=new IMUMeasurement;
    IMUMeasurement& imu(*pimu);
    log >> imu.ax >> imu.ay >> imu.az >> imu.q0 >> imu.q1 >> imu.q2 >> imu.q3 >> imu.mx >> imu.my >> imu.mz >> imu.gx >> imu.gy >> imu.gz;
    imu.yaw = getYaw(imu.q0, imu.q1, imu.q2, imu.q3);
    return pimu;
  }
  return 0;
}


void boundaries(DVector2& min, DVector2& max, DPose2 offset, double maxrange, const LaserScan& scan){
  DTransformation2 tp=DTransformation2(offset)*DTransformation2(scan.laserPose)*DTransformation2(scan.laserParams.laserPose);
  
  //robot pose;
  DVector2 rp=tp.translation();
  min.x()=min.x()<rp.x()?min.x():rp.x();
  min.y()=min.y()<rp.y()?min.y():rp.y();
  max.x()=max.x()>rp.x()?max.x():rp.x();
  max.y()=max.y()>rp.y()?max.y():rp.y();
  for (int i=0; i<(int)scan.ranges.size(); i++){
    double r=scan.ranges[i];
    if (r>=maxrange)
      continue;
    
    DVector2 bp(r,0);
    bp=scan.laserParams.beams[i]*bp;
    bp=tp*bp;;
    min.x()=min.x()<bp.x()?min.x():bp.x();
    min.y()=min.y()<bp.y()?min.y():bp.y();
    max.x()=max.x()>bp.x()?max.x():bp.x();
    max.y()=max.y()>bp.y()?max.y():bp.y();
  }
}

void integrateScan(LocalizeMap& lmap, DPose2 offset, double maxrange, double usableRange, const LaserScan& scan){
  DTransformation2 tp=DTransformation2(offset)*DTransformation2(scan.laserPose)*DTransformation2(scan.laserParams.laserPose);
  
  //robot pose;
  DVector2 rp=tp.translation();
  IVector2 start=lmap.world2map(rp);
  for (int i=0; i<(int)scan.ranges.size(); i++){
    double r=scan.ranges[i];
    if (r>=maxrange)
      continue;
    
    bool cropped=false;
    if (r>usableRange){
      r=usableRange;
      cropped=true;
    }
    static GridLineTraversalLine line;
    DVector2 bp(r,0);
    bp=scan.laserParams.beams[i]*bp;
    bp=tp*bp;;

    IVector2 end=lmap.world2map(bp);

    GridLineTraversal::gridLine(start, end, &line);
    for (int i=0; i<line.num_points; i++){
      lmap.cell(line.points[i]).distance+=1.;
    }
    if (! cropped)
      lmap.cell(end).occupancy+=1.;
  }
}

int main (int argc, const char ** argv){

  if (argc<2){
    const char**v=message;
    while (*v){
      cout << *v << endl;
      v++;
    }
    return 0;
  }
  double maxrange=20;
  double usablerange=20;
  double resolution=0.1;
  double off_x=0., off_y=0., off_theta=0.;
  bool debug=false;
  double occFactor=5;
  double distance_threshold=LOCALIZE_MAP_DISTANCE_THRESHOLD;
  double full_threshold=LOCALIZE_MAP_DISTANCE_FULL_CELL_THRESHOLD;
  const char* logfile=0;
  const char* mapfile=0;
  double border=10;
  
  int c=1;
  while (c<argc){
    if (!strcmp(argv[c],"-debug")){
      debug=true;
      c++;
    } else
    if (!strcmp(argv[c],"-res")){
      c++;
      resolution=atof(argv[c]);
      c++;
    } else 
    if (!strcmp(argv[c],"-offset")){
      c++;
      off_x=atof(argv[c]);
      c++;
      off_y=atof(argv[c]);
      c++;
      off_theta=atof(argv[c]);
      c++;
    } else
    if (!strcmp(argv[c],"-occFactor")){
      c++;
      occFactor=atof(argv[c]);
      c++;
    } else
    if (!strcmp(argv[c],"-maxrange")){
      c++;
      maxrange=atof(argv[c]);
      c++;
    } else
    if (!strcmp(argv[c],"-usablerange")){
      c++;
      usablerange=atof(argv[c]);
      c++;
    } else
    if (!strcmp(argv[c],"-border")){
      c++;
      usablerange=atof(argv[c]);
      c++;
    } else
    if (!strcmp(argv[c],"-full")){
      c++;
      full_threshold=atof(argv[c]);
      c++;
    } else
    if (!strcmp(argv[c],"-dmapDist")){
      c++;
      distance_threshold=atof(argv[c]);
      c++;
    } else
    if (! logfile){
      logfile=argv[c];
      c++;
    } else {
      mapfile=argv[c];
      break;
    }
  }

  if (! logfile){
    cout << "pgm2lmap: image file not specified" << endl;
    cout << "          Run the program without arguments for the help banner.";
    return 0;
  }

  if (! logfile){
    cout << "pgm2lmap: map file not specified" << endl;
    cout << "          Run the program without arguments for the help banner.";
    return 0;
  }

  cout << "clf2lmap, generating map from file "<< logfile << " to file " << mapfile << endl;
  cout << "Parameters: " << endl;
  cout << " resolution=" << resolution << endl;
  cout << " offset    =" << off_x << " " << off_y << off_theta << endl;
  cout << " full      =" << full_threshold<< endl;
  cout << " dmapDist  =" << distance_threshold<< endl;
  cout << " maxrange  =" << maxrange << endl;
  cout << " usablerange  =" << usablerange  << endl;
  cout << " occFactor  =" << occFactor  << endl;


  DPose2 offset(off_x,off_y,off_theta);
  
  ifstream is(logfile);
  if (! is)
    return 0;
  LaserScan scan;
  std::list<Data*> log;
  while (is){
    Data* data=readLogLine(is);
    if (data){
      log.push_back(data);
      cerr << "*";
    }
  }
  
  cerr << "computing boundaries.." << endl;
  DVector2 min(MAXDOUBLE,MAXDOUBLE), max(-MAXDOUBLE, -MAXDOUBLE);
  for ( std::list<Data*>::const_iterator it=log.begin(); it!=log.end(); it++){
    const LaserScan * pscan= dynamic_cast<LaserScan *>(*it);
    if (! pscan)
      continue;
    const LaserScan& scan(*pscan); 
    boundaries(min, max, offset, maxrange, scan);
  }
  cerr << "Boundaries= " << min.x() << " " << min.y() << " " << max.x() << " " << max.y() << endl;
  
  min=min-DVector2(border, border);
  max=max+DVector2(border, border);
  cerr << "Extended Boundaries= " << min.x() << " " << min.y() << " " << max.x() << " " << max.y() << endl;


  DVector2 dsize=max-min;
  IVector2 isize((int)(dsize.x()/resolution), (int)(dsize.y()/resolution));
  LocalizeMapCell unknown;
  unknown.occupancy=0;
  unknown.distance=0;
  unknown.visited=false;
  cerr << "Allocating map, size : " << isize.x() << " x " << isize.y() << " [pixels] , " <<
    dsize.x() << " x " << dsize.y() << " [m] " <<  endl;
  cerr << "Origin : " << -min.x()/resolution << " x " << isize.y() + min.y()/resolution << endl;
  
  LocalizeMap lmap(isize, resolution, min, unknown);

  DVector2 pc = lmap.map2world(IVector2(isize.x()/2,isize.y()/2));
  
  cerr << "Origin player : " << pc.x() << " , " << pc.y() << " [m]" << endl;
  
  cerr << "Lines to add to Stage world: " << endl << endl;  
  cerr << "   size [  " << dsize.x() << " " <<  dsize.y() << " ] \n" <<
          "   origin  [ " << pc.x() << " " << pc.y() << " 0 ]\n" << endl;

  cerr << "World file \n\n" <<
  resolution << "\n0.0\n0.0\n" << -resolution << "\n" <<
          min.x() << "\n" << max.y() << "\n" << endl;

  cerr << "integrating scans ... " << endl;
  
  double snorth=0, cnorth=1;
  bool takeImu=false;
  bool foundIMU=false;
  DTransformation2 previousPose;
  for ( std::list<Data*>::const_iterator it=log.begin(); it!=log.end(); it++){
    const LaserScan * pscan= dynamic_cast<LaserScan *>(*it);
    if (pscan){
      const LaserScan& scan(*pscan); 
      integrateScan(lmap, offset, maxrange, usablerange, scan);
      cerr << "*";
      takeImu=true;
      previousPose=DTransformation2(offset)*DTransformation2(scan.laserPose);
      continue;
    }
    const IMUMeasurement * pimu= dynamic_cast<IMUMeasurement *>(*it);
    if (pimu&&takeImu){
      foundIMU=true;
      const IMUMeasurement& imu(*pimu);
      DPose2 p=previousPose.toPoseType();
      double robotHeading=p.theta();
      double delta=robotHeading-imu.yaw;
      snorth+=sin(delta);
      cnorth+=cos(delta);
      takeImu=false;
      continue;
    }
  }

  cerr << "generating occupancy map " << endl;
  for (int x=0; x<lmap.size.x(); x++)
    for (int y=0; y<lmap.size.y(); y++){
      IVector2 p(x,y);
      double occ=occFactor*lmap.cell(p).occupancy;
      double visit=occ+lmap.cell(p).distance;
      if (visit<=0){
	lmap.cell(p).visited=false;
	lmap.cell(p).distance=0;
	lmap.cell(p).occupancy=0;
      } else {
	lmap.cell(p).occupancy=occ/visit;
	lmap.cell(p).distance=0;
	lmap.cell(p).visited=true;
      }
    }
  if (foundIMU==true)
    lmap.north()=atan2(snorth, cnorth);
  lmap.distanceMap(distance_threshold, full_threshold);
  ofstream os (mapfile);
  os << lmap;
  os.close();
  if (debug){
    os.open("clf2lmap_omap.pgm");
    lmap.saveToPGM(os);
    os.close();
    os.open("clf2lmap_dmap.pgm");
    lmap.saveDistanceMapToPGM(os, false);
    os.close();
  }
  return 0;
}
