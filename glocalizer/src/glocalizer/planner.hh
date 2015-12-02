#ifndef _PLANNER_HH_
#define _PLANNER_HH_

#include "localizemap.hh"
#include <list>
#include <vector>
#include <deque>
#include <algorithm>

typedef std::list<LocalizeMapCell*> LocalizeCellList;

struct PlannerParameters{
  double robotRadius;
  double obstacleGain;
  double distanceGain;
  PlannerParameters();
};


struct Planner{
  Planner(PlannerParameters* params, LocalizeMap* map); 
  bool computePath(const DVector2& start, const DVector2& goal, LocalizeCellList& path);
  virtual double heuristic(LocalizeMapCell* current, LocalizeMapCell* goal);
  virtual ~Planner();
protected:
  std::vector<LocalizeMapCell*> _markedCells;
  PlannerParameters* _params;
  LocalizeMap* _lmap;

};


#endif
