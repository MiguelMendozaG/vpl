#include "nbvplanner.h"
#include <viewsynthesis.h>
#include "workspacenbvplanner.h"
#include <vector>
#include <string>
#include <sstream>

using namespace std;

class TrainingPlanner : public  WorkspaceNBVPlanner 
{
public:
TrainingPlanner(RobotSensor* rs, PartialModelBase* pm);

virtual bool planNBV(ViewStructure &v);

virtual bool init();

protected:
  void vistas_dirigidas(std::vector< std::vector< double > > points, ViewList& views);
  float radius, cx, cy, cz;
  double compareClouds(std::vector< std::vector< double > > a, std::vector< std::vector< double > > reference, double gap);
};
