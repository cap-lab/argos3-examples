#ifndef STA_H
#define STA_H

#include "task_allocation.h"

using namespace argos;

class STA : public TaskAllocation {
public:
   void Init(int robotId, int numOfRobots, int sumOfWorkload, int numOfGroups, double dThreshold, std::vector<SGroupInfo> &candidateGroups);
   void Reset();
   int GetGroup();
   int GetNeighborNum();
   void SelectGroup();
   void ShareDecisions(CCI_RangeAndBearingActuator* pcRABA, CCI_RangeAndBearingSensor* pcRABS);
   bool StopCheck();
private:
};
#endif /* STA_H */
