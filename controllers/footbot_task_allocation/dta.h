#ifndef DTA_H
#define DTA_H

#include "task_allocation.h"

using namespace argos;

class DTA : public TaskAllocation {
public:
   void Init(int robotId, int numOfRobots, int sumOfWorkload, int numOfGroups, double dThreshold, std::vector<SGroupInfo> &candidateGroups);
   void Reset();
   int GetGroup();
   int GetNeighborNum();
   void SelectGroup();
   void ShareDecisions(int dataIndex, CCI_RangeAndBearingActuator* pcRABA, CCI_RangeAndBearingSensor* pcRABS);
   bool StopCheck();
#pragma pack(push, 2)
   struct SDecision {
      unsigned int unRobotId;
      unsigned int unCount;
      short sGroupId;
   };
#pragma pack(pop)
private:
   double GetTaskProb();
   std::vector<double> GetTaskRatio();
   double GetStopProb();
   std::vector<SDecision> m_vsSharedData;
   int m_nWorkloadSum;
   int m_nGroupingIteration;
   int m_nNumOfGroups;
   double m_dThreshold;
};
#endif /* DTA_H */
