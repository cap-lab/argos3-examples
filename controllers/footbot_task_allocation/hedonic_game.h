#ifndef HEDONIC_H
#define HEDONIC_H

#include "task_allocation.h"

using namespace argos;

class Hedonic : public TaskAllocation {
public:
   void Init(int robotId, int numOfRobots, int sumOfWorkload, int numOfGroups, double dThreshold, std::vector<SGroupInfo> &candidateGroups);
   void Reset();
   int GetGroup();
   int GetNeighborNum();
   void SelectGroup();
   void ShareDecisions(CCI_RangeAndBearingActuator *pcRABA, CCI_RangeAndBearingSensor *pcRABS);
   bool StopCheck();
#pragma pack(push, 2)
   struct SDecision {
      unsigned short usMessageId;
      unsigned short iteration;
      unsigned short usRobotId;
      unsigned short usGroupId;
      unsigned short dummy;
   };
#pragma pack(pop)

private:
   double GetUtil(int participants, SGroupInfo& group);
   std::vector<double> GetTaskRatio();
   int m_nSumOfWorkload;
   std::vector<SDecision> m_vsSharedData;
   bool m_bSatisfiedFlag;
   int m_nNumOfGroups;
   double m_dThreshold;
   int m_nMaxIteration;

};
#endif /* HEDONIC_H */
