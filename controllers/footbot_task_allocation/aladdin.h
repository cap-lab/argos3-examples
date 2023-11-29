#ifndef ALADDIN_H
#define ALADDIN_H

#include "task_allocation.h"

using namespace argos;

class ALADDIN : public TaskAllocation {
public:
   void Init(int robotId, int numOfRobots, int sumOfWorkload, int numOfGroups, double dThreshold, std::vector<SGroupInfo> &candidateGroups);
   void Reset();
   int GetGroup();
   int GetNeighborNum();
   void SelectGroup();
   void ShareDecisions(CCI_RangeAndBearingActuator* pcRABA, CCI_RangeAndBearingSensor* pcRABS);
   bool StopCheck();
#pragma pack(push, 2)
   struct SDecision {
      unsigned short usMessageId;
      unsigned short usRobotId;
      unsigned short usCount;
      unsigned short usGroupId;
      unsigned short dummy;
   };
#pragma pack(pop)
private:
   double CalculateUtility();
   std::vector<SDecision> m_vsSharedData;
   double m_dBeta;
   int m_nNumOfGroups;
   int m_nGroupingIteration;
};
#endif /* ALADDIN_H */
