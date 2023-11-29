#ifndef GCAA_H
#define GCAA_H

#include "task_allocation.h"

using namespace argos;

class GCAA : public TaskAllocation {
public:
   void Init(int robotId, int numOfRobots, int sumOfWorkload, int numOfGroups, double dThreshold, std::vector<SGroupInfo> &candidateGroups);
   void Reset();
   int GetGroup();
   int GetNeighborNum();
   void SelectGroup();
   void ShareDecisions(int dataIndex, CCI_RangeAndBearingActuator* pcRABA, CCI_RangeAndBearingSensor* pcRABS);
   bool StopCheck();
#pragma pack(push, 1)
   struct SDecision {
      unsigned short usMessageId;
      unsigned char ucRobotId;
      unsigned char ucGroupId;
      unsigned short usUtility;
      unsigned char ucChange;
      unsigned char ucPrevChange;
      unsigned short usCount;
   };
#pragma pack(pop)
private:
   void CalculateUtility();
   void updateVector();
   std::vector<SDecision> m_vsSharedData;
   int m_nNumOfGroups;
   int m_nGroupingIteration;
};
#endif /* POTENTIAL_H */
