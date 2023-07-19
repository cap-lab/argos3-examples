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
   void ShareDecisions(int dataIndex, CCI_RangeAndBearingActuator *pcRABA, CCI_RangeAndBearingSensor *pcRABS);
   bool StopCheck();
#pragma pack(push, 2)
   struct SDecision {
       int iteration;
       int unRobotId;
       short sGroupId;
   };
#pragma pack(pop)

private:
   double GetUtil(int participants, SGroupInfo& group);
   std::vector<double> GetTaskRatio();
   int m_nIteration;
   int m_nSumOfWorkload;
   std::vector<SDecision> m_vsSharedData;
   bool m_bSatisfiedFlag;
   int m_nNumOfRobots;
   int m_nNumOfGroups;
   double m_dThreshold;

};
#endif /* HEDONIC_H */
