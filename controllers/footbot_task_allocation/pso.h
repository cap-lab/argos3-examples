#ifndef PSO_H
#define PSO_H
#include "task_allocation.h"
using namespace argos;
class PSO : public TaskAllocation {
public:
   void Init(int robotId, int numOfRobots, int sumOfWorkload, int numOfGroups, double dThreshold, std::vector<SGroupInfo> &candidateGroups);
   void Reset();
   int GetGroup();
   int GetNeighborNum();
   double GetFitness(std::vector<unsigned short> &robot_assigned);
   void SelectGroup();
   void ShareDecisions(int dataIndex, CCI_RangeAndBearingActuator* pcRABA, CCI_RangeAndBearingSensor* pcRABS);
   bool StopCheck();
#pragma pack(push, 2)
   struct SDecision {
      unsigned short usSenderId;
      unsigned short usRobotId;
      unsigned short usGroupId;
      unsigned int unUtility;
   };
#pragma pack(pop)
private:
   void GetFitness();
   std::vector<double> GetTaskRatio();
   int m_nUtility;
   std::vector<unsigned short> m_vnCurPosition;
   std::vector<double> m_vnCurVelocity;
   std::vector<std::vector<unsigned short>> m_vnPerBestPosition;
   std::vector<unsigned short> m_vnGloBestPosition;
   std::vector<SDecision> m_vsSharedData;
   int m_nFitness;
   int m_nNumOfRobots;
   int m_nNumOfGroups;
   int m_nSumOfWorkload;
   int m_nIteration;
   double m_dThreshold;
};
#endif /* PSO_H */
