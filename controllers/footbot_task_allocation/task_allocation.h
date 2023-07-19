#ifndef TASK_ALLOCATION_H
#define TASK_ALLOCATION_H
/* RABA */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* RABS */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Color */
#include <argos3/core/utility/datatypes/color.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>

#define MYLOG LOG<<"["<<m_nRobotId<<"] "
#define MYLOGERR LOGERR<<"["<<m_nRobotId<<"] "

using namespace argos;

class TaskAllocation {
public:
   struct SGroupInfo {
      short sGroupId;
      int nRequirement;
      int nWorkload;
      CColor cLedColor;
   };

   virtual void Init(int robotId, int numOfRobots, int sumOfWorkload, int numOfGroups, double dThreshold, std::vector<SGroupInfo> &candidateGroups){}
   virtual void Reset(){}
   virtual int GetGroup(){return 0;}
   virtual int GetNeighborNum(){return 0;}
   virtual void SelectGroup(){}
   virtual void ShareDecisions(int dataIndex, CCI_RangeAndBearingActuator* pcRABA, CCI_RangeAndBearingSensor* pcRABS){}
   virtual bool StopCheck(){return false;}
   static TaskAllocation* createAllocator(std::string &type);
   
protected:
   std::vector<SGroupInfo> m_vsCandidateGroups;
   int m_nRobotId;
};
#endif /* TASK_ALLOCATION_H */
