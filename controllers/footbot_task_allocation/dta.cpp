#include "dta.h"
#include <algorithm>
#include <random>

void DTA::Init(int robotId, int numOfRobots, int sumOfWorkload, int numOfGroups, double dThreshold, std::vector<SGroupInfo> &candidateGroups) {
   /* The robot id */
   m_nRobotId = robotId;
   /* Candiate groups */
   m_vsCandidateGroups = candidateGroups;
   /* Number Of Robots */
   m_nRobotNum = numOfRobots;
   /* My Group Selection Data Structure Init */
   SDecision data;
   m_vsSharedData.push_back(data);
   m_vsSharedData.at(0).usMessageId = TaskAllocation::MESSAGE_ALLOCATION;
   m_vsSharedData.at(0).usGroupId = candidateGroups.at(0).usGroupId;
   m_vsSharedData.at(0).usRobotId = robotId;
   m_vsSharedData.at(0).usCount = 0;
   for (int i = 1 ; i < m_nRobotNum ; i++) {
      SDecision newData;
      newData.usMessageId = TaskAllocation::MESSAGE_ALLOCATION;
      newData.usGroupId = -1;
      newData.usRobotId = -1;
      newData.usCount = 0;
      m_vsSharedData.push_back(newData);
   }

   /* Threshold & Sum Of Workload Init */
   m_dThreshold = dThreshold;
   m_nWorkloadSum = sumOfWorkload;
   m_nNumOfGroups = numOfGroups;
   /* Flag Init */
   m_nSharedRobotNum = 1;
   m_nGroupingIteration = 0;
}

/****************************************/
/****************************************/

void DTA::Reset() {
  m_nGroupingIteration = 0;
}

/****************************************/
/****************************************/

int DTA::GetGroup() {
   return (int) m_vsSharedData.at(0).usGroupId;
}

/****************************************/
/****************************************/

int DTA::GetNeighborNum() {
   return m_nSharedRobotNum;
}

/****************************************/
/****************************************/

double DTA::GetTaskProb() {
   int NumOfAssignedRobot = 0;
   for (int i = 0 ; i < m_nSharedRobotNum ; i++) {
      if (m_vsSharedData.at(i).usGroupId == m_vsSharedData.at(0).usGroupId) {
         NumOfAssignedRobot++;
      }
   }
   auto it = std::find_if(m_vsCandidateGroups.begin(), m_vsCandidateGroups.end(),
         [this](const SGroupInfo& structObj) {
         return structObj.usGroupId == this->m_vsSharedData.at(0).usGroupId;
         });
   if (it != m_vsCandidateGroups.end()) {
      return (m_nSharedRobotNum * it->nWorkload * 1.0) / (m_nWorkloadSum * NumOfAssignedRobot);
   } else {
      MYLOGERR << "There's no group id " << m_vsSharedData.at(0).usGroupId << " in candidate groups" << std::endl;
      return -1;
   }
}

/****************************************/
/****************************************/

void DTA::SelectGroup() {
   m_nGroupingIteration++;
   std::random_device rd;
   std::mt19937 gen(rd());
   if (m_vsSharedData.at(0).usCount == 0) {
      m_vsSharedData.at(0).usCount = 1;
      std::uniform_int_distribution<> d(0, m_vsCandidateGroups.size() - 1);
      m_vsSharedData.at(0).usGroupId = m_vsCandidateGroups.at(d(gen)).usGroupId;
      MYLOG << "group: " << m_vsSharedData.at(0).usGroupId << ", count: " << m_vsSharedData.at(0).usCount << std::endl;
      return;
   }

   double task_prob = GetTaskProb();

   if (task_prob == -1) {
      return;
   }

   if (task_prob < 1.0 && m_nSharedRobotNum > 1) {
      std::vector<double> weight_list;
      double other_task_prob = (1.0-task_prob) / (m_vsCandidateGroups.size() - 1);
      for (auto& group : m_vsCandidateGroups) {
         if (group.usGroupId == m_vsSharedData.at(0).usGroupId) {
            weight_list.push_back(task_prob);
         } else {
            weight_list.push_back(other_task_prob);
         }
      }
      std::discrete_distribution<> d(weight_list.begin(), weight_list.end());
      m_vsSharedData.at(0).usGroupId = m_vsCandidateGroups.at(d(gen)).usGroupId;
      m_vsSharedData.at(0).usCount++;
      MYLOG << "group: " << m_vsSharedData.at(0).usGroupId << ", count: " << m_vsSharedData.at(0).usCount << std::endl;

      return;
   }
}

/****************************************/
/****************************************/

void DTA::ShareDecisions(CCI_RangeAndBearingActuator* pcRABA, CCI_RangeAndBearingSensor* pcRABS) {
   /* Send Data */
   unsigned char* byteArray = reinterpret_cast<unsigned char*>(m_vsSharedData.data());
   CByteArray cSendByteArray = CByteArray(byteArray,sizeof(SDecision)*m_nRobotNum);
   pcRABA->SetData(cSendByteArray);

   /* Receive Data */
   const CCI_RangeAndBearingSensor::TReadings& tReadings = pcRABS->GetReadings();
   for(auto& tReading : tReadings) {
      CByteArray data = tReading.Data;
      SDecision* receivedData = reinterpret_cast<SDecision*>(data.ToCArray());
      if (receivedData->usMessageId != TaskAllocation::MESSAGE_ALLOCATION) continue;
      for (int i = 0 ; i < m_nRobotNum ; i++) {
         if (receivedData->usRobotId > m_nRobotNum) break;
         if (receivedData->usCount == 0) {
            receivedData += 1;
            continue;
         }
         auto existData = std::find_if(m_vsSharedData.begin(), m_vsSharedData.end(),
                                 [receivedData](SDecision& structObj) {
                                     return structObj.usRobotId == receivedData->usRobotId;
                                 });
         if (existData != m_vsSharedData.end()) {
            if (existData->usCount < receivedData->usCount) {
               existData->usCount = receivedData->usCount;
               existData->usGroupId = receivedData->usGroupId;
            }
         } else {
            m_vsSharedData.at(m_nSharedRobotNum).usCount = receivedData->usCount;
            m_vsSharedData.at(m_nSharedRobotNum).usGroupId = receivedData->usGroupId;
            m_vsSharedData.at(m_nSharedRobotNum).usRobotId = receivedData->usRobotId;
            m_nSharedRobotNum++;
            m_nGroupingIteration = 0;
         }
         receivedData += 1;
      }
   }
}

/****************************************/
/****************************************/

std::vector<double> DTA::GetTaskRatio() {
   std::vector<int> SumOfRobotCap(m_nNumOfGroups, 0);
   for (int i = 0 ; i < m_nSharedRobotNum ; i++) {
      SumOfRobotCap.at(m_vsSharedData.at(i).usGroupId) += 1;
   }

   std::vector<double> TaskRatios;
   for (auto& group : m_vsCandidateGroups) {
      TaskRatios.push_back(SumOfRobotCap.at(group.usGroupId) / std::ceil((group.nWorkload*1.0)/ group.nRequirement));
   }

   return TaskRatios;
}

/****************************************/
/****************************************/

double DTA::GetStopProb() {
   return m_nGroupingIteration>50 ? 1 : 0;
}

/****************************************/
/****************************************/

bool DTA::StopCheck() {
   if (m_vsCandidateGroups.size() == 1) {
      return true;
   }
   std::vector<double> taskRatios = GetTaskRatio();
   bool isGroupingDone = true;
   for (int i = 0 ; i < taskRatios.size() ; i++) {
      for (int j = 0 ; j < taskRatios.size() ; j++) {
         if (taskRatios.at(i) - taskRatios.at(j) > m_dThreshold || 
            taskRatios.at(i) < 1 || taskRatios.at(j) < 1) {
            isGroupingDone = false;
            break;
         }
      }
      if (isGroupingDone == false) break;
   }
   if (isGroupingDone == true) return true;
   //std::random_device rd;
   //std::mt19937 gen(rd());
   //std::vector<double> weight_list;
   //double stopProb = GetStopProb();
   //weight_list.push_back(1-stopProb);
   //weight_list.push_back(stopProb);
   //std::discrete_distribution<> d(weight_list.begin(), weight_list.end());
   //int random_index = d(gen);
   //if (random_index == 1) {
   if(m_nGroupingIteration > 15){
      isGroupingDone = true;
   }
   return isGroupingDone;
}

