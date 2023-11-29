#include "dta.h"
#include <algorithm>
#include <random>

void DTA::Init(int robotId, int numOfRobots, int sumOfWorkload, int numOfGroups, double dThreshold, std::vector<SGroupInfo> &candidateGroups) {
   /* The robot id */
   m_nRobotId = robotId;
   /* Candiate groups */
   m_vsCandidateGroups = candidateGroups;
   /* My Group Selection Data Structure Init */
   SDecision data;
   m_vsSharedData.push_back(data);
   m_vsSharedData.at(0).sGroupId = -1;
   m_vsSharedData.at(0).unRobotId = robotId;
   m_vsSharedData.at(0).unCount = 0;

   /* Threshold & Sum Of Workload Init */
   m_dThreshold = dThreshold;
   m_nWorkloadSum = sumOfWorkload;
   m_nNumOfGroups = numOfGroups;
   /* Flag Init */
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
   return (int) m_vsSharedData.at(0).sGroupId;
}

/****************************************/
/****************************************/

int DTA::GetNeighborNum() {
   return m_vsSharedData.size();
}

/****************************************/
/****************************************/

double DTA::GetTaskProb() {
   int NumOfAssignedRobot = 0;
   for (auto& data : m_vsSharedData) {
      if (data.sGroupId == m_vsSharedData.at(0).sGroupId) {
         NumOfAssignedRobot++;
      }
   }
   auto it = std::find_if(m_vsCandidateGroups.begin(), m_vsCandidateGroups.end(),
         [this](const SGroupInfo& structObj) {
         return structObj.sGroupId == this->m_vsSharedData.at(0).sGroupId;
         });
   if (it != m_vsCandidateGroups.end()) {
      return (m_vsSharedData.size() * it->nWorkload * 1.0) / (m_nWorkloadSum * NumOfAssignedRobot);
   } else {
      MYLOGERR << "There's no group id " << m_vsSharedData.at(0).sGroupId << " in candidate groups" << std::endl;
      return -1;
   }
}

/****************************************/
/****************************************/

void DTA::SelectGroup() {
   m_nGroupingIteration++;
   std::random_device rd;
   std::mt19937 gen(rd());
   if (m_vsSharedData.at(0).unCount == 0) {
      m_vsSharedData.at(0).unCount = 1;
      std::uniform_int_distribution<> d(0, m_vsCandidateGroups.size() - 1);
      m_vsSharedData.at(0).sGroupId = m_vsCandidateGroups.at(d(gen)).sGroupId;
      MYLOG << "group: " << m_vsSharedData.at(0).sGroupId << ", count: " << m_vsSharedData.at(0).unCount << std::endl;
      return;
   }

   double task_prob = GetTaskProb();

   if (task_prob == -1) {
      return;
   }

   if (task_prob < 1.0 && m_vsSharedData.size() > 1) {
      std::vector<double> weight_list;
      double other_task_prob = (1.0-task_prob) / (m_vsCandidateGroups.size() - 1);
      for (auto& group : m_vsCandidateGroups) {
         if (group.sGroupId == m_vsSharedData.at(0).sGroupId) {
            weight_list.push_back(task_prob);
         } else {
            weight_list.push_back(other_task_prob);
         }
      }
      std::discrete_distribution<> d(weight_list.begin(), weight_list.end());
      m_vsSharedData.at(0).sGroupId = m_vsCandidateGroups.at(d(gen)).sGroupId;
      m_vsSharedData.at(0).unCount++;
      MYLOG << "group: " << m_vsSharedData.at(0).sGroupId << ", count: " << m_vsSharedData.at(0).unCount << std::endl;

      return;
   }
}

/****************************************/
/****************************************/

void DTA::ShareDecisions(int dataIndex, CCI_RangeAndBearingActuator* pcRABA, CCI_RangeAndBearingSensor* pcRABS) {
   /* Send Data */
   if (dataIndex < m_vsSharedData.size()) {
      unsigned char* byteArray = reinterpret_cast<unsigned char*>(&m_vsSharedData.at(dataIndex));
      CByteArray cSendByteArray = CByteArray(byteArray,sizeof(SDecision));
      pcRABA->SetData(cSendByteArray);
   }

   /* Receive Data */
   const CCI_RangeAndBearingSensor::TReadings& tReadings = pcRABS->GetReadings();
   for(auto& tReading : tReadings) {
      CByteArray data = tReading.Data;
      SDecision* receivedData = reinterpret_cast<SDecision*>(data.ToCArray());
      if (receivedData->unCount == 0) continue;
      auto existData = std::find_if(m_vsSharedData.begin(), m_vsSharedData.end(),
                              [receivedData](SDecision& structObj) {
                                  return structObj.unRobotId == receivedData->unRobotId;
                              });
      if (existData != m_vsSharedData.end()) {
         if (existData->unCount < receivedData->unCount) {
            existData->unCount = receivedData->unCount;
            existData->sGroupId = receivedData->sGroupId;
         }
      } else {
         SDecision newData;
         newData.unCount = receivedData->unCount;
         newData.sGroupId = receivedData->sGroupId;
         newData.unRobotId = receivedData->unRobotId;
         m_vsSharedData.push_back(newData);
         m_nGroupingIteration = 0;
      }
   }

  
}

/****************************************/
/****************************************/

std::vector<double> DTA::GetTaskRatio() {
   std::vector<int> SumOfRobotCap(m_nNumOfGroups, 0);
   for (auto& data : m_vsSharedData) {
      SumOfRobotCap.at(data.sGroupId) += 1;
   }

   std::vector<double> TaskRatios;
   for (auto& group : m_vsCandidateGroups) {
      TaskRatios.push_back(SumOfRobotCap.at(group.sGroupId) / std::ceil((group.nWorkload*1.0)/group.nRequirement));
   }

   return TaskRatios;
}

/****************************************/
/****************************************/

double DTA::GetStopProb() {
   int desiredIter = 25;
   return m_nGroupingIteration > desiredIter ? 1 : 0;
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
   std::random_device rd;
   std::mt19937 gen(rd());
   std::vector<double> weight_list;
   double stopProb = GetStopProb();
   weight_list.push_back(1-stopProb);
   weight_list.push_back(stopProb);
   std::discrete_distribution<> d(weight_list.begin(), weight_list.end());
   int random_index = d(gen);
   if (random_index == 1) {
      isGroupingDone = true;
   }
   return isGroupingDone;
}

