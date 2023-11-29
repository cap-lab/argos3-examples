#include "hedonic_game.h"
#include <algorithm>

void Hedonic::Init(int robotId, int numOfRobots, int sumOfWorkload, int numOfGroups, double dThreshold, std::vector<SGroupInfo> &candidateGroups) {
   /* The robot id */
   m_nRobotId = robotId;
   m_nRobotNum = numOfRobots;
   /* Candiate groups */
   m_vsCandidateGroups = candidateGroups;

   SDecision decision;
   decision.iteration = 0;
   decision.usRobotId = robotId;
   decision.usGroupId = candidateGroups.at(0).usGroupId;
   decision.usMessageId = TaskAllocation::MESSAGE_ALLOCATION;
   m_vsSharedData.push_back(decision);

   for (int i = 1 ; i < m_nRobotNum ; i++) {
      SDecision newData;
      newData.usMessageId = TaskAllocation::MESSAGE_ALLOCATION;
      newData.usGroupId = -1;
      newData.usRobotId = -1;
      newData.iteration = 0;
      m_vsSharedData.push_back(newData);
   }

   m_nSharedRobotNum = 1;
   m_nMaxIteration = -1;
   m_bSatisfiedFlag = false;
   m_nSumOfWorkload = sumOfWorkload;
   m_dThreshold = dThreshold;
   m_nNumOfGroups = numOfGroups;
}

/****************************************/
/****************************************/

void Hedonic::Reset() {
   m_bSatisfiedFlag = false;
}

/****************************************/
/****************************************/

int Hedonic::GetGroup() {
   return (int)m_vsSharedData[0].usGroupId;
}

/****************************************/
/****************************************/

int Hedonic::GetNeighborNum() {
   return m_nSharedRobotNum;
}

/****************************************/
/****************************************/

double Hedonic::GetUtil(int participants, SGroupInfo& group) {
   return std::abs(m_nRobotNum*1.0/m_nSumOfWorkload - participants*1.0/group.nWorkload);
}

/****************************************/
/****************************************/

void Hedonic::SelectGroup() {
   std::vector<double> candidate(m_vsCandidateGroups.size(), std::numeric_limits<double>::lowest());
   for (auto &group: m_vsCandidateGroups) {
      std::vector<bool> currentMembers(m_nSharedRobotNum, false);
      for(int i = 0 ; i < currentMembers.size() ; i++) {
         currentMembers[i] = (m_vsSharedData[i].usGroupId == group.usGroupId);
      }
      currentMembers[0] = true;
      int participants = std::count(currentMembers.begin(), currentMembers.end(), true); 
      candidate[group.usGroupId] = GetUtil(participants, group);
   }
   auto minElementIter = std::min_element(candidate.begin(), candidate.end());

   int bestUtility = *minElementIter;
   int bestTask = m_vsCandidateGroups[std::distance(candidate.begin(), minElementIter)].usGroupId;
   int currentTask = m_vsSharedData.at(0).usGroupId; 
   m_vsSharedData.at(0).usGroupId = bestTask;
   m_bSatisfiedFlag = true;
   if (currentTask != m_vsSharedData.at(0).usGroupId) {
      MYLOG << "change " << currentTask << m_vsSharedData.at(0).usGroupId << std::endl;
      m_vsSharedData.at(0).iteration++;
      if (m_vsSharedData.at(0).iteration > m_nMaxIteration) {
         m_nMaxIteration = m_vsSharedData.at(0).iteration;
      }
   }
}

/****************************************/
/****************************************/

void Hedonic::ShareDecisions(CCI_RangeAndBearingActuator *pcRABA, CCI_RangeAndBearingSensor *pcRABS) {
   // Send Data
   unsigned char* byteArray = reinterpret_cast<unsigned char*>(m_vsSharedData.data());
   CByteArray cSendByteArray = CByteArray(byteArray, sizeof(SDecision)*m_nRobotNum);
   pcRABA->SetData(cSendByteArray);
   // Receive Data
   int currentTask = m_vsSharedData.at(0).usGroupId;
   const CCI_RangeAndBearingSensor::TReadings& tReadings = pcRABS->GetReadings();
   for (auto& tReading : tReadings) {
      CByteArray data = tReading.Data;
      SDecision* receivedData = reinterpret_cast<SDecision*>(data.ToCArray());
      if (receivedData->usMessageId != TaskAllocation::MESSAGE_ALLOCATION) continue;
      if (m_nMaxIteration < receivedData->iteration) {
         m_nMaxIteration = receivedData->iteration;
         for (int i = 0 ; i < m_nRobotNum ; i++) {
            if (receivedData->usRobotId > m_nRobotNum) break;
            auto existData = std::find_if(m_vsSharedData.begin(), m_vsSharedData.end(),
                    [receivedData](SDecision& structObj) {
                       return structObj.usRobotId == receivedData->usRobotId;
                    });
            if (existData != m_vsSharedData.end()) {
               existData->iteration = receivedData->iteration;
               if (existData->usGroupId != receivedData->usGroupId) {
                  m_bSatisfiedFlag = false;
                  existData->usGroupId = receivedData->usGroupId;
               }
            } else {
               m_vsSharedData.at(m_nSharedRobotNum).usMessageId = receivedData->usMessageId;
               m_vsSharedData.at(m_nSharedRobotNum).usRobotId = receivedData->usRobotId;
               m_vsSharedData.at(m_nSharedRobotNum).usGroupId = receivedData->usGroupId;
               m_nSharedRobotNum++;
               m_bSatisfiedFlag = false;
            }
            receivedData += 1;
         }    
      }
   }
}

/****************************************/
/****************************************/

std::vector<double> Hedonic::GetTaskRatio() {
   std::vector<int> SumOfRobotCap(m_nNumOfGroups, 0);
   for (int i = 0 ; i < m_nSharedRobotNum ; i++) {
      SumOfRobotCap.at(m_vsSharedData.at(i).usGroupId) += 1;
   }

   std::vector<double> TaskRatios;
   for (auto& group : m_vsCandidateGroups) {
      TaskRatios.push_back(SumOfRobotCap.at(group.usGroupId) / std::ceil((group.nWorkload*1.0)/group.nRequirement));
   }

   return TaskRatios;
}

/****************************************/
/****************************************/

bool Hedonic::StopCheck() {
   return m_bSatisfiedFlag;
}
