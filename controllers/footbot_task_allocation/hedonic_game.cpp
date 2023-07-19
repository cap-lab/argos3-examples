#include "hedonic_game.h"
#include <algorithm>

void Hedonic::Init(int robotId, int numOfRobots, int sumOfWorkload, int numOfGroups, double dThreshold, std::vector<SGroupInfo> &candidateGroups) {
   /* The robot id */
   m_nRobotId = robotId;
   m_nNumOfRobots = numOfRobots;
   /* Candiate groups */
   m_vsCandidateGroups = candidateGroups;
   SDecision decision;
   decision.iteration = 0;
   decision.unRobotId = robotId;
   decision.sGroupId = -1;
   m_vsSharedData.push_back(decision);
   m_nIteration = 0;
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
   return (int)m_vsSharedData[0].sGroupId;
}

/****************************************/
/****************************************/

int Hedonic::GetNeighborNum() {
   return m_vsSharedData.size();
}

/****************************************/
/****************************************/

double Hedonic::GetUtil(int participants, SGroupInfo& group) {
   return std::abs(m_nNumOfRobots*1.0/m_nSumOfWorkload - participants*1.0/group.nWorkload);
}

/****************************************/
/****************************************/

void Hedonic::SelectGroup() {
   std::vector<double> candidate(m_vsCandidateGroups.size(), std::numeric_limits<double>::lowest());
   for (auto &group: m_vsCandidateGroups) {
      std::vector<bool> currentMembers(m_vsSharedData.size(), false);
      for(int i = 0 ; i < currentMembers.size() ; i++) {
         currentMembers[i] = (m_vsSharedData[i].sGroupId == group.sGroupId);
      }
      currentMembers[0] = true;
      int participants = std::count(currentMembers.begin(), currentMembers.end(), true); 
      candidate[group.sGroupId] = GetUtil(participants, group);
   }
   auto minElementIter = std::min_element(candidate.begin(), candidate.end());

   int bestUtility = *minElementIter;
   int bestTask = m_vsCandidateGroups[std::distance(candidate.begin(), minElementIter)].sGroupId;
   int currentTask = m_vsSharedData[0].sGroupId; 
   m_vsSharedData[0].sGroupId = bestTask;
   m_bSatisfiedFlag = true;
   if (currentTask != m_vsSharedData[0].sGroupId) {
      MYLOG << "change " << currentTask << m_vsSharedData[0].sGroupId << std::endl;
      m_nIteration += 1;
   }
}

/****************************************/
/****************************************/

void Hedonic::ShareDecisions(int dataIndex, CCI_RangeAndBearingActuator *pcRABA, CCI_RangeAndBearingSensor *pcRABS) {
   // Send Data
   if (dataIndex < m_vsSharedData.size()) {
      SDecision decision;
      decision.iteration = m_nIteration;
      decision.unRobotId = m_vsSharedData[dataIndex].unRobotId;
      decision.sGroupId = m_vsSharedData[dataIndex].sGroupId;
      unsigned char* byteArray = reinterpret_cast<unsigned char*>(&decision);
      CByteArray cSendByteArray = CByteArray(byteArray, sizeof(SDecision));
      pcRABA->SetData(cSendByteArray);
   }
   // Receive Data
   int currentTask = m_vsSharedData[0].sGroupId;
   const CCI_RangeAndBearingSensor::TReadings& tReadings = pcRABS->GetReadings();
   for (auto& tReading : tReadings) {
      CByteArray data = tReading.Data;
      SDecision* receivedData = reinterpret_cast<SDecision*>(data.ToCArray());
      if (m_nIteration < receivedData->iteration) {
         auto existData = std::find_if(m_vsSharedData.begin(), m_vsSharedData.end(),
                 [receivedData](SDecision& structObj) {
                    return structObj.unRobotId == receivedData->unRobotId;
                 });
         if (existData != m_vsSharedData.end()) {
            if (existData->iteration < receivedData->iteration) {
               existData->iteration = receivedData->iteration;
               if (existData->sGroupId != receivedData->sGroupId) {
                  m_bSatisfiedFlag = false;
                  existData->sGroupId = receivedData->sGroupId;
               }
            }
         } else {
            SDecision newData;
            newData.iteration = receivedData->iteration;
            newData.unRobotId = receivedData->unRobotId;
            newData.sGroupId = receivedData->sGroupId;
            m_vsSharedData.push_back(newData);
            m_bSatisfiedFlag = false;
         }
      }    
   }
}

/****************************************/
/****************************************/

std::vector<double> Hedonic::GetTaskRatio() {
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

bool Hedonic::StopCheck() {
   return m_bSatisfiedFlag;
}
