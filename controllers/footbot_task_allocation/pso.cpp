#include "pso.h"
#include <random>
#include <algorithm>

void PSO::Init(int robotId, int numOfRobots, int sumOfWorkload, int numOfGroups, double dThreshold, std::vector<SGroupInfo> &candidateGroups) {
   m_nRobotId = robotId;
   m_vsCandidateGroups = candidateGroups;
   m_nSumOfWorkload = sumOfWorkload;
   m_nNumOfGroups = numOfGroups;
   m_nNumOfRobots = numOfRobots;
   m_dThreshold = dThreshold;
   m_nIteration = 0;
   std::random_device rd;
   std::mt19937 gen(rd());
   std::uniform_int_distribution<> d(0, numOfGroups - 1);
   for (int i = 0 ; i < m_nNumOfRobots ; i++) {
      m_vnCurPosition.push_back(d(gen));
      m_vnCurVelocity.push_back(d(gen));
      m_vnPerBestPosition.push_back(std::vector<unsigned short>());
      m_vnGloBestPosition.push_back(m_vnCurPosition.at(i));
      for (int j = 0 ; j < m_nNumOfRobots ; j++) {
         if (i!=m_nRobotId) {
            m_vnPerBestPosition[i].push_back(-1);
         } else {
            m_vnPerBestPosition[i].push_back(m_vnCurPosition[j]);
         }
      }
   }
}

/****************************************/
/****************************************/

void PSO::Reset() {}

/****************************************/
/****************************************/

int PSO::GetGroup() {
   return m_vnCurPosition[m_nRobotId];
}

/****************************************/
/****************************************/

int PSO::GetNeighborNum() {
   return m_nNumOfRobots;
}

/****************************************/
/****************************************/

double PSO::GetFitness(std::vector<unsigned short> &robot_assigned) {
   std::vector<int> task_assigned;
   for (int i = 0 ; i < m_vsCandidateGroups.size(); i++) {
      task_assigned.push_back(0);
   }
   for (int i = 0 ; i < robot_assigned.size() ; i++) {
      if (robot_assigned[i] >= 0 && robot_assigned[i] < m_vsCandidateGroups.size()) {
         task_assigned[robot_assigned[i]] += 1;
      }
   }
   std::vector<double> finish_time;
   for (int i = 0 ; i < m_vsCandidateGroups.size() ; i++) {
      if (task_assigned[i] != 0) {
         finish_time.push_back(m_vsCandidateGroups[i].nWorkload / task_assigned[i]);
      } else {
         finish_time.push_back(9999999);
      }
   }
   auto max_iter = std::max_element(finish_time.begin(), finish_time.end());
   if (max_iter != finish_time.end()) {
      return *max_iter;
   } else {
      return 9999999;
   }
}
   
/****************************************/
/****************************************/

void PSO::SelectGroup() {
   std::random_device rd;
   std::mt19937 gen(rd());
   std::uniform_int_distribution<> d(0, 1);
   for (int i = 0 ; i < m_vnPerBestPosition.size() ; i++) {
      if (GetFitness(m_vnGloBestPosition) > GetFitness(m_vnPerBestPosition[i])) {
         m_vnGloBestPosition = m_vnPerBestPosition[i];
      }
   }
   for (int i = 0 ; i < m_vnCurVelocity.size() ; i++) {
      m_vnCurVelocity[i] = 0.9*m_vnCurVelocity[i] + 0.3*d(gen)*(m_vnPerBestPosition[m_nRobotId][i] - m_vnCurPosition[i]) + 0.3*d(gen)*(m_vnGloBestPosition[i]-m_vnCurPosition[i]);
      m_vnCurPosition[i] = m_vnCurVelocity[i] + m_vnCurPosition[i];
      m_vnCurPosition[i] = std::max(0, (int)m_vnCurPosition[i]);
      m_vnCurPosition[i] = std::min((int)m_vnCurPosition[i], (int)m_vsCandidateGroups.size()-1);
   }
   double cur_fitness = GetFitness(m_vnCurPosition);
   double old_pFitness = GetFitness(m_vnPerBestPosition[m_nRobotId]);
   if (cur_fitness < old_pFitness) {
      m_vnPerBestPosition[m_nRobotId] = m_vnCurPosition;
   }
   m_nIteration++;
}

/****************************************/
/****************************************/

void PSO::ShareDecisions(int dataIndex, CCI_RangeAndBearingActuator* pcRABA, CCI_RangeAndBearingSensor* pcRABS) {
   /* Send Data */
   if (dataIndex < m_vsSharedData.size()) {
      SDecision decision;
      decision.usSenderId = m_nRobotId;
      decision.usRobotId = dataIndex;
      decision.usGroupId = m_vnPerBestPosition[m_nRobotId][dataIndex];
      decision.unUtility = m_nFitness;
      unsigned char* byteArray = reinterpret_cast<unsigned char*>(&m_vsSharedData.at(dataIndex));
      CByteArray cSendByteArray = CByteArray(byteArray,sizeof(SDecision));
      pcRABA->SetData(cSendByteArray);
   }
   const CCI_RangeAndBearingSensor::TReadings& tReadings = pcRABS->GetReadings();
   for(auto& tReading : tReadings) {
      CByteArray data = tReading.Data;
      SDecision* receivedData = reinterpret_cast<SDecision*>(data.ToCArray());
      m_vnPerBestPosition[receivedData->usSenderId][receivedData->usRobotId] = receivedData->usGroupId;
   }
}

/****************************************/
/****************************************/

std::vector<double> PSO::GetTaskRatio() {
   std::vector<int> SumOfRobotCap(m_nNumOfGroups, 0);
   for (auto& data : m_vsSharedData) {
      SumOfRobotCap.at(data.usGroupId) += 1;
   }

   std::vector<double> TaskRatios;
   for (auto& group : m_vsCandidateGroups) {
      TaskRatios.push_back(SumOfRobotCap.at(group.sGroupId) / std::ceil((group.nWorkload*1.0)/group.nRequirement));
   }

   return TaskRatios;
}

/****************************************/
/****************************************/

bool PSO::StopCheck() {
   if (GetFitness(m_vnCurPosition) < m_dThreshold) {
      return true;
   } else {
      if (m_nIteration >= 1000){
         return true;
      } else {
         return false;
      }
   }
}

