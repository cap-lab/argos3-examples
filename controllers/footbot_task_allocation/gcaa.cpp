#include "gcaa.h"
#include <cmath>
#include <algorithm>
#include <random>

#define DEADLINE 1
static bool customCompare(const TaskAllocation::SGroupInfo &a, const TaskAllocation::SGroupInfo &b);

void GCAA::Init(int robotId, int numOfRobots, int sumOfWorkload, int numOfGroups, double dThreshold, std::vector<SGroupInfo> &candidateGroups) {
   /* The robot id */
   m_nRobotId = robotId;
   m_nRobotNum = numOfRobots;
   /* Candiate groups */
   m_vsCandidateGroups = candidateGroups;
   std::sort(m_vsCandidateGroups.begin(), m_vsCandidateGroups.end(), customCompare);
   /* My Group Selection Data Structure Init */
   std::random_device rd;
   std::mt19937 gen(rd());
   std::vector<double> weights;
   double p = 1.0/(m_nNumOfGroups-1);
   for (int i = 0 ; i < m_nNumOfGroups ; i++) {
      weights.push_back(p);
   }

   std::discrete_distribution<int> dist(weights.begin(), weights.end());
   int index = dist(gen);

   SDecision data;
   m_vsSharedData.push_back(data);
   m_vsSharedData.at(0).usMessageId = TaskAllocation::MESSAGE_ALLOCATION;
   m_vsSharedData.at(0).ucGroupId = candidateGroups.at(index).usGroupId;
   m_vsSharedData.at(0).ucRobotId = robotId;
   m_vsSharedData.at(0).usCount = 0;
   m_vsSharedData.at(0).usUtility = 0;
   m_vsSharedData.at(0).ucChange = 0;
   m_vsSharedData.at(0).ucPrevChange = 0;
   for (int i = 1 ; i < m_nRobotNum ; i++) {
      SDecision newData;
      newData.usMessageId = TaskAllocation::MESSAGE_ALLOCATION;
      newData.ucGroupId = -1;
      newData.ucRobotId = -1;
      newData.usCount = 0;
      newData.usUtility = 0;
      newData.ucChange = 0;
      newData.ucPrevChange = 0;
      m_vsSharedData.push_back(newData);
   }
   /* Beta & Num Of Groups Init */
   m_nNumOfGroups = numOfGroups;
   /* Flag Init */
   m_nSharedRobotNum = 1;
   m_nGroupingIteration = 0;
}

/****************************************/
/****************************************/

int GCAA::GetGroup(){
   return m_vsSharedData.at(0).ucGroupId;
}

/****************************************/
/****************************************/

int GCAA::GetNeighborNum() {
   return m_nSharedRobotNum;
}

/****************************************/
/****************************************/

void GCAA::SelectGroup() {
   if (m_vsSharedData.at(0).ucChange == 0 && m_nGroupingIteration>0){
      CalculateUtility();
      m_vsSharedData.at(0).usCount++;
   }
   m_nGroupingIteration++;
}

/****************************************/
/****************************************/

void GCAA::Reset() {
   m_nGroupingIteration = 0;
}

/****************************************/
/****************************************/

void GCAA::ShareDecisions(int dataIndex, CCI_RangeAndBearingActuator* pcRABA, CCI_RangeAndBearingSensor* pcRABS){
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
         if (receivedData->ucRobotId > m_nRobotNum) break;
         if (receivedData->usCount == 0) {
            receivedData += 1;
            continue;
         }
         auto existData = std::find_if(m_vsSharedData.begin(), m_vsSharedData.end(),
                                 [receivedData](SDecision& structObj) {
                                     return structObj.ucRobotId == receivedData->ucRobotId;
                                 });
         if (existData != m_vsSharedData.end()) {
            if (existData->usCount < receivedData->usCount) {
               existData->ucGroupId = receivedData->ucGroupId;
               existData->usUtility = receivedData->usUtility;
               existData->ucChange = receivedData->ucChange;
               existData->usCount = receivedData->usCount;
            }
         } else {
            m_vsSharedData.at(m_nSharedRobotNum).usMessageId = receivedData->usMessageId;
            m_vsSharedData.at(m_nSharedRobotNum).ucRobotId = receivedData->ucRobotId;
            m_vsSharedData.at(m_nSharedRobotNum).ucGroupId = receivedData->ucGroupId;
            m_vsSharedData.at(m_nSharedRobotNum).usUtility = receivedData->usUtility;
            m_vsSharedData.at(m_nSharedRobotNum).ucChange = receivedData->ucChange;
            m_vsSharedData.at(m_nSharedRobotNum).usCount = receivedData->usCount;
            m_vsSharedData.at(0).ucChange = 0;
            m_nSharedRobotNum++;
            m_nGroupingIteration = 1;
         }
         receivedData += 1;
      }
   }
}

/****************************************/
/****************************************/

bool GCAA::StopCheck() {
   if (m_vsSharedData.at(0).ucChange == 0) updateVector();
   bool done = true;
   for (auto &data : m_vsSharedData) {
      if (data.ucChange == 0 && data.ucPrevChange == 0) {
         done = false;
      }
   }
   if (m_nGroupingIteration >= 20) {
      m_vsSharedData.at(0).ucChange == 1;
      done = true;
   }
   return done;
}

/****************************************/
/****************************************/

void GCAA::CalculateUtility() {
   std::vector<int> numOfAssignedRobot(m_nNumOfGroups, 0);
   std::vector<double> marginalUtility(m_nNumOfGroups);
   for (int i = 0 ; i < m_nSharedRobotNum ; i++) {
      if (m_vsSharedData.at(i).ucChange == 1) numOfAssignedRobot[m_vsSharedData.at(i).ucGroupId]++;
   }
   for (int i = 0 ; i < m_nNumOfGroups ; i++) {
      double finishTime_0, finishTime_i;
      if (numOfAssignedRobot[i] == 0){
         finishTime_0 = m_vsCandidateGroups[i].nWorkload;
      } else {
         finishTime_0 = m_vsCandidateGroups[i].nWorkload / (double)numOfAssignedRobot[i];
      }
      finishTime_i = m_vsCandidateGroups[i].nWorkload / (double)(numOfAssignedRobot[i] + 1);
      marginalUtility[i] = finishTime_0 - finishTime_i;
   }
   
   int maxIndex = std::distance(marginalUtility.begin(), std::max_element(marginalUtility.begin(), marginalUtility.end()));
   m_vsSharedData.at(0).ucGroupId = maxIndex;
   m_vsSharedData.at(0).usUtility = (unsigned short) (marginalUtility[maxIndex]*100);
   std::cout << m_nRobotId << " " << numOfAssignedRobot[0] << "," << numOfAssignedRobot[1] << "," << numOfAssignedRobot[2] << "," << numOfAssignedRobot[3] << std::endl;
   std::cout << m_nRobotId << " " << marginalUtility[0] << "," << marginalUtility[1] << "," << marginalUtility[2] << "," << marginalUtility[3] << std::endl;
   std::cout << m_nRobotId << " cal " << m_vsSharedData.at(0).ucGroupId << " " << m_vsSharedData.at(0).usUtility << std::endl;
}

/****************************************/
/****************************************/

void GCAA::updateVector() {
   int maxIndex = 0;
   unsigned short maxUtility = 0;
   for (int i = 0 ; i < m_vsSharedData.size() ; i++) {
      if (m_vsSharedData.at(i).ucGroupId == m_vsSharedData.at(0).ucGroupId && 
            m_vsSharedData.at(i).usUtility > maxUtility && 
            m_vsSharedData.at(i).ucChange == 0) {
         maxUtility = m_vsSharedData.at(i).usUtility;
         maxIndex = i;
      }
   }
   m_vsSharedData.at(maxIndex).ucChange = 1;
   std::cout << m_nRobotId << " up " << m_vsSharedData.at(maxIndex).ucRobotId << " " << m_vsSharedData.at(maxIndex).ucGroupId << " " << m_vsSharedData.at(maxIndex).usUtility << std::endl;
}

/****************************************/
/****************************************/
bool customCompare(const TaskAllocation::SGroupInfo &a, const TaskAllocation::SGroupInfo &b) {
       return a.usGroupId < b.usGroupId;
}
