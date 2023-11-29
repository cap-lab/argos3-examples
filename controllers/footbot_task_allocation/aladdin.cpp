#include "aladdin.h"
#include <cmath>
#include <algorithm>
#include <random>

#define DEADLINE 1
static bool customCompare(const TaskAllocation::SGroupInfo &a, const TaskAllocation::SGroupInfo &b);

void ALADDIN::Init(int robotId, int numOfRobots, int sumOfWorkload, int numOfGroups, double dThreshold, std::vector<SGroupInfo> &candidateGroups) {
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
   m_vsSharedData.at(0).usGroupId = candidateGroups.at(index).usGroupId;
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
   /* Beta & Num Of Groups Init */
   m_nNumOfGroups = numOfGroups;
   m_dBeta = 0.9;
   /* Flag Init */
   m_nSharedRobotNum = 1;
   m_nGroupingIteration = 0;
}

/****************************************/
/****************************************/

int ALADDIN::GetGroup(){
   return m_vsSharedData.at(0).usGroupId;
}

/****************************************/
/****************************************/

int ALADDIN::GetNeighborNum() {
   return m_nSharedRobotNum;
}

/****************************************/
/****************************************/

void ALADDIN::SelectGroup() {
   m_vsSharedData.at(0).usGroupId = CalculateUtility();
   m_vsSharedData.at(0).usCount++;
   m_nGroupingIteration++;
}

/****************************************/
/****************************************/

void ALADDIN::Reset() {
   m_nGroupingIteration = 0;

}

/****************************************/
/****************************************/

void ALADDIN::ShareDecisions(CCI_RangeAndBearingActuator* pcRABA, CCI_RangeAndBearingSensor* pcRABS){
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
            m_vsSharedData.at(m_nSharedRobotNum).usMessageId = receivedData->usMessageId;
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

bool ALADDIN::StopCheck() {
   if (m_nGroupingIteration >= 5){
      return true;
   } else {
      return false;
   }
}

/****************************************/
/****************************************/

double ALADDIN::CalculateUtility() {
   std::vector<int> numOfAssignedRobot(m_nNumOfGroups, 0);
   std::vector<double> marginalUtility(m_nNumOfGroups);
   for (int i = 0 ; i < m_nSharedRobotNum ; i++) {
      numOfAssignedRobot[m_vsSharedData.at(i).usGroupId]++;
   }
   numOfAssignedRobot[m_vsSharedData[0].usGroupId]--;
   for (int i = 0 ; i < m_nNumOfGroups ; i++) {
      double finishTime_0, finishTime_i, s_0, s_i;
      
      if (numOfAssignedRobot[i] > 0) {
         finishTime_0 = m_vsCandidateGroups[i].nWorkload / (double)numOfAssignedRobot[i];
         //if (finishTime_0 <= DEADLINE) {
            s_0 = pow(m_dBeta, finishTime_0);
         //} else {
         //   s_0 = 0.0;
         //}
      } else {
         s_0 = 0.0;
      }
      finishTime_i = m_vsCandidateGroups[i].nWorkload / (double)(numOfAssignedRobot[i] + 1);
      //if (finishTime_i <= DEADLINE) {
         s_i = pow(m_dBeta, finishTime_i);
      //} else {
      //   s_i = 0.0;
      //}
      std::cout << i << " " << m_vsCandidateGroups[i].usGroupId << " " << m_vsCandidateGroups[i].nWorkload << " " << numOfAssignedRobot[i] << " " << finishTime_0 << " " << finishTime_i << " " << s_i << " " << s_0 << std::endl;
      marginalUtility[i] = s_i-s_0;
   }
   
   int maxIndex = std::distance(marginalUtility.begin(), std::max_element(marginalUtility.begin(), marginalUtility.end()));
   int ret = m_vsSharedData.at(0).usGroupId;
   if (maxIndex != m_vsSharedData.at(0).usGroupId) {
      double p = 0.9;
      int randomNum;
      std::random_device rd;
      std::mt19937 gen(rd());
      //std::vector<double> weights;
      //double remain_p = (1-p)/(m_nNumOfGroups-1);
      //for (int i = 0 ; i < m_nNumOfGroups ; i++) {
      //   if (i == maxIndex) weights.push_back(p);
      //   else weights.push_back(remain_p);
      //}
      //std::discrete_distribution<int> dist(weights.begin(), weights.end());
      std::discrete_distribution<int> dist({p,1-p});
      randomNum = dist(gen);
      if (randomNum == 0) ret = maxIndex;
   }
   return ret;
   
}

/****************************************/
/****************************************/

bool customCompare(const TaskAllocation::SGroupInfo &a, const TaskAllocation::SGroupInfo &b) {
       return a.usGroupId < b.usGroupId;
}
