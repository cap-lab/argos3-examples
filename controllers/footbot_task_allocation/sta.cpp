#include "sta.h"

void STA::Init(int robotId, int numOfRobots, int sumOfWorkload, int numOfGroups, double dThreshold, std::vector<SGroupInfo> &candidateGroups) {
   m_vsCandidateGroups = candidateGroups;
}

/****************************************/
/****************************************/

int STA::GetGroup(){
   return (int)m_vsCandidateGroups[0].usGroupId;
}

/****************************************/
/****************************************/

int STA::GetNeighborNum() {
   return 0;
}

/****************************************/
/****************************************/

void STA::SelectGroup() {}

/****************************************/
/****************************************/

void STA::Reset() {}

/****************************************/
/****************************************/

void STA::ShareDecisions(CCI_RangeAndBearingActuator* pcRABA, CCI_RangeAndBearingSensor* pcRABS){}

/****************************************/
/****************************************/

bool STA::StopCheck() {
   return true;
}
