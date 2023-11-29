/* Include the controller definition */
#include "footbot_task_allocation.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Random */
#include <random>
/* Math */
#include <cmath>
/* Json */
#include <nlohmann/json.hpp>
/* Task allocation algorithm */
#include "dta.h"
#include "pso.h"
#include "hedonic_game.h"
#include "sta.h"

#define sign(n) ((n)<0?-1:1)

/****************************************/
/****************************************/

CFootBotTaskAllocation::SFoodData::SFoodData() :
   HasFoodItem(false),
   FoodItemIdx(0),
   TotalFoodItems(0) {}

void CFootBotTaskAllocation::SFoodData::Reset() {
   HasFoodItem = false;
   FoodItemIdx = 0;
   TotalFoodItems = 0;
}

/****************************************/
/****************************************/

CFootBotTaskAllocation::SDiffusionParams::SDiffusionParams() :
   GoStraightAngleRange(CRadians(-1.0f), CRadians(1.0f)) {}

void CFootBotTaskAllocation::SDiffusionParams::Init(TConfigurationNode& t_node) {
   try {
      CRange<CDegrees> cGoStraightAngleRangeDegrees(CDegrees(-10.0f), CDegrees(10.0f));
      GetNodeAttribute(t_node, "go_straight_angle_range", cGoStraightAngleRangeDegrees);
      GoStraightAngleRange.Set(ToRadians(cGoStraightAngleRangeDegrees.GetMin()),
                               ToRadians(cGoStraightAngleRangeDegrees.GetMax()));
      GetNodeAttribute(t_node, "delta", Delta);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller diffusion parameters.", ex);
   }
}

/****************************************/
/****************************************/

void CFootBotTaskAllocation::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

/****************************************/
/****************************************/

void CFootBotTaskAllocation::SStateData::Init() {
   State = STATE_MOVING;
}

void CFootBotTaskAllocation::SStateData::Reset() {
   State = STATE_MOVING;
}

/****************************************/
/****************************************/

void CFootBotTaskAllocation::SArea::Init() {

}

/****************************************/
/****************************************/

CFootBotTaskAllocation::CFootBotTaskAllocation() :
   m_pcWheels(NULL),
   m_pcLEDs(NULL),
   m_pcRABA(NULL),
   m_pcRABS(NULL),
   m_pcProximity(NULL),
   m_pcRNG(NULL) {}

/****************************************/
/****************************************/

void CFootBotTaskAllocation::GroupSelectionInit(TConfigurationNode &g_node) {
   /* Robot Id */
   std::string RobotId = GetId();
   RobotId.erase(std::remove_if(RobotId.begin(), RobotId.end(), [](char c) { return !std::isdigit(c); }), RobotId.end());
   m_nRobotId = std::stoi(RobotId);
   /* File open and read */
   std::string candidateFileName;
   GetNodeAttribute(g_node, "candidate_group_file_name", candidateFileName);
   std::ifstream file(candidateFileName);
   if (!file.is_open()) {
       std::cerr << "Failed to open the file.\n";
       exit(1);
   }
   nlohmann::json_abi_v3_11_2::json jCandidateGroups;
   try{
     file >> jCandidateGroups;
   } catch(nlohmann::json::parse_error& e) {
      std::cerr << "JSON parse error: " << e.what() << '\n';
      exit(1);
   }
   try{
      /* Candidate Group List Init */
      for (auto& jCandidateGroup : jCandidateGroups[RobotId]) {
         TaskAllocation::SGroupInfo groupInfo;
         groupInfo.usGroupId = jCandidateGroup["name"];
         groupInfo.nRequirement = jCandidateGroup["requ"];
         groupInfo.nWorkload = jCandidateGroup["load"];
         groupInfo.cLedColor = AColorList[groupInfo.usGroupId];
         m_vsCandidateGroups.push_back(groupInfo);
      }
   } catch(nlohmann::json::parse_error& e) {
      std::cerr << "JSON parse error: " << e.what() << '\n';
      exit(1);
   }
   /* Allocator Init */
   std::string allocatorType;
   GetNodeAttribute(g_node, "allocator_type", candidateFileName);
   m_pcAllocator = TaskAllocation::createAllocator(candidateFileName);
   m_pcAllocator->Init(std::stoi(RobotId), jCandidateGroups["num_of_robots"], jCandidateGroups["workload_sum"], 
         jCandidateGroups["num_of_group"], jCandidateGroups["threshold"], m_vsCandidateGroups);
   m_nRobotNum = jCandidateGroups["num_of_robots"];
}

/****************************************/
/****************************************/

void CFootBotTaskAllocation::TargetAreaInit(TConfigurationNode& t_node) {
   for (auto& candidate : m_vsCandidateGroups) {
      CRange<Real> x, y; 
      GetNodeAttribute(t_node, "t_" + std::to_string(candidate.usGroupId) + "_x", x);
      GetNodeAttribute(t_node, "t_" + std::to_string(candidate.usGroupId) + "_y", y);
      m_cTargetAreaX.push_back(x);
      m_cTargetAreaY.push_back(y);
   }
}

/****************************************/
/****************************************/

void CFootBotTaskAllocation::CornerAreaInit(TConfigurationNode& t_node) {
   for (auto& candidate : m_vsCandidateGroups) {
      CRange<Real> x, y; 
      GetNodeAttribute(t_node, "t_" + std::to_string(candidate.usGroupId) + "_x", x);
      GetNodeAttribute(t_node, "t_" + std::to_string(candidate.usGroupId) + "_y", y);
      m_cCornerAreaX.push_back(x);
      m_cCornerAreaY.push_back(y);
   }
}

/****************************************/
/****************************************/

void CFootBotTaskAllocation::Init(TConfigurationNode& t_node) {
   try {
      /*
       * Initialize sensors/actuators
       */
      m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
      m_pcLEDs      = GetActuator<CCI_LEDsActuator                >("leds"                 );
      m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
      m_pcRABA->ClearData();
      m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
      m_pcPosition  = GetSensor  <CCI_PositioningSensor           >("positioning"          );
      m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );

      /* Diffusion algorithm */
      m_sDiffusionParams.Init(GetNode(t_node, "diffusion"));
      /* Wheel turning */
      m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
      /* Controller state */
      m_sStateData.Init();
      TConfigurationNode p_node =  GetNode(t_node, "position");
      /* Task allocation */
      GroupSelectionInit(GetNode(t_node, "grouping"));
      /* Position */
      float x,y,ran;
      GetNodeAttribute(p_node, "grouping_position_x", x);
      GetNodeAttribute(p_node, "grouping_position_y", y);
      GetNodeAttribute(p_node, "grouping_range", ran);
      m_cGroupingPoint = CVector2(x,y);
      m_sGroupingArea.left_bottom = CVector2(x-ran, y-ran);
      m_sGroupingArea.right_ceiling = CVector2(x+ran, y+ran);
      TargetAreaInit(GetNode(t_node, "target_area"));
      CornerAreaInit(GetNode(t_node, "nest_area"));
      /* Random number generator */
      m_pcRNG = CRandom::CreateRNG("argos");
      m_pcRNG->SetSeed(std::chrono::system_clock::now().time_since_epoch().count());
      m_pcRNG->Reset();
      Reset();
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the foot-bot task_allocation controller for robot \"" << GetId() << "\"", ex);
   }
}

/****************************************/
/****************************************/

void CFootBotTaskAllocation::SetNumOfGroupFoodItem(std::vector<unsigned int> &foodItemNumList) {
   m_vunGroupTotalFoodItem = foodItemNumList;
}

/****************************************/
/****************************************/

void CFootBotTaskAllocation::SetGroupingState() {
   m_sStateData.State = SStateData::STATE_GROUPING;
   m_pcAllocator->Reset();
   m_nSharingDataNum = 0;
   m_nSharingDataIndex = 0;
}

/****************************************/
/****************************************/

void CFootBotTaskAllocation::SetWaitingState() {
   m_sStateData.State = SStateData::STATE_WAITING;
   m_nWaitingIteration = 0;
   m_nLastNeighborNum = m_pcAllocator->GetNeighborNum();
}

/****************************************/
/****************************************/

void CFootBotTaskAllocation::SetSplitingState() {
   MYLOG << "set spliting" << std::endl;
   m_sStateData.State = SStateData::STATE_SPLITING;
   for (int i = 0 ; i < m_vsCandidateGroups.size() ; i++) {
      if (m_pcAllocator->GetGroup() == m_vsCandidateGroups.at(i).usGroupId) {
         m_cTargetX = m_cTargetAreaX.at(i);
         m_cTargetY = m_cTargetAreaY.at(i);
         m_cCornerX = m_cCornerAreaX.at(i);
         m_cCornerY = m_cCornerAreaY.at(i);

      }
   }
   SFoundCount data;
   m_vsSharedData.push_back(data);
   m_vsSharedData.at(0).usGroupId = m_pcAllocator->GetGroup();
   m_vsSharedData.at(0).usRobotId = m_nRobotId;
   m_vsSharedData.at(0).usFound = 0;
   m_vsSharedData.at(0).usMessageId = (unsigned short) TaskAllocation::MESSAGE_FOOD_ITEM;
   m_nSharedFoodCount = 1;
   for (int i = 1 ; i < m_nRobotNum ; i++) {
      SFoundCount data;
      data.usMessageId = (unsigned short) TaskAllocation::MESSAGE_FOOD_ITEM;
      data.usGroupId = m_pcAllocator->GetGroup();
      data.usRobotId = -1;
      data.usFound = 0;
      m_vsSharedData.push_back(data);
   }
   m_pcRABA->ClearData();
}

/****************************************/
/****************************************/

void CFootBotTaskAllocation::ControlStep() {
   switch(m_sStateData.State) {
      case SStateData::STATE_MOVING: {
         Moving(m_cGroupingPoint);
         break;
      }
      case SStateData::STATE_GROUPING: {
         Grouping(CVector2(m_cGroupingPoint.GetX()+0.5, m_cGroupingPoint.GetY()));
         break;
      }
      case SStateData::STATE_WAITING: {
         Waiting(m_cGroupingPoint);
         break;
      }
      case SStateData::STATE_SPLITING: {
         Spliting();
         break;
      }
      case SStateData::STATE_EXPLORING: {
         Explore();
         ShareFoundCount();
         StopCheck();
         break;
      }
      case SStateData::STATE_RETURN_TO_NEST: {
         GoToNest();
         break;
      }
      case SStateData::STATE_FAIL: {
         m_pcWheels->SetLinearVelocity(0,0);
         break;
      }
      case SStateData::STATE_FINISH: {
         ShareFoundCount();
         break;
      }
      default: {
         MYLOGERR << "We can't be here, there's a bug!" << std::endl;
      }
   }
}

/****************************************/
/****************************************/

void CFootBotTaskAllocation::Reset() {
   /* Reset robot state */
   m_sStateData.Reset();
   /* Reset food data */
   m_sFoodData.Reset();
   /* Set LED color */
   m_pcLEDs->SetAllColors(CColor::WHITE);
   /* Clear up the last exploration result */
   m_pcRABA->ClearData();
}

/****************************************/
/****************************************/

CRadians CFootBotTaskAllocation::GetCurrentOrientation() {
   const CQuaternion current_orientation = m_pcPosition->GetReading().Orientation;
   CRadians z_angle, y_angle, x_angle;
   current_orientation.ToEulerAngles(z_angle, y_angle, x_angle);
   return z_angle;
}

/****************************************/
/****************************************/

bool CFootBotTaskAllocation::InBound(CRange<Real> x_range, CRange<Real> y_range) {
   CVector2 curPos = GetCurrentPosition();
   if (x_range.GetMin() <= curPos.GetX() && curPos.GetX() <= x_range.GetMax()
         && y_range.GetMin() <= curPos.GetY() && curPos.GetY() <= y_range.GetMax()) {
      return true;
   } else {
      return false;
   }
}

/****************************************/
/****************************************/

Real CFootBotTaskAllocation::GetBoundValue(CRange<Real> *x_bound, CRange<Real> *y_bound, const CRadians& angle) {
   if (x_bound == NULL && y_bound == NULL) {
      return 0.0;
   }
   Real MAX_DISTANCE = 30.0;
   /* get current state */
   CVector2 curPos = GetCurrentPosition();
   CRadians curAngle = GetCurrentOrientation();

   /* get x distance */
   double xDistance, xDistanceMin, xDistanceMax;
   if (x_bound != NULL) {
      xDistanceMin = std::abs(curPos.GetX() - x_bound->GetMin()) / Cos(curAngle + angle);
      xDistanceMin = xDistanceMin <= 0 ? -xDistanceMin : MAX_DISTANCE;
      xDistanceMax = std::abs(curPos.GetX() - x_bound->GetMax()) / Cos(curAngle + angle);
      xDistanceMax = xDistanceMax >= 0 ? xDistanceMax : MAX_DISTANCE;
      xDistance = xDistanceMin < xDistanceMax ? xDistanceMin : xDistanceMax;
   } else {
      xDistance = MAX_DISTANCE;
   }

   /* get y distance */
   double yDistance, yDistanceMin, yDistanceMax;
   if (y_bound != NULL) {
      yDistanceMin = std::abs(curPos.GetY() - y_bound->GetMin()) / Sin(curAngle + angle);
      yDistanceMin = yDistanceMin <= 0 ? -yDistanceMin : MAX_DISTANCE;
      yDistanceMax = std::abs(curPos.GetY() - y_bound->GetMax()) / Sin(curAngle + angle);
      yDistanceMax = yDistanceMax >= 0 ? yDistanceMax : MAX_DISTANCE;
      yDistance = yDistanceMin < yDistanceMax ? yDistanceMin : yDistanceMax;
   } else {
      yDistance = MAX_DISTANCE;
   }

   /* get bound value */
   double distance = xDistance < yDistance ? xDistance : yDistance;
   if(distance < 0.009889556) {
      return 1.0;
   } else if (distance > 0.5) {
      return 0.0;
   } else {
      return 0.0100527 / (distance + 0.000163144);
   }
}

/****************************************/
/****************************************/

CVector2 CFootBotTaskAllocation::DiffusionVector(bool& b_collision, CRange<Real> *x_bound, CRange<Real> *y_bound) {
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cDiffusionVector;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      double value = GetBoundValue(x_bound, y_bound, tProxReads[i].Angle);
      value = value > tProxReads[i].Value ? value : tProxReads[i].Value;
      cDiffusionVector += CVector2(value, tProxReads[i].Angle);
   }
   cDiffusionVector /= tProxReads.size();
   /* If the angle of the vector is small enough and the closest obstacle
      is far enough, ignore the vector and go straight, otherwise return
      it */
   CRadians cAngle = cDiffusionVector.Angle();
   if(m_sDiffusionParams.GoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
      cDiffusionVector.Length() < m_sDiffusionParams.Delta ) {
      b_collision = false;
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_real_distribution<> dis(-1.0, 1.0);
      return CVector2(0.75, CRadians(dis(gen)));
   }
   else {
      b_collision = true;
      cDiffusionVector.Normalize();
      return -cDiffusionVector;
   }
}

/****************************************/
/****************************************/

void CFootBotTaskAllocation::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
   /* State transition logic */
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
      if(Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         /* Just go straight */
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         break;
      }
      case SWheelTurningParams::SOFT_TURN: {
         /* Both wheels go straight, but one is faster than the other */
         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }
      case SWheelTurningParams::HARD_TURN: {
         /* Opposite wheel speeds */
         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
         break;
      }
   }
   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

CVector2 CFootBotTaskAllocation::VectorToTargetPoint(const CVector2& target_point) {
   CRadians curAngle = GetCurrentOrientation();
   CVector2 curPos = GetCurrentPosition();
   return CVector2(
           (target_point - curPos).Length(),  
           ((target_point - curPos).Angle() - curAngle).SignedNormalize());
}

/****************************************/
/****************************************/

void CFootBotTaskAllocation::MoveToTargetPoint(const CVector2& target_point) {
   bool bCollision;
   CVector2 diffusion_vector = DiffusionVector(bCollision, NULL, NULL);
   if (bCollision == true){
      SetWheelSpeedsFromVector(
         m_sWheelTurningParams.MaxSpeed * diffusion_vector + VectorToTargetPoint(target_point));
   } else {
      SetWheelSpeedsFromVector(
         m_sWheelTurningParams.MaxSpeed * VectorToTargetPoint(target_point));
   }
}

/****************************************/
/****************************************/

void CFootBotTaskAllocation::Moving(const CVector2& target_point) {
   CVector2 currentPosition = GetCurrentPosition();
   if(m_sGroupingArea.left_bottom.GetX() <= currentPosition.GetX() && currentPosition.GetX() <= m_sGroupingArea.right_ceiling.GetX()
      && m_sGroupingArea.left_bottom.GetY() <= currentPosition.GetY() && currentPosition.GetY() <= m_sGroupingArea.right_ceiling.GetY()) {
      SetGroupingState();
   } else {
      MoveToTargetPoint(target_point);
   }
}

/****************************************/
/****************************************/

void CFootBotTaskAllocation::SetLedForGroup() {
   auto it = std::find_if(m_vsCandidateGroups.begin(), m_vsCandidateGroups.end(),
         [this](const TaskAllocation::SGroupInfo& structObj) {
         return structObj.usGroupId == this->m_pcAllocator->GetGroup();
         });
   if (it != m_vsCandidateGroups.end()) {
      m_pcLEDs->SetAllColors(it->cLedColor);
   } else {
      MYLOGERR << "There's no group id " << m_pcAllocator->GetGroup() << " in candidate groups" << std::endl;
   }
}

/****************************************/
/****************************************/

void CFootBotTaskAllocation::Grouping(const CVector2& target_point) {
   MoveToTargetPoint(target_point);
   m_pcAllocator->SelectGroup();
   m_pcAllocator->ShareDecisions(m_pcRABA, m_pcRABS);
   SetLedForGroup();
   if (m_pcAllocator->StopCheck() == true) {
      MYLOG << "Group Selection Finish: " << m_pcAllocator->GetGroup() << std::endl;
      SetWaitingState();
   }
}

/****************************************/
/****************************************/

void CFootBotTaskAllocation::Waiting(const CVector2& target_point) {
   MoveToTargetPoint(target_point);
   m_pcAllocator->ShareDecisions(m_pcRABA, m_pcRABS);
   m_nSharingDataIndex++;
   if (m_pcAllocator->GetNeighborNum() > m_nLastNeighborNum) {
      SetGroupingState();
   } else if (m_nWaitingIteration > 5){
      SetSplitingState();
      MYLOG << "Waiting Done" << std::endl;
   }
   m_nWaitingIteration++;
}

/****************************************/
/****************************************/

void CFootBotTaskAllocation::Spliting() {
   m_pcAllocator->ShareDecisions(m_pcRABA, m_pcRABS);
   CVector2 targetPoint = CVector2(
         m_cGroupingPoint.GetX()+2*(sign(m_cCornerX.GetMin())), 
         m_cGroupingPoint.GetY()+2*(sign(m_cCornerY.GetMin())));
   MoveToTargetPoint(targetPoint);
   CVector2 currentPosition = GetCurrentPosition();
   CRange<Real> x_bound, y_bound;
   x_bound.Set(m_cTargetX.GetMin() + 0.7, m_cTargetX.GetMax() - 0.7);
   y_bound.Set(m_cTargetY.GetMin() + 0.7, m_cTargetY.GetMax() - 0.7);
   if(InBound(x_bound, y_bound)){
      MYLOG << "Arrived Target Area" << std::endl;
      m_sStateData.State = SStateData::STATE_EXPLORING;
      m_pcLEDs->SetAllColors(CColor::BLACK);
   }
}

/****************************************/
/****************************************/

void CFootBotTaskAllocation::ShareFoundCount() {
   unsigned char* byteArray = reinterpret_cast<unsigned char*>(m_vsSharedData.data());
   CByteArray cSendByteArray = CByteArray(byteArray,sizeof(SFoundCount)*m_nRobotNum);
   m_pcRABA->SetData(cSendByteArray);
   const CCI_RangeAndBearingSensor::TReadings& tReadings = m_pcRABS->GetReadings();
   for(auto& tReading : tReadings) {
      CByteArray data = tReading.Data;
      SFoundCount* receivedData = reinterpret_cast<SFoundCount*>(data.ToCArray());
      if (receivedData->usMessageId != TaskAllocation::MESSAGE_FOOD_ITEM) continue;
      for (int i = 0 ; i < m_nRobotNum ; i++) {
         if (receivedData->usRobotId > m_nRobotNum) break;
         if (receivedData->usGroupId != m_pcAllocator->GetGroup()) {
            receivedData += 1;
            continue;
         }
         auto existData = std::find_if(m_vsSharedData.begin(), m_vsSharedData.end(),
                                 [receivedData](SFoundCount& structObj) {
                                     return structObj.usRobotId == receivedData->usRobotId;
                                 });
         if (existData != m_vsSharedData.end()) {
               if (existData->usFound < receivedData->usFound)
                  existData->usFound = receivedData->usFound;
         } else {
            m_vsSharedData.at(m_nSharedFoodCount).usFound = receivedData->usFound;
            m_vsSharedData.at(m_nSharedFoodCount).usRobotId = receivedData->usRobotId;
            m_nSharedFoodCount++;
         }
         receivedData += 1;
      }
   }
}

/****************************************/
/****************************************/

void CFootBotTaskAllocation::StopCheck() {
   int totalFound = 0;
   for (SFoundCount& data : m_vsSharedData) {
      totalFound += data.usFound;
   }
   if (totalFound >= m_vunGroupTotalFoodItem.at(m_pcAllocator->GetGroup())){
      m_sStateData.State = SStateData::STATE_FINISH;
      m_vsSharedData.at(0).usFound = m_vunGroupTotalFoodItem.at(m_pcAllocator->GetGroup());
      m_pcWheels->SetLinearVelocity(0,0);
   }
}

/****************************************/
/****************************************/

void CFootBotTaskAllocation::Explore() {
   if(m_sFoodData.HasFoodItem) {
      m_pcLEDs->SetAllColors(CColor::WHITE);
      m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
   } else {
      bool bCollision;
      CVector2 cDiffusion = DiffusionVector(bCollision, &m_cTargetX, &m_cTargetY);
      SetWheelSpeedsFromVector(m_sWheelTurningParams.MaxSpeed * cDiffusion);
   }
}

/****************************************/
/****************************************/

CVector2 CFootBotTaskAllocation::GetCurrentPosition() {
   const CVector3 current_position3 = m_pcPosition->GetReading().Position;
   return CVector2(current_position3.GetX(), current_position3.GetY());
}

/****************************************/
/****************************************/

void CFootBotTaskAllocation::GoToNest() {
   CVector2 currentPosition = GetCurrentPosition();
       
   if(!m_sFoodData.HasFoodItem) {
      m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
      m_pcLEDs->SetAllColors(CColor::BLACK);
      m_sStateData.State = SStateData::STATE_EXPLORING;
      m_vsSharedData.at(0).usFound++;
      return;
   }

   CVector2 targetPoint = CVector2(
         (m_cCornerX.GetMin() + m_cCornerX.GetMax())/2, 
         (m_cCornerY.GetMin() + m_cCornerY.GetMax())/2);
   MoveToTargetPoint(targetPoint);
}

/****************************************/
/****************************************/

int CFootBotTaskAllocation::GetGroup() {
   return m_pcAllocator->GetGroup();
}

/****************************************/
/****************************************/
/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the XML configuration file to refer to
 * this controller.
 * When ARGoS reads that string in the XML file, it knows which controller
 * class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotTaskAllocation, "footbot_task_allocation_controller")
