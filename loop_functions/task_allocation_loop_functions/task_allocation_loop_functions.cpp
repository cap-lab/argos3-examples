#include "task_allocation_loop_functions.h"
#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/footbot_task_allocation/footbot_task_allocation.h>

/****************************************/
/****************************************/

CTaskAllocationLoopFunctions::CTaskAllocationLoopFunctions() :
   m_pcFloor(NULL),
   m_pcRNG(NULL),
   m_unCollectedFood(0),
   m_unRooms(4) {
   for(UInt32 i = 0; i < m_unRooms; ++i) {
      m_cTaskAllocationArenaSideX.push_back(CRange<Real>());
      m_cTaskAllocationArenaSideY.push_back(CRange<Real>());
      m_cTaskAllocationCornerSideX.push_back(CRange<Real>());
      m_cTaskAllocationCornerSideY.push_back(CRange<Real>());
   }
}

/****************************************/
/****************************************/

void CTaskAllocationLoopFunctions::CornerAreaInit(TConfigurationNode& t_corner){
   for(int i = 0 ; i < m_unRooms ; i++) {
      GetNodeAttribute(t_corner, "t_"+std::to_string(i)+"_x", m_cTaskAllocationCornerSideX[i]);
      GetNodeAttribute(t_corner, "t_"+std::to_string(i)+"_y", m_cTaskAllocationCornerSideY[i]);
   }
}

/****************************************/
/****************************************/

void CTaskAllocationLoopFunctions::TargetAreaInit(TConfigurationNode& t_target){
   for(int i = 0 ; i < m_unRooms ; i++) {
      GetNodeAttribute(t_target, "t_"+std::to_string(i)+"_x", m_cTaskAllocationArenaSideX[i]);
      GetNodeAttribute(t_target, "t_"+std::to_string(i)+"_y", m_cTaskAllocationArenaSideY[i]);
      m_cTaskAllocationArenaSideX[i].Set(m_cTaskAllocationArenaSideX[i].GetMin() + 0.5, m_cTaskAllocationArenaSideX[i].GetMax() - 0.5);
      m_cTaskAllocationArenaSideY[i].Set(m_cTaskAllocationArenaSideY[i].GetMin() + 0.5, m_cTaskAllocationArenaSideY[i].GetMax() - 0.5);
   }
}

/****************************************/
/****************************************/

void CTaskAllocationLoopFunctions::AisleAreaInit(TConfigurationNode& t_asile){
   GetNodeAttribute(t_asile, "x", m_cAisleX);
   GetNodeAttribute(t_asile, "y", m_cAisleY);
   m_cAisleY.Set(m_cAisleY.GetMin() + 0.5, m_cAisleY.GetMax() - 0.5);
}

/****************************************/
/****************************************/

void CTaskAllocationLoopFunctions::RangeInit(TConfigurationNode& t_node){
   AisleAreaInit(GetNode(t_node, "aisle"));
   TargetAreaInit(GetNode(t_node, "target_area"));
   CornerAreaInit(GetNode(t_node, "nest_area"));
}

/****************************************/
/****************************************/

void CTaskAllocationLoopFunctions::Init(TConfigurationNode& t_node) {
   try {
      TConfigurationNode& tTaskAllocation = GetNode(t_node, "task_allocation");
      m_pcFloor = &GetSpace().GetFloorEntity();

      RangeInit(t_node);

      std::vector<UInt32> vnFoodItems;
      TConfigurationNode& t_item = GetNode(t_node, "item");
      m_unTotalFood = 0;
      for(UInt32 i = 0 ; i < m_unRooms; ++i) {
         int num;
         GetNodeAttribute(t_item, "t_" + std::to_string(i), num);
         vnFoodItems.push_back(num);
         m_unTotalFood += num;
      }
      UInt32 unTraps;
      GetNodeAttribute(tTaskAllocation, "traps", unTraps);

      Real rSquareRadius;
      GetNodeAttribute(tTaskAllocation, "food_radius", rSquareRadius);
      m_fFoodSquareRadius = rSquareRadius*rSquareRadius;
      GetNodeAttribute(tTaskAllocation, "trap_radius", rSquareRadius);
      m_fTrapSquareRadius = rSquareRadius*rSquareRadius;

      m_pcRNG = CRandom::CreateRNG("argos");
      m_pcRNG->SetSeed(std::chrono::system_clock::now().time_since_epoch().count());
      m_pcRNG->Reset();

      for(UInt32 i = 0; i < unTraps; ++i) {
         m_cTrapPos.push_back(
           CVector2(m_pcRNG->Uniform(m_cAisleX),
                    m_pcRNG->Uniform(m_cAisleY)));
      }
      for(UInt32 r = 0; r < m_unRooms; ++r) {
         for(UInt32 i = 0; i < vnFoodItems[r]; ++i) {
            m_cFoodPos.push_back(
               CVector2(m_pcRNG->Uniform(m_cTaskAllocationArenaSideX[r]),
                        m_pcRNG->Uniform(m_cTaskAllocationArenaSideY[r])));
         }
      }

      m_sState.State = SAllocationState::PRE_ALLOCATION;
      m_bDone = false;
      std::string outputDir;
      GetNodeAttribute(tTaskAllocation, "output_dir", outputDir);
      GetNodeAttribute(tTaskAllocation, "output_file", m_strOutput);
      auto t = std::time(nullptr);
      auto tm = *std::localtime(&t);
      std::ostringstream oss;
      oss << std::put_time(&tm, "/%Y_%m_%d_%H_%M_%S_");
      m_strOutput = outputDir + oss.str() + m_strOutput;
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
      exit(1);
   }
}

/****************************************/
/****************************************/

void CTaskAllocationLoopFunctions::Reset() {
   m_unCollectedFood = 0;
   m_bDone = false;
   m_sState.State = SAllocationState::PRE_ALLOCATION;
   for(UInt32 i = 0; i < m_cTrapPos.size(); ++i) {
      m_cTrapPos.push_back(
         CVector2(m_pcRNG->Uniform(m_cAisleX),
                  m_pcRNG->Uniform(m_cAisleY)));
   }
   for(UInt32 r = 0; r < m_unRooms; ++r) {
      for(UInt32 i = 0; i < m_cFoodPos.size(); ++i) {
         m_cFoodPos.push_back(
            CVector2(m_pcRNG->Uniform(m_cTaskAllocationArenaSideX[r]),
                     m_pcRNG->Uniform(m_cTaskAllocationArenaSideY[r])));
      }
   }
}

/****************************************/
/****************************************/

void CTaskAllocationLoopFunctions::Destroy() {}

/****************************************/
/****************************************/

CColor CTaskAllocationLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
   for(UInt32 i = 0; i < m_cTrapPos.size(); ++i) {
      if((c_position_on_plane - m_cTrapPos[i]).SquareLength() < m_fTrapSquareRadius) {
         return CColor::RED;
      }
   }
   for(UInt32 i = 0; i < m_cFoodPos.size(); ++i) {
      if((c_position_on_plane - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius) {
         return CColor::BLACK;
      }
   }
   for(UInt32 i = 0; i < m_unRooms; ++i) {
      if(m_cTaskAllocationCornerSideX[i].GetMin() <= c_position_on_plane.GetX() && m_cTaskAllocationCornerSideX[i].GetMax() >= c_position_on_plane.GetX()
       && m_cTaskAllocationCornerSideY[i].GetMin() <= c_position_on_plane.GetY() && m_cTaskAllocationCornerSideY[i].GetMax() >= c_position_on_plane.GetY()) {
          return CColor::GRAY50;
      }
   }
   return CColor::WHITE;
}

/****************************************/
/****************************************/

bool CTaskAllocationLoopFunctions::InNest(const CVector2& cPos) {
   for (UInt32 i = 0; i < m_unRooms; ++i) {
      if(m_cTaskAllocationCornerSideX[i].GetMin() <= cPos.GetX() && m_cTaskAllocationCornerSideX[i].GetMax() >= cPos.GetX()
       && m_cTaskAllocationCornerSideY[i].GetMin() <= cPos.GetY() && m_cTaskAllocationCornerSideY[i].GetMax() >= cPos.GetY()) {
         return true;
      }
   }
   return false;
}

/****************************************/
/****************************************/

void CTaskAllocationLoopFunctions::WriteOutput(std::string output) {
   std::ofstream file(m_strOutput, std::ios::app);
   if (file.is_open()) {
      file << output << std::endl;
      file.close();
   } else {
      exit(1);
   }
}

/****************************************/
/****************************************/

void CTaskAllocationLoopFunctions::PreStep() {
   UInt32 unWalkingFBs = 0;
   UInt32 unRestingFBs = 0;
   CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
   bool allocationFinish = true;
   if (m_sState.State != SAllocationState::ALLOCATION_WORKING) {
      allocationFinish = false;
   }

   for(CSpace::TMapPerType::iterator it = m_cFootbots.begin(); it != m_cFootbots.end(); ++it) {
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      CFootBotTaskAllocation& cController = dynamic_cast<CFootBotTaskAllocation&>(cFootBot.GetControllableEntity().GetController());
      CFootBotTaskAllocation::SStateData& sStateData = cController.GetStateData();
      if (m_sState.State == SAllocationState::PRE_ALLOCATION 
            && sStateData.State == CFootBotTaskAllocation::SStateData::STATE_GROUPING) {
         m_sState.State = SAllocationState::ONLY_ALLOCATION;
         WriteOutput("alloc_start," + std::to_string(GetSpace().GetSimulationClock()));
      } else if (m_sState.State == SAllocationState::ONLY_ALLOCATION 
            && sStateData.State == CFootBotTaskAllocation::SStateData::STATE_SPLITING) {
         m_sState.State = SAllocationState::ALLOCATION_WORKING;
         WriteOutput("working_start," + std::to_string(GetSpace().GetSimulationClock()));
      } else if (m_sState.State == SAllocationState::ALLOCATION_WORKING && 
            !(sStateData.State == CFootBotTaskAllocation::SStateData::STATE_SPLITING 
               || sStateData.State == CFootBotTaskAllocation::SStateData::STATE_EXPLORING 
               || sStateData.State == CFootBotTaskAllocation::SStateData::STATE_RETURN_TO_NEST
               || sStateData.State == CFootBotTaskAllocation::SStateData::STATE_FAIL)) {
         allocationFinish = false;
      }

      CVector2 cPos;
      cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
               cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
      for(size_t i = 0; i < m_cTrapPos.size(); ++i) {
         if((cPos - m_cTrapPos[i]).SquareLength() < m_fTrapSquareRadius) {
            sStateData.State = CFootBotTaskAllocation::SStateData::STATE_FAIL;
         }
      }

      CFootBotTaskAllocation::SFoodData& sFoodData = cController.GetFoodData();
      if(sFoodData.HasFoodItem) {
         if(InNest(cPos)) {
            sFoodData.HasFoodItem = false;
            sFoodData.FoodItemIdx = 0;
            ++sFoodData.TotalFoodItems;
            ++m_unCollectedFood;
            if(m_unCollectedFood == m_unTotalFood) {
               WriteOutput("working_done," + std::to_string(GetSpace().GetSimulationClock()));
               m_bDone = true;
            }
         }
      }
      else {
         bool bDone = false;
         for(size_t i = 0; i < m_cFoodPos.size() && !bDone; ++i) {
            if((cPos - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius) {
               m_cFoodPos.erase(m_cFoodPos.begin() + i);
               sFoodData.HasFoodItem = true;
               sFoodData.FoodItemIdx = i;
               m_pcFloor->SetChanged();
               bDone = true;
            }
         }
      }
   }
   if (m_sState.State == SAllocationState::ALLOCATION_WORKING && allocationFinish == true) {
      m_sState.State = SAllocationState::WORKING;
      WriteOutput("alloc_done," + std::to_string(GetSpace().GetSimulationClock()));
   }
   if(GetSpace().GetSimulationClock() >= 20000){
      WriteOutput("working_done_fail," + std::to_string(GetSpace().GetSimulationClock()));
      m_bDone = true;
   }
}

/****************************************/
/****************************************/

bool CTaskAllocationLoopFunctions::IsExperimentFinished() {
   if(m_bDone == true) {
      CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
      for(CSpace::TMapPerType::iterator it = m_cFootbots.begin(); it != m_cFootbots.end(); ++it) {
         CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
         CFootBotTaskAllocation& cController = dynamic_cast<CFootBotTaskAllocation&>(cFootBot.GetControllableEntity().GetController());
         CFootBotTaskAllocation::SStateData& sStateData = cController.GetStateData();
         if (sStateData.State == CFootBotTaskAllocation::SStateData::STATE_FAIL) {
            WriteOutput(cController.GetId() + ",fail");
         } else {
            WriteOutput(cController.GetId() + "," + std::to_string(cController.GetGroup()));
         }
      }
      return true;
   } else {
      return false;
   }
}

/****************************************/
/****************************************/
REGISTER_LOOP_FUNCTIONS(CTaskAllocationLoopFunctions, "task_allocation_loop_functions")
