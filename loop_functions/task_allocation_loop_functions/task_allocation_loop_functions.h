#ifndef TASK_ALLOCATION_LOOP_FUNCTIONS_H
#define TASK_ALLOCATION_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>

using namespace argos;

class CTaskAllocationLoopFunctions : public CLoopFunctions {

public:
   struct SAllocationState {
      enum EState{
         PRE_ALLOCATION,
         ONLY_ALLOCATION,
         ALLOCATION_WORKING,
         WORKING,
      } State;
   };

   CTaskAllocationLoopFunctions();
   virtual ~CTaskAllocationLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
   virtual bool InNest(const CVector2& cPos);
   virtual void PreStep();
   virtual void CornerAreaInit(TConfigurationNode& t_corner);
   virtual void TargetAreaInit(TConfigurationNode& t_target);
   virtual void AisleAreaInit(TConfigurationNode& t_asile);
   virtual void RangeInit(TConfigurationNode& t_node);
   virtual bool IsExperimentFinished();

private:
   void WriteOutput(std::string output);

   Real m_fFoodSquareRadius;
   Real m_fTrapSquareRadius;
   std::vector<CRange<Real>> m_cTaskAllocationArenaSideX;
   std::vector<CRange<Real>> m_cTaskAllocationArenaSideY;
   CRange<Real> m_cAisleX;
   CRange<Real> m_cAisleY;
   std::vector<CRange<Real>> m_cTaskAllocationCornerSideX;
   std::vector<CRange<Real>> m_cTaskAllocationCornerSideY;
   std::vector<CVector2> m_cFoodPos;
   std::vector<CVector2> m_cTrapPos;
   CFloorEntity* m_pcFloor;
   CRandom::CRNG* m_pcRNG;

   std::string m_strOutput;

   SAllocationState m_sState;
   UInt32 m_unCollectedFood;
   UInt32 m_unTotalFood;
   UInt32 m_unRooms;
   bool m_bDone;
};

#endif /* TASK_ALLOCATION_LOOP_FUNCTIONS_H */
