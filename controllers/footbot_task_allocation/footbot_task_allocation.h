/*
 * AUTHOR: wecracy <wecracy@snu.ac.kr>
 *
 * An example task_allocation controller for the foot-bot.
 *
 * This controller is meant to be used with the XML file:
 *    experiments/task_allocation.argos
 */

#ifndef FOOTBOT_TASK_ALLOCATION_H
#define FOOTBOT_TASK_ALLOCATION_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the foot-bot light sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
/* Definition of the foot-bot motor ground sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>
/* Definitions for random number generation */
#include <vector>
/* Definitions for task allocation algorithms */
#include "task_allocation.h"

using namespace argos;

class CFootBotTaskAllocation : public CCI_Controller {

public:

   /*
    * This structure holds data about food collecting by the robots
    */
   struct SFoodData {
      bool HasFoodItem;      // true when the robot is carrying a food item
      size_t FoodItemIdx;    // the index of the current food item in the array of available food items
      size_t TotalFoodItems; // the total number of food items carried by this robot during the experiment

      SFoodData();
      void Reset();
   };

   /*
    * The following variables are used as parameters for the
    * diffusion algorithm. You can set their value in the <parameters>
    * section of the XML configuration file, under the
    * <controllers><footbot_task_allocation_controller><parameters><diffusion>
    * section.
    */
   struct SDiffusionParams {
      /*
       * Maximum tolerance for the proximity reading between
       * the robot and the closest obstacle.
       * The proximity reading is 0 when nothing is detected
       * and grows exponentially to 1 when the obstacle is
       * touching the robot.
       */
      Real Delta;
      /* Angle tolerance range to go straight. */
      CRange<CRadians> GoStraightAngleRange;

      /* Constructor */
      SDiffusionParams();

      /* Parses the XML section for diffusion */
      void Init(TConfigurationNode& t_tree);
   };

   /*
    * The following variables are used as parameters for
    * turning during navigation. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><footbot_task_allocation_controller><parameters><wheel_turning>
    * section.
    */
   struct SWheelTurningParams {
      /*
       * The turning mechanism.
       * The robot can be in three different turning states.
       */
      enum ETurningMechanism
      {
         NO_TURN = 0, // go straight
         SOFT_TURN,   // both wheels are turning forwards, but at different speeds
         HARD_TURN    // wheels are turning with opposite speeds
      } TurningMechanism;
      /*
       * Angular thresholds to change turning state.
       */
      CRadians HardTurnOnAngleThreshold;
      CRadians SoftTurnOnAngleThreshold;
      CRadians NoTurnAngleThreshold;
      /* Maximum wheel speed */
      Real MaxSpeed;

      void Init(TConfigurationNode& t_tree);
   };

   /*
    * Contains all the state information about the controller.
    */
   struct SStateData {
      enum EState {
         STATE_MOVING = 0,
         STATE_FAIL,
         STATE_GROUPING,
         STATE_WAITING,
         STATE_SPLITING,
         STATE_EXPLORING,
         STATE_RETURN_TO_NEST
      } State;
      
      void Init();
      void Reset();
      CVector2 position;
   };

   /*
    * Contain the area information.
    */
   struct SArea {
      void Init();
      CVector2 left_bottom;
      CVector2 right_ceiling;
   };

   /*
    * Color list for grouping.
    */
   CColor AColorList[9] = {
      CColor::RED,
      CColor::GREEN,
      CColor::BLUE,
      CColor::YELLOW, 
      CColor::ORANGE,
      CColor::MAGENTA,
      CColor::CYAN,
      CColor::BROWN
   };
   

public:

   CFootBotTaskAllocation();
   virtual ~CFootBotTaskAllocation() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_task_allocation_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    */
   virtual void Reset();

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}

   /*
    * Returns the food data
    */
   inline SFoodData& GetFoodData() {
      return m_sFoodData;
   }

   /*
    * Returns the state data
    */
   inline SStateData& GetStateData() {
      return m_sStateData;
   }

   /*
    * Returns the group data
    */
   int GetGroup();
private:

   /*
    * Get current position of the robot by a position sensor 
    */
   CVector2 GetCurrentPosition();

   /*
    * Get current position of the robot by a position sensor 
    */
   CRadians GetCurrentOrientation();

   /*
    * Calculates the imitate proximity sensor value to hold the robot
    * in the target area.
    */
   Real GetBoundValue(CRange<Real> *x_bound, CRange<Real> *y_bound, const CRadians& angle);

   /*
    * Calculates the diffusion vector. If there is a close obstacle,
    * it points away from it; it there is none, it points forwards.
    * The b_collision parameter is used to return true or false whether
    * a collision avoidance just happened or not. It is necessary for the
    * collision rule.
    */
   CVector2 DiffusionVector(bool& b_collision, CRange<Real> *x_bound, CRange<Real> *y_bound);

   /*
    * Gets a direction vector as input and transforms it into wheel
    * actuation.
    */
   void SetWheelSpeedsFromVector(const CVector2& c_heading);

   /*
    * Get the vector for target point.
    */
   CVector2 VectorToTargetPoint(const CVector2& target_point);

   /*
    * Move to target point.
    */
   void MoveToTargetPoint(const CVector2& target_point);

   /*
    * Get the vector for flocking.
    */
   CVector2 FlockingVector();

   /*
    * Executes the moving state.
    */
   void Moving(const CVector2& target_point);

   /*
    * Set state as grouping
    */
   void SetGroupingState();

   /*
    * Executes the moving state.
    */
   void Grouping(const CVector2& target_point);

   /*
    * Set state as spliting
    */
   void SetSplitingState();

   /*
    * Split to a target area
    */
   void Spliting();

   /*
    * Init the target area
    */
   void TargetAreaInit(TConfigurationNode& t_node);

   /*
    * Init the corner nest area
    */
   void CornerAreaInit(TConfigurationNode& t_node);

   /*
    * Executes the exploring state.
    */
   void Explore();

   /*
    * Executes the return to nest state.
    */
   void GoToNest();

   /*
    * Init info related to group selection.
    */
   void GroupSelectionInit(TConfigurationNode& g_node);

   /*
    * Set led for a selected group
    */
   void SetLedForGroup();

   /*
    * Set State as waiting
    */
   void SetWaitingState();

   /*
    * Wating for some times
    */
   void Waiting(const CVector2& target_point);

   /*
    * Return whether a robot is in the range
    */
   bool InBound(CRange<Real> x_range, CRange<Real> y_range);
private:
   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the LEDs actuator */
   CCI_LEDsActuator* m_pcLEDs;
   /* Pointer to the range and bearing actuator */
   CCI_RangeAndBearingActuator*  m_pcRABA;
   /* Pointer to the range and bearing sensor */
   CCI_RangeAndBearingSensor* m_pcRABS;
   /* Pointer to the positioning sensor */
   CCI_PositioningSensor* m_pcPosition;
   /* Pointer to the foot-bot proximity sensor */
   CCI_FootBotProximitySensor* m_pcProximity;

   /* The random number generator */
   CRandom::CRNG* m_pcRNG;

   /* The robot id */
   int m_nRobotId;
   /* The controller state information */
   SStateData m_sStateData;
   /* The turning parameters */
   SWheelTurningParams m_sWheelTurningParams;
   /* The diffusion parameters */
   SDiffusionParams m_sDiffusionParams;
   /* The food data */
   SFoodData m_sFoodData;
   /* The task allocation algorithm */
   TaskAllocation *m_pcAllocator;
   /* Variables for information sharing */
   int m_nSharingDataNum;
   int m_nSharingDataIndex;
   int m_nWaitingIteration;
   int m_nLastNeighborNum;
   /* Candidate group info */
   std::vector<TaskAllocation::SGroupInfo> m_vsCandidateGroups;
   /* Map Info */
      /* Group */
   CVector2 m_cGroupingPoint;
   SArea m_sGroupingArea;
      /* Task */
   CRange<Real> m_cTargetX;
   CRange<Real> m_cTargetY;
   std::vector<CRange<Real>> m_cTargetAreaX;
   std::vector<CRange<Real>> m_cTargetAreaY;
      /* Corner */
   CRange<Real> m_cCornerX;
   CRange<Real> m_cCornerY;
   std::vector<CRange<Real>> m_cCornerAreaX;
   std::vector<CRange<Real>> m_cCornerAreaY;
};

#endif /* FOOTBOT_TASK_ALLOCATION_H */
