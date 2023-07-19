#include "task_allocation_qt_user_functions.h"
#include <controllers/footbot_task_allocation/footbot_task_allocation.h>
#include <argos3/core/simulator/entity/controllable_entity.h>

using namespace argos;

/****************************************/
/****************************************/

CTaskAllocationQTUserFunctions::CTaskAllocationQTUserFunctions() {
   RegisterUserFunction<CTaskAllocationQTUserFunctions,CFootBotEntity>(&CTaskAllocationQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CTaskAllocationQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   CFootBotTaskAllocation& cController = dynamic_cast<CFootBotTaskAllocation&>(c_entity.GetControllableEntity().GetController());
   CFootBotTaskAllocation::SFoodData& sFoodData = cController.GetFoodData();
   if(sFoodData.HasFoodItem) {
      DrawCylinder(
         CVector3(0.0f, 0.0f, 0.3f), 
         CQuaternion(),
         0.1f,
         0.05f,
         CColor::BLACK);
   }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CTaskAllocationQTUserFunctions, "task_allocation_qt_user_functions")
