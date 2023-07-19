#ifndef TASK_ALLOCATION_QT_USER_FUNCTIONS_H
#define TASK_ALLOCATION_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;

class CTaskAllocationQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CTaskAllocationQTUserFunctions();

   virtual ~CTaskAllocationQTUserFunctions() {}

   void Draw(CFootBotEntity& c_entity);
   
};

#endif /* TASK_ALLOCATION_QT_USER_FUNCTIONS_H */
