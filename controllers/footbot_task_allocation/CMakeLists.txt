add_library(footbot_task_allocation SHARED 
   task_allocation.h task_allocation.cpp 
   footbot_task_allocation.h footbot_task_allocation.cpp 
   dta.h dta.cpp 
   hedonic_game.h hedonic_game.cpp 
   pso.h pso.cpp
   sta.h sta.cpp)

list(APPEND CMAKE_PREFIX_PATH "~/tools/anaconda/anaconda3")

find_package(nlohmann_json REQUIRED)


target_link_libraries(footbot_task_allocation
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  nlohmann_json::nlohmann_json)
