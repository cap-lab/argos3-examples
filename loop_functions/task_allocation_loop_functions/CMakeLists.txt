link_directories(${CMAKE_BINARY_DIR}/controllers/footbot_foraging)
set(task_allocation_loop_functions_SOURCES task_allocation_loop_functions.cpp)

if(ARGOS_QTOPENGL_FOUND)
  include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
  set(task_allocation_loop_functions_SOURCES
    ${task_allocation_loop_functions_SOURCES}
    task_allocation_qt_user_functions.cpp)
endif(ARGOS_QTOPENGL_FOUND)

add_library(task_allocation_loop_functions MODULE ${task_allocation_loop_functions_SOURCES})
target_link_libraries(task_allocation_loop_functions
  footbot_task_allocation
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media)

if(ARGOS_QTOPENGL_FOUND)
  target_link_libraries(task_allocation_loop_functions argos3plugin_simulator_qtopengl)
endif(ARGOS_QTOPENGL_FOUND)
