#include "task_allocation.h"
#include "dta.h"
#include "pso.h"
#include "hedonic_game.h"
#include "sta.h"


TaskAllocation* TaskAllocation::createAllocator(std::string &type) {
   if (type == "hedonic") {
      return new Hedonic();
   } else if(type == "pso") {
      return new PSO();
   } else if (type == "dta") {
      return new DTA();
   } else if (type == "sta") {
      return new STA();
   } else {
      LOGERR << "Wrong Task Allocator Type: " << type << std::endl;
      exit(1);
   }
}
