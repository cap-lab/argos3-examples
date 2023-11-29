#include "task_allocation.h"
#include "dta.h"
#include "hedonic_game.h"
#include "pso.h"
#include "sta.h"
#include "aladdin.h"
#include "gcaa.h"


TaskAllocation* TaskAllocation::createAllocator(std::string &type) {
   if (type == "dta") {
      return new DTA();
   } else if(type == "hedonic") {
      return new Hedonic();
   } else if (type == "pso") {
      return new PSO();
   } else if (type == "sta") {
      return new STA();
   } else if (type == "aladdin") {
      return new ALADDIN();
   } else if (type == "gcaa") {
      return new GCAA();
   } else {
      LOGERR << "Wrong Task Allocator Type: " << type << std::endl;
      exit(1);
   }
}
