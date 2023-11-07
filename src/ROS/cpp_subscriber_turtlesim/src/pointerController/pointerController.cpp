#include "pointerController.h"
#include <iostream>

PointerController::PointerController(std::string nodeName) : TurtleController(nodeName){
   this->points_ = std::make_shared<std::vector<std::tuple<float, float>>>();
   std::cout << "PointerController created" << std::endl;
}

// PointerController::~PointerController(){
//    std::cout << "PointerController destroyer called, initiating turtles killing..." << std::endl;

//    for (auto it = turtleNames_.rbegin(); it != turtleNames_.rend(); ++it)
//    {
//       std::cout << "Killing turtle: " << *it << std::endl;
//       this->killTurtle(*it);
//    }
// }

bool PointerController::generatePoints(std::shared_ptr<std::vector<TurtleInfo>> turtles)
{
   for (auto turtle : *turtles)
   {
      std::cout << "Spawning turtle: " << turtle.name << " at (" << turtle.x << ", " << turtle.y << ")" << std::endl;

      if (!this->spawnTurtle(turtle.name, turtle.x, turtle.y))
      {
         std::cout << "Failed to spawn turtle" << std::endl;
         return false;
      } else {
         this->points_->push_back(std::make_tuple(turtle.x, turtle.y));
      }
   }

   return true;
}