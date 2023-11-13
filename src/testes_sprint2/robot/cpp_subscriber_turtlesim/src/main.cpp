#include "pointerController/pointerController.h"
#include "mainTurtle/mainTurtle.h"
#include <memory>

int main(int argc, char **argv)
{
   // Iniciando o pacote ros
   rclcpp::init(argc, argv);

   auto pointerController = std::make_shared<PointerController>("pointerController");

   auto mainTurtle = std::make_shared<MainTurtle>("mainTurtle");

   // points - x --> min: 1 max: 10   y --> min: 1 max: 10
   auto turtles = std::make_shared<std::vector<TurtleInfo>>(std::vector<TurtleInfo>({
       TurtleInfo("turtle2"),
       TurtleInfo("turtle3"),
       TurtleInfo("turtle4"),
       TurtleInfo("turtle5"),
       TurtleInfo("turtle6"),
       TurtleInfo("turtle7"),
   }));

   if (pointerController->generatePoints(turtles))
   {
      // pointerController.reset();
      auto points = pointerController->getPoints();
      auto shortest_path = mainTurtle->findShortestPath(points);

      std::cout << "Shortest path: " << std::endl;
      for (auto point : *shortest_path)
      {
         std::cout << "(" << std::get<0>(point) << ", " << std::get<1>(point) << ")" << std::endl;
      }

      std::thread spinner([&]() {
        rclcpp::spin(mainTurtle);
    });

      mainTurtle->followPath(shortest_path);

      spinner.join();
   };

   // Finalizando o ros
   rclcpp::shutdown();

   return 0;
}