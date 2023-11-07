#include "mainTurtle.h"

MainTurtle::MainTurtle(std::string nodeName) : TurtleController(nodeName)
{
   this->turtleName_ = nodeName;
   this->spawnTurtle(1.00f, 1.00f);

   this->poseSub_ = this->create_subscription<turtlesim::msg::Pose>("/" + this->turtleName_ + "/pose", 10, std::bind(&MainTurtle::poseCallback, this, std::placeholders::_1));

   this->velPub_ = this->create_publisher<geometry_msgs::msg::Twist>("/" + this->turtleName_ + "/cmd_vel", 10);
}

// MainTurtle::~MainTurtle()
// {
//    this->killTurtle();
// }

bool MainTurtle::killTurtle()
{

   auto killReq = std::make_unique<turtlesim::srv::Kill::Request>();
   killReq->name = this->turtleName_;
   auto promise = this->killClient_->async_send_request(std::move(killReq));

   try
   {
      rclcpp::spin_until_future_complete(this->get_node_base_interface(), promise);
   }
   catch (...)
   {
      return false;
   }

   return true;
};

bool MainTurtle::spawnTurtle(float x, float y)
{

   auto spawnReq = std::make_unique<turtlesim::srv::Spawn::Request>();
   spawnReq->name = this->turtleName_;
   spawnReq->x = x;
   spawnReq->y = y;
   auto response_spawn = this->spawnClient_->async_send_request(std::move(spawnReq));

   auto res = rclcpp::spin_until_future_complete(this->get_node_base_interface(), response_spawn);

   if (res == rclcpp::FutureReturnCode::SUCCESS)
   {
      return true;
   }
   else
   {
      std::cout << "Failed to spawn mainTurtle" << std::endl;
      return false;
   }
};

void MainTurtle::poseCallback(const turtlesim::msg::Pose::SharedPtr pose)
{
   this->current_pose_ = *pose;
}

turtlesim::msg::Pose MainTurtle::getPose()
{
   return this->current_pose_;
}

bool MainTurtle::move(double linear_velocity, double angular_velocity)
{
   auto velMsg = std::make_unique<geometry_msgs::msg::Twist>();
   velMsg->linear.x = linear_velocity;
   velMsg->angular.z = angular_velocity;
   this->velPub_->publish(std::move(velMsg));

   return true;
}

float MainTurtle::euclideanDistance(const std::tuple<float, float> &a, const std::tuple<float, float> &b)
{
   float dx = std::get<0>(a) - std::get<0>(b);
   float dy = std::get<1>(a) - std::get<1>(b);
   return std::hypot(dx, dy);
};

float MainTurtle::pathLength(const std::shared_ptr<std::vector<std::tuple<float, float>>> &pointsPtr)
{
   float totalDistance = 0.0;
   for (size_t i = 0; i < pointsPtr->size() - 1; ++i)
   {
      totalDistance += euclideanDistance((*pointsPtr)[i], (*pointsPtr)[i + 1]);
   }
   totalDistance += euclideanDistance(pointsPtr->back(), pointsPtr->front());
   return totalDistance;
};

std::shared_ptr<std::vector<std::tuple<float, float>>> MainTurtle::findShortestPath(std::shared_ptr<std::vector<std::tuple<float, float>>> pointsPtr)
{
   auto shortestPath = std::make_shared<std::vector<std::tuple<float, float>>>(pointsPtr->begin(), pointsPtr->end());
   float minPathLength = std::numeric_limits<float>::max();
   std::sort(shortestPath->begin() + 1, shortestPath->end());

   do
   {
      float currentPathLength = pathLength(pointsPtr);
      if (currentPathLength < minPathLength)
      {
         minPathLength = currentPathLength;
         *shortestPath = *pointsPtr;
      }
   } while (std::next_permutation(pointsPtr->begin() + 1, pointsPtr->end()));

   return shortestPath;
};

void MainTurtle::rotateToTarget(float target_x, float target_y)
{
   float target_angle = std::atan2(target_y - current_pose_.y, target_x - current_pose_.x);
   std::cout << "Rotating to target to the target angle: " << target_angle << std::endl;
   float angular_error = target_angle - current_pose_.theta;
   std::cout << "Angular error: " << angular_error << std::endl;

   // Normalizar o erro angular para o intervalo [-pi, pi]
   angular_error = std::atan2(std::sin(angular_error), std::cos(angular_error));
   std::cout << "Normalized angular error: " << angular_error << std::endl;

   // Aqui você precisará chamar move com uma velocidade angular até que a tartaruga esteja orientada corretamente.

   const double kp = 0.5;
   while (std::fabs(angular_error) > 0.1)
   {
      const float start_theta = current_pose_.theta;
      std::cout << "While condition: " << std::fabs(angular_error) << std::endl;
      move(0.0, angular_error * kp);
      // Atualizar erro angular aqui (assumindo que current_pose_ é atualizado em algum lugar do seu código)
      while (start_theta == current_pose_.theta)
      {
      }
      angular_error = target_angle - current_pose_.theta;
      angular_error = std::atan2(std::sin(angular_error), std::cos(angular_error));
   }
   std::cout << "Finished rotating to target" << std::endl;
   // Pare a rotação
   move(0.0, 0.0);
};

void MainTurtle::moveToTarget(float target_x, float target_y)
{
   // Primeiro gire para o alvo
   rotateToTarget(target_x, target_y);

   // Em seguida, mova-se em linha reta para o ponto
   float distance = std::hypot(target_x - current_pose_.x, target_y - current_pose_.y);
   // distance = distance >  ? 
   std::cout << "Moving to target at distance: " << distance << std::endl;
   while (distance > 0.5)
   {
      const float start_x = current_pose_.x;
      const float start_y = current_pose_.y;

      move(distance * 0.5, 0.0); // move straight forward
      std::cout << "Distancia: " << distance << std::endl;

      while (start_x == current_pose_.x && start_y == current_pose_.y)
      {
      }
      // Atualizar a distância aqui (assumindo que current_pose_ é atualizado em algum lugar do seu código)
      distance = std::hypot(target_x - current_pose_.x, target_y - current_pose_.y);
      rotateToTarget(target_x, target_y);
   }
   std::cout << "Finished moving to target" << std::endl;
   // Parar o movimento
   move(0.0, 0.0);
};

void MainTurtle::followPath(std::shared_ptr<std::vector<std::tuple<float, float>>> points)
{
   for (const auto &point : *points)
   {
      std::cout << "Moving to point (" << std::get<0>(point) << ", " << std::get<1>(point) << ")" << std::endl;

      float target_x = std::get<0>(point);
      float target_y = std::get<1>(point);
      moveToTarget(target_x, target_y);
   }
};