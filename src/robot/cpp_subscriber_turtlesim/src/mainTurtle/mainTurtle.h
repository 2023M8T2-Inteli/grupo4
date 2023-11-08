#include "../turtleController/turtleController.h"

class MainTurtle : public TurtleController
{
private:
   std::string turtleName_;
   rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr poseSub_;
   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velPub_;
   turtlesim::msg::Pose current_pose_;

   bool spawnTurtle(float x, float y);
   bool killTurtle();
   void poseCallback(const turtlesim::msg::Pose::SharedPtr pose);

   float euclideanDistance(const std::tuple<float, float> &a, const std::tuple<float, float> &b);

   float pathLength(const std::shared_ptr<std::vector<std::tuple<float, float>>> &pointsPtr);

   void rotateToTarget(float target_x, float target_y);

   void moveToTarget(float target_x, float target_y);

   bool move(double linear_velocity, double angular_velocity);

public:
   MainTurtle(std::string nodeName);
   // ~MainTurtle();
   turtlesim::msg::Pose getPose();
   std::shared_ptr<std::vector<std::tuple<float, float>>> findShortestPath(std::shared_ptr<std::vector<std::tuple<float, float>>> pointsPtr);
   void followPath(std::shared_ptr<std::vector<std::tuple<float, float>>> points);
};