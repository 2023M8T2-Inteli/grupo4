#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/kill.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <string.h>
#include <memory>
#include <iostream>

using namespace std;

class Turtle : public rclcpp::Node
{

public:
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr killClient;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawnClient;

    Turtle(string turtleName) : rclcpp::Node(turtleName)
    {
        cout << "constructor" << endl;
        killClient = this->create_client<turtlesim::srv::Kill>("kill");
        spawnClient = this->create_client<turtlesim::srv::Spawn>("spawn");

        while (!killClient->wait_for_service(std::chrono::seconds(1)))
        {
            cout << "waiting for kill service" << endl;
        }

        while (!spawnClient->wait_for_service(std::chrono::seconds(1)))
        {
            cout << "waiting for spawn service" << endl;
        }

        // se já existe uma tartaruga com esse nome, vamos mata-la
        auto killReq = std::make_shared<turtlesim::srv::Kill::Request>();
        killReq->name = turtleName;
        auto promise = this->killClient->async_send_request(killReq);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), promise);

        //spawnando a tartruga
        auto spawnReq = std::make_shared<turtlesim::srv::Spawn::Request>();
        spawnReq->name = turtleName;
        spawnReq->x = 5.54;
        spawnReq->y = 5.54;
        this->spawnClient->async_send_request(spawnReq);
    }
        

};

int main(int argc, char** argv)
{
    // Coloque aqui o nome da sua tartaruga
    auto turtleName = ""
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Turtle>(turtleName));
    rclcpp::shutdown(); 
    return 0;
}