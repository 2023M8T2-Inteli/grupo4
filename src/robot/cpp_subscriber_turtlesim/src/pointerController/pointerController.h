#include "../turtleController/turtleController.h"
#include <vector>
#include <tuple>
#include <random>

struct TurtleInfo
{
   std::string name;
   float x;
   float y;

   TurtleInfo(const std::string name) : name(name)
   {
      std::random_device rd;  // Dispositivo usado para obter uma semente para o gerador de números
      std::mt19937 gen(rd()); // Gerador de números aleatórios baseado em Mersenne Twister

      // Distribuição uniforme entre 1 e 10
      std::uniform_real_distribution<> dis(1, 10);

      // Gerar um número aleatório
      this->x = dis(gen);
      this->y = dis(gen);
   }
};

class PointerController : public TurtleController
{
private:
   std::shared_ptr<std::vector<std::tuple<float, float>>> points_;

public:
   PointerController(std::string nodeName);
   // ~PointerController();
   bool generatePoints(std::shared_ptr<std::vector<TurtleInfo>> turtles);
   std::shared_ptr<std::vector<std::tuple<float, float>>> getPoints() { return this->points_; };
};