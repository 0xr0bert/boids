#include <iostream>

#include "simulation.h"

#include <string>

namespace {
struct Config {
  size_t num_boids = 100;
  size_t num_steps = 500;
  unsigned int seed = 2342;
};

Config parse_args(const int argc, char *argv[]) {
  Config config;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--boids") {
      config.num_boids = std::stoul(argv[++i]);
    } else if (std::string(argv[i]) == "--steps") {
      config.num_steps = std::stoul(argv[++i]);
    } else if (std::string(argv[i]) == "--seed") {
      config.seed = std::stoul(argv[++i]);
    }
  }
  return config;
}
} // namespace

int main(const int argc, char *argv[]) {
  const auto [num_boids, num_steps, seed] = parse_args(argc, argv);

  auto params = std::make_unique<boids::SimulationParams>();
  boids::Simulation sim(num_boids, std::move(params), seed);

  std::cout << "step,id,x,y,vx,vy" << std::endl;

  for (size_t i = 0; i < num_steps; ++i) {
    const auto &boids = sim.boids();
    for (size_t j = 0; j < boids.size(); ++j) {
      std::cout << i << "," << j << "," << boids[j].position.x << ","
                << boids[j].position.y << "," << boids[j].velocity.x << ","
                << boids[j].velocity.y << std::endl;
    }
    sim.step();
  }

  return 0;
}
