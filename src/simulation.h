#ifndef BOIDS_SIMULATION_H
#define BOIDS_SIMULATION_H

#include "boid.h"
#include <vector>

#include <memory>

namespace boids {

/**
 * @brief Parameters that control the boids simulation.
 *
 * These values tune the size of the world, the neighborhood rules used to
 * compute flocking behavior, and the limits applied to velocity and force.
 */
struct SimulationParams {
  /**
   * @brief Side length of the square simulation world.
   *
   * Boids are kept inside a `world_size` by `world_size` area using wrap-around
   * behavior at the edges.
   */
  double world_size = 100.0;

  /**
   * @brief Maximum distance at which other boids influence separation,
   * alignment, and cohesion.
   */
  double neighbour_radius = 10.0;

  /**
   * @brief Distance threshold for stronger separation behavior.
   *
   * Boids closer than this radius are considered too close and should repel
   * one another more aggressively.
   */
  double separation_radius = 3.0;

  /**
   * @brief Weight applied to the separation steering force.
   */
  double separation_weight = 1.5;

  /**
   * @brief Weight applied to the alignment steering force.
   */
  double alignment_weight = 1.0;

  /**
   * @brief Weight applied to the cohesion steering force.
   */
  double cohesion_weight = 1.0;

  /**
   * @brief Upper bound on boid speed.
   */
  double max_speed = 2.0;

  /**
   * @brief Upper bound on the magnitude of the combined steering force.
   */
  double max_force = 0.1;

  /**
   * @brief Time delta applied each simulation step.
   */
  double time_step = 1.0;
};

/**
 * @brief Represents and advances a boids flock simulation.
 *
 * A `Simulation` owns the flock state and the parameters that govern its
 * behavior. Each call to `step()` computes separation, alignment, and cohesion
 * forces for every boid, updates velocities, advances positions, and wraps
 * positions around the world boundaries.
 */
class Simulation {
public:
  /**
   * @brief Creates a simulation with a random initial flock.
   *
   * @param num_boids Number of boids to create.
   * @param params Simulation parameters to use; ownership is transferred to the
   *        simulation.
   * @param seed Seed used to initialize the random number generator for the
   *        initial positions and velocities.
   */
  Simulation(size_t num_boids, std::unique_ptr<SimulationParams> params,
             unsigned int seed);

  /**
   * @brief Advances the simulation by one time step.
   *
   * This updates each boid's velocity using the weighted flocking rules and
   * then moves the boid according to its velocity and the configured time step.
   */
  void step();

  /**
   * @brief Returns the current flock state.
   *
   * @return Read-only reference to the internal boid container.
   */
  [[nodiscard]] const std::vector<Boid> &boids() const { return boids_; }

  /**
   * @brief Returns the simulation parameters in use.
   *
   * @return Read-only reference to the owned simulation parameters.
   */
  [[nodiscard]] const SimulationParams &params() const { return *params_; }

private:
  /**
   * @brief Computes the separation steering vector for one boid.
   *
   * The result pushes the boid away from nearby neighbors.
   *
   * @param i Index of the boid to evaluate.
   * @return Separation force vector.
   */
  [[nodiscard]] Coords separation(size_t i) const;

  /**
   * @brief Computes the alignment steering vector for one boid.
   *
   * The result nudges the boid toward the average heading of nearby neighbors.
   *
   * @param i Index of the boid to evaluate.
   * @return Alignment force vector.
   */
  [[nodiscard]] Coords alignment(size_t i) const;

  /**
   * @brief Computes the cohesion steering vector for one boid.
   *
   * The result nudges the boid toward the average position of nearby neighbors.
   *
   * @param i Index of the boid to evaluate.
   * @return Cohesion force vector.
   */
  [[nodiscard]] Coords cohesion(size_t i) const;

  /**
   * @brief Wraps a position back into the simulation world.
   *
   * Positions that move outside the square world are translated to the opposite
   * side, creating toroidal boundary behavior.
   *
   * @param position Position to adjust in place.
   */
  void wrap(Coords &position) const;

  /**
   * @brief The flock being simulated.
   */
  std::vector<Boid> boids_;

  /**
   * @brief Parameters that control flock behavior.
   */
  std::unique_ptr<SimulationParams> params_;
};

} // namespace boids
#endif // BOIDS_SIMULATION_H
