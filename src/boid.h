#ifndef BOIDS_BOID_H
#define BOIDS_BOID_H
#include "coords.h"

namespace boids {

/**
 * @brief Represents a single boid in the simulation.
 *
 * A boid stores its current position and velocity in 2D space. The values are
 * expressed using `Coords`, which can represent both points and vectors.
 */
struct Boid {
  /**
   * @brief Current position of the boid.
   */
  Coords position;

  /**
   * @brief Current velocity of the boid.
   */
  Coords velocity;
};

} // namespace boids

#endif // BOIDS_BOID_H
