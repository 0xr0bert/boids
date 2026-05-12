#include "simulation.h"

#include <random>

namespace boids {
Simulation::Simulation(const size_t num_boids,
                       std::unique_ptr<SimulationParams> params,
                       unsigned int seed)
    : boids_(num_boids), params_(std::move(params)) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> pos_dist(0.0, params_->world_size);
  std::uniform_real_distribution<double> vel_dist(-params_->max_speed,
                                                  params_->max_speed);

  for (auto &[position, velocity] : boids_) {
    position = {pos_dist(rng), pos_dist(rng)};
    velocity = {vel_dist(rng), vel_dist(rng)};
  }
}

void Simulation::step() {
  std::vector<Coords> new_velocities(boids_.size());

  for (size_t i = 0; i < boids_.size(); ++i) {
    const Coords sep = separation(i) * params_->separation_weight;
    const Coords ali = alignment(i) * params_->alignment_weight;
    const Coords coh = cohesion(i) * params_->cohesion_weight;

    const Coords acceleration = (sep + ali + coh).limited(params_->max_force);
    new_velocities[i] =
        (boids_[i].velocity + acceleration).limited(params_->max_speed);
  }

  for (size_t i = 0; i < boids_.size(); ++i) {
    boids_[i].velocity = new_velocities[i];
    boids_[i].position += boids_[i].velocity * params_->time_step;
    wrap(boids_[i].position);
  }
}

Coords Simulation::separation(const size_t i) const {
  Coords steer{0.0, 0.0};
  size_t count = 0;

  for (size_t j = 0; j < boids_.size(); ++j) {
    if (i == j) {
      continue;
    }
    const Coords dist = boids_[i].position - boids_[j].position;
    if (const double dist_len = dist.length();
        dist_len < params_->neighbour_radius) {
      steer += dist.normalized() / dist_len;
      ++count;
    }
  }

  if (count > 0) {
    steer = steer / static_cast<double>(count);
  }

  return steer;
}

Coords Simulation::alignment(const size_t i) const {
  Coords avg_vel{0.0, 0.0};
  size_t count = 0;

  for (size_t j = 0; j < boids_.size(); ++j) {
    if (i == j) {
      continue;
    }
    if (const double dist = (boids_[i].position - boids_[j].position).length();
        dist < params_->neighbour_radius) {
      avg_vel += boids_[j].velocity;
      ++count;
    }
  }

  if (count == 0) {
    return Coords{0.0, 0.0};
  }
  avg_vel = avg_vel / static_cast<double>(count);
  return avg_vel - boids_[i].velocity;
}

Coords Simulation::cohesion(const size_t i) const {
  Coords centre{0.0, 0.0};
  size_t count = 0;

  for (size_t j = 0; j < boids_.size(); ++j) {
    if (i == j) {
      continue;
    }
    if (const double dist = (boids_[i].position - boids_[j].position).length();
        dist < params_->neighbour_radius) {
      centre += boids_[j].position;
      ++count;
    }
  }

  if (count == 0) {
    return Coords{0.0, 0.0};
  }
  centre = centre / static_cast<double>(count);
  return centre - boids_[i].position;
}

void Simulation::wrap(Coords &position) const {
  const double w = params_->world_size;
  if (position.x < 0.0) {
    position.x += w;
  }
  if (position.x >= w) {
    position.x -= w;
  }
  if (position.y < 0.0) {
    position.y += w;
  }
  if (position.y >= w) {
    position.y -= w;
  }
}
} // namespace boids
