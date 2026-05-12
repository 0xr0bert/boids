#ifndef BOIDS_COORDS_H
#define BOIDS_COORDS_H
#include <cmath>

namespace boids {

/**
 * @brief Represents a two-dimensional coordinate or vector.
 *
 * `Coords` stores an `(x, y)` pair using double-precision floating-point
 * values. It can be used both as a position in 2D space and as a vector for
 * common vector operations such as addition, subtraction, scaling,
 * normalization, and measuring length.
 */
struct Coords {
  /**
   * @brief Horizontal component.
   */
  double x = 0.0;

  /**
   * @brief Vertical component.
   */
  double y = 0.0;

  /**
   * @brief Computes the Euclidean length of this coordinate/vector.
   *
   * The length is calculated as:
   *
   * `sqrt(x^2 + y^2)`
   *
   * @return The distance from the origin `(0, 0)` to this coordinate.
   */
  [[nodiscard]] double length() const { return sqrt(x * x + y * y); }

  /**
   * @brief Returns a normalized copy of this vector.
   *
   * The normalized vector has the same direction as this vector but a length
   * of `1.0`. If this vector has zero length, `{0.0, 0.0}` is returned to avoid
   * division by zero.
   *
   * @return A unit-length vector in the same direction, or `{0.0, 0.0}` if this
   * vector has zero length.
   */
  [[nodiscard]] Coords normalized() const {
    const auto len = length();
    return len > 0.0 ? Coords{x / len, y / len} : Coords{0.0, 0.0};
  }

  /**
   * @brief Returns a copy of this vector with its length capped.
   *
   * If the vector's length exceeds `max_length`, it is scaled down to that
   * maximum length while keeping the same direction. Otherwise, the vector
   * is returned unchanged.
   *
   * @param max_length The maximum allowed length.
   * @return A vector whose length is at most `max_length`.
   */
  [[nodiscard]] Coords limited(const double max_length) const {
    const auto len = length();
    return len > max_length ? normalized() * max_length : *this;
  }

  /**
   * @brief Adds another coordinate/vector to this one.
   *
   * @param other The coordinate/vector to add.
   * @return A new `Coords` containing the component-wise sum.
   */
  Coords operator+(const Coords &other) const {
    return Coords{x + other.x, y + other.y};
  }

  /**
   * @brief Subtracts another coordinate/vector from this one.
   *
   * @param other The coordinate/vector to subtract.
   * @return A new `Coords` containing the component-wise difference.
   */
  Coords operator-(const Coords &other) const {
    return Coords{x - other.x, y - other.y};
  }

  /**
   * @brief Multiplies this coordinate/vector by a scalar.
   *
   * @param scale The scalar value to multiply both components by.
   * @return A new `Coords` with both components scaled by `scale`.
   */
  Coords operator*(const double scale) const {
    return Coords{x * scale, y * scale};
  }

  /**
   * @brief Divides this coordinate/vector by a scalar.
   *
   * @param scale The scalar value to divide both components by.
   * @return A new `Coords` with both components divided by `scale`.
   *
   * @warning Passing `0.0` as `scale` will result in floating-point division by
   * zero behavior.
   */
  Coords operator/(const double scale) const {
    return Coords{x / scale, y / scale};
  }

  /**
   * @brief Adds another coordinate/vector to this one in place.
   *
   * Both `x` and `y` are increased by the corresponding components of `other`.
   *
   * @param other The coordinate/vector to add to this one.
   * @return A copy of this coordinate/vector after modification.
   */
  Coords operator+=(const Coords &other) {
    x += other.x;
    y += other.y;
    return *this;
  }
};
} // namespace boids

#endif // BOIDS_COORDS_H
