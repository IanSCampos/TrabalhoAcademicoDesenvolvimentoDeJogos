#include "Vector2D.hpp"
#include <cmath>

// Returns a unit-length copy with the same direction.
Vector2D Vector2D::normalized() const {
    float len = length();
    return {x / len, y / len};
}

// Scales this vector in place to unit length.
void Vector2D::normalize() {
    float len = length();
    x /= len;
    y /= len;
}

// Approximate equality within a tolerance.
bool Vector2D::equals(const Vector2D& rhs, float tolerance) const noexcept {
    return std::abs(x - rhs.x) <= tolerance && std::abs(y - rhs.y) <= tolerance;
}

// Addition operator.
Vector2D Vector2D::operator+(const Vector2D& rhs) const noexcept {
    return {x + rhs.x, y + rhs.y};
}

// Subtraction operator.
Vector2D Vector2D::operator-(const Vector2D& rhs) const noexcept {
    return {x - rhs.x, y - rhs.y};
}

// Right-scalar multiplication.
Vector2D Vector2D::operator*(float scalar) const noexcept {
    return {x * scalar, y * scalar};
}

// Division operator.
Vector2D Vector2D::operator/(float scalar) const {
    return {x / scalar, y / scalar};
}

// In-place addition.
Vector2D& Vector2D::operator+=(const Vector2D& rhs) noexcept {
    x += rhs.x;
    y += rhs.y;
    return *this;
}

// In-place subtraction.
Vector2D& Vector2D::operator-=(const Vector2D& rhs) noexcept {
    x -= rhs.x;
    y -= rhs.y;
    return *this;
}

// In-place scalar multiplication.
Vector2D& Vector2D::operator*=(float scalar) noexcept {
    x *= scalar;
    y *= scalar;
    return *this;
}

// In-place division.
Vector2D& Vector2D::operator/=(float scalar) {
    x /= scalar;
    y /= scalar;
    return *this;
}

// Left-scalar multiplication (Global function).
Vector2D operator*(float scalar, const Vector2D& vec) noexcept {
    return {vec.x * scalar, vec.y * scalar};
}