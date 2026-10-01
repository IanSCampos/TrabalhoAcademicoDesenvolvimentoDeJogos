#include "Transform2D.hpp"
#include <cmath>

// Creates a translation matrix.
Transform2D Transform2D::translation(float tx, float ty) noexcept {
    Transform2D res;
    res.m[2][0] = tx;
    res.m[2][1] = ty;
    return res;
}

// Creates a rotation matrix.
Transform2D Transform2D::rotation(float angle_rad) noexcept {
    Transform2D res;
    float c = std::cos(angle_rad);
    float s = std::sin(angle_rad);
    res.m[0][0] = c;  res.m[0][1] = s;
    res.m[1][0] = -s; res.m[1][1] = c;
    return res;
}

// Creates a scaling matrix.
Transform2D Transform2D::scale(float sx, float sy) noexcept {
    Transform2D res;
    res.m[0][0] = sx;
    res.m[1][1] = sy;
    return res;
}

// Matrix product (left to right).
Transform2D Transform2D::operator*(const Transform2D& rhs) const noexcept {
    Transform2D res;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            res.m[i][j] = m[i][0] * rhs.m[0][j] + 
                          m[i][1] * rhs.m[1][j] + 
                          m[i][2] * rhs.m[2][j];
        }
    }
    return res;
}

// In-place matrix product.
Transform2D& Transform2D::operator*=(const Transform2D& rhs) noexcept {
    *this = *this * rhs;
    return *this;
}

// Transforms a POINT (w = 1).
Vector2D Transform2D::transform_point(const Vector2D& point) const noexcept {
    return {
        point.x * m[0][0] + point.y * m[1][0] + m[2][0],
        point.x * m[0][1] + point.y * m[1][1] + m[2][1]
    };
}

// Transforms a DIRECTION (w = 0).
Vector2D Transform2D::transform_vector(const Vector2D& direction) const noexcept {
    return {
        direction.x * m[0][0] + direction.y * m[1][0],
        direction.x * m[0][1] + direction.y * m[1][1]
    };
}