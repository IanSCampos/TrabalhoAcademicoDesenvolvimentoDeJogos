#include <Collision2D.hpp>

bool AABB::intersects(const AABB& other) const noexcept {
    // Check for overlap on both axes.
    // Touching borders count as overlapping.
    return min.x <= other.max.x && max.x >= other.min.x &&
           min.y <= other.max.y && max.y >= other.min.y;
}

AABB Collision2D::bounds(const Vector2D& position) const noexcept {
    // Calculate AABB centered on the entity's position.
    // min is <= max since halfExtents is positive.
    return AABB{
        {position.x - halfExtents.x, position.y - halfExtents.y},
        {position.x + halfExtents.x, position.y + halfExtents.y}
    };
}