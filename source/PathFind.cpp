#include "PathFind.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace {

/// Single node in the A* open/closed set.
struct AStarNode {
    int   index;    ///< Flat index into the grid (row * width + col).
    int   parent;   ///< Flat index of the predecessor, or -1 for the start.
    float g;        ///< Cost from start to this node.
    float f;        ///< g + h (estimated total cost).
};

/// Heuristic: octile distance – admissible for 8-connectivity with diagonal
/// cost = sqrt(2) and straight cost = 1.
inline float octile(int dc, int dr) noexcept {
    int ax = dc < 0 ? -dc : dc;
    int ay = dr < 0 ? -dr : dr;
    int mn = ax < ay ? ax : ay;
    int mx = ax > ay ? ax : ay;
    return static_cast<float>(mx) + (std::sqrt(2.0f) - 1.0f) * static_cast<float>(mn);
}

/// Min-heap comparator (lowest f on top).
struct MinF {
    bool operator()(const AStarNode& a, const AStarNode& b) const noexcept {
        return a.f > b.f; // inverted so std::make_heap/push_heap acts as min-heap
    }
};

} // anonymous namespace

// ---------------------------------------------------------------------------
// PathFind
// ---------------------------------------------------------------------------

PathFind::PathFind(const int* grid, int width, int height)
    : m_grid(grid), m_width(width), m_height(height) {}

std::vector<Vector2D> PathFind::findPath(const Vector2D& startWorld,
                                         const Vector2D& goalWorld) const {
    // --- World -> tile mapping -----------------------------------------------
    const auto toCol = [](float wx) {
        return static_cast<int>(std::floor(wx / TILE_SIZE));
    };
    const auto toRow = [](float wy) {
        return static_cast<int>(std::floor(wy / TILE_SIZE));
    };
    const auto toCentre = [](int col, int row) -> Vector2D {
        return { (static_cast<float>(col) + 0.5f) * TILE_SIZE,
                 (static_cast<float>(row) + 0.5f) * TILE_SIZE };
    };
    const auto inBounds = [this](int col, int row) {
        return col >= 0 && col < m_width && row >= 0 && row < m_height;
    };
    const auto isWalkable = [this](int col, int row) {
        return m_grid[row * m_width + col] == EMPTY_TILE;
    };
    const auto flatIndex = [this](int col, int row) {
        return row * m_width + col;
    };

    // --- Validate endpoints -------------------------------------------------
    const int sc = toCol(startWorld.x);
    const int sr = toRow(startWorld.y);
    const int gc = toCol(goalWorld.x);
    const int gr = toRow(goalWorld.y);

    if (!inBounds(sc, sr) || !isWalkable(sc, sr)) return {};
    if (!inBounds(gc, gr) || !isWalkable(gc, gr)) return {};

    const int startIdx = flatIndex(sc, sr);
    const int goalIdx  = flatIndex(gc, gr);

    if (startIdx == goalIdx) {
        return { toCentre(sc, sr) };
    }

    const int totalCells = m_width * m_height;

    // --- A* -----------------------------------------------------------------
    // Per-cell state: g cost (-1 = unvisited) and parent flat index.
    std::vector<float> gCost(totalCells, -1.0f);
    std::vector<int>   parent(totalCells, -1);

    // Open set as a min-heap on f.
    std::vector<AStarNode> heap;
    heap.reserve(256);

    const float h0 = octile(gc - sc, gr - sr);
    heap.push_back({ startIdx, -1, 0.0f, h0 });
    std::make_heap(heap.begin(), heap.end(), MinF{});
    gCost[startIdx] = 0.0f;

    // 8-directional neighbours: {dc, dr, cost}
    constexpr struct { int dc, dr; float cost; } kNeighbours[8] = {
        {  1,  0, 1.0f        },
        { -1,  0, 1.0f        },
        {  0,  1, 1.0f        },
        {  0, -1, 1.0f        },
        {  1,  1, 1.4142135f  },
        { -1,  1, 1.4142135f  },
        {  1, -1, 1.4142135f  },
        { -1, -1, 1.4142135f  },
    };

    bool found = false;

    while (!heap.empty()) {
        // Pop the node with the lowest f.
        std::pop_heap(heap.begin(), heap.end(), MinF{});
        AStarNode current = heap.back();
        heap.pop_back();

        // Skip stale entries (a cheaper path was already found for this cell).
        if (gCost[current.index] >= 0.0f && current.g > gCost[current.index] + EPSILON) {
            continue;
        }

        if (current.index == goalIdx) {
            found = true;
            break;
        }

        // Expand neighbours.
        const int curCol = current.index % m_width;
        const int curRow = current.index / m_width;

        for (const auto& nb : kNeighbours) {
            const int nc = curCol + nb.dc;
            const int nr = curRow + nb.dr;

            if (!inBounds(nc, nr) || !isWalkable(nc, nr)) continue;

            // Diagonal corner-clipping prevention:
            // Both orthogonal cells adjacent to this diagonal move must be walkable.
            if (nb.dc != 0 && nb.dr != 0) {
                if (!isWalkable(curCol + nb.dc, curRow) ||
                    !isWalkable(curCol,          curRow + nb.dr)) {
                    continue;
                }
            }

            const float tentativeG = current.g + nb.cost;
            const int   nbIdx      = flatIndex(nc, nr);

            if (gCost[nbIdx] < 0.0f || tentativeG < gCost[nbIdx] - EPSILON) {
                gCost[nbIdx]  = tentativeG;
                parent[nbIdx] = current.index;

                const float h = octile(gc - nc, gr - nr);
                heap.push_back({ nbIdx, current.index, tentativeG, tentativeG + h });
                std::push_heap(heap.begin(), heap.end(), MinF{});
            }
        }
    }

    if (!found) return {};

    // --- Reconstruct path ---------------------------------------------------
    std::vector<Vector2D> path;
    path.reserve(64);

    for (int idx = goalIdx; idx != -1; idx = parent[idx]) {
        const int col = idx % m_width;
        const int row = idx / m_width;
        path.push_back(toCentre(col, row));
    }

    std::reverse(path.begin(), path.end());
    return path;
}
