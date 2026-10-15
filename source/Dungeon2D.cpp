#include "Dungeon2D.hpp"
#include <random>

namespace {
// Minimum room size (5x5) and minimum partition dimension.
constexpr int MIN_ROOM_SIZE = 5;
constexpr int MIN_PARTITION_SIZE = 7;

// Finds an arbitrary room within a subtree.
const Rect* findLeafRoom(const BSPNode* node) {
    if (!node) return nullptr;
    if (node->isLeaf()) return &node->room;
    const Rect* leftRoom = findLeafRoom(node->left.get());
    return leftRoom ? leftRoom : findLeafRoom(node->right.get());
}

// Carves a room of at least 5x5 tiles inside a leaf node.
void carveRoom(BSPNode* node, std::mt19937& rng) {
    int regionW = node->region.right - node->region.left;
    int regionH = node->region.bottom - node->region.top;

    int maxW = (regionW >= MIN_ROOM_SIZE + 2) ? (regionW - 2) : MIN_ROOM_SIZE;
    int maxH = (regionH >= MIN_ROOM_SIZE + 2) ? (regionH - 2) : MIN_ROOM_SIZE;

    int roomW = (maxW > MIN_ROOM_SIZE) ? std::uniform_int_distribution<int>(MIN_ROOM_SIZE, maxW)(rng) : MIN_ROOM_SIZE;
    int roomH = (maxH > MIN_ROOM_SIZE) ? std::uniform_int_distribution<int>(MIN_ROOM_SIZE, maxH)(rng) : MIN_ROOM_SIZE;

    int maxStartX = node->region.right - roomW - 1;
    int minStartX = node->region.left + 1;
    int startX = (minStartX <= maxStartX) ? std::uniform_int_distribution<int>(minStartX, maxStartX)(rng) : node->region.left;

    int maxStartY = node->region.bottom - roomH - 1;
    int minStartY = node->region.top + 1;
    int startY = (minStartY <= maxStartY) ? std::uniform_int_distribution<int>(minStartY, maxStartY)(rng) : node->region.top;

    node->room = Rect{startX, startY, startX + roomW, startY + roomH};
}

// Recursively splits regions using binary space partitioning.
void splitNode(BSPNode* node, std::mt19937& rng) {
    int nodeW = node->region.right - node->region.left;
    int nodeH = node->region.bottom - node->region.top;

    bool canSplitV = nodeW >= 2 * MIN_PARTITION_SIZE;
    bool canSplitH = nodeH >= 2 * MIN_PARTITION_SIZE;

    if (!canSplitV && !canSplitH) {
        carveRoom(node, rng);
        return;
    }

    bool splitH = false;
    if (canSplitH && canSplitV) {
        if (static_cast<double>(nodeW) / nodeH >= 1.25) splitH = false;
        else if (static_cast<double>(nodeH) / nodeW >= 1.25) splitH = true;
        else splitH = std::uniform_int_distribution<int>(0, 1)(rng) == 1;
    } else {
        splitH = canSplitH;
    }

    node->left = std::make_unique<BSPNode>();
    node->right = std::make_unique<BSPNode>();

    if (splitH) {
        int splitOffset = std::uniform_int_distribution<int>(MIN_PARTITION_SIZE, nodeH - MIN_PARTITION_SIZE)(rng);
        node->left->region = Rect{node->region.left, node->region.top, node->region.right, node->region.top + splitOffset};
        node->right->region = Rect{node->region.left, node->region.top + splitOffset, node->region.right, node->region.bottom};
    } else {
        int splitOffset = std::uniform_int_distribution<int>(MIN_PARTITION_SIZE, nodeW - MIN_PARTITION_SIZE)(rng);
        node->left->region = Rect{node->region.left, node->region.top, node->region.left + splitOffset, node->region.bottom};
        node->right->region = Rect{node->region.left + splitOffset, node->region.top, node->region.right, node->region.bottom};
    }

    splitNode(node->left.get(), rng);
    splitNode(node->right.get(), rng);
}

// Connects left and right subtrees with 3-tile wide corridors.
void buildCorridors(BSPNode* node, std::vector<Corridor>& corridors) {
    if (!node || node->isLeaf()) return;

    buildCorridors(node->left.get(), corridors);
    buildCorridors(node->right.get(), corridors);

    const Rect* rA = findLeafRoom(node->left.get());
    const Rect* rB = findLeafRoom(node->right.get());
    if (!rA || !rB) return;

    int ax = rA->left + (rA->right - rA->left) / 2;
    int ay = rA->top + (rA->bottom - rA->top) / 2;
    int bx = rB->left + (rB->right - rB->left) / 2;
    int by = rB->top + (rB->bottom - rB->top) / 2;

    if (ax != bx) {
        int xMin = std::min(ax, bx) - 1;
        int xMax = std::max(ax, bx) + 2;
        corridors.push_back(Corridor{xMin, ay - 1, xMax, ay + 2});
    }

    if (ay != by) {
        int yMin = std::min(ay, by) - 1;
        int yMax = std::max(ay, by) + 2;
        corridors.push_back(Corridor{bx - 1, yMin, bx + 2, yMax});
    }
}
} // namespace

// Constructor initializing dimensions and ensuring root starts as null.
Dungeon2D::Dungeon2D(int widthTiles, int heightTiles)
    : width(widthTiles > 0 ? widthTiles : 0),
      height(heightTiles > 0 ? heightTiles : 0),
      root(nullptr) {}

// Builds the BSP tree, carves the rooms, and lays 3-tile wide corridors.
void Dungeon2D::generate(unsigned seed) {
    // Width and height must be positive and capable of holding a room.
    if (width < MIN_ROOM_SIZE || height < MIN_ROOM_SIZE) {
        return;
    }

    // Root must be null before generation begins.
    root = nullptr;
    corridors.clear();

    std::mt19937 rng(seed);

    root = std::make_unique<BSPNode>();
    root->region = Rect{0, 0, width, height};

    // Partition space and carve rooms in leaves.
    splitNode(root.get(), rng);

    // Recursively connect all subtrees.
    buildCorridors(root.get(), corridors);
}