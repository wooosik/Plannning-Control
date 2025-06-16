#include <iostream>
#include <queue>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <utility>

// Simple A* pathfinding on a 2D grid
// 0 = free cell, 1 = obstacle

struct Node {
    int x, y;
    double g;
    double f;
    int parent;
};

static int index(int x, int y, int width) {
    return y * width + x;
}

std::vector<std::pair<int, int>> reconstruct(const std::vector<Node>& nodes, int current) {
    std::vector<std::pair<int, int>> path;
    while (current != -1) {
        path.push_back({nodes[current].x, nodes[current].y});
        current = nodes[current].parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<std::pair<int, int>> astar(const std::vector<int>& grid, int width, int height,
                                       std::pair<int, int> start, std::pair<int, int> goal) {
    auto heuristic = [&](int x, int y) {
        return std::abs(goal.first - x) + std::abs(goal.second - y);
    };

    std::vector<Node> nodes(width * height);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int id = index(x, y, width);
            nodes[id] = {x, y, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), -1};
        }
    }

    auto cmp = [&](int lhs, int rhs) { return nodes[lhs].f > nodes[rhs].f; };
    std::priority_queue<int, std::vector<int>, decltype(cmp)> open(cmp);

    int startIdx = index(start.first, start.second, width);
    nodes[startIdx].g = 0;
    nodes[startIdx].f = heuristic(start.first, start.second);
    open.push(startIdx);

    const int moves[4][2] = { {1,0},{-1,0},{0,1},{0,-1} };

    while (!open.empty()) {
        int current = open.top();
        open.pop();
        if (nodes[current].x == goal.first && nodes[current].y == goal.second) {
            return reconstruct(nodes, current);
        }
        for (auto m : moves) {
            int nx = nodes[current].x + m[0];
            int ny = nodes[current].y + m[1];
            if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
            int nid = index(nx, ny, width);
            if (grid[nid]) continue; // obstacle
            double tentative_g = nodes[current].g + 1.0;
            if (tentative_g < nodes[nid].g) {
                nodes[nid].parent = current;
                nodes[nid].g = tentative_g;
                nodes[nid].f = tentative_g + heuristic(nx, ny);
                open.push(nid);
            }
        }
    }
    return {};
}

int main() {
    int width = 5;
    int height = 5;
    std::vector<int> grid(width * height, 0);
    grid[index(2,2,width)] = 1; // obstacle

    auto path = astar(grid, width, height, {0,0}, {4,4});
    for (auto p : path) {
        std::cout << "(" << p.first << "," << p.second << ") ";
    }
    std::cout << std::endl;
    return 0;
}
