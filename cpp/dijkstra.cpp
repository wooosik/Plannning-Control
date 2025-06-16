#include <iostream>
#include <queue>
#include <vector>
#include <limits>

// Simple Dijkstra shortest path for weighted graph represented as adjacency list

struct Edge {
    int to;
    double cost;
};

std::vector<double> dijkstra(int n, const std::vector<std::vector<Edge>>& adj, int start) {
    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    dist[start] = 0.0;
    using P = std::pair<double,int>; // cost, node
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
    pq.push({0.0, start});

    while (!pq.empty()) {
        auto [d,u] = pq.top();
        pq.pop();
        if (d != dist[u]) continue;
        for (const auto& e : adj[u]) {
            double nd = dist[u] + e.cost;
            if (nd < dist[e.to]) {
                dist[e.to] = nd;
                pq.push({nd, e.to});
            }
        }
    }
    return dist;
}

int main() {
    int n = 5;
    std::vector<std::vector<Edge>> adj(n);
    adj[0].push_back({1, 2.0});
    adj[0].push_back({2, 4.0});
    adj[1].push_back({2, 1.0});
    adj[1].push_back({3, 7.0});
    adj[2].push_back({4, 3.0});
    adj[3].push_back({4, 1.0});

    auto dist = dijkstra(n, adj, 0);
    for (int i = 0; i < n; ++i) {
        std::cout << "0 -> " << i << " = " << dist[i] << std::endl;
    }
    return 0;
}
