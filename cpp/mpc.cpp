#include <iostream>
#include <vector>
#include <cmath>
#include <limits>

// Very simple MPC example for a 1D point mass
// Control input: acceleration

struct State {
    double x;
    double v;
};

State simulate(const State& s, double a, double dt) {
    State next;
    next.x = s.x + s.v * dt + 0.5 * a * dt * dt;
    next.v = s.v + a * dt;
    return next;
}

// Evaluate cost for a sequence of accelerations
double evaluate(const State& start, const std::vector<double>& uSeq, double dt, double target) {
    State cur = start;
    double cost = 0.0;
    for (double u : uSeq) {
        cur = simulate(cur, u, dt);
        double err = cur.x - target;
        cost += err * err + 0.1 * u * u; // simple quadratic cost
    }
    return cost;
}

std::vector<double> mpc(const State& start, double target, int horizon, double dt) {
    std::vector<double> bestSeq(horizon, 0.0);
    double bestCost = std::numeric_limits<double>::infinity();
    // Brute-force search over {-1,0,1} accelerations
    std::vector<double> options = {-1.0, 0.0, 1.0};
    std::vector<double> seq(horizon);
    std::function<void(int)> dfs = [&](int depth) {
        if (depth == horizon) {
            double c = evaluate(start, seq, dt, target);
            if (c < bestCost) {
                bestCost = c;
                bestSeq = seq;
            }
            return;
        }
        for (double u : options) {
            seq[depth] = u;
            dfs(depth + 1);
        }
    };
    dfs(0);
    return bestSeq;
}

int main() {
    State s{0.0, 0.0};
    double target = 10.0;
    double dt = 1.0;
    int horizon = 3;
    auto seq = mpc(s, target, horizon, dt);
    std::cout << "Chosen accelerations:";
    for (double u : seq) {
        std::cout << " " << u;
    }
    std::cout << std::endl;
    return 0;
}
