#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <iostream>

using namespace gtsam;

int main() {
    // Create a factor graph
    NonlinearFactorGraph graph;

    // Add a prior on pose x1
    Pose2 priorMean(0.0, 0.0, 0.0); // prior at origin
    auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
    graph.emplace_shared<PriorFactor<Pose2>>(Symbol('x', 1), priorMean, priorNoise);

    // Add odometry between x1 and x2
    Pose2 odometry(2.0, 0.0, 0.0); // 2m forward
    auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
    graph.emplace_shared<BetweenFactor<Pose2>>(Symbol('x', 1), Symbol('x', 2), odometry, odometryNoise);

    // Create initial estimate
    Values initialEstimate;
    initialEstimate.insert(Symbol('x', 1), Pose2(0.5, 0.0, 0.2));
    initialEstimate.insert(Symbol('x', 2), Pose2(2.3, 0.1, -0.2));

    // Optimize
    GaussNewtonOptimizer optimizer(graph, initialEstimate);
    Values result = optimizer.optimize();

    // Print result
    result.print("Final Result:\n");

    std::cout << "GTSAM test completed successfully!" << std::endl;
    return 0;
}
