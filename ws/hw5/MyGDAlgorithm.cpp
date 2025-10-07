#include "MyGDAlgorithm.h"

// Implement your plan method here, similar to HW2:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    // Make path object
    amp::Path2D path;
    // Make potential function object
    MyPotentialFunction potential(problem,d_star,zetta,Q_star,eta);
    // Settings
    double step_size = 0.01; // Step size for gradient descent
    double goal_rad = 0.25;
    double tol = 1e-5;
    int max_iters = 1000000;

    // Initialize at start
    Eigen::Vector2d q = problem.q_init;
    path.waypoints.push_back(q);

    for (int iter = 0; iter < max_iters; ++iter) {
        // Compute gradient
        Eigen::Vector2d grad = potential.getGradient(q);
        // Check for invalid gradients
        if (!grad.allFinite() || grad.norm() > 1e6) {
            std::cout << "Invalid gradient detected at iteration " << iter << ", terminating." << std::endl;
            break;
        }
        // Update position
        q = q + step_size * grad;
        path.waypoints.push_back(q);
        //std::cout << "Iter: " << iter << ", q: " << q.transpose() << ", Grad: " << grad.transpose() << std::endl;
        // Check for convergence to goal
        if ((q - problem.q_goal).norm() <= goal_rad) {
            path.valid = true;
            std::cout << "Terminated due to reaching goal at iteration " << iter << "." << std::endl;
            path.waypoints.push_back(problem.q_goal); // Ensure exact goal is included
            break;
        }
        // Check for small gradient
        if (grad.norm() < tol) {
            //std::cout << "Small gradient at iteration " << iter << ", for q: " << q.transpose() << std::endl;
            // Check what kind of minima/stuck point it's in
            bool is_not_minima = false;
            Eigen::Vector2d random_dir;
            for (double i = 0; i < 2*M_PI; i += 0.1) {
                Eigen::Vector2d dir = Eigen::Vector2d(std::cos(i), std::sin(i));
                // Check if this direction is a descent direction
                random_dir = dir;
                // Take a small step in this direction and see if potential decreases
                Eigen::Vector2d test_q = q + step_size/10.0 * random_dir;
                double test_U = potential(test_q);
                double current_U = potential(q);
                if (test_U < current_U) {
                    is_not_minima = true;
                    //std::cout << "Found descent direction at iteration " << iter << " in random direction " << random_dir.transpose() << std::endl;
                    break;
                }
            }

            if (!is_not_minima) {
                std::cout << "Terminated due to small gradient at iteration " << iter << "." << std::endl;
                return path; // Stuck in local minima, terminate
            }
            // Otherwise, nudge in direction found above
            // q = q + step_size * random_dir.normalized();
            // path.waypoints.push_back(q);
            //std::cout << "Nudged at iteration " << iter << ", new q: " << q.transpose() << std::endl;
        }
    }
    if (path.waypoints.size() >= max_iters) {
        std::cout << "Terminated due to reaching max iterations." << std::endl;
    }
    return path;
}

Eigen::Vector2d MyPotentialFunction::getNearestVertex(const Eigen::Vector2d&q) const {
    double min_dist = std::numeric_limits<double>::infinity();
    Eigen::Vector2d nearest_vertex = Eigen::Vector2d::Zero();
    // Loop over obstacles
    for (const amp::Obstacle2D& obs : problem.obstacles) {
        // Loop over vertices
        const std::vector<Eigen::Vector2d>& vertices = obs.verticesCCW();
        for (const Eigen::Vector2d& v : vertices) {
            double dist = (q - v).norm();
            if (dist < min_dist) {
                min_dist = dist;
                nearest_vertex = v;
            }
        }
    }
    return nearest_vertex;
}

std::vector<std::pair<double, Eigen::Vector2d>> MyPotentialFunction::getNearestObstacleInfo(const Eigen::Vector2d& q) const {
    std::vector<std::pair<double, Eigen::Vector2d>> obstacle_candidates;

    for (const amp::Obstacle2D& obs : problem.obstacles) {
        const auto& vertices = obs.verticesCCW();
        double min_dist = std::numeric_limits<double>::infinity();
        Eigen::Vector2d closest_point = Eigen::Vector2d::Zero();

        // Loop over each edge (v_i, v_{i+1})
        for (size_t i = 0; i < vertices.size(); ++i) {
            const Eigen::Vector2d& v1 = vertices[i];
            const Eigen::Vector2d& v2 = vertices[(i + 1) % vertices.size()];
            Eigen::Vector2d edge = v2 - v1;

            double t = (q - v1).dot(edge) / edge.squaredNorm();
            t = std::max(0.0, std::min(1.0, t)); // clamp to segment

            Eigen::Vector2d proj = v1 + t * edge;  // closest point on edge
            double dist = (q - proj).norm();

            if (dist < min_dist) {
                min_dist = dist;
                closest_point = proj;
            }
        }

        obstacle_candidates.push_back({min_dist, closest_point - q});
    }

    return obstacle_candidates;
}

double MyPotentialFunction::operator()(const Eigen::Vector2d& q) const {
    // --- Attractive potential ---
    Eigen::Vector2d to_goal = problem.q_goal - q;
    double U_att;
    if (to_goal.norm() <= d_star) {
        U_att = 0.5 * zetta * std::pow(to_goal.norm(), 2);
    } else {
        U_att = d_star * zetta * to_goal.norm() - 0.5 * zetta * std::pow(d_star, 2);
    }

    // --- Repulsive potential (boundary-based) ---
    double U_rep = 0.0;
    for (const auto& [dist, vec] : getNearestObstacleInfo(q)) {
        double safe_dist = std::max(dist, 1e-6); // avoid div by 0
        if (safe_dist <= Q_star) {
            U_rep += 0.5 * eta * std::pow((1.0 / safe_dist - 1.0 / Q_star), 2);
        }
    }

    // --- Centroid repulsion (milder) ---
    double U_centroid = 0.0;
    double alpha = 15.0 * eta; // scale factor (tune as needed)
    // double alpha = 0.0 * eta; // weaker than boundary repulsion
    for (const amp::Obstacle2D& obs : problem.obstacles) {
        // Compute centroid of obstacle polygon
        Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
        const auto& verts = obs.verticesCCW();
        for (const auto& v : verts) centroid += v;
        centroid /= verts.size();

        double dist = (q - centroid).norm();
        double safe_dist = std::max(dist, 1e-6);
        U_centroid += 0.5 * alpha * std::pow((1.0 / safe_dist), 2);
    }

    return U_att + U_rep + U_centroid;
}

Eigen::Vector2d MyPotentialFunction::getGradient(const Eigen::Vector2d& q) const {
    // --- Attractive gradient ---
    Eigen::Vector2d to_goal = problem.q_goal - q;
    Eigen::Vector2d grad_U_att;
    if (to_goal.norm() <= d_star) {
        grad_U_att = zetta * to_goal;
    } else {
        grad_U_att = d_star * zetta * to_goal.normalized();
    }

    // --- Repulsive gradient (boundary-based) ---
    Eigen::Vector2d grad_U_rep = Eigen::Vector2d::Zero();
    for (const auto& [dist, vec] : getNearestObstacleInfo(q)) {
        double safe_dist = std::max(dist, 1e-6);
        if (safe_dist <= Q_star) {
            Eigen::Vector2d dir = (-vec).normalized(); 
            // vec = (closest_point - q), so -vec = (q - closest_point)
            grad_U_rep += eta * (1.0 / safe_dist - 1.0 / Q_star)
                               * (1.0 / (safe_dist * safe_dist))
                               * dir;
        }
    }

    // --- Centroid repulsion gradient ---
    Eigen::Vector2d grad_U_centroid = Eigen::Vector2d::Zero();
    double alpha = 15.0 * eta; // weaker than boundary repulsion
    // double alpha = 0.0 * eta; // weaker than boundary repulsion
    for (const amp::Obstacle2D& obs : problem.obstacles) {
        Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
        const auto& verts = obs.verticesCCW();
        for (const auto& v : verts) centroid += v;
        centroid /= verts.size();

        Eigen::Vector2d vec = centroid - q; // pointing from q to centroid
        double dist = vec.norm();
        double safe_dist = std::max(dist, 1e-6);
        Eigen::Vector2d dir = (-vec).normalized(); // (q - centroid)
        grad_U_centroid += alpha * (1.0 / safe_dist)
                                    * (1.0 / (safe_dist * safe_dist))
                                    * dir;
    }

    return grad_U_att + grad_U_rep + grad_U_centroid;
}
