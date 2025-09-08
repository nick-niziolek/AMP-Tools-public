#include "CollisionChecker.h"

bool CollisionChecker::PointCollisionChecker(const Eigen::Vector2d& point, const amp::Problem2D& problem) {
    // Loop over each obstacle in the problem
    for (const auto& obstacle : problem.obstacles) {
        // Check if the point is inside the obstacle by using ray casting and horizontal line
        const auto& vertices = obstacle.verticesCCW();
        int intersection_count = 0;
        for (size_t i = 0; i < vertices.size(); ++i) {
            const auto& v1 = vertices[i];
            const auto& v2 = vertices[(i + 1) % vertices.size()]; // wraps around to first vertex
            // Check if the horizontal ray from 'point' intersects the edge (v1, v2)
            std::cout << "v1.y(): " << v1.y() << ", v2.y(): " << v2.y() << ", point.y(): " << point.y() << std::endl;
            if ((v1.y() > point.y()) != (v2.y() > point.y())) {
                intersection_count++;
            }
        }

        // If the intersection count is odd, the point is inside the obstacle
        std::cout << "Intersection count for obstacle: " << intersection_count << std::endl;
        if (intersection_count % 2 == 1) {
            return true;
        }
    }

    // If no obstacles contain the point, return false
    return false;
}

bool CollisionChecker::LineCollisionChecker(const Eigen::Vector2d& start,
                                            const Eigen::Vector2d& end,
                                            const amp::Problem2D& problem) {
    auto cross = [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
        return a.x() * b.y() - a.y() * b.x();
    };

    for (const auto& obstacle : problem.obstacles) {
        const auto& vertices = obstacle.verticesCCW();
        for (size_t i = 0; i < vertices.size(); ++i) {
            const Eigen::Vector2d& v1 = vertices[i];
            const Eigen::Vector2d& v2 = vertices[(i + 1) % vertices.size()];

            Eigen::Vector2d r = end - start;
            Eigen::Vector2d s = v2 - v1;

            double rxs = cross(r, s);
            double qpxr = cross(v1 - start, r);

            // Case 1: parallel
            if (std::abs(rxs) < 1e-9) {
                if (std::abs(qpxr) < 1e-9) {
                    // Collinear case: check overlap
                    auto onSegment = [](const Eigen::Vector2d& a,
                                        const Eigen::Vector2d& b,
                                        const Eigen::Vector2d& c) {
                        return (c.x() >= std::min(a.x(), b.x()) &&
                                c.x() <= std::max(a.x(), b.x()) &&
                                c.y() >= std::min(a.y(), b.y()) &&
                                c.y() <= std::max(a.y(), b.y()));
                    };
                    if (onSegment(start, end, v1) || onSegment(start, end, v2) ||
                        onSegment(v1, v2, start) || onSegment(v1, v2, end)) {
                        return true; // overlap
                    }
                }
                continue; // parallel non-collinear
            }

            // Case 2: not parallel
            double t = cross(v1 - start, s) / rxs;
            double u = cross(v1 - start, r) / rxs;

            if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
                return true; // intersection
            }
        }
    }
    return false; // no collisions
}