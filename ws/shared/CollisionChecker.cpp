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
                                            const amp::Environment2D& env) {
    auto cross = [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
        return a.x() * b.y() - a.y() * b.x();
    };

    for (const auto& obstacle : env.obstacles) {
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

float CollisionChecker::PointToNearestVertexDistance(const Eigen::Vector2d& point, const amp::Problem2D& problem) {
    float min_distance = std::numeric_limits<float>::max();
    for (const auto& obstacle : problem.obstacles) {
        const auto& vertices = obstacle.verticesCCW();
        for (size_t i = 0; i < vertices.size(); ++i) {
            float distance = (point - vertices[i]).norm();
            if (distance < min_distance) {
                min_distance = distance;
            }
        }
    }
    return min_distance;
}

float CollisionChecker::PointToLineAndPoint(const Eigen::Vector2d& point,
                                            const Eigen::Vector2d& line_direction,
                                            const Eigen::Vector2d& line_point) {
    //Point Line Distance
    float numerator = std::abs((point - line_point).x() * line_direction.y() - (point - line_point).y() * line_direction.x());
    float denominator = line_direction.norm();
    return numerator / denominator;
}

bool CollisionChecker::isManipStateFeasible(const amp::LinkManipulator2D& manip, const amp::ManipulatorState& state, const amp::Environment2D& env) {
    for (uint32_t i = 0; i < manip.nLinks(); ++i) {
        Eigen::Vector2d joint_start = manip.getJointLocation(state, i);
        Eigen::Vector2d joint_end = manip.getJointLocation(state, i + 1);
        if (LineCollisionChecker(joint_start, joint_end, env)) {
            return false; // In collision
        }
    }
    return true; // No collisions
}

bool CollisionChecker::isCellColliding(const Eigen::Vector2d& cell_center, double cell_size_x, double cell_size_y, const amp::Environment2D& env) {
    // Calculate the half size of the cell in each dimension
    double half_size_x = cell_size_x / 2.0;
    double half_size_y = cell_size_y / 2.0;

    // Define the four corners of the cell
    Eigen::Vector2d bottom_left(cell_center.x() - half_size_x, cell_center.y() - half_size_y);
    Eigen::Vector2d bottom_right(cell_center.x() + half_size_x, cell_center.y() - half_size_y);
    Eigen::Vector2d top_right(cell_center.x() + half_size_x, cell_center.y() + half_size_y);
    Eigen::Vector2d top_left(cell_center.x() - half_size_x, cell_center.y() + half_size_y);

    // Get vector of cell corners
    std::vector<Eigen::Vector2d> cell_corners = {bottom_left, bottom_right, top_right, top_left};

    // Loop over obstacles in the environment
    for (const auto& obstacle : env.obstacles) {
        const auto& vertices = obstacle.verticesCCW();
        if (convexPolygonsIntersect(cell_corners, vertices)) {
            return true; // Cell collides with obstacle
        }
    }

    return false; // Cell is free of collisions
}

// Helper: project polygon onto axis and get [min, max]
std::pair<double, double> CollisionChecker::projectPolygon(const std::vector<Eigen::Vector2d>& poly, const Eigen::Vector2d& axis) {
    double min_proj = std::numeric_limits<double>::infinity();
    double max_proj = -std::numeric_limits<double>::infinity();
    for (const auto& v : poly) {
        double proj = v.dot(axis);
        if (proj < min_proj) min_proj = proj;
        if (proj > max_proj) max_proj = proj;
    }
    return {min_proj, max_proj};
}

// Helper: check if projections overlap
bool CollisionChecker::overlapOnAxis(const std::vector<Eigen::Vector2d>& polyA, const std::vector<Eigen::Vector2d>& polyB, const Eigen::Vector2d& axis) {
    auto [minA, maxA] = projectPolygon(polyA, axis);
    auto [minB, maxB] = projectPolygon(polyB, axis);
    return !(maxA < minB || maxB < minA);
}

// Main intersection test
bool CollisionChecker::convexPolygonsIntersect(const std::vector<Eigen::Vector2d>& polyA, const std::vector<Eigen::Vector2d>& polyB) {
    // Collect all potential separating axes (edge normals)
    auto checkAxes = [&](const std::vector<Eigen::Vector2d>& poly1, const std::vector<Eigen::Vector2d>& poly2) {
        size_t n = poly1.size();
        for (size_t i = 0; i < n; ++i) {
            Eigen::Vector2d edge = poly1[(i + 1) % n] - poly1[i];
            Eigen::Vector2d normal(-edge.y(), edge.x());  // perpendicular vector

            // Normalize to avoid numerical issues
            normal.normalize();

            // If projections don't overlap on this axis → no intersection
            if (!overlapOnAxis(poly1, poly2, normal))
                return false;
        }
        return true;
    };

    // Test all edges from both polygons
    return checkAxes(polyA, polyB) && checkAxes(polyB, polyA);
}