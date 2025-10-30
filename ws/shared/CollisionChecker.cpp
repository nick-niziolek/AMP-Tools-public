#include "CollisionChecker.h"

bool CollisionChecker::PointCollisionChecker(const Eigen::Vector2d& point, const amp::Environment2D& env) {
    // Loop over each obstacle in the environment
    for (const auto& obstacle : env.obstacles) {
        const auto& vertices = obstacle.verticesCCW();
        int intersection_count = 0;
        
        for (size_t i = 0; i < vertices.size(); ++i) {
            const auto& v1 = vertices[i];
            const auto& v2 = vertices[(i + 1) % vertices.size()]; // wraps around to first vertex
            
            // Check if the horizontal ray from 'point' intersects the edge (v1, v2)
            // The ray extends to the right (positive x direction) from point
            
            // First check: does the edge cross the horizontal line at point.y()?
            if ((v1.y() > point.y()) != (v2.y() > point.y())) {
                // Calculate the x-coordinate of the intersection
                // Using linear interpolation: x = v1.x + (point.y - v1.y) * (v2.x - v1.x) / (v2.y - v1.y)
                double x_intersect = v1.x() + (point.y() - v1.y()) * (v2.x() - v1.x()) / (v2.y() - v1.y());
                
                // Only count if intersection is to the RIGHT of the point
                if (x_intersect > point.x()) {
                    intersection_count++;
                }
            }
        }

        // If the intersection count is odd, the point is inside the obstacle
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

float CollisionChecker::PointToNearestVertexDistance(const Eigen::Vector2d& point, const amp::Environment2D& env) {
    float min_distance = std::numeric_limits<float>::max();
    for (const auto& obstacle : env.obstacles) {
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

bool CollisionChecker::pointInPolygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& vertices) {
    int n = vertices.size();
    bool inside = false;
    
    double x = point.x();
    double y = point.y();
    
    double p1x = vertices[0].x();
    double p1y = vertices[0].y();
    
    for (int i = 1; i <= n; i++) {
        double p2x = vertices[i % n].x();
        double p2y = vertices[i % n].y();
        
        if (y > std::min(p1y, p2y)) {
            if (y <= std::max(p1y, p2y)) {
                if (x <= std::max(p1x, p2x)) {
                    if (p1y != p2y) {
                        double xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x;
                        if (p1x == p2x || x <= xinters) {
                            inside = !inside;
                        }
                    }
                }
            }
        }
        p1x = p2x;
        p1y = p2y;
    }
    
    return inside;
}

Eigen::Vector2d CollisionChecker::closestPointOnSegment(const Eigen::Vector2d& point, 
                                      const Eigen::Vector2d& segStart, 
                                      const Eigen::Vector2d& segEnd) {
    Eigen::Vector2d ab = segEnd - segStart;
    
    // If segment has zero length
    double lengthSquared = ab.squaredNorm();
    if (lengthSquared < 1e-10) {
        return segStart;
    }
    
    // Parameter t for closest point (clamped to [0, 1])
    double t = (point - segStart).dot(ab) / lengthSquared;
    t = std::max(0.0, std::min(1.0, t));
    
    // Closest point on segment
    return segStart + t * ab;
}

bool CollisionChecker::circlePolygonCollision(const Eigen::Vector2d& point, 
                           double radius, 
                           const std::vector<Eigen::Vector2d>& vertices) {
    // First check if center is inside polygon
    if (pointInPolygon(point, vertices)) {
        return true;
    }
    
    // Find minimum distance to any edge
    double minDistanceSquared = std::numeric_limits<double>::infinity();
    
    for (size_t i = 0; i < vertices.size(); i++) {
        const Eigen::Vector2d& segStart = vertices[i];
        const Eigen::Vector2d& segEnd = vertices[(i + 1) % vertices.size()];
        
        Eigen::Vector2d closest = closestPointOnSegment(point, segStart, segEnd);
        double distSquared = (point - closest).squaredNorm();
        
        minDistanceSquared = std::min(minDistanceSquared, distSquared);
    }
    
    // Collision if closest distance is less than radius
    return minDistanceSquared < (radius * radius);
}

/////////////////////////////////////////////////////////////////

bool CollisionChecker::GenericCollisionChecker(const std::vector<Eigen::Vector2d>& agent_start, const std::vector<Eigen::Vector2d>& agent_end, const amp::Environment2D& env) {
    if (agent_start.size() == 1 && agent_end.size() == 1) {
        // Point agent
        if (PointCollisionChecker(agent_start[0], env) || PointCollisionChecker(agent_end[0], env)) {
            return true; // Collision detected
        } else {
            if (agent_start[0] == agent_end[0]) {
                return false; // No movement, no collision
            }
            // Check path collision
            if (LineCollisionChecker(agent_start[0], agent_end[0], env)) {
                return true; // Collision detected
            } else {
                return false; // No collisions
            }
        }
    } else if (agent_start.size() == 2 && agent_end.size() == 2) {
        // Error, not a real polygon
        throw std::invalid_argument("Agent with 2 vertices is not a valid polygon.");
    } else {
        // Polygon agent
        for (const auto& obstacle : env.obstacles) {
            const auto& vertices = obstacle.verticesCCW();
            if (convexPolygonsIntersect(agent_start, vertices) || convexPolygonsIntersect(agent_end, vertices)) {
                return true; // Collision detected
            }
        }
        // Check if path between vertices collides
        if (agent_start == agent_end) {
            return false; // No movement, no collision
        }
        for (size_t i = 0; i < agent_start.size(); ++i) {
            Eigen::Vector2d start_vertex = agent_start[i];
            Eigen::Vector2d end_vertex = agent_end[i];
            if (LineCollisionChecker(start_vertex, end_vertex, env)) {
                return true; // Collision detected
            }
        }
        
        return false; // No collisions
    }
}

///////////////////////////////////////////////////////////////

bool MultiDiscConfigSpace::inCollision(const Eigen::VectorXd& cspace_state) const {
    // Check if the cspace_state is twice the size, and use line collision if true
    if (cspace_state.size() == 4 * m_problem.numAgents()) {
        if (inLineCollision(cspace_state.head(cspace_state.size() / 2), cspace_state.tail(cspace_state.size() / 2))) {
            return true;
        }
        for (size_t i = 0; i < 2; i++) {
            Eigen::VectorXd state = cspace_state.segment(i * (cspace_state.size() / 2), cspace_state.size() / 2);
            if (inPointCollision(state)) {
                return true;
            }
        }
    }
    else {
        if (inPointCollision(cspace_state)) {
            return true;
        }
    }
    return false; // No collisions
}

bool MultiDiscConfigSpace::inPointCollision(const Eigen::VectorXd& cspace_state) const {
    // Each agent has 2 dimensions (x, y)
    std::size_t n_agents = m_problem.numAgents();

    // Reuse a single checker instance for efficiency
    CollisionChecker checker;

    for (std::size_t i = 0; i < n_agents; ++i) {
        Eigen::Vector2d agent_pos(cspace_state(2 * i), cspace_state(2 * i + 1));
        double agent_radius = m_problem.agent_properties[i].radius;

        // Check collision with obstacles
        for (const auto& obstacle : m_problem.obstacles) {
            if (checker.circlePolygonCollision(agent_pos, agent_radius, obstacle.verticesCCW())) {
                return true; // Collision detected
            }
        }

        // Check collision with other agents
        for (std::size_t j = i + 1; j < n_agents; ++j) {
            Eigen::Vector2d other_agent_pos(cspace_state(2 * j), cspace_state(2 * j + 1));
            double other_agent_radius = m_problem.agent_properties[j].radius;

            double dist = (agent_pos - other_agent_pos).norm();
            if (dist < (agent_radius + other_agent_radius)) {
                return true; // Collision between agents
            }
        }
    }
    return false; // No collisions
}

bool MultiDiscConfigSpace::inLineCollision(const Eigen::VectorXd& cspace_start, const Eigen::VectorXd& cspace_end) const {
    std::size_t n_agents = m_problem.numAgents();
    CollisionChecker checker;

    for (std::size_t i = 0; i < n_agents; ++i) {
        Eigen::Vector2d start_pos(cspace_start(2 * i), cspace_start(2 * i + 1));
        Eigen::Vector2d end_pos(cspace_end(2 * i), cspace_end(2 * i + 1));
        double agent_radius = m_problem.agent_properties[i].radius;

        // Get lines offset radius from agent path
        Eigen::Vector2d path_dir = end_pos - start_pos;
        Eigen::Vector2d normal(-path_dir.y(), path_dir.x());
        normal.normalize();
        Eigen::Vector2d offset = normal * agent_radius;
        Eigen::Vector2d line1_start = start_pos + offset;
        Eigen::Vector2d line1_end = end_pos + offset;
        // Check collision with obstacles for line1
        if (checker.LineCollisionChecker(line1_start, line1_end, m_problem)) {
            return true; // Collision detected
        }
        Eigen::Vector2d line2_start = start_pos - offset;
        Eigen::Vector2d line2_end = end_pos - offset;
        // Check collision with obstacles for line2
        if (checker.LineCollisionChecker(line2_start, line2_end, m_problem)) {
            return true; // Collision detected
        }
    }
    return false; // No collisions
}