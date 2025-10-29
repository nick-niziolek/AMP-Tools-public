# include "MySamplingBasedPlanners.h"
# include "CollisionChecker.h"
# include "CSConstructors.h"
# include "MyAStar.h"
# include "AMPCore.h"

// Implement your PRM algorithm here
amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    double x_min = problem.x_min;
    double x_max = problem.x_max;
    double y_min = problem.y_min;
    double y_max = problem.y_max;
    CollisionChecker checker;

    // Make graph
    auto graphPtr = std::make_shared<amp::Graph<double>>();
    m_roadmap = graphPtr;    
    std::map<amp::Node, Eigen::Vector2d> nodes;

    // Sample points in the configuration space
    std::vector<Eigen::Vector2d> vertex_set;
    vertex_set.push_back(problem.q_init);
    vertex_set.push_back(problem.q_goal);
    while (vertex_set.size() < m_n_samples + 2) {
        Eigen::Vector2d sample = Eigen::Vector2d::Random();
        sample.x() = x_min + (x_max - x_min) * (sample.x() + 1) / 2;
        sample.y() = y_min + (y_max - y_min) * (sample.y() + 1) / 2;
        // Check if point is valid
        if (!checker.PointCollisionChecker(sample,problem)) {
            vertex_set.push_back(sample);
        }
    }
    // Add vertices to the graph
    for (amp::Node i = 0; i < vertex_set.size(); ++i) nodes[i] = vertex_set[i]; // Add point-index pair to the map

    m_nodes.clear();  // Clear previous nodes
    for (amp::Node i = 0; i < vertex_set.size(); ++i) {
        m_nodes[i] = vertex_set[i];  // Store in member variable
    }

    // Create edges between points within a certain radius
    std::vector<std::tuple<amp::Node, amp::Node, double>> edges;
    for (size_t i = 0; i < vertex_set.size(); ++i) {
        for (size_t j = 0; j < vertex_set.size(); ++j) {
            if (i != j) {
                double distance = (vertex_set[i] - vertex_set[j]).norm();
                if (distance <= m_radius) {
                    // Check if the edge is valid
                    if (!checker.LineCollisionChecker(vertex_set[i], vertex_set[j], problem)) {
                        edges.push_back(std::make_tuple(i, j, distance));
                    }
                }
            }
        }
    }

    // Connect the edges in the graph
    for (const auto& [from, to, weight] : edges) graphPtr->connect(from, to, weight); 

    // Use A* to find the path
    // Construct amp::ShortestPathProblem
    amp::ShortestPathProblem problem_astar;
    problem_astar.init_node = 0;
    problem_astar.goal_node = 1;
    problem_astar.graph = graphPtr;
    MyAStarAlgo astar;

    // Create zero heuristic as std::function
    amp::SearchHeuristic zero_heuristic;

    auto results = astar.search(problem_astar, zero_heuristic);

    if (!results.success) {
        // std::cout << "No path found!" << std::endl;
        return path; // Return empty path
    }

    if (!m_path_smooth) {
            for (const auto& node : results.node_path) {
                path.waypoints.push_back(vertex_set[node]);
            }
            return path;
        } else {
            int smoothing_iter = 1000;
            std::list<amp::Node> smoothed_path = results.node_path;
            for (int i = 0; i < smoothing_iter; ++i) {
                if (smoothed_path.size() <= 2) break;
                int idx1 = rand() % (smoothed_path.size() - 1);
                int idx2 = rand() % (smoothed_path.size() - 1);
                if (idx1 == idx2) continue;
                if (idx1 > idx2) std::swap(idx1, idx2);
                if (idx2 - idx1 < 2) continue; // Need at least one node between
                // Check if line between point 1 and point 2 is valid
                auto it1 = smoothed_path.begin();
                std::advance(it1, idx1);
                auto it2 = smoothed_path.begin();
                std::advance(it2, idx2);
                if (!checker.LineCollisionChecker(vertex_set[*it1], vertex_set[*it2], problem)) {
                    // Remove points between point 1 and point 2
                    smoothed_path.erase(std::next(it1), it2);
                }
            }
            for (const auto& node : smoothed_path) {
                path.waypoints.push_back(vertex_set[node]);
            }
            return path;
        }

    return path;
}

// Constructor
// MyRRT::MyRRT(double goal_bias, double step_size, int max_iterations, double epsilon)
//     : m_goal_bias(goal_bias), m_step_size(step_size), m_max_iterations(max_iterations), m_epsilon(epsilon) {}

// Implement your RRT algorithm here
amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    double x_min = problem.x_min;
    double x_max = problem.x_max;
    double y_min = problem.y_min;
    double y_max = problem.y_max;
    CollisionChecker checker;

    // Make graph
    auto graphPtr = std::make_shared<amp::Graph<double>>();
    m_roadmap = graphPtr;    
    std::map<amp::Node, Eigen::Vector2d> nodes;
    nodes[0] = problem.q_init;
    m_nodes.clear();  // Clear previous nodes
    m_nodes[0] = problem.q_init;  // Store in member variable

    // Build tree until goal reached
    int iter = 0;
    std::vector<Eigen::Vector2d> vertex_set;
    vertex_set.push_back(problem.q_init);
    bool goal_reached = false;

    while(iter <= m_max_iterations) {
        iter++;
        Eigen::Vector2d sample;
        // Get sample with goal bias
        if (((double) rand() / RAND_MAX) < m_goal_bias) {
            sample = problem.q_goal; 
        } else {
            sample = Eigen::Vector2d::Random();
            sample.x() = x_min + (x_max - x_min) * (sample.x() + 1) / 2;
            sample.y() = y_min + (y_max - y_min) * (sample.y() + 1) / 2;
        }
        // Get nearest vertex
        double min_distance = std::numeric_limits<double>::infinity();
        int nearest_index = -1;
        for (int i = 0; i < vertex_set.size(); ++i) {
            double distance = (vertex_set[i] - sample).norm();
            if (distance < min_distance) {
                min_distance = distance;
                nearest_index = i;
            }
        }
        // Grow tree from vertex towards sample for step
        Eigen::Vector2d direction = (sample - vertex_set[nearest_index]).normalized();
        Eigen::Vector2d new_point = vertex_set[nearest_index] + direction * m_step_size;
        // Check if new point is valid
        if (!checker.PointCollisionChecker(new_point,problem) && !checker.LineCollisionChecker(vertex_set[nearest_index], new_point, problem)) {
            // Add new point to vertex set
            vertex_set.push_back(new_point);
            int new_index = vertex_set.size() - 1;
            nodes[new_index] = new_point; // Add point-index pair to the map
            m_nodes[new_index] = new_point;  // Store in member variable
            // Connect new point to nearest vertex
            double distance = (new_point - vertex_set[nearest_index]).norm();
            graphPtr->connect(nearest_index, new_index, distance);
            if ((new_point - problem.q_goal).norm() < m_epsilon) {
                // Goal reached
                graphPtr->connect(new_index, 1, (new_point - problem.q_goal).norm());
                goal_reached = true;
                break;
            }
        }
    }

    if (goal_reached) {
        // Make the path from the nodes
        // Use A* to find the path
        // Construct amp::ShortestPathProblem
        amp::ShortestPathProblem problem_astar;
        problem_astar.init_node = 0;
        problem_astar.goal_node = vertex_set.size() - 1;
        problem_astar.graph = graphPtr;
        MyAStarAlgo astar;

        // Create zero heuristic as std::function
        amp::SearchHeuristic zero_heuristic;

        auto results = astar.search(problem_astar, zero_heuristic);

        if (!results.success) {
            // std::cout << "No path found!" << std::endl;
            return path; // Return empty path
        }

        for (const auto& node : results.node_path) {
            path.waypoints.push_back(vertex_set[node]);
        }

        path.waypoints.push_back(problem.q_goal); // Add goal at the end
    }

    return path;
}

std::vector<Eigen::VectorXd> N_Dim_RRT::plan(const amp::ConfigurationSpace& space, 
    const Eigen::VectorXd& q_init,
    const Eigen::VectorXd& q_goal,
    const std::vector<amp::Path2D>& other_paths,
    const std::vector<double>& other_radii,
    const double agent_radii) {
    std::vector<Eigen::VectorXd> path;
    auto graphPtr = std::make_shared<amp::Graph<double>>();
    m_roadmap = graphPtr;
    std::map<amp::Node, Eigen::VectorXd> nodes;
    nodes[0] = q_init;
    m_nodes.clear();
    m_nodes[0] = q_init;

    int iter = 0;
    std::vector<Eigen::VectorXd> vertex_set;
    std::vector<int> time_step;  // Track which time step each node corresponds to
    vertex_set.push_back(q_init);
    time_step.push_back(0);
    bool goal_reached = false;
    
    for (size_t i = 0; i < m_max_iterations; ++i) {
        iter++;
        Eigen::VectorXd sample;
        
        // Get sample with goal bias
        if (((double) rand() / RAND_MAX) < m_goal_bias) {
            sample = q_goal; 
        } else {
            sample = Eigen::VectorXd::Zero(space.dimension());
            for (int d = 0; d < space.dimension(); ++d) {
                double lb = space.lowerBounds()[d];
                double ub = space.upperBounds()[d];
                sample[d] = lb + static_cast<double>(rand()) / RAND_MAX * (ub - lb);
            }
        }
        
        // Get nearest vertex
        double min_distance = std::numeric_limits<double>::infinity();
        int nearest_index = -1;
        for (int i = 0; i < vertex_set.size(); ++i) {
            double distance = (vertex_set[i] - sample).norm();
            if (distance < min_distance) {
                min_distance = distance;
                nearest_index = i;
            }
        }
        
        // Grow tree from vertex towards sample for step
        Eigen::VectorXd direction = (sample - vertex_set[nearest_index]).normalized();
        Eigen::VectorXd new_point = vertex_set[nearest_index] + direction * m_step_size;

        // Check if new point collides with any other agents
        bool collision = false;
        int parent_time = time_step[nearest_index];
        int new_time = parent_time + 1;  // Advance one time step
        
        if (!other_paths.empty()) {
            // Check collision along the edge by sampling multiple points
            int num_checks = 10;  // Sample 10 points along the edge
            for (int check_idx = 0; check_idx <= num_checks && !collision; ++check_idx) {
                double t = static_cast<double>(check_idx) / num_checks;
                Eigen::VectorXd interpolated_pos = vertex_set[nearest_index] + t * (new_point - vertex_set[nearest_index]);
                
                // Interpolate time
                int check_time = parent_time + static_cast<int>(t * (new_time - parent_time));
                
                // Check against all other agents at this time step
                for (size_t agent_idx = 0; agent_idx < other_paths.size(); ++agent_idx) {
                    const amp::Path2D& other_path = other_paths[agent_idx];
                    if (other_path.waypoints.empty()) continue;
                    
                    // Clamp time index to valid range
                    int safe_time = std::min(check_time, static_cast<int>(other_path.waypoints.size()) - 1);
                    Eigen::Vector2d other_robot_pos = other_path.waypoints[safe_time];
                    
                    // Check if circles collide
                    double dist = (interpolated_pos.head(2) - other_robot_pos).norm();
                    if (dist < other_radii[agent_idx] + agent_radii) {
                        collision = true;
                        break;
                    }
                }
            }
        }
        
        if (collision) continue;

        // Check if new point is valid (obstacle collision)
        Eigen::VectorXd stacked = Eigen::VectorXd(2 * new_point.size());
        stacked << vertex_set[nearest_index], new_point;
        if (!space.inCollision(stacked)) {
            // Add new point to vertex set
            vertex_set.push_back(new_point);
            time_step.push_back(new_time);
            int new_index = vertex_set.size() - 1;
            nodes[new_index] = new_point;
            m_nodes[new_index] = new_point;
            
            // Connect new point to nearest vertex
            double distance = (new_point - vertex_set[nearest_index]).norm();
            graphPtr->connect(nearest_index, new_index, distance);
            
            if ((new_point - q_goal).norm() < m_epsilon) {
                // Goal reached
                graphPtr->connect(new_index, 1, (new_point - q_goal).norm());
                goal_reached = true;
                break;
            }
        }
    }

    // Get path from graph
    if (goal_reached) {
        amp::ShortestPathProblem problem_astar;
        problem_astar.init_node = 0;
        problem_astar.goal_node = vertex_set.size() - 1;
        problem_astar.graph = graphPtr;
        MyAStarAlgo astar;

        amp::SearchHeuristic zero_heuristic;
        auto results = astar.search(problem_astar, zero_heuristic);

        if (!results.success) {
            return path;
        }

        for (const auto& node : results.node_path) {
            path.push_back(vertex_set[node]);
        }

        path.push_back(q_goal);
    } else {
        // std::cout << "N-dim RRT Failed to find path!" << std::endl;
        return path;
    }

    return path;
}