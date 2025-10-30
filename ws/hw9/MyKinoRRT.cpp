#include "MyKinoRRT.h"
#include "CollisionChecker.h"
#include "MyAStar.h"
#include "cassert"
#include <nanoflann.hpp>

amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) {
    amp::KinoPath path;
    Eigen::VectorXd state = problem.q_init;
    CollisionChecker checker;
    // Set length and width if it's a car; keep a pointer to MyDynamicAgent for generic use
    MyDynamicAgent* my_agent = nullptr;
    if (problem.agent_type == amp::AgentType::SimpleCar) {
        MySimpleCar* car_agent = dynamic_cast<MySimpleCar*>(&agent);
        assert(car_agent != nullptr && "Agent must be of type MySimpleCar");
        car_agent->agent_dim = problem.agent_dim;
        my_agent = car_agent;
    } else {
        my_agent = dynamic_cast<MyDynamicAgent*>(&agent);
        assert(my_agent != nullptr && "Agent must be of type MyDynamicAgent");
    }

    auto graphPtr = std::make_shared<amp::Graph<double>>();
    m_roadmap = graphPtr;    
    std::map<amp::Node, Eigen::VectorXd> nodes;
    std::vector<Eigen::VectorXd> vertex_set;
    std::vector<Eigen::VectorXd> controls;
    vertex_set.push_back(problem.q_init);
    controls.push_back(Eigen::VectorXd::Zero(problem.u_bounds.size()));
    nodes[0] = problem.q_init;
    m_nodes.clear();  // Clear previous nodes
    m_nodes[0] = problem.q_init;  // Store in member variable
    bool goal_reached = false;

    int state_out_of_bounds = 0;
    int state_in_collision = 0;
    int path_in_collision = 0;

    for (size_t i = 0; i < m_max_iter; i++) {

        if (i % 5000 == 0) {
            std::cout << "Iteration: " << i << " Tree Size: " << m_nodes.size() << " State Out of Bounds: " << state_out_of_bounds << " State In Collision: " << state_in_collision << " Path In Collision: " << path_in_collision << std::endl;
            // // Do quick search for debugging, shortest distance to goal in current tree
            // double min_goal_dist = std::numeric_limits<double>::max();
            // double min_dist_to_start = std::numeric_limits<double>::max();
            // int num_checks = 0;
            // for (const auto& node_pair : m_nodes) {
            //     Eigen::VectorXd node_state = node_pair.second;
            //     double dist_to_goal = 0.0;
            //     double dist_to_start = 0.0;
            //     for (size_t k = 0; k < problem.q_goal.size(); k++) {
            //         double goal_center = 0.5 * (problem.q_goal[k].first + problem.q_goal[k].second);
            //         dist_to_goal += std::pow(node_state(k) - goal_center, 2);
            //         double start_center = problem.q_init(k);
            //         dist_to_start += std::pow(node_state(k) - start_center, 2);
            //     }
            //     dist_to_goal = std::sqrt(dist_to_goal);
            //     dist_to_start = std::sqrt(dist_to_start);
            //     if (dist_to_goal < min_goal_dist)
            //         min_goal_dist = dist_to_goal;
            //     if (dist_to_start < min_dist_to_start)
            //         min_dist_to_start = dist_to_start;
            //     num_checks++;
            //     // std::cout << "Node " << node_pair.first << " Distance to goal: " << dist_to_goal << std::endl;
            // }
            // std::cout << "Minimum distance to goal in tree: " << min_goal_dist << " Minimum distance to start: " << min_dist_to_start << " in " << num_checks << " checks." << std::endl;
        }
        

        // Sample random position within bounds with goal bias
        Eigen::VectorXd sample = Eigen::VectorXd::Zero(problem.q_init.size());
        for (size_t k = 0; k < problem.q_bounds.size(); k++) {
            double q_min = problem.q_bounds[k].first;
            double q_max = problem.q_bounds[k].second;
            if (static_cast<double>(rand()) / RAND_MAX < m_goal_bias) {
                // Sample towards goal
                double goal_center = 0.5 * (problem.q_goal[k].first + problem.q_goal[k].second);
                sample(k) = goal_center;
            } else {
                // Uniform random sample
                sample(k) = q_min + static_cast<double>(rand()) / RAND_MAX * (q_max - q_min);
            }
        }

        // std::cout << "Sampled state: " << sample.transpose() << std::endl;

        // Get nearest current tree point to sample
        double min_dist = std::numeric_limits<double>::max();
        Eigen::VectorXd nearest_state = state;
        amp::Node nearest_node = 0;
        for (const auto& node_pair : nodes) {
            Eigen::VectorXd node_state = node_pair.second;
            double dist = (node_state - sample).norm();
            if (dist < min_dist) {
                min_dist = dist;
                nearest_state = node_state;
                nearest_node = node_pair.first;
            }
        }

        state = nearest_state;

        // std::cout << "Nearest state: " << state.transpose() << std::endl;

        // Sample random control within bounds, get the closest to the state if allowed multiple samples
        Eigen::VectorXd control = Eigen::VectorXd::Zero(problem.u_bounds.size());
        min_dist = std::numeric_limits<double>::max();
        for (size_t j = 0; j < m_u_samples; j++) {
            Eigen::VectorXd temp_control = Eigen::VectorXd::Zero(problem.u_bounds.size());
            for (size_t k = 0; k < problem.u_bounds.size(); k++) {
                double u_min = problem.u_bounds[k].first;
                double u_max = problem.u_bounds[k].second;
                temp_control(k) = u_min + static_cast<double>(rand()) / RAND_MAX * (u_max - u_min);
            }
            // Check which control gets state closest to sample
            Eigen::VectorXd temp_state = state;
            my_agent->propagate(temp_state, temp_control, m_time_step);
            double dist = (temp_state - sample).norm();
            if (dist < min_dist) {
                min_dist = dist;
                control = temp_control;
            }
        }

        // std::cout << "Selected control: " << control.transpose() << std::endl;

        // Propagate state
        Eigen::VectorXd new_state = state;
        agent.propagate(new_state, control, m_time_step);

        state = new_state;

        // std::cout << "New state after propagation: " << state.transpose() << std::endl;

        // std::cin.get();
        
        // Check state bounds
        bool state_in_bounds = true;
        for (size_t k = 0; k < state.size(); k++) {
            double q_min = problem.q_bounds[k].first;
            double q_max = problem.q_bounds[k].second;
            if (state(k) < q_min || state(k) > q_max) {
                // std::cout << "State out of bounds!" << std::endl;
                state_out_of_bounds++;
                state_in_bounds = false;
                break;
            }
        }
        // If car, check corners if they're in bounds
        if (problem.agent_type == amp::AgentType::SimpleCar && state_in_bounds) {
            std::vector<Eigen::Vector2d> corner_vertices;
            my_agent->getVertices(state, corner_vertices);
            for (const auto& corner : corner_vertices) {
                if (corner.x() < problem.q_bounds[0].first || corner.x() > problem.q_bounds[0].second ||
                    corner.y() < problem.q_bounds[1].first || corner.y() > problem.q_bounds[1].second) {
                    state_in_bounds = false;
                    state_out_of_bounds++;
                    break;
                }
            }
        }

        if (!state_in_bounds) {
            continue;
        }

        // Get agent vertices
        std::vector<Eigen::Vector2d> agent_vertices;
        my_agent->getVertices(state, agent_vertices);
        
        // Check if state is valid (not in collision)
        if (checker.GenericCollisionChecker(agent_vertices, agent_vertices, problem)) {
            // std::cout << "State in collision!" << std::endl;
            state_in_collision++;
            continue;
        } else {
            // Check line between start and end first 
            std::vector<Eigen::Vector2d> start_vertices;
            my_agent->getVertices(nearest_state, start_vertices);
            if (checker.GenericCollisionChecker(start_vertices, agent_vertices, problem)) {
                // std::cout << "Straight Path in collision!" << std::endl;
                path_in_collision++;
                continue;
            }
            // Check along path
            bool subdiv_collision = false;
            for (size_t j = 0; j < my_agent->getPropagationPath().size()-1; j++) {
                Eigen::VectorXd interp_start = my_agent->getPropagationPath()[j];
                Eigen::VectorXd interp_end = my_agent->getPropagationPath()[j+1];
                std::vector<Eigen::Vector2d> interp_vertices_start;
                my_agent->getVertices(interp_start, interp_vertices_start);
                std::vector<Eigen::Vector2d> interp_vertices_end;
                my_agent->getVertices(interp_end, interp_vertices_end);
                if (checker.GenericCollisionChecker(interp_vertices_start, interp_vertices_end, problem)) {
                    // std::cout << "Subdiv Path in collision!" << std::endl;
                    path_in_collision++;
                    subdiv_collision = true;
                    break;
                }
            }
            if (subdiv_collision) {
                continue;
            }
        }

        // If checks passed, add node and connection in graph
        int new_node_id = nodes.size();
        nodes[new_node_id] = state;
        m_nodes[new_node_id] = state;
        vertex_set.push_back(state);
        controls.push_back(control);
        amp::Node from_node = nearest_node;
        amp::Node to_node = new_node_id;
        m_roadmap->connect(from_node, to_node, 1.0);

        // Check if goal reached
        goal_reached = true;
        for (size_t k = 0; k < problem.q_goal.size(); k++) {
            double q_min = problem.q_goal[k].first;
            double q_max = problem.q_goal[k].second;
            if (state(k) < q_min || state(k) > q_max) {
                goal_reached = false;
                break;
            }
        }
        if (goal_reached) {
            // std::cout << "Goal reached!" << std::endl;
            break;
        }
    }
    // Use A* to find path from start to goal in graph
    if (goal_reached) {
        amp::ShortestPathProblem problem_astar;
        problem_astar.init_node = 0;
        problem_astar.goal_node = vertex_set.size() - 1;
        problem_astar.graph = graphPtr;
        MyAStarAlgo astar;

        amp::SearchHeuristic zero_heuristic;
        auto results = astar.search(problem_astar, zero_heuristic);

        if (!results.success) {
            path.valid = false;
            std::cout << "A* failed to find path!" << std::endl;
            graphPtr->print();
            return path;
        }

        for (const auto& node : results.node_path) {
            path.waypoints.push_back(vertex_set[node]);
            path.controls.push_back(controls[node]);
            path.durations.push_back(m_time_step);
        }

        std::cout << "Goal Reached!" << std::endl;
        path.valid = true;
        return path;

        // path.waypoints.push_back(problem.q_goal);
    } else {
        path.valid = false;
        std::cout << "Kino RRT Failed to find path!" << std::endl;
        // Get best effort path to closest node to goal
        double min_goal_dist = std::numeric_limits<double>::max();
        amp::Node closest_node = 0;
        for (const auto& node_pair : m_nodes) {
            Eigen::VectorXd node_state = node_pair.second;
            double dist_to_goal = 0.0;
            for (size_t k = 0; k < problem.q_goal.size(); k++) {
                double goal_center = 0.5 * (problem.q_goal[k].first + problem.q_goal[k].second);
                dist_to_goal += std::pow(node_state(k) - goal_center, 2);
            }
            if (dist_to_goal < min_goal_dist) {
                min_goal_dist = dist_to_goal;
                closest_node = node_pair.first;
            }
        }
        // Get best effort path to closest node to goal
        if (closest_node != 0) {
            std::cout << "Best effort path found to node: " << closest_node << std::endl;
        }
        // Step back through tree to get to start
        amp::Node current_node = closest_node;
        std::vector<amp::Node> reverse_path;
        while (current_node != 0) {
            reverse_path.push_back(current_node);
            auto parents = m_roadmap->parents(current_node);
            if (parents.empty()) {
                break;
            }
            current_node = parents[0];
        }
        reverse_path.push_back(0);
        // Reverse to get correct order
        for (auto it = reverse_path.rbegin(); it != reverse_path.rend(); ++it) {
            amp::Node node = *it;
            path.waypoints.push_back(vertex_set[node]);
            path.controls.push_back(Eigen::VectorXd::Zero(problem.u_bounds.size())); // Placeholder
            path.durations.push_back(m_time_step);
        }
        return path;
    }
    
}

double MyKinoRRT::getPathLength(const amp::KinoPath& path) const {
    double length = 0.0;
    for (size_t i = 0; i < path.waypoints.size() - 1; i++) {
        length += (path.waypoints[i+1] - path.waypoints[i]).norm();
    }
    return length;
}

void MySingleIntegrator::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    int subdiv = 10;
    double dt_act = dt / subdiv;
    prop_path.clear();
    prop_path.push_back(state);
    for (int i = 0; i < subdiv; i++) {
        state += dt_act * control;
        prop_path.push_back(state);
    }
}

void MySingleIntegrator::getVertices(const Eigen::VectorXd& state, std::vector<Eigen::Vector2d>& vertices) {
    // Point agent - just the position
    vertices.clear();
    vertices.push_back(Eigen::Vector2d(state(0), state(1)));
}

void MyFirstOrderUnicycle::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    // Dynamics function to pass to RungeKutta4
    // state: [x, y, theta]
    Integrator integrator;
    double r = 0.25; // wheel radius
    int subdiv = 10;
    double dt_act = dt / subdiv;
    prop_path.clear();
    prop_path.push_back(state);
    auto dynamics = [r](const Eigen::VectorXd& s, const Eigen::VectorXd& u) {
            Eigen::VectorXd dsdt(3);
            double theta = s(2);
            dsdt(0) = u(0) * r * cos(theta); // dx/dt
            dsdt(1) = u(0) * r * sin(theta); // dy/dt
            dsdt(2) = u(1);                  // dtheta/dt
            return dsdt;
        };
    for (int i = 0; i < subdiv; i++) {
        integrator.RungeKutta4(state, control, dt_act, dynamics);
        prop_path.push_back(state);
    }
}

void MyFirstOrderUnicycle::getVertices(const Eigen::VectorXd& state, std::vector<Eigen::Vector2d>& vertices) {
    // Point agent - just the position
    vertices.clear();
    vertices.push_back(Eigen::Vector2d(state(0), state(1)));
}

void MySecondOrderUnicycle::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    // Dynamics function to pass to RungeKutta4
    // state: [x, y, theta, sigma, omega]
    Integrator integrator;
    double r = 0.25; // wheel radius
    int subdiv = 10;
    double dt_act = dt / subdiv;
    prop_path.clear();
    prop_path.push_back(state);
    auto dynamics = [r](const Eigen::VectorXd& s, const Eigen::VectorXd& u) {
        Eigen::VectorXd dsdt(5);
        double theta = s(2);
        dsdt(0) = s(3) * r * cos(theta); // dx/dt
        dsdt(1) = s(3) * r * sin(theta); // dy/dt
        dsdt(2) = s(4);                   // dtheta/dt
        dsdt(3) = u(0);                   // dsigma/dt
        dsdt(4) = u(1);                   // domega/dt
        return dsdt;
    };
    for (int i = 0; i < subdiv; i++) {
        integrator.RungeKutta4(state, control, dt_act, dynamics);
        prop_path.push_back(state);
    }
}

void MySecondOrderUnicycle::getVertices(const Eigen::VectorXd& state, std::vector<Eigen::Vector2d>& vertices) {
    // Point agent - just the position
    vertices.clear();
    vertices.push_back(Eigen::Vector2d(state(0), state(1)));
}

void MySimpleCar::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    // Dynamics function to pass to RungeKutta4
    // state: [x, y, theta, v, phi]
    if (!agent_dim.length || !agent_dim.width) {
        throw std::runtime_error("Agent dimensions not set in MySimpleCar");
    }
    if (!std::isfinite(state.sum()) || !std::isfinite(control.sum())) {
        std::cerr << "propagate: got NaN/Inf in state or control; state=" << state.transpose()
                  << " control=" << control.transpose() << std::endl;
        return;
    }
    if (state.size() < 5 || control.size() < 2) {
        std::cerr << "propagate: state or control size too small; state size=" << state.size()
                  << " control size=" << control.size() << std::endl;
        return;
    }
    Integrator integrator;
    prop_path.clear();
    prop_path.push_back(state);
    int subdiv = 10;
    double dt_act = dt / subdiv;
    double L = agent_dim.length;
    double W = agent_dim.width;
    auto dynamics = [L, W](const Eigen::VectorXd& s, const Eigen::VectorXd& u) {
        Eigen::VectorXd dsdt(5);
        double theta = s(2);
        double v = s(3);
        double phi = s(4);
        dsdt(0) = v * cos(theta);               // dx/dt
        dsdt(1) = v * sin(theta);               // dy/dt
        dsdt(2) = (v / L) * tan(phi);          // dtheta/dt
        dsdt(3) = u(0);                        // dv/dt
        dsdt(4) = u(1);                        // dphi/dt
        return dsdt;
    };
    for (int i = 0; i < subdiv; i++) {
        integrator.RungeKutta4(state, control, dt_act, dynamics);
        prop_path.push_back(state);
    }
}

void MySimpleCar::getVertices(const Eigen::VectorXd& state, std::vector<Eigen::Vector2d>& vertices) {
    double length = agent_dim.length;
    double width = agent_dim.width;
    double x = state(0);
    double y = state(1);
    double theta = state(2);

    Eigen::Matrix2d R;
    R << cos(theta), -sin(theta),
         sin(theta),  cos(theta);

    Eigen::Vector2d front_left_local(length, width / 2);
    Eigen::Vector2d front_right_local(length, -width / 2);
    Eigen::Vector2d rear_right_local(0, -width / 2);
    Eigen::Vector2d rear_left_local(0, width / 2);

    vertices.clear();
    // CCW order
    vertices.push_back(R * front_left_local + Eigen::Vector2d(x, y));
    vertices.push_back(R * rear_left_local + Eigen::Vector2d(x, y));
    vertices.push_back(R * rear_right_local + Eigen::Vector2d(x, y));
    vertices.push_back(R * front_right_local + Eigen::Vector2d(x, y));
}

void Integrator::RungeKutta4(Eigen::VectorXd& state, const Eigen::VectorXd& control, double dt,
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>& dynamics) {
    Eigen::VectorXd k1 = dynamics(state, control);
    Eigen::VectorXd k2 = dynamics(state + 0.5 * dt * k1, control);
    Eigen::VectorXd k3 = dynamics(state + 0.5 * dt * k2, control);
    Eigen::VectorXd k4 = dynamics(state + dt * k3, control);
    state += (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);
}