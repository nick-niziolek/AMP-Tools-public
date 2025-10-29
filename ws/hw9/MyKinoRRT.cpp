#include "MyKinoRRT.h"

amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) {
    amp::KinoPath path;
    Eigen::VectorXd state = problem.q_init;
    
    auto graphPtr = std::make_shared<amp::Graph<double>>();
    m_roadmap = graphPtr;    
    std::map<amp::Node, Eigen::VectorXd> nodes;
    nodes[0] = problem.q_init;
    m_nodes.clear();  // Clear previous nodes
    m_nodes[0] = problem.q_init;  // Store in member variable

    for (size_t i = 0; i < m_max_iter; i++) {
        for (size_t j = 0; j < m_u_samples; j++) {
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
            // Get nearest current tree point to sample
            double min_dist = std::numeric_limits<double>::max();
            Eigen::VectorXd nearest_state = state;
            for (const auto& node_pair : nodes) {
                Eigen::VectorXd node_state = node_pair.second;
                double dist = (node_state - sample).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    nearest_state = node_state;
                }
            }
            state = nearest_state;

            // Sample random control within bounds, get the closest to the state if allowed multiple samples
            Eigen::VectorXd control = Eigen::VectorXd::Zero(problem.u_bounds.size());
            for (size_t k = 0; k < problem.u_bounds.size(); k++) {
                double u_min = problem.u_bounds[k].first;
                double u_max = problem.u_bounds[k].second;
                control(k) = u_min + static_cast<double>(rand()) / RAND_MAX * (u_max - u_min);
            }
            // Propagate state
            Eigen::VectorXd new_state = state;
            agent.propagate(new_state, control, 1.0);

            state = new_state;
        }
        // Check if state is valid (not in collision)
            // If so, check if the path leading to the state is valid
        
    }


    path.waypoints.push_back(state);
    for (int i = 0; i < 10; i++) {
        Eigen::VectorXd control = Eigen::VectorXd::Random(problem.q_init.size());
        agent.propagate(state, control, 1.0);
        path.waypoints.push_back(state);
        path.controls.push_back(control);
        path.durations.push_back(1.0);
    }
    path.valid = true;
    return path;
}

void MySingleIntegrator::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    int subdiv = 10;
    double dt_act = dt / subdiv;
    for (int i = 0; i < subdiv; i++) {
        state += dt_act * control;
    }
}

void MyFirstOrderUnicycle::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    // Dynamics function to pass to RungeKutta4
    // state: [x, y, theta]
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
        RungeKutta4(state, control, dt_act, dynamics);
        prop_path.push_back(state);
    }
    
}

void MySecondOrderUnicycle::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    // Dynamics function to pass to RungeKutta4
    // state: [x, y, theta, sigma, omega]
    double r = 0.25; // wheel radius
    prop_path.clear();
    prop_path.push_back(state);
    auto dynamics = [r](const Eigen::VectorXd& s, const Eigen::VectorXd& u) {
        Eigen::VectorXd dsdt(4);
        double theta = s(2);
        dsdt(0) = s(3) * r * cos(theta); // dx/dt
        dsdt(1) = s(3) * r * sin(theta); // dy/dt
        dsdt(2) = s(4);                   // dtheta/dt
        dsdt(3) = u(0);                   // dsigma/dt
        dsdt(4) = u(1);                   // domega/dt
        return dsdt;
    };

    RungeKutta4(state, control, dt, dynamics);
    prop_path.push_back(state);
}

void MySimpleCar::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    // Dynamics function to pass to RungeKutta4
    // state: [x, y, theta, v, phi]
    double L = 5; // wheelbase
    prop_path.clear();
    prop_path.push_back(state);
    int subdiv = 10;
    double dt_act = dt / subdiv;
    auto dynamics = [L](const Eigen::VectorXd& s, const Eigen::VectorXd& u) {
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
        RungeKutta4(state, control, dt_act, dynamics);
        prop_path.push_back(state);
    }
}

void RungeKutta4(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt, Eigen::VectorXd (*dynamics)(const Eigen::VectorXd&, const Eigen::VectorXd&)) {
    Eigen::VectorXd k1 = dynamics(state, control);
    Eigen::VectorXd k2 = dynamics(state + 0.5 * dt * k1, control);
    Eigen::VectorXd k3 = dynamics(state + 0.5 * dt * k2, control);
    Eigen::VectorXd k4 = dynamics(state + dt * k3, control);
    state += (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);
}

