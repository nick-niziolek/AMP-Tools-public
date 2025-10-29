#include "MyMultiAgentPlanners.h"
#include "CollisionChecker.h"
#include "MySamplingBasedPlanners.h"

amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path(problem.numAgents());
    auto graphPtr = std::make_shared<amp::Graph<double>>();
    m_roadmap = graphPtr;
    std::map<amp::Node, Eigen::VectorXd> nodes;
    
    // Initial States
    Eigen::VectorXd q_init;
    Eigen::VectorXd q_goal;

    // Stack initial states to get q_init
    for (size_t i = 0; i < problem.numAgents(); ++i) {
        q_init.conservativeResize(q_init.size() + 2);
        q_init[q_init.size() - 2] = problem.agent_properties[i].q_init[0];
        q_init[q_init.size() - 1] = problem.agent_properties[i].q_init[1];

        q_goal.conservativeResize(q_goal.size() + 2);
        q_goal[q_goal.size() - 2] = problem.agent_properties[i].q_goal[0];
        q_goal[q_goal.size() - 1] = problem.agent_properties[i].q_goal[1];
    } 

    MultiDiscConfigSpace cs(problem);

    N_Dim_RRT rrt(m_goal_bias, m_step_size, m_max_iterations, m_epsilon);
    std::vector<Eigen::VectorXd> q_path = rrt.plan(cs, q_init, q_goal);
    if (q_path.empty()) {
        // std::cout << "No path found by Centralized RRT." << std::endl;
        return path; // Return empty path
    }
    // Save graph and nodes
    m_roadmap = rrt.m_roadmap;
    m_nodes = rrt.m_nodes;

    // Decompose q_path into individual agent paths
    std::vector<amp::Path2D> agent_paths(problem.numAgents());
    for (const auto& q : q_path) {
        for (size_t i = 0; i < problem.numAgents(); ++i) {
            Eigen::Vector2d waypoint;
            waypoint[0] = q[2 * i];
            waypoint[1] = q[2 * i + 1];
            agent_paths[i].waypoints.push_back(waypoint);
        }
    }

    // Combine the individual agent paths into the final multi-agent path
    for (size_t i = 0; i < problem.numAgents(); ++i) {
        // std::cout << "Agent " << i << " path length: " << agent_paths[i].waypoints.size() << std::endl;
        path.agent_paths[i] = agent_paths[i];
    }

    return path;
}

amp::MultiAgentPath2D MyDecentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path;
    m_nodes_size = 0;
    // Implement decentralized RRT planning for each agent
    for (size_t i = 0; i < problem.numAgents(); ++i) {
        // std::cout << "Planning for agent " << i << std::endl;
        // Create single agent problem
        Eigen::Vector2d q_init = problem.agent_properties[i].q_init;
        Eigen::Vector2d q_goal = problem.agent_properties[i].q_goal;
        amp::MultiAgentProblem2D single_agent_problem;
        single_agent_problem.x_min = problem.x_min;
        single_agent_problem.x_max = problem.x_max;
        single_agent_problem.y_min = problem.y_min;
        single_agent_problem.y_max = problem.y_max;
        single_agent_problem.obstacles = problem.obstacles;
        amp::CircularAgentProperties agent_prop;
        agent_prop.radius = problem.agent_properties[i].radius;
        agent_prop.q_init = q_init;
        agent_prop.q_goal = q_goal;
        single_agent_problem.agent_properties.push_back(agent_prop);
        MultiDiscConfigSpace cs(single_agent_problem);
        amp::Path2D agent_path;
        // Plan for single agent and include other paths for collision checking
        N_Dim_RRT rrt(m_goal_bias, m_step_size, m_max_iterations, m_epsilon);
        std::vector<Eigen::VectorXd> found_path;
        std::vector<double> other_agent_radii;
        for (size_t j = 0; j < i; ++j)
        {
            other_agent_radii.push_back(problem.agent_properties[j].radius);
        }
        found_path = rrt.plan(cs, q_init, q_goal, path.agent_paths, other_agent_radii, problem.agent_properties[i].radius);
        if (found_path.empty()) {
            std::cout << "No path found for agent " << i << " by Decentralized RRT." << std::endl;
            return amp::MultiAgentPath2D(); // Return empty path
        }
        // Check if found path has right dimension
        for (const auto& q : found_path) {
            if (q.size() != 2) {
                std::cout << "Error: Found path for agent " << i << " has incorrect dimension." << std::endl;
                return amp::MultiAgentPath2D(); // Return empty path
            }
        }
        // Convert found_path to agent_path
        m_nodes_size += rrt.m_nodes.size();
        for (const auto& q : found_path) {
            if (q.size() != 2) continue; // Safety check
            agent_path.waypoints.push_back(Eigen::Vector2d(q[0], q[1]));
        }
        path.agent_paths.push_back(agent_path);

    }
    
    return path;
}