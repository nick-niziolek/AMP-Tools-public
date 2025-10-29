#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW8.h"

// This is file is loaded from the shared/ directory
// Overwrite with your MySamplingBasedPlanners.h and MySamplingBasedPlanners.cpp from hw7
#include "MySamplingBasedPlanners.h" 


class MyCentralPlanner : public amp::CentralizedMultiAgentRRT {
    public:
        MyCentralPlanner(double goal_bias = 0.05, double step_size = 0.25, int max_iterations = 40000, double epsilon = 0.25) 
            : m_goal_bias(goal_bias), m_step_size(step_size), m_max_iterations(max_iterations), m_epsilon(epsilon) {}
        
        // Override the plan method from MultiAgentCircleMotionPlanner2D
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;
        int getTreeSize() const { return m_nodes.size(); }
    private:
        double m_goal_bias;
        double m_step_size;
        int m_max_iterations;
        double m_epsilon;
        std::shared_ptr<amp::Graph<double>> m_roadmap;
        std::map<amp::Node, Eigen::VectorXd> m_nodes;
};


class MyDecentralPlanner : public amp::DecentralizedMultiAgentRRT {
    public:
        MyDecentralPlanner(double goal_bias = 0.05, double step_size = 0.25, int max_iterations = 40000, double epsilon = 0.25) 
            : m_goal_bias(goal_bias), m_step_size(step_size), m_max_iterations(max_iterations), m_epsilon(epsilon) {}
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;
        int getTreeSize() const { return m_nodes_size; }
    private:
        double m_goal_bias;
        double m_step_size;
        int m_max_iterations;
        double m_epsilon;
        int m_nodes_size;
};