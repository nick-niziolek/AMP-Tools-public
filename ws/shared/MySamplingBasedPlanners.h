#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW7.h"
#include "hw/HW8.h"

class MyPRM : public amp::PRM2D {
    public:
        MyPRM(int n_samples = 200, double radius = 1.0, bool path_smooth = false) : m_n_samples(n_samples), m_radius(radius), m_path_smooth(path_smooth) {};
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;
        std::shared_ptr<amp::Graph<double>> getRoadmap() const { return m_roadmap; }
        std::map<amp::Node, Eigen::Vector2d> getNodes() const { return m_nodes; }  // Add this

    private:
        int m_n_samples;
        double m_radius;
        bool m_path_smooth;
        std::shared_ptr<amp::Graph<double>> m_roadmap;
        std::map<amp::Node, Eigen::Vector2d> m_nodes;  // Add this member variable
};

class MyRRT : public amp::GoalBiasRRT2D {
    public:
        MyRRT(double goal_bias = 0.05, double step_size = 0.5, int max_iterations = 5000, double epsilon = 0.25) : m_goal_bias(goal_bias), m_step_size(step_size), m_max_iterations(max_iterations), m_epsilon(epsilon) {}
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 
        std::shared_ptr<amp::Graph<double>> getRoadmap() const { return m_roadmap; }
        std::map<amp::Node, Eigen::Vector2d> getNodes() const { return m_nodes; }  // Add this
    private:
        double m_goal_bias;
        double m_step_size;
        int m_max_iterations;
        double m_epsilon;
        std::shared_ptr<amp::Graph<double>> m_roadmap;
        std::map<amp::Node, Eigen::Vector2d> m_nodes;
};

class N_Dim_RRT {
    public:
        N_Dim_RRT(double goal_bias = 0.05, double step_size = 0.5, int max_iterations = 5000, double epsilon = 0.25) 
            : m_goal_bias(goal_bias), m_step_size(step_size), m_max_iterations(max_iterations), m_epsilon(epsilon) {}

        // Override the plan method from MultiAgentCircleMotionPlanner2D
        virtual std::vector<Eigen::VectorXd> plan(const amp::ConfigurationSpace& space, const Eigen::VectorXd& q_init, const Eigen::VectorXd& q_goal, 
            const std::vector<amp::Path2D>& other_paths = {},const std::vector<double>& other_radii = {}, const double agent_radii = 0.5); ;
        std::shared_ptr<amp::Graph<double>> m_roadmap;
        std::map<amp::Node, Eigen::VectorXd> m_nodes;
    private:
        double m_goal_bias;
        double m_step_size;
        int m_max_iterations;
        double m_epsilon;
        std::list<std::vector<Eigen::VectorXd>> m_other_paths;
};

// Centralized Multi-Agent RRT - inherits from CentralizedMultiAgentRRT
class MyCentralizedMultiAgentRRT : public amp::CentralizedMultiAgentRRT {
    public:
        
        
    private:
        std::shared_ptr<amp::Graph<double>> m_roadmap;
        std::map<amp::Node, Eigen::VectorXd> m_nodes;
        double m_goal_bias;
        double m_step_size;
        int m_max_iterations;
        double m_epsilon;
};

// Decentralized Multi-Agent RRT - inherits from DecentralizedMultiAgentRRT
class MyDecentralizedMultiAgentRRT : public amp::DecentralizedMultiAgentRRT {
    public:
        MyDecentralizedMultiAgentRRT(double goal_bias = 0.05, double step_size = 0.5, int max_iterations = 5000, double epsilon = 0.25) 
            : m_goal_bias(goal_bias), m_step_size(step_size), m_max_iterations(max_iterations), m_epsilon(epsilon) {}
        
        // Override the plan method from MultiAgentCircleMotionPlanner2D
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;
        
    private:
        double m_goal_bias;
        double m_step_size;
        int m_max_iterations;
        double m_epsilon;
};
