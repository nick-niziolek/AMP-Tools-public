#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW9.h"

class MyKinoRRT : public amp::KinodynamicRRT {
    public:
        MyKinoRRT(int max_iter=50000, int u_samples=1, double goal_bias=0.05) : m_max_iter(max_iter), m_u_samples(u_samples), m_goal_bias(goal_bias) {}
        virtual amp::KinoPath plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) override;
        std::shared_ptr<amp::Graph<double>> m_roadmap;
        std::map<amp::Node, Eigen::VectorXd> m_nodes;
    private:
        int m_max_iter;
        int m_u_samples;
        double m_goal_bias;
};  

class MySingleIntegrator : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override {};
        std::vector<Eigen::VectorXd> prop_path;
};

class MyFirstOrderUnicycle : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override {};
        std::vector<Eigen::VectorXd> prop_path;
};

class MySecondOrderUnicycle : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override {};
        std::vector<Eigen::VectorXd> prop_path;
};

class MySimpleCar : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override {};
        std::vector<Eigen::VectorXd> prop_path;
};