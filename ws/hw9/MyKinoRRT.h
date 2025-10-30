#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW9.h"

class MyDynamicAgent : public amp::DynamicAgent {
    public:
        virtual std::vector<Eigen::VectorXd> getPropagationPath() const = 0;
        virtual void getVertices(const Eigen::VectorXd& state, std::vector<Eigen::Vector2d>& vertices) = 0;
};

class MyKinoRRT : public amp::KinodynamicRRT {
    public:
        MyKinoRRT(int max_iter=50000, int u_samples=1, double goal_bias=0.05, double time_step=0.1) : m_max_iter(max_iter), m_u_samples(u_samples), m_goal_bias(goal_bias), m_time_step(time_step) {}
        virtual amp::KinoPath plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) override;
        double getPathLength(const amp::KinoPath& path) const;
        std::shared_ptr<amp::Graph<double>> m_roadmap;
        std::map<amp::Node, Eigen::VectorXd> m_nodes;
    private:
        int m_max_iter;
        int m_u_samples;
        double m_goal_bias;
        double m_time_step;
};  

class MySingleIntegrator : public MyDynamicAgent {
    public:
        std::vector<Eigen::VectorXd> prop_path;
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
        virtual std::vector<Eigen::VectorXd> getPropagationPath() const override { return prop_path; }
        virtual void getVertices(const Eigen::VectorXd& state, std::vector<Eigen::Vector2d>& vertices) override;
};

class MyFirstOrderUnicycle : public MyDynamicAgent {
    public:
        std::vector<Eigen::VectorXd> prop_path;
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
        virtual std::vector<Eigen::VectorXd> getPropagationPath() const override { return prop_path; }
        virtual void getVertices(const Eigen::VectorXd& state, std::vector<Eigen::Vector2d>& vertices) override;
};

class MySecondOrderUnicycle : public MyDynamicAgent {
    public:
        std::vector<Eigen::VectorXd> prop_path;
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
        virtual std::vector<Eigen::VectorXd> getPropagationPath() const override { return prop_path; }
        virtual void getVertices(const Eigen::VectorXd& state, std::vector<Eigen::Vector2d>& vertices) override;
};

class MySimpleCar : public MyDynamicAgent {
    public:
        std::vector<Eigen::VectorXd> prop_path;
        amp::AgentDimensions agent_dim; // Add member to store dimensions
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
        virtual std::vector<Eigen::VectorXd> getPropagationPath() const override { return prop_path; }
        virtual void getVertices(const Eigen::VectorXd& state, std::vector<Eigen::Vector2d>& vertices) override;
};

class Integrator {
    public:
        void RungeKutta4(Eigen::VectorXd& state, const Eigen::VectorXd& control, double dt,
            const std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>& dynamics);
};