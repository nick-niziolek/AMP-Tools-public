#include "AMPCore.h"

class CollisionChecker {
    public:
        bool PointCollisionChecker(const Eigen::Vector2d& point, const amp::Environment2D& env);
        bool LineCollisionChecker(const Eigen::Vector2d& start, const Eigen::Vector2d& end, const amp::Environment2D& env);
        float PointToNearestVertexDistance(const Eigen::Vector2d& point, const amp::Environment2D& env);
        float PointToLineAndPoint(const Eigen::Vector2d& point, const Eigen::Vector2d& line_direction, const Eigen::Vector2d& line_point);
        bool isManipStateFeasible(const amp::LinkManipulator2D& manip, const amp::ManipulatorState& state, const amp::Environment2D& env);
        bool isCellColliding(const Eigen::Vector2d& cell_center, double cell_size_x, double cell_size_y, const amp::Environment2D& env);
        bool pointInPolygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& vertices);
        Eigen::Vector2d closestPointOnSegment(const Eigen::Vector2d& point, 
                                      const Eigen::Vector2d& segStart, 
                                      const Eigen::Vector2d& segEnd);
        bool circlePolygonCollision(const Eigen::Vector2d& point, 
                           double radius, 
                           const std::vector<Eigen::Vector2d>& vertices);
        bool GenericCollisionChecker(const std::vector<Eigen::Vector2d>& agent, const amp::Environment2D& env);

    private:
        bool convexPolygonsIntersect(const std::vector<Eigen::Vector2d>& polyA, const std::vector<Eigen::Vector2d>& polyB);
        bool overlapOnAxis(const std::vector<Eigen::Vector2d>& polyA, const std::vector<Eigen::Vector2d>& polyB, const Eigen::Vector2d& axis);
        std::pair<double, double> projectPolygon(const std::vector<Eigen::Vector2d>& poly, const Eigen::Vector2d& axis);
    };

class MultiDiscConfigSpace : public amp::ConfigurationSpace {
    public:
        MultiDiscConfigSpace(const amp::MultiAgentProblem2D& problem)
        : amp::ConfigurationSpace(
            [&problem]() -> Eigen::VectorXd {
            int n = static_cast<int>(2 * problem.numAgents());
            Eigen::VectorXd v(n);
            for (int i = 0; i < n/2; ++i) { v[2*i] = problem.x_min; v[2*i+1] = problem.y_min; }
            return v;
            }(),
            [&problem]() -> Eigen::VectorXd {
            int n = static_cast<int>(2 * problem.numAgents());
            Eigen::VectorXd v(n);
            for (int i = 0; i < n/2; ++i) { v[2*i] = problem.x_max; v[2*i+1] = problem.y_max; }
            return v;
            }()
        )
        , m_problem(problem) {}

        virtual bool inCollision(const Eigen::VectorXd& cspace_state) const override;
        bool inLineCollision(const Eigen::VectorXd& cspace_start, const Eigen::VectorXd& cspace_end) const;
        bool inPointCollision(const Eigen::VectorXd& cspace_state) const;
    private:
        amp::MultiAgentProblem2D m_problem;
};