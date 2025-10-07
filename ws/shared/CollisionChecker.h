#include "AMPCore.h"

class CollisionChecker {
    public:
        bool PointCollisionChecker(const Eigen::Vector2d& point, const amp::Problem2D& problem);
        bool LineCollisionChecker(const Eigen::Vector2d& start, const Eigen::Vector2d& end, const amp::Environment2D& env);
        float PointToNearestVertexDistance(const Eigen::Vector2d& point, const amp::Problem2D& problem);
        float PointToLineAndPoint(const Eigen::Vector2d& point, const Eigen::Vector2d& line_direction, const Eigen::Vector2d& line_point);
        bool isManipStateFeasible(const amp::LinkManipulator2D& manip, const amp::ManipulatorState& state, const amp::Environment2D& env);
        bool isCellColliding(const Eigen::Vector2d& cell_center, double cell_size_x, double cell_size_y, const amp::Environment2D& env);

    private:
        bool convexPolygonsIntersect(const std::vector<Eigen::Vector2d>& polyA, const std::vector<Eigen::Vector2d>& polyB);
        bool overlapOnAxis(const std::vector<Eigen::Vector2d>& polyA, const std::vector<Eigen::Vector2d>& polyB, const Eigen::Vector2d& axis);
        std::pair<double, double> projectPolygon(const std::vector<Eigen::Vector2d>& poly, const Eigen::Vector2d& axis);
    };