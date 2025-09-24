#include "AMPCore.h"

class CollisionChecker {
    public:
        bool PointCollisionChecker(const Eigen::Vector2d& point, const amp::Problem2D& problem);
        bool LineCollisionChecker(const Eigen::Vector2d& start, const Eigen::Vector2d& end, const amp::Environment2D& env);
        float PointToNearestVertexDistance(const Eigen::Vector2d& point, const amp::Problem2D& problem);
        float PointToLineAndPoint(const Eigen::Vector2d& point, const Eigen::Vector2d& line_direction, const Eigen::Vector2d& line_point);
        bool isManipStateFeasible(const amp::LinkManipulator2D& manip, const amp::ManipulatorState& state, const amp::Environment2D& env);
    };