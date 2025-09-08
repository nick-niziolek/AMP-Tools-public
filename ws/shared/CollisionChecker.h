#include "AMPCore.h"

class CollisionChecker {
    public:
        bool PointCollisionChecker(const Eigen::Vector2d& point, const amp::Problem2D& problem);
        bool LineCollisionChecker(const Eigen::Vector2d& start, const Eigen::Vector2d& end, const amp::Problem2D& problem);
    };