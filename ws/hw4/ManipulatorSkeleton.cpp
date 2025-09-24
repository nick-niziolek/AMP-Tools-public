#include "ManipulatorSkeleton.h"
#include "CollisionChecker.h"
#include <Eigen/Cholesky>

Manipulator2D::Manipulator2D()
    : LinkManipulator2D({1.0, 1.0}) // Default to a 2-link with all links of 1.0 length
{}

Manipulator2D::Manipulator2D(size_t n_links)
    : LinkManipulator2D(std::vector<double>(n_links, 1.0)) // n-link manipulator, all links of length 1.0
{}

Manipulator2D::Manipulator2D(const std::vector<double>& link_lengths)
    : LinkManipulator2D(link_lengths)
{}

// Override this method for implementing forward kinematics
Eigen::Vector2d Manipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const {

    Eigen::Vector2d joint_location = getBaseLocation();
    std::vector<double> link_lengths = getLinkLengths();
    std::vector<Eigen::Vector2d> joint_positions;
    joint_positions.push_back(joint_location); // Base position
    double cumulative_angle = 0.0;

    if (state.size() < nLinks()) {
        throw std::runtime_error(
            "State vector too small for number of links. State vector size: " +
            std::to_string(state.size()) +
            ", required: " + std::to_string(nLinks()) +
            ". State vector received: " + ([&state](){
                std::ostringstream oss;
                oss << state.transpose();
                return oss.str();
            })()
        );
    }
    if (link_lengths.size() < nLinks()) {
        throw std::runtime_error(
            "Link lengths vector too small. Link lengths vector size: " +
            std::to_string(link_lengths.size()) +
            ", required: " + std::to_string(nLinks()) + "."
        );
    }
        
    // Implement forward kinematics to calculate the joint position given the manipulator state (angles
    // for n number links
    if (joint_index > nLinks()) {
        throw std::runtime_error("Joint index out of bounds: " + std::to_string(joint_index) +
                     ", number of links: " + std::to_string(nLinks()));
    }

    for (size_t i = 0; i < nLinks(); ++i) {
        double angle = state(i);
        double used_angle = angle + cumulative_angle;
        double length = link_lengths[i];
        joint_location += Eigen::Vector2d(length * cos(used_angle), length * sin(used_angle));
        joint_positions.push_back(joint_location);
        cumulative_angle += angle;
    }

    return joint_positions[joint_index];
}

// Override this method for implementing inverse kinematics
amp::ManipulatorState Manipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    // Implement inverse kinematics here

    amp::ManipulatorState joint_angles(nLinks());
    joint_angles.setZero();
    
    // If you have different implementations for 2/3/n link manipulators, you can separate them here
    if (nLinks() == 2) {
        if (end_effector_location.norm() > getLinkLengths()[0] + getLinkLengths()[1]) {
            throw std::runtime_error("Target location is out of reach for the 2-link manipulator.");
        } else if (end_effector_location.norm() < fabs(getLinkLengths()[0] - getLinkLengths()[1])) {
            throw std::runtime_error("Target location is too close for the 2-link manipulator.");
        }

        double x = end_effector_location.x();
        double y = end_effector_location.y();
        double l1 = getLinkLengths()[0];
        double l2 = getLinkLengths()[1];
        double r = sqrt(x*x + y*y);
        double phi = atan2(y, x);

        // Law of cosines for shoulder angle (theta1)
        double cos_theta1 = - (l2*l2 - l1*l1 - r*r) / (2 * l1 * r);
        // Clamp to [-1, 1] to avoid NaN from acos
        cos_theta1 = std::max(-1.0, std::min(1.0, cos_theta1));
        double theta1 = phi + getIKFlag() * acos(cos_theta1);

        // Get angle from first link end to target
        Eigen::Vector2d link1_end(l1 * cos(theta1), l1 * sin(theta1));
        Eigen::Vector2d to_target = end_effector_location - link1_end;
        double theta2 = atan2(to_target.y(), to_target.x());
        theta2 -= theta1;

        // Assign to joint angles
        joint_angles(0) = theta1;
        joint_angles(1) = theta2;

        return joint_angles;
    } else {
        // FABRIK Algorithm
        std::vector<double> link_lengths = getLinkLengths();
        std::vector<Eigen::Vector2d> joint_positions(nLinks() + 1);
        Eigen::Vector2d direction;
        double tol = 1e-8;

        // Initialize joint positions based on current joint angles
        for (int i = 0; i <= nLinks(); i++) {
            joint_positions[i] = getJointLocation(joint_angles, i);
        }

        for (int i = 0; i < 1000; i++) {
            // Backward Pass
            joint_positions[nLinks()] = end_effector_location;
            for (int j = nLinks()-1; j >= 0; j--) {
                direction = (joint_positions[j] - joint_positions[j+1]).normalized();
                joint_positions[j] = joint_positions[j+1] + direction * link_lengths[j];
            }

            // Forward Pass
            joint_positions[0] = getBaseLocation();
            for (int j = 0; j < nLinks(); j++) {
                direction = (joint_positions[j+1] - joint_positions[j]).normalized();
                joint_positions[j+1] = joint_positions[j] + direction * link_lengths[j];
            }

            // Check for convergence
            if ((joint_positions[nLinks()] - end_effector_location).norm() < tol) {
                // Calculate joint angles from positions
                // Joint angles based on previous link
                for (int j = 0; j < nLinks(); j++) {
                    Eigen::Vector2d vec = joint_positions[j+1] - joint_positions[j];
                    Eigen::Vector2d ref = (j == 0) ? Eigen::Vector2d(1, 0) : (joint_positions[j] - joint_positions[j-1]).normalized();
                    joint_angles(j) = atan2(vec.y(), vec.x()) - atan2(ref.y(), ref.x());
                }
                return joint_angles;
            }

        }

        double final_error = (joint_positions[nLinks()] - end_effector_location).norm();
        std::ostringstream oss;
        oss << "Failed to converge to a solution for the given end effector location.\n"
            << "Target location: [" << end_effector_location.x() << ", " << end_effector_location.y() << "]\n"
            << "Final error: " << final_error << "\n"
            << "Closest reached: [" << joint_positions[nLinks()].x() << ", "
            << joint_positions[nLinks()].y() << "]";
        throw std::runtime_error(oss.str());
    }

    return joint_angles;
}





    //     // Implement IK for n-link manipulator (if needed)
    //     // Cyclical Coordinate Descent
    //     for (int i = 0; i < 100; ++i) {
    //         double step_size = 1.0/(i * i + 1.0);
    //         for (int j = nLinks() - 1; j >= 0; --j) {
    //             Eigen::Vector2d joint_pos = getJointLocation(joint_angles, j);
    //             // Loop over a lot of test angles and find the angle that gets the end effector closest to target
    //             double best_angle = joint_angles(j);
    //             double best_distance = (getJointLocation(joint_angles, nLinks()) - end_effector_location).norm();
    //             for (double test_angle = -M_PI; test_angle <= M_PI; test_angle += step_size) {
    //                 joint_angles(j) = test_angle;
    //                 double distance = (getJointLocation(joint_angles, nLinks()) - end_effector_location).norm();
    //                 if (distance < best_distance) {
    //                     best_distance = distance;
    //                     best_angle = test_angle;
    //                 }
    //             }
    //             joint_angles(j) = best_angle;

    //             // Normalize angle to [-pi, pi]
    //             if (joint_angles(j) > M_PI) {
    //                 joint_angles(j) -= 2 * M_PI;
    //             } else if (joint_angles(j) < -M_PI) {
    //                 joint_angles(j) += 2 * M_PI;
    //             }
    //         }

    //         // Check for convergence
    //         if ((getJointLocation(joint_angles, nLinks()) - end_effector_location).norm() < 1e-4) {
    //             // std::cout << "Converged in " << i << " iterations with error: "
    //             //           << (getJointLocation(joint_angles, nLinks()) - end_effector_location).norm()
    //             //           << std::endl;
    //             return joint_angles;
    //         }
    //     }

    //     double final_error = (getJointLocation(joint_angles, nLinks()) - end_effector_location).norm();
    //     std::ostringstream oss;
    //     oss << "Failed to converge to a solution for the given end effector location.\n"
    //         << "Target location: [" << end_effector_location.x() << ", " << end_effector_location.y() << "]\n"
    //         << "Final error: " << final_error << "\n"
    //         << "Closest reached: [" << getJointLocation(joint_angles, nLinks()).x() << ", "
    //         << getJointLocation(joint_angles, nLinks()).y() << "]";
    //     throw std::runtime_error(oss.str());
    // }

// bool Manipulator2D::isPointFeasible(const Eigen::Vector2d& point, const amp::Problem2D& env){
//     amp::ManipulatorState state; // To hold the manipulator state from IK
//     CollisionChecker collisionchecker; // Create an instance of CollisionChecker
//     for (int j = -1; j < 2; j += 2) {
//         setIKFlag(j); // Try other direction for shoulder
//         state = getConfigurationFromIK(point); // Get the manipulator state from IK
//         int checks = 0; // To count collision-free links
//         // Check for collisions
//         for (int i = 0; i < nLinks(); i++) {
//             Eigen::Vector2d joint_start = getJointLocation(state, i);
//             Eigen::Vector2d joint_end = getJointLocation(state, i + 1);
//             if (!collisionchecker.LineCollisionChecker(joint_start, joint_end, env)){
//                 checks++; // Increment if no collision
//             }
//         }
//         if (checks == nLinks() - 1) {
//             return true;
//         }
//     }
//     return false;
// }

// bool Manipulator2D::isStateFeasible(const amp::ManipulatorState& state, const amp::Problem2D& env) {
//     CollisionChecker collisionchecker; // Create an instance of CollisionChecker
//     int checks = 0; // To count collision-free links
//     // Check for collisions
//     for (int i = 0; i < nLinks(); i++) {
//         Eigen::Vector2d joint_start = getJointLocation(state, i);
//         Eigen::Vector2d joint_end = getJointLocation(state, i + 1);
//         if (!collisionchecker.LineCollisionChecker(joint_start, joint_end, env)){
//             checks++; // Increment if no collision
//         }
//     }
//     return (checks == nLinks() - 1);
// }