#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Derive the amp::LinkManipulator2D class
class Manipulator2D : public amp::LinkManipulator2D {
    public:
        // Default constructor
        Manipulator2D();

        // Constructor with number of links
        Manipulator2D(size_t num_links);

        // Constructor with custom link lengths
        Manipulator2D(const std::vector<double>& link_lengths);

        // Override this method for implementing forward kinematics
        virtual Eigen::Vector2d getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const override;

        // Override this method for implementing inverse kinematics
        virtual amp::ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const override;

        void setIKFlag(int flag) { ik_flag_ = flag; }
        int getIKFlag() const { return ik_flag_; }
    private:
        int ik_flag_ = -1;  // Default to elbow-down configuration
};