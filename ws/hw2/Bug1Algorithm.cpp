#include "Bug1Algorithm.h"
#include "CollisionChecker.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D Bug1Algorithm::plan(const amp::Problem2D& problem) {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;

    CollisionChecker checker;

    const double step_size = 0.001;
    const double turn_step = 0.05;

    Eigen::Vector2d current_position = problem.q_init;
    path.waypoints.push_back(current_position);
    //std::cout << "Bug is at position: " << current_position.transpose() << std::endl;

    bool stop = false;
    while (stop != true) {
        // Move towards goal in increments of step_size
        Eigen::Vector2d direction = (problem.q_goal - current_position).normalized();
        Eigen::Vector2d next_position = current_position + direction * step_size;

        // Check for collision at next_position
        if (checker.LineCollisionChecker(current_position,next_position,problem)) {
            //std::cout << "Collision detected at position: " << next_position.transpose() << " Starting Loop" << std::endl;
            // Follow boundary, left turning bug algorithm
            // Turn left until valid move is found 
            bool found_valid_move = false;
            double angle = std::atan2(direction.y(), direction.x());
            Eigen::Vector2d boundary_start = current_position;
            int boundary_start_index = path.waypoints.size() - 1;
            while (!found_valid_move) {
                // Rotate direction vector left by turn_step radians
                angle += turn_step; // turn left
                direction = Eigen::Vector2d(std::cos(angle), std::sin(angle));
                next_position = current_position + direction * step_size;

                if (!checker.LineCollisionChecker(current_position,next_position,problem)) {
                    found_valid_move = true;
                    boundary_start = next_position;
                    boundary_start_index = path.waypoints.size();
                    Eigen::Vector2d direction = (next_position - current_position).normalized();
                    angle = std::atan2(direction.y(), direction.x());
                    current_position = next_position;
                    path.waypoints.push_back(current_position);
                    //std::cout << boundary_start.transpose() << " vs " << path.waypoints[boundary_start_index].transpose() << std::endl;
                    //std::cout << "Bug is at position: " << current_position.transpose() << std::endl;

                }
            }

            //std::cout << "Found valid move to start boundary following at position: " << current_position.transpose() << std::endl;

            // Follow boundary till back at start point
            bool found_boundary_start = false;
            float min_dist_to_goal = (current_position - problem.q_goal).norm();
            float min_dist_to_start = 100;
            int min_dist_to_goal_index = -1;
            Eigen::Vector2d closest_point_to_goal = current_position;
            Eigen::Vector2d test_position = current_position + Eigen::Vector2d(std::cos(angle), std::sin(angle)) * step_size;

            int j = 0;
            int i = 0;
            double eff_step_size = step_size;
            while(!found_boundary_start) {
                // Check if next turn is collision, if so turn left until clear, else turn right until it almost hits
                j++;

                /*if (j % 50 == 0) {
                    std::cout << "Boundary following iteration " << j << ", current position: " << current_position.transpose() << ", distance to goal: " << (current_position - problem.q_goal).norm() << ", min distance to goal: " << min_dist_to_goal << std::endl;
                }*/

                test_position = current_position + Eigen::Vector2d(std::cos(angle), std::sin(angle)) * eff_step_size;
                i = 0;
                if (checker.LineCollisionChecker(current_position,test_position,problem)) {
                    // Collision, turn left
                    while (checker.LineCollisionChecker(current_position,test_position,problem)) {
                        angle += turn_step; 
                        test_position = current_position + Eigen::Vector2d(std::cos(angle), std::sin(angle)) * eff_step_size;
                        if (i++ > 1000) {
                            std::cout << "Stuck turning left in boundary following, stopping." << std::endl;
                            return path;
                        }
                    }
                } else {
                    // No collision, turn right
                    while (!checker.LineCollisionChecker(current_position,test_position,problem)) {
                        angle -= turn_step; 
                        test_position = current_position + Eigen::Vector2d(std::cos(angle), std::sin(angle)) * eff_step_size;
                        if (i++ > 1000) {
                            std::cout << "Stuck turning right in boundary following, stopping." << std::endl;
                            return path;
                        }
                    }
                    angle += turn_step; 
                    test_position = current_position + Eigen::Vector2d(std::cos(angle), std::sin(angle)) * eff_step_size;
                }
                
                // Make the move
                next_position = test_position;
                current_position = next_position;
                path.waypoints.push_back(current_position);
                //std::cout << "Bug is at position: " << current_position.transpose() << std::endl;
                // Record distance to goal if it is the new lowest
                if ((current_position - problem.q_goal).norm() < min_dist_to_goal){
                    min_dist_to_goal = (current_position - problem.q_goal).norm();
                    closest_point_to_goal = current_position;
                    min_dist_to_goal_index = path.waypoints.size() - 1;
                }

                // Check if back at boundary start
                if (((current_position - boundary_start).norm() <= eff_step_size * 1.5)&&(j > 10)) {
                    if (!checker.LineCollisionChecker(current_position,boundary_start,problem)) {
                        found_boundary_start = true;
                    }
                    //std::cout << "Completed boundary loop. Distance is " << (current_position - boundary_start).norm() << std::endl;
                }

                if (((current_position - boundary_start).norm() < min_dist_to_start)&&(j > 10)){
                    min_dist_to_start = (current_position - boundary_start).norm();
                }

                if (j > 1000000) {
                    std::cout << "Too many boundary iterations, stopping. Closest at " << min_dist_to_start << std::endl;
                    return path;
                }
            }

            //std::cout << "Boundary loop complete. Closest distance to goal was " << min_dist_to_goal << " at point " << closest_point_to_goal.transpose() << std::endl;
            // Move bug back to the closest point to boundary start found during boundary following
            if (min_dist_to_goal_index != -1 && min_dist_to_goal_index > boundary_start_index) {
                for (int i = boundary_start_index + 1; i <= min_dist_to_goal_index; ++i) {
                    path.waypoints.push_back(path.waypoints[i]);
                    current_position = path.waypoints[i];
                }
            }

            /*
            // Follow boundary till closest point to goal
            while (!((current_position - closest_point_to_goal).norm() <= step_size)) {
                // Check if next turn is collision, if so turn left, else turn right

                int i = 0;
                test_position = current_position + Eigen::Vector2d(std::cos(angle), std::sin(angle)) * step_size;
                if (checker.LineCollisionChecker(current_position,test_position,problem)) {
                    // Collision, turn left
                    while (checker.LineCollisionChecker(current_position,test_position,problem)) {
                        angle += turn_step; 
                        test_position = current_position + Eigen::Vector2d(std::cos(angle), std::sin(angle)) * step_size;
                    }
                } else {
                    // No collision, turn right
                    while (!checker.LineCollisionChecker(current_position,test_position,problem)) {
                        angle -= turn_step; 
                        test_position = current_position + Eigen::Vector2d(std::cos(angle), std::sin(angle)) * step_size;
                    }
                    angle += turn_step; 
                    test_position = current_position + Eigen::Vector2d(std::cos(angle), std::sin(angle)) * step_size;
                }
                
                // Make the move
                Eigen::Vector2d direction = (test_position - current_position).normalized();
                angle = std::atan2(direction.y(), direction.x());
                next_position = current_position + direction * step_size;
                current_position = next_position;
                path.waypoints.push_back(current_position);
                //std::cout << "Bug is at position: " << current_position.transpose() << std::endl;
            }
            */

            //std::cout << "Reached closest point to goal on boundary. Distance is " << (current_position - closest_point_to_goal).norm() << std::endl;

            next_position = current_position + (problem.q_goal - current_position).normalized() * step_size;
            if (checker.LineCollisionChecker(current_position,next_position,problem)) {
                std::cout << "Still in collision after boundary following, stopping." << std::endl;
                return path;
            }

        } else {
            // No collision, move to next position
            current_position = next_position;
            path.waypoints.push_back(current_position);
            //std::cout << "Bug is at position: " << current_position.transpose() << std::endl;
        }

        // Check if goal is reached
        if ((current_position - problem.q_goal).norm() < step_size) {
            stop = true;
            path.waypoints.push_back(problem.q_goal);
            //std::cout << "Bug is at position: " << current_position.transpose() << std::endl;
        }
    }

    Eigen::Vector2d segment_start = path.waypoints[-1];
    Eigen::Vector2d segment_end = path.waypoints[-1];
    for (int i = 1; i < path.waypoints.size(); i++) {
        segment_start = path.waypoints[i-1];
        segment_end = path.waypoints[i];
        if (checker.LineCollisionChecker(segment_start,segment_end,problem)) {
            std::cout << "Final path has collision between " << segment_start.transpose() << " and " << segment_end.transpose() << std::endl;
        }
    }

    return path;
}
