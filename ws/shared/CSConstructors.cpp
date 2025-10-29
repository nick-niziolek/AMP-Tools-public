#include "CSConstructors.h"
#include "CollisionChecker.h"

////////////////////// THIS IS FROM HW4 //////////////////////

std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // Get bounds on each dimension using getter methods
    auto [x0_min, x0_max] = x0Bounds();
    auto [x1_min, x1_max] = x1Bounds();

    // Get number of cells in each dimension using size()
    auto [num_cells_x0, num_cells_x1] = size();

    // Compute normalized positions in [0, 1]
    double norm_x0 = (x0 - x0_min) / (x0_max - x0_min);
    double norm_x1 = (x1 - x1_min) / (x1_max - x1_min);

    // Clamp normalized values to [0, 1]
    norm_x0 = std::max(0.0, std::min(1.0, norm_x0));
    norm_x1 = std::max(0.0, std::min(1.0, norm_x1));

    // Compute cell indices
    std::size_t cell_x = static_cast<std::size_t>(norm_x0 * num_cells_x0);
    std::size_t cell_y = static_cast<std::size_t>(norm_x1 * num_cells_x1);

    // Clamp indices to valid range
    if (cell_x >= num_cells_x0) cell_x = num_cells_x0 - 1;
    if (cell_y >= num_cells_x1) cell_y = num_cells_x1 - 1;

    return {cell_x, cell_y};
}

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::cout << "Constructing C-space for manipulator" << std::endl;
    double theta_min = -2 * M_PI;
    double theta_max = 4 * M_PI;
    double grid_size = 0.25; // radians
    std::size_t m_cells_per_dim_x = static_cast<std::size_t>(std::ceil((theta_max - theta_min) / grid_size));
    std::size_t m_cells_per_dim_y = static_cast<std::size_t>(std::ceil((theta_max - theta_min) / grid_size));
    // double step_size = 0.005;
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim_x, m_cells_per_dim_y, theta_min, theta_max, theta_min, theta_max);
    //std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;

    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    CollisionChecker collision_checker;
    amp::ManipulatorState state(2);
    // Set all cells to true first
    for (std::size_t i = 0; i < m_cells_per_dim_x; ++i) {
        for (std::size_t j = 0; j < m_cells_per_dim_y; ++j) {
            cspace(i, j) = false; // Initialize all cells to false (not in collision)
        }
    }
    for (std::size_t i = 0; i < m_cells_per_dim_x; ++i) {
        for (std::size_t j = 0; j < m_cells_per_dim_y; ++j) {
            // Get cell center
            double theta1 = theta_min + (i + 0.5) * (theta_max - theta_min) / m_cells_per_dim_x;
            double theta2 = theta_min + (j + 0.5) * (theta_max - theta_min) / m_cells_per_dim_y;
            state << theta1,theta2; // Set the manipulator state to the current angles
            bool no_collision = collision_checker.isManipStateFeasible(manipulator, state, env);
            if (!no_collision) {
                cspace(i,j) = !no_collision; // Store the collision status
            }
        }
    }

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

/////////////////////////////HW6/////////////////////////////////

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyPointAgentCSConstructor::construct(const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    // make grid cells of 0.25 edge lengths
    double cell_size = 0.25;
    std::size_t m_cells_per_dim_x = static_cast<std::size_t>(std::ceil((env.x_max - env.x_min) / cell_size));
    std::size_t m_cells_per_dim_y = static_cast<std::size_t>(std::ceil((env.y_max - env.y_min) / cell_size));

    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim_x, m_cells_per_dim_y, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for point agent" << std::endl;
    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    CollisionChecker collision_checker;
    double x0_min = env.x_min, x0_max = env.x_max;
    double x1_min = env.y_min, x1_max = env.y_max;
    double cell_size_x = (x0_max - x0_min) / m_cells_per_dim_x;
    double cell_size_y = (x1_max - x1_min) / m_cells_per_dim_y;
    // Loop over each cell in the grid
    for (std::size_t i = 0; i < m_cells_per_dim_x; ++i) {
        for (std::size_t j = 0; j < m_cells_per_dim_y; ++j) {
            // Compute the center of the cell
            double cell_center_x = x0_min + (i + 0.5) * cell_size_x;
            double cell_center_y = x1_min + (j + 0.5) * cell_size_y;
            Eigen::Vector2d cell_center(cell_center_x, cell_center_y);
            // Check if the cell is in collision using the CollisionChecker
            bool in_collision = collision_checker.isCellColliding(cell_center, cell_size_x, cell_size_y, env);
            cspace(i, j) = in_collision; // Store the collision status (true if in collision, false if free)
        }
    }
    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator) {
    // std::cout << "Building Wavefront" << std::endl;
    std::cout << "Starting Position: (" << q_init(0) << ", " << q_init(1) << ")" << std::endl;
    std::cout << "Goal Position: (" << q_goal(0) << ", " << q_goal(1) << ")" << std::endl;
    amp::Path2D path;
    Eigen::Vector2d q_init_mod = q_init;
    Eigen::Vector2d q_goal_mod = q_goal;
    if (isManipulator) {
        // Get start/end within 0-2*pi
        while (q_init_mod(0) < 0) q_init_mod(0) += 2 * M_PI;
        while (q_init_mod(0) >= 2 * M_PI) q_init_mod(0) -= 2 * M_PI;
        while (q_init_mod(1) < 0) q_init_mod(1) += 2 * M_PI;
        while (q_init_mod(1) >= 2 * M_PI) q_init_mod(1) -= 2 * M_PI;
        while (q_goal_mod(0) < 0) q_goal_mod(0) += 2 * M_PI;
        while (q_goal_mod(0) >= 2 * M_PI) q_goal_mod(0) -= 2 * M_PI;
        while (q_goal_mod(1) < 0) q_goal_mod(1) += 2 * M_PI;
        while (q_goal_mod(1) >= 2 * M_PI) q_goal_mod(1) -= 2 * M_PI;
        std::cout << "Modified Starting Position: (" << q_init_mod(0) << ", " << q_init_mod(1) << ")" << std::endl;
        std::cout << "Modified Goal Position: (" << q_goal_mod(0) << ", " << q_goal_mod(1) << ")" << std::endl;
    }

    // Make dense grid object to store wavefront values
    amp::DenseArray2D<int> wavefront_values(grid_cspace.size().first, grid_cspace.size().second, 0); // Initialize all values to 0 

    // Propagate wavefront values from goal to init
    auto [cell_x_init, cell_y_init] = grid_cspace.getCellFromPoint(q_init_mod(0), q_init_mod(1));
    // Support multiple goal cells (for manipulators with wrapping, or for other multi-goal cases)
    std::vector<std::pair<std::size_t, std::size_t>> goal_cells;
    goal_cells.emplace_back(grid_cspace.getCellFromPoint(q_goal_mod(0), q_goal_mod(1)));
    // For manipulators, add shifted goals (handled below), but for general case, just use this one

    // For manipulators, add shifted goal cells (within 2*pi wrapping)
    if (isManipulator) {
        goal_cells.clear(); // We'll add all shifted goals below
        for (int i = -1; i <= 1; ++i) {
            for (int j = -1; j <= 1; ++j) {
                auto [shifted_x_goal, shifted_y_goal] = grid_cspace.getCellFromPoint(q_goal_mod(0) + i * 2 * M_PI, q_goal_mod(1) + j * 2 * M_PI);
                goal_cells.emplace_back(shifted_x_goal, shifted_y_goal);
            }
        }
    }

    // Set all goal cells to 2 in the wavefront
    for (const auto& [cell_x_goal, cell_y_goal] : goal_cells) {
        wavefront_values(cell_x_goal, cell_y_goal) = 2;
    }

    // Set all obstacle cells to 1
    for (std::size_t i = 0; i < grid_cspace.size().first; ++i) {
        for (std::size_t j = 0; j < grid_cspace.size().second; ++j) {
            if (grid_cspace(i, j)) { // If cell is in collision
                wavefront_values(i, j) = 1; // Mark as obstacle
            }
        }
    }

    bool at_start = false;
    int current_wave_value = 2;
    int max_wave_value = 10000; // Prevent infinite loops
    bool any_changed = false;
    while(!at_start) {
        // std::cout << "Current wave value: " << current_wave_value << std::endl;
        // Check if we have reached the start cell
        at_start = (wavefront_values(cell_x_init, cell_y_init) > 0); // If start cell has been reached, we are done
        if (at_start) break;
        // Loop over all cells and propagate wavefront values
        any_changed = false;
        for (std::size_t i = 0; i < grid_cspace.size().first; ++i) {
            for (std::size_t j = 0; j < grid_cspace.size().second; ++j) {
                if (wavefront_values(i, j) == current_wave_value) { // If this cell has the current wave value
                    // Check neighbors (4-connectivity)
                    std::vector<std::pair<int, int>> neighbors = {
                        {static_cast<int>(i) - 1, static_cast<int>(j)}, // Left
                        {static_cast<int>(i) + 1, static_cast<int>(j)}, // Right
                        {static_cast<int>(i), static_cast<int>(j) - 1}, // Down
                        {static_cast<int>(i), static_cast<int>(j) + 1}  // Up
                    };
                    for (const auto& [ni, nj] : neighbors) {
                        // Check bounds and if neighbor is free and unvisited
                        if (ni >= 0 && ni < static_cast<int>(grid_cspace.size().first) &&
                            nj >= 0 && nj < static_cast<int>(grid_cspace.size().second) &&
                            !grid_cspace(ni, nj) && // Not occupied
                            wavefront_values(ni, nj) == 0) { // Unvisited
                            wavefront_values(ni, nj) = current_wave_value + 1; // Set wave value
                            any_changed = true;
                        }
                    }
                }
            }
        }
        current_wave_value++;
        if (current_wave_value > max_wave_value) {
            std::cerr << "Error: Exceeded maximum wave values without reaching start!" << std::endl;
            path.waypoints.push_back(q_init_mod);
            path.waypoints.push_back(q_goal_mod);
            return path; // Return empty path
        }
        if (!any_changed) {
            std::cerr << "Error: No more cells to propagate to, but start not reached!" << std::endl;
            path.waypoints.push_back(q_init_mod);
            path.waypoints.push_back(q_goal_mod);
            return path; // Return empty path
        }
    }

    // std::cout << "Finished Building Wavefront" << std::endl;

    path.waypoints.push_back(q_init_mod); // Start path with initial configuration
    int iteration = 0;
    int max_iterations = 10000; // Prevent infinite loops
    // Start at init cell and step towards lowest cell until at goal
    auto [cell_x_curr, cell_y_curr] = grid_cspace.getCellFromPoint(q_init_mod(0), q_init_mod(1));
    Eigen::Vector2d q = q_init_mod;
    // Find if current cell matches any of the goal cells
    auto is_goal_cell = [&](std::size_t x, std::size_t y) {
        for (const auto& [gx, gy] : goal_cells) {
            if (x == gx && y == gy) return true;
        }
        return false;
    };

    while (!is_goal_cell(cell_x_curr, cell_y_curr)) {
        iteration++;
        if (iteration > max_iterations) {
            std::cerr << "Error: Exceeded maximum iterations while tracing path!" << std::endl;
            break;
        }
        // std::cout << "Current cell: (" << cell_x_curr << ", " << cell_y_curr << "), Wave value: " << wavefront_values(cell_x_curr, cell_y_curr) << std::endl;
        // Find lowest neighbor (4-connectivity)
        std::vector<std::pair<int, int>> neighbors = {
            {static_cast<int>(cell_x_curr) - 1, static_cast<int>(cell_y_curr)}, // Left
            {static_cast<int>(cell_x_curr) + 1, static_cast<int>(cell_y_curr)}, // Right
            {static_cast<int>(cell_x_curr), static_cast<int>(cell_y_curr) - 1}, // Down
            {static_cast<int>(cell_x_curr), static_cast<int>(cell_y_curr) + 1}  // Up
        };
        int lowest_value = wavefront_values(cell_x_curr, cell_y_curr);
        if (lowest_value == 1) {
            std::cerr << "Error: Stuck at an obstacle cell!" << std::endl;
            break; // Should not happen if path exists
        }
        std::pair<int, int> next_cell = {static_cast<int>(cell_x_curr), static_cast<int>(cell_y_curr)};
        for (const auto& [ni, nj] : neighbors) {
            // Check bounds
            if (ni >= 0 && ni < static_cast<int>(grid_cspace.size().first) &&
                nj >= 0 && nj < static_cast<int>(grid_cspace.size().second)) {
                int neighbor_value = wavefront_values(ni, nj);
                if (neighbor_value > 1 && neighbor_value < lowest_value) { // Valid wave value and lower than current lowest
                    lowest_value = neighbor_value;
                    next_cell = {ni, nj};
                }
            }
        }
        // Move to next cell
        cell_x_curr = next_cell.first;
        cell_y_curr = next_cell.second;
        // Convert cell indices back to continuous configuration space coordinates
        auto [x0_min, x0_max] = grid_cspace.x0Bounds();
        auto [x1_min, x1_max] = grid_cspace.x1Bounds();
        double cell_size_x = (x0_max - x0_min) / grid_cspace.size().first;
        double cell_size_y = (x1_max - x1_min) / grid_cspace.size().second;
        double config_x0 = x0_min + (cell_x_curr + 0.5) * cell_size_x;
        double config_x1 = x1_min + (cell_y_curr + 0.5) * cell_size_y;
        // Add to path waypoints
        q << config_x0, config_x1;
        path.waypoints.push_back(q);
    }

    // Ensure the goal is the last waypoint, with wrapping consistent with the last q point
    Eigen::Vector2d q_goal_wrapped = q_goal_mod;
    if (isManipulator && path.waypoints.size() > 0) {
        // Wrap each joint to be as close as possible to the last q
        for (int d = 0; d < 2; ++d) {
            double diff = q_goal_mod(d) - q(d);
            // Wrap diff to [-pi, pi]
            diff = std::fmod(diff + M_PI, 2 * M_PI);
            if (diff < 0) diff += 2 * M_PI;
            diff -= M_PI;
            q_goal_wrapped(d) = q(d) + diff;
        }
    }
    path.waypoints.push_back(q_goal_wrapped); // Ensure goal is the last waypoint

    // if (isManipulator) {
    //     Eigen::Vector2d bounds0 = Eigen::Vector2d(0.0, 0.0);
    //     Eigen::Vector2d bounds1 = Eigen::Vector2d(2*M_PI, 2*M_PI);
    //     amp::unwrapWaypoints(path.waypoints, bounds0, bounds1);
    // }
    return path;
}
