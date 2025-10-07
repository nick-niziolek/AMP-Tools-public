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
    double theta_min = 0;
    double theta_max = 2 * M_PI;
    double step_size = 0.005;
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, theta_min, theta_max, theta_min, theta_max);
    //std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;

    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    CollisionChecker collision_checker;
    amp::ManipulatorState state(2);
    for (double i = theta_min; i < theta_max; i += step_size) {
        for (double j = theta_min; j < theta_max; j += step_size) {
            state << i, j; // Set the manipulator state to the current angles
            bool in_collision = collision_checker.isManipStateFeasible(manipulator, state, env);
            if (!in_collision) {
                auto [cell_x, cell_y] = cspace.getCellFromPoint(i, j);
                cspace(cell_x, cell_y) = !in_collision;
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
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for point agent" << std::endl;
    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    CollisionChecker collision_checker;
    double x0_min = env.x_min, x0_max = env.x_max;
    double x1_min = env.y_min, x1_max = env.y_max;
    double cell_size_x = (x0_max - x0_min) / m_cells_per_dim;
    double cell_size_y = (x1_max - x1_min) / m_cells_per_dim;
    // Loop over each cell in the grid
    for (std::size_t i = 0; i < m_cells_per_dim; ++i) {
        for (std::size_t j = 0; j < m_cells_per_dim; ++j) {
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
    std::cout << "Planning with Wavefront" << std::endl;
    // Propagate wavefront values from goal to init
    auto [cell_x_init, cell_y_init] = grid_cspace.getCellFromPoint(q_init(0), q_init(1));
    auto [cell_x_goal, cell_y_goal] = grid_cspace.getCellFromPoint(q_goal(0), q_goal(1));
    // Make dense grid object to store wavefront values
    amp::DenseArray2D<int> wavefront_values(grid_cspace.size().first, grid_cspace.size().second, 0); // Initialize all values to 0 
    // Set goal cell to 2
    wavefront_values(cell_x_goal, cell_y_goal) = 2; 
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
    while(!at_start) {
        at_start = (wavefront_values(cell_x_init, cell_y_init) > 0); // If start cell has been reached, we are done
        // Loop over all cells and propagate wavefront values
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
                        }
                    }
                }
            }
        }
        current_wave_value++;
    }

    amp::Path2D path;
    path.waypoints.push_back(q_init); // Start path with initial configuration

    // Start at init cell and step towards lowest cell until at goal
    auto [cell_x_curr, cell_y_curr] = grid_cspace.getCellFromPoint(q_init(0), q_init(1));
    Eigen::Vector2d q = q_init;
    while (cell_x_curr != cell_x_goal || cell_y_curr != cell_y_goal) {
        std::cout << "Current cell: (" << cell_x_curr << ", " << cell_y_curr << "), Wave value: " << wavefront_values(cell_x_curr, cell_y_curr) << std::endl;
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

    path.waypoints.push_back(q_goal); // Ensure goal is the last waypoint

    if (isManipulator) {
        Eigen::Vector2d bounds0 = Eigen::Vector2d(0.0, 0.0);
        Eigen::Vector2d bounds1 = Eigen::Vector2d(2*M_PI, 2*M_PI);
        amp::unwrapWaypoints(path.waypoints, bounds0, bounds1);
    }
    return path;
}
