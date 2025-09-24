#include "CSpaceSkeleton.h"
#include "CollisionChecker.h"

// Override this method for returning whether or not a point is in collision

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
