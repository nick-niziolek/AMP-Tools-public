// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the headers for HW4 code
#include "CSpaceSkeleton.h"
#include "ManipulatorSkeleton.h"

// Include the header of the shared class
#include "HelpfulClass.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    Manipulator2D manipulator(std::vector<double>{1.0,1.0,1.0,1.0,1.0,1.0}); // 3-link manipulator

    // // You can visualize your manipulator given an angle state like so:

    // // amp::ManipulatorState test_state(manipulator.nLinks()); // zero state
    // // test_state.setZero();

    // //amp::ManipulatorState test_state(manipulator.nLinks());
    // //test_state << M_PI/6.0, M_PI/3.0, 7.0 * (M_PI/4.0);

    // //manipulator.setIKFlag(1); 
    amp::ManipulatorState test_state = manipulator.getConfigurationFromIK(Eigen::Vector2d(3,3));

    // // The visualizer uses your implementation of forward kinematics to show the joint positions so you can use that to test your FK algorithm
    // std::cout << "Test state: " << test_state.transpose() << std::endl;
    Visualizer::makeFigure(manipulator, test_state); 

    // // Create the collision space constructor
    //std::size_t n_cells = 500;
    //MyManipulatorCSConstructor cspace_constructor(n_cells);

    // // Create the collision space using a given manipulator and environment
    // std::unique_ptr<amp::GridCSpace2D> cspace1 = cspace_constructor.construct(manipulator, HW4::getEx3Workspace1());
    // std::unique_ptr<amp::GridCSpace2D> cspace2 = cspace_constructor.construct(manipulator, HW4::getEx3Workspace2());
    // std::unique_ptr<amp::GridCSpace2D> cspace3 = cspace_constructor.construct(manipulator, HW4::getEx3Workspace3());

    // // You can visualize your cspace
    // Visualizer::makeFigure(*cspace1);
    // Visualizer::makeFigure(*cspace2);
    // Visualizer::makeFigure(*cspace3);

    // // Visualize the workspaces
    // Visualizer::makeFigure(HW4::getEx3Workspace1());
    // Visualizer::makeFigure(HW4::getEx3Workspace2());
    // Visualizer::makeFigure(HW4::getEx3Workspace3());

    Visualizer::saveFigures(true, "hw4_figs");

    //Grade method
    //amp::HW4::grade<Manipulator2D>(cspace_constructor, "nicholas.niziolek@colorado.edu", argc, argv);
    return 0;
}