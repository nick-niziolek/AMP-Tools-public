// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"
#include "hw/HW2.h"

// Include any custom headers you created in your workspace
#include "MyGDAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Set initial values
    double d_star = 1.0;
    double zetta = 1.0;
    double Q_star = 0.05;
    double eta = 0.01;

    // Visualization and parameter sweep variables
    // amp::Path2D path;
    // amp::Problem2D prob;
    // MyGDAlgorithm algo(d_star, zetta, 0,0);
    // path = algo.plan(HW5::getWorkspace1());
    // prob = HW5::getWorkspace1();
    // bool success = HW5::check(path, prob, true);
    // Visualizer::makeFigure(prob, path);  // Fixed
    // MyPotentialFunction potential_function(prob, d_star, zetta, 0,0);
    // Visualizer::makeFigure(potential_function, prob, 50);

    // For HW2
    // MyGDAlgorithm algo2(d_star, zetta, Q_star, eta);
    // path = algo2.plan(HW2::getWorkspace1());
    // std::cout << "Path Length: " << path.length() << std::endl;
    // prob = HW2::getWorkspace1();
    // bool success = HW5::check(path, prob, true);
    // Visualizer::makeFigure(prob, path);  // Fixed
    // MyPotentialFunction potential_function2(prob, d_star, zetta, Q_star, eta);
    // Visualizer::makeFigure(potential_function2, prob, 50);

    // // For HW2
    // MyGDAlgorithm algo3(d_star, zetta, Q_star, eta);
    // path = algo3.plan(HW2::getWorkspace2());
    // std::cout << "Path Length: " << path.length() << std::endl;
    // prob = HW2::getWorkspace2();
    // success = HW5::check(path, prob, true);
    // Visualizer::makeFigure(prob, path);  // Fixed
    // MyPotentialFunction potential_function3(prob, d_star, zetta, Q_star, eta);
    // Visualizer::makeFigure(potential_function3, prob, 50);

    // std::vector<double> best_vals = {Q_star, eta};
    // int best_score = 0;

    // amp::Path2D path;
    // amp::Problem2D prob;
    // MyGDAlgorithm algo(d_star, zetta, Q_star, eta);

    // std::vector<double> best_HW5_params{0,0};
    // std::vector<double> best_HW2_params{Q_star, eta};

    // // // Sweep Q_star and eta to find best parameters for HW5 WS 1
    // // for (double Q_star = 0.01; Q_star <= 1.0; Q_star += 0.01) {
    // //     bool success = false;
    // //     for (double eta = 0.01; eta <= 1.0; eta += 0.01) {
    // //         std::cout << "Testing Q*: " << Q_star << ", eta: " << eta << std::endl;
    // //         algo = MyGDAlgorithm(d_star, zetta, Q_star, eta);
    // //         success = HW5::check(algo.plan(HW5::getWorkspace1()), HW5::getWorkspace1(), false);
    // //         if (success) {
    // //             best_HW5_params = {Q_star, eta};
    // //             break;
    // //         }
    // //     }
    // //     if (success) break;
    // // }

    // // Visualize best parameters for HW5 WS 1
    // Q_star = best_HW5_params[0];
    // eta = best_HW5_params[1];
    // algo = MyGDAlgorithm(d_star, zetta, Q_star, eta);
    // Visualizer::makeFigure(HW5::getWorkspace1(), algo.plan(HW5::getWorkspace1()));
    // MyPotentialFunction potential_function(HW5::getWorkspace1(), d_star, zetta, Q_star, eta);
    // Visualizer::makeFigure(potential_function, HW5::getWorkspace1(), 30);

    // // Sweep Q_star and eta to find best parameters for HW2 WS 1/2
    // double best_score = 0.0;
    // std::vector<double> best_HW2_params{Q_star, eta};
    // bool any_success = false;
    // for (double Q_star = 0.05; Q_star <= 0.3; Q_star += 0.05) {
    //     bool success = false;
    //     for (double eta = 0.01; eta <= 0.5; eta += 0.01) {
    //         std::cout << "Testing Q*: " << Q_star << ", eta: " << eta << std::endl;
    //         algo = MyGDAlgorithm(d_star, zetta, Q_star, eta);
    //         bool success1 = HW2::check(algo.plan(HW2::getWorkspace1()), HW2::getWorkspace1(), false);
    //         bool success2 = HW2::check(algo.plan(HW2::getWorkspace2()), HW2::getWorkspace2(), false);
    //         int score = (success1 ? 1 : 0) + (success2 ? 1 : 0);
    //         if (score > best_score) {
    //             best_score = score;
    //             best_HW2_params = {Q_star, eta};
    //         }
    //         if (score == 2) {
    //             success = true;
    //             break;
    //         }
    //         if (success1 || success2) any_success = true;
    //     }
    //     if (success) break;
    // }

    // std::vector<double> best_HW2_params{0.05, 0.01};
    // //std::vector<double> best_HW2_params{Q_star, eta};
    // Q_star = best_HW2_params[0];
    // eta = best_HW2_params[1];
    // algo = MyGDAlgorithm(d_star, zetta, Q_star, eta);
    // std::cout << "Visualizing best HW2 params: Q*: " << Q_star << ", eta: " << eta << std::endl;

    // // // Visualize for HW2 Workspace 1
    // Visualizer::makeFigure(HW2::getWorkspace1(), algo.plan(HW2::getWorkspace1()));
    // MyPotentialFunction potential_function4(HW2::getWorkspace1(), d_star, zetta, Q_star, eta);
    // Visualizer::makeFigure(potential_function4, HW2::getWorkspace1(), 50);
    // std::cout << "Path Length: " << algo.plan(HW2::getWorkspace1()).length() << std::endl;

    // Visualize for HW2 Workspace 2
    // Visualizer::makeFigure(HW2::getWorkspace2(), algo.plan(HW2::getWorkspace2()));
    // MyPotentialFunction potential_function5(HW2::getWorkspace2(), d_star, zetta, Q_star, eta);
    // Visualizer::makeFigure(potential_function5, HW2::getWorkspace2(), 50);

    // std::cout << "Path Length: " << algo.plan(HW2::getWorkspace2()).length() << std::endl;

    // // // Do sweep over random scenarios
    // // best_score = 0.0;
    // // best_vals = {Q_star, eta};
    // // for (double Q_star = 0.0; Q_star <= 2.0; Q_star += 0.1) {
    // //     for (double eta = 0.0; eta <= 2.0; eta += 0.1) {
    // //         std::cout << "Testing Q*: " << Q_star << ", eta: " << eta << std::endl;
    // //         algo = MyGDAlgorithm(d_star, zetta, Q_star, eta);
    // //         int score = 0;
    // //         for (int i = 0; i < 25; i++) {
    // //             bool success = HW5::generateAndCheck(algo, path, prob, false);
    // //             score += success ? 1 : 0; 
    // //         }
    // //         if (score > best_score) {
    // //             best_score = score;
    // //             best_vals = {Q_star, eta};
    // //         }
    // //     }
    // // }

    // std::cout << "Best HW5 params: Q*: " << best_HW5_params[0] << ", eta: " << best_HW5_params[1] << std::endl;
    // std::cout << "Best HW2 params: Q*: " << best_HW2_params[0] << ", eta: " << best_HW2_params[1] << std::endl;
    // std::cout << "Best random sweep params: Q*: " << best_vals[0] << ", eta: " << best_vals[1] << std::endl;

    // // Visualize best parameters for HW2 WS 1/2
    // Q_star = best_HW2_params[0];
    // eta = best_HW2_params[1];
    // algo = MyGDAlgorithm(d_star, zetta, Q_star, eta);
    // std::cout << "Visualizing best HW2 params: Q*: " << Q_star << ", eta: " << eta << std::endl;

    // // Visualize for HW2 Workspace 1
    // Visualizer::makeFigure(HW2::getWorkspace1(), algo.plan(HW2::getWorkspace1()));
    // MyPotentialFunction potential_function1(HW2::getWorkspace1(), d_star, zetta, Q_star, eta);
    // Visualizer::makeFigure(potential_function1, HW2::getWorkspace1(), 50);

    // // Visualize for HW2 Workspace 2
    // Visualizer::makeFigure(HW2::getWorkspace2(), algo.plan(HW2::getWorkspace2()));
    // MyPotentialFunction potential_function2(HW2::getWorkspace2(), d_star, zetta, Q_star, eta);
    // Visualizer::makeFigure(potential_function2, HW2::getWorkspace2(), 50);

    // // std::ofstream log("sweep_results.csv");
    // // log << "Q_star,eta,score\n";
    // // for (double Q_star = 0.01; Q_star <= 0.2; Q_star += 0.01) {
    // //     for (double eta = 0.01; eta <= 0.2; eta += 0.01) {
    // //         algo = MyGDAlgorithm(d_star, zetta, Q_star, eta);
    // //         int score = 0;
    // //         for (int i = 0; i < 25; i++) {
    // //             bool success = HW5::generateAndCheck(algo, path, prob, false);
    // //             score += success ? 1 : 0; 
    // //         }
    // //         if (score > best_score) {
    // //             best_score = score;
    // //             best_vals = {Q_star, eta};
    // //         }
    // //         log << Q_star << "," << eta << "," << score << "\n";
    // //     }
    // // }
    // // std::cout << "Best score: " << best_score << " with Q*: " << best_vals[0] << ", eta: " << best_vals[1] << std::endl;
    // // // // Set best parameters found
    // Q_star = best_vals[0];
    // eta = best_vals[1];
    // // Continuously test random problems until the algorithm fails
    // int test_count = 0;
    // algo = MyGDAlgorithm(d_star, zetta, Q_star, eta);
    // while (true) {
    //     bool success = HW5::generateAndCheck(algo, path, prob);
    //     if (!success) {
    //         std::cout << "Test failed on iteration " << test_count << std::endl;
    //         Visualizer::makeFigure(prob, path);
    //         MyPotentialFunction potential_function(prob, d_star, zetta, Q_star, eta);
    //         Visualizer::makeFigure(potential_function, prob, 50);
    //         break;
    //     }
    //     ++test_count;
    // }

    // Visualizer::saveFigures(true, "hw5_figs");
    
    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    HW5::grade<MyGDAlgorithm>("nicholas.niziolek@colorado.edu", argc, argv, d_star, zetta, Q_star, eta);
    return 0;
}