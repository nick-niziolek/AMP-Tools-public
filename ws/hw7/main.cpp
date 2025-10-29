// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MySamplingBasedPlanners.h"
#include "tools/Time.h"
#include "hw/HW7.h"
#include "hw/HW6.h"

using namespace amp;

int main(int argc, char** argv) {
    // HW7::hint(); // Consider implementing an N-dimensional planner 

    // Test PRM on Workspace1 of HW2
    // Problem2D problem = HW2::getWorkspace2();
    // MyPRM prm(200,2.0,false);
    // Path2D path;
    // // Run until a valid solution is found, then plot
    // Path2D valid_path;
    // int iter = 0;
    // int max_iter = 1000;
    // while (iter < max_iter) {
    //     iter++;
    //     valid_path = prm.plan(problem);
    //     if (HW7::check(valid_path, problem, true)) {
    //         break;
    //     }       
    // }
    // Visualizer::makeFigure(problem, valid_path, *prm.getRoadmap(), prm.getNodes());
    // std::cout << "Valid path length: " << valid_path.length() << std::endl;

    // // Benchmark over a set of algorithm constructor arguments
    // // std::vector<std::pair<int, double>> prm_params = {
    // //     {200, 0.5},{200, 1},{200, 1.5},{200, 2},{500, 0.5},{500, 1},{500, 1.5},{500, 2}
    // // };
    // std::vector<std::pair<int, double>> prm_params = {
    //     {200, 1},{200, 2},{500, 1},{500, 2},{1000, 1},{1000, 2}
    // };

    // // Saving data sets for # of valid solutions, path lengths, and compute time
    // std::vector<int> valid_solutions_list;
    // std::list<std::vector<double>> path_lengths_list;
    // std::list<std::vector<double>> compute_times_list;

    // for (const auto& param : prm_params) {
    //     int n_samples = param.first;
    //     double radius = param.second;
    //     // Plan over 100 runs, return number of valid solutions, best path length, best path, best graph, best nodes
    //     int valid_solutions = 0;
    //     double best_path_length = std::numeric_limits<double>::infinity();
    //     Path2D best_path;
    //     std::vector<double> path_lengths;
    //     std::vector<double> compute_times;
    //     // Get the roadmap and create node-to-coordinate map
    //     std::shared_ptr<amp::Graph<double>> graphPtr;
    //     // Get populated map from PRM
    //     std::map<amp::Node, Eigen::Vector2d> nodes;  
    //     for (int i = 0; i < 100; ++i) {
    //         MyPRM prm(n_samples, radius, true); // Re-initialize PRM each run
    //         Timer timer("prm_run");
    //         path = prm.plan(problem);
    //         timer.stop();
    //         if (HW7::check(path, problem, false)) {
    //             valid_solutions++;
    //             path_lengths.push_back(path.length());
    //             compute_times.push_back(Profiler::getMostRecentProfile("prm_run", TimeUnit::s));
    //             // Update best path if this one is shorter
    //             double path_length = path.length();
    //             if (path_length < best_path_length) {
    //                 best_path_length = path_length;
    //                 best_path = path;
    //                 graphPtr = prm.getRoadmap(); // Get the roadmap from PRM
    //                 nodes = prm.getNodes(); // Get the node coordinates from PRM
    //             }
    //         }
    //     }
    //     if (valid_solutions == 0) {
    //         std::cout << "PRM found no valid solutions out of 100 runs for parameters (" 
    //               << n_samples << ", " << radius << "). Skipping visualization and data collection." << std::endl;
    //         valid_solutions_list.push_back(0);
    //         path_lengths_list.push_back({-1.0});
    //         compute_times_list.push_back({-1.0});
    //         continue;
    //     }

    //     std::cout << "PRM found " << valid_solutions << " valid solutions out of 100 runs." << std::endl;
    //     std::cout << "Best path length: " << best_path_length << std::endl;

    //     valid_solutions_list.push_back(valid_solutions);
    //     path_lengths_list.push_back(path_lengths);
    //     compute_times_list.push_back(compute_times);

    //     // Visualize - need to dereference the shared_ptr
    //     // graphPtr->print();
    //     // Visualizer::makeFigure(problem, best_path, *graphPtr, nodes);
    // }

    // // Make box plots only if there is data
    // if (!path_lengths_list.empty() && !compute_times_list.empty()) {
    //     // std::vector<std::string> labels = {"200,0.5", "200,1", "200,1.5", "200,2", "500,0.5", "500,1", "500,1.5", "500,2"};
    //     std::vector<std::string> labels = {"200,1", "200,2", "500,1", "500,2", "1000,1", "1000,2"};
    //     // Only use labels for sets with data
    //     std::vector<std::string> filtered_labels;
    //     auto filter_labels = [&](const std::list<std::vector<double>>& data_list) {
    //         filtered_labels.clear();
    //         auto it = labels.begin();
    //         for (const auto& data : data_list) {
    //             // std::cout << "Data size: " << data.size() << ", first element: " << data[0] << std::endl;
    //             if (data[0] >= 0 && it != labels.end()) {
    //                 filtered_labels.push_back(*it);
    //             }
    //             ++it;
    //         }
    //         //std::cout << "Filtered labels size: " << filtered_labels.size() << std::endl;
    //     };
    //     auto filtered_data = [&](const std::list<std::vector<double>>& data_list) {
    //         std::list<std::vector<double>> filtered;
    //         auto it = labels.begin();
    //         for (const auto& data : data_list) {
    //             if (data[0] >= 0 && it != labels.end()) {
    //                 filtered.push_back(data);
    //             }
    //             ++it;
    //         }
    //         return filtered;
    //     };

    //     filter_labels(path_lengths_list);
    //     if (!filtered_labels.empty()) {
    //         Visualizer::makeBoxPlot(filtered_data(path_lengths_list), filtered_labels, "Path Length Comparison", "PRM Parameters (n_samples, radius)", "Path Length");
    //     }

    //     filter_labels(compute_times_list);
    //     if (!filtered_labels.empty()) {
    //         Visualizer::makeBoxPlot(filtered_data(compute_times_list), filtered_labels, "Computation Time Comparison", "PRM Parameters (n_samples, radius)", "Compute Time (s)");
    //     }
    // } else {
    //     std::cout << "No data available for box plots. Skipping visualization." << std::endl;
    // }

    // // Bar plot for valid solutions
    // if (!valid_solutions_list.empty()) {
    //     // std::vector<std::string> labels = {"200,0.5", "200,1", "200,1.5", "200,2", "500,0.5", "500,1", "500,1.5", "500,2"};
    //     std::vector<std::string> labels = {"200,1", "200,2", "500,1", "500,2", "1000,1", "1000,2"};
    //     std::vector<double> valid_solutions_double;
    //     for (const auto& count : valid_solutions_list) {
    //         valid_solutions_double.push_back(static_cast<double>(count));
    //     }
    //     Visualizer::makeBarGraph(valid_solutions_double, labels, "Valid Solutions Comparison", "PRM Parameters (n_samples, radius)", "Number of Valid Solutions");
    // } else {
    //     std::cout << "No valid solutions data available for bar plot. Skipping visualization." << std::endl;
    // }

    // // Test RRT against three environments and benchmark with 100 runs each like above, including plotting the best solution to each of the three environments
    // MyRRT rrt(0.05, 0.5, 5000, 0.25);

    // std::vector<int> valid_solutions_list;
    // std::list<std::vector<double>> path_lengths_list;
    // std::list<std::vector<double>> compute_times_list;

    // for (int env_idx = 1; env_idx <= 3; ++env_idx) {
    //     Problem2D problem;
    //     if (env_idx == 1) {
    //         problem = HW5::getWorkspace1();
    //     } else if (env_idx == 2) {
    //         problem = HW2::getWorkspace1();
    //     } else if (env_idx == 3) {
    //         problem = HW2::getWorkspace2();
    //     }
        
    //     // Benchmark over 100 runs
    //     int valid_solutions = 0;
    //     double best_path_length = std::numeric_limits<double>::infinity();
    //     Path2D best_path;
    //     std::vector<double> path_lengths;
    //     std::vector<double> compute_times;
    //     std::shared_ptr<amp::Graph<double>> graphPtr;
    //     std::map<amp::Node, Eigen::Vector2d> nodes;  
    //     for (int i = 0; i < 100; ++i) {
    //         rrt = MyRRT(0.05, 0.5, 5000, 0.25); // Re-initialize RRT each run
    //         Timer timer("rrt_run");
    //         Path2D path = rrt.plan(problem);
    //         timer.stop();
    //         if (HW7::check(path, problem, false)) {
    //             valid_solutions++;
    //             path_lengths.push_back(path.length());
    //             compute_times.push_back(Profiler::getMostRecentProfile("rrt_run", TimeUnit::s));
    //             double path_length = path.length();
    //             if (path_length < best_path_length) {
    //                 best_path_length = path_length;
    //                 best_path = path;
    //                 graphPtr = rrt.getRoadmap(); // Get the roadmap from RRT
    //                 nodes = rrt.getNodes(); // Get the node coordinates from RRT
    //             }
    //         }
    //     }
    //     if (valid_solutions == 0) {
    //         std::cout << "RRT found no valid solutions out of 100 runs for Environment " 
    //               << env_idx << ". Skipping visualization and data collection." << std::endl;
    //         valid_solutions_list.push_back(0);
    //         continue;
    //     } else {
    //         valid_solutions_list.push_back(valid_solutions);
    //         path_lengths_list.push_back(path_lengths);
    //         compute_times_list.push_back(compute_times);
    //     }
    //     std::cout << "RRT found " << valid_solutions << " valid solutions out of 100 runs for Environment " 
    //               << env_idx << "." << std::endl;
    //     std::cout << "Best path length: " << best_path_length << std::endl;
    //     Visualizer::makeFigure(problem, best_path, *graphPtr, nodes);
    // }

    // // Make box and bar plots for RRT results
    // // Similar to above!
    // if (!path_lengths_list.empty() && !compute_times_list.empty()) {
    //     std::vector<std::string> labels = {"Env HW5 1", "Env HW2 1", "Env HW2 2"};

    //     // Path lengths box plot
    //     Visualizer::makeBoxPlot(path_lengths_list, labels, "RRT Path Length Comparison", "Environment", "Path Length");

    //     // Compute times box plot
    //     Visualizer::makeBoxPlot(compute_times_list, labels, "RRT Computation Time Comparison", "Environment", "Compute Time (s)");

    //     // Valid solutions bar plot
    //     std::vector<double> valid_solutions_double;
    //     for (const auto& count : valid_solutions_list) {
    //         valid_solutions_double.push_back(static_cast<double>(count));
    //     }
    //     Visualizer::makeBarGraph(valid_solutions_double, labels, "Valid Solutions Comparison", "PRM Parameters (n_samples, radius)", "Number of Valid Solutions");
    // }
    
    // //Visualizer::showFigures();
    // Visualizer::saveFigures(true, "hw7_figs");

    // Grade method
    HW7::grade<MyPRM, MyRRT>("nicholas.niziolek@colorado.edu", argc, argv, std::make_tuple(1000,2), std::make_tuple());
    return 0;
}