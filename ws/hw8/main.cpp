// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiAgentPlanners.h"
#include <fstream>
#include <iostream>
#include <filesystem>

using namespace amp;

void timer_example() {
    double startTime;
    amp::Timer timer("timer");
    for (int i=0; i < 5; ++i) {
        startTime = timer.now(TimeUnit::ms);  
        std::cout << "Press any key to continue...\n";
        std::cin.get();
        std::cout << "Time since last run: " << timer.now(TimeUnit::ms) - startTime << std::endl;
    }
    timer.stop();
    std::cout << "Total time since last run: " << Profiler::getTotalProfile("timer") << std::endl;
}

int main(int argc, char** argv) {
    // // Initializing workspace 1 with 3 agents
    // amp::RNG::seed(amp::RNG::randiUnbounded());
    // MultiAgentPath2D path;
    // std::vector<std::vector<Eigen::Vector2d>> collision_states;

    // // bool valid = false;
    // // while (!valid) {
    // //     // valid = HW8::generateAndCheck(central_planner,path,problem,collision_states);
    // //     path = central_planner.plan(problem);
    // //     valid = HW8::check(path, problem, collision_states,false);
    // // }
    // // Visualizer::makeFigure(problem, path, collision_states);
    // // valid = false;
    // // while (!valid) {
    // //     // valid = HW8::generateAndCheck(decentral_planner,path,problem,collision_states);
    // //     path = decentral_planner.plan(problem);
    // //     valid = HW8::check(path, problem, collision_states,false);
    // // }
    // // Visualizer::makeFigure(problem, path, collision_states);

    // // Benchmark the algorithm by sweeping over number of agents and running each 100 times
    // // Want to save to a csv tree size, compute time, number of valid solutions
    // std::list<std::vector<double>> num_agents_list;
    // std::list<std::vector<double>> tree_size_list;
    // std::list<std::vector<double>> compute_time_list;
    // std::vector<std::tuple<int, int, double>> results; // num_agents, tree_size, compute_time
    // amp::Timer timer("benchmark");
    // double start_time = timer.now(TimeUnit::ms);
    // for (size_t i = 1; i <= 6; ++i) {
    //     int valid_solutions = 0;
    //     std::vector<double> tree_size;
    //     std::vector<double> compute_time;
    //     for (size_t j = 0; j < 100; ++j) {
    //         MyCentralPlanner central_planner(0.05, 0.5, 7500, 0.25);
    //         MultiAgentProblem2D problem = HW8::getWorkspace1(i);
            
    //         // start timer
    //         start_time = timer.now(TimeUnit::ms);
    //         path = central_planner.plan(problem);
    //         // std::cout << "Number of agents in problem: " << i << ", Trial: " << j << ", Tree Size: " << central_planner.getTreeSize() << std::endl;
    //         // std::cout << "Number of agents in path: " << path.agent_paths.size() << std::endl;
    //         bool isValid = HW8::check(path, problem, collision_states,false);
    //         // stop timer
    //         timer.stop();
    //         // save results
    //         if (isValid) {
    //             results.push_back(std::make_tuple(i, central_planner.getTreeSize(), timer.now(TimeUnit::ms) - start_time));
    //             tree_size.push_back(central_planner.getTreeSize());
    //             compute_time.push_back(timer.now(TimeUnit::ms) - start_time);
    //             valid_solutions++;
    //         }
    //     }
    //     num_agents_list.push_back(std::vector<double>(valid_solutions, i));
    //     tree_size_list.push_back(tree_size);
    //     compute_time_list.push_back(compute_time);
    //     std::cout << "Number of agents: " << i << ", Valid Solutions: " << valid_solutions << "/100" << std::endl;
    // }
    // // Write results to csv
    // std::cout << "cwd: " << std::filesystem::current_path() << std::endl;
    // // std::ofstream file("benchmark_results.csv");
    // std::ofstream file("/workspaces/AMP_HW/AMP-Tools-public/file_dump/out/benchmark_results.csv");
    // file << "num_agents,tree_size,compute_time_ms\n";
    // for (const auto& result : results) {
    //     file << std::get<0>(result) << "," << std::get<1>(result) << "," << std::get<2>(result) << "\n";
    // }
    // file.close(); 

    // // Make results into box plots using Visualizer
    // Visualizer::makeBoxPlot(tree_size_list, {"1 Agent", "2 Agents", "3 Agents", "4 Agents", "5 Agents", "6 Agents"}, "Tree Size vs Number of Agents", "Number of Agents", "Tree Size");
    // Visualizer::makeBoxPlot(compute_time_list, {"1 Agent", "2 Agents", "3 Agents", "4 Agents", "5 Agents", "6 Agents"}, "Compute Time vs Number of Agents", "Number of Agents", "Compute Time (ms)");

    // // Compute averages
    // std::vector<double> avg_tree_sizes;
    // std::vector<double> avg_compute_times;
    
    // for (const auto& tree_size_vec : tree_size_list) {
    //     if (!tree_size_vec.empty()) {
    //         double sum = 0.0;
    //         for (double val : tree_size_vec) {
    //             sum += val;
    //         }
    //         avg_tree_sizes.push_back(sum / tree_size_vec.size());
    //     } else {
    //         avg_tree_sizes.push_back(0.0);  // Handle empty case
    //     }
    // }
    
    // for (const auto& compute_time_vec : compute_time_list) {
    //     if (!compute_time_vec.empty()) {
    //         double sum = 0.0;
    //         for (double val : compute_time_vec) {
    //             sum += val;
    //         }
    //         avg_compute_times.push_back(sum / compute_time_vec.size());
    //     } else {
    //         avg_compute_times.push_back(0.0);  // Handle empty case
    //     }
    // }
    // Visualizer::makeBarGraph(avg_tree_sizes, {"1 Agent", "2 Agents", "3 Agents", "4 Agents", "5 Agents", "6 Agents"},"Average Tree Size vs Number of Agents", "Number of Agents", "Average Tree Size");
    // Visualizer::makeBarGraph(avg_compute_times, {"1 Agent", "2 Agents", "3 Agents", "4 Agents", "5 Agents", "6 Agents"},"Average Compute Time vs Number of Agents", "Number of Agents", "Average Compute Time (ms)");

    // // Benchmark the decentralized algorithm the same as the previous
    // std::list<std::vector<double>> num_agents_list_dec;
    // std::list<std::vector<double>> tree_size_list_dec;
    // std::list<std::vector<double>> compute_time_list_dec;
    // std::vector<std::tuple<int, int, double>> results_dec; // num_agents, tree_size, compute_time
    // amp::Timer timer_dec("benchmark_dec");
    // double start_time_dec = timer_dec.now(TimeUnit::ms);
    // for (size_t i = 1; i <= 6; ++i) {
    //     int valid_solutions = 0;
    //     std::vector<double> tree_size;
    //     std::vector<double> compute_time;
    //     for (size_t j = 0; j < 100; ++j) {
    //         MyDecentralPlanner decentral_planner(0.05, 0.5, 7500, 0.25);
    //         MultiAgentProblem2D problem = HW8::getWorkspace1(i);
    //         // start timer
    //         start_time_dec = timer_dec.now(TimeUnit::ms);
    //         path = decentral_planner.plan(problem);
    //         // std::cout << "Number of agents in problem: " << i << ", Trial: " << j << ", Tree Size: " << decentral_planner.getTreeSize() << std::endl;
    //         // std::cout << "Number of agents in path: " << path.agent_paths.size() << std::endl;
    //         bool isValid = HW8::check(path, problem, collision_states,false);
    //         // stop timer
    //         timer_dec.stop();
    //         // save results
    //         if (isValid) {
    //             results_dec.push_back(std::make_tuple(i, decentral_planner.getTreeSize(), timer_dec.now(TimeUnit::ms) - start_time_dec));
    //             tree_size.push_back(decentral_planner.getTreeSize());
    //             compute_time.push_back(timer_dec.now(TimeUnit::ms) - start_time_dec);
    //             valid_solutions++;
    //         }
    //     }
    //     num_agents_list_dec.push_back(std::vector<double>(valid_solutions, i));
    //     tree_size_list_dec.push_back(tree_size);
    //     compute_time_list_dec.push_back(compute_time);
    //     std::cout << "Number of agents: " << i << ", Valid Solutions: " << valid_solutions << "/100" << std::endl;
    // }
    // // Write results to csv
    // std::cout << "cwd: " << std::filesystem::current_path() << std::endl;
    // // std::ofstream file_dec("benchmark_results_decentralized.csv");
    // std::ofstream file_dec("/workspaces/AMP_HW/AMP-Tools-public/file_dump/out/benchmark_results_decentralized.csv");
    // file_dec << "num_agents,tree_size,compute_time_ms\n";
    // for (const auto& result : results_dec) {
    //     file_dec << std::get<0>(result) << "," << std::get<1>(result) << "," << std::get<2>(result) << "\n";
    // }
    // file_dec.close();
    // // Make results into box plots using Visualizer
    // Visualizer::makeBoxPlot(tree_size_list_dec, {"1 Agent", "2 Agents", "3 Agents", "4 Agents", "5 Agents", "6 Agents"}, "Decentralized: Tree Size vs Number of Agents", "Number of Agents", "Tree Size");
    // Visualizer::makeBoxPlot(compute_time_list_dec, {"1 Agent", "2 Agents", "3 Agents", "4 Agents", "5 Agents", "6 Agents"}, "Decentralized: Compute Time vs Number of Agents", "Number of Agents", "Compute Time (ms)");
    // // Compute averages
    // std::vector<double> avg_tree_sizes_dec;
    // std::vector<double> avg_compute_times_dec;
    // for (const auto& tree_size_vec : tree_size_list_dec) {
    //     if (!tree_size_vec.empty()) {
    //         double sum = 0.0;
    //         for (double val : tree_size_vec) {
    //             sum += val;
    //         }
    //         avg_tree_sizes_dec.push_back(sum / tree_size_vec.size());
    //     } else {
    //         avg_tree_sizes_dec.push_back(0.0);  // Handle empty case
    //     }
    // }
    // for (const auto& compute_time_vec : compute_time_list_dec) {
    //     if (!compute_time_vec.empty()) {
    //         double sum = 0.0;
    //         for (double val : compute_time_vec) {
    //             sum += val;
    //         }
    //         avg_compute_times_dec.push_back(sum / compute_time_vec.size());
    //     } else {
    //         avg_compute_times_dec.push_back(0.0);  // Handle empty case
    //     }
    // }
    // Visualizer::makeBarGraph(avg_tree_sizes_dec, {"1 Agent", "2 Agents", "3 Agents", "4 Agents", "5 Agents", "6 Agents"},"Decentralized: Average Tree Size vs Number of Agents", "Number of Agents", "Average Tree Size");
    // Visualizer::makeBarGraph(avg_compute_times_dec, {"1 Agent", "2 Agents", "3 Agents", "4 Agents", "5 Agents", "6 Agents"},"Decentralized: Average Compute Time vs Number of Agents", "Number of Agents", "Average Compute Time (ms)");

    // // Visualize and grade methods
    // Visualizer::saveFigures(false, "hw8_figs");
    HW8::grade<MyCentralPlanner, MyDecentralPlanner>("nicholas.niziolek@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}