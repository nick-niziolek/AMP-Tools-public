// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW9.h"
#include "hw/HW2.h"
#include "MyKinoRRT.h"

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

// Load problems and map agent for quick testing
std::vector<KinodynamicProblem2D> problems = {HW9::getStateIntProblemWS1(), HW9::getStateIntProblemWS2(), HW9::getFOUniProblemWS1(), HW9::getFOUniProblemWS2(), HW9::getSOUniProblemWS1(), HW9::getSOUniProblemWS2(), HW9::getCarProblemWS1(), HW9::getParkingProblem()};
std::unordered_map<AgentType, std::function<std::shared_ptr<amp::DynamicAgent>()>> agentFactory = {
    {AgentType::SingleIntegrator, []() { return std::make_shared<MySingleIntegrator>(); }},
    {AgentType::FirstOrderUnicycle, []() { return std::make_shared<MyFirstOrderUnicycle>(); }},
    {AgentType::SecondOrderUnicycle, []() { return std::make_shared<MySecondOrderUnicycle>(); }},
    {AgentType::SimpleCar, []() { return std::make_shared<MySimpleCar>(); }}
};

int main(int argc, char** argv) {
    // Select problem, plan, check, and visualize
    int select = 7;
    KinodynamicProblem2D prob = problems[select];
    std::list<std::vector<double>> planning_times;
    std::list<std::vector<double>> path_lengths;
    std::list<int> valid_solutions;
    std::vector<std::tuple<int, int, double, double>> results; // u_samples, valid_solutions, planning_time, path_length

    std::vector<int> u_samples_list = {1,5,10,15};
    for (size_t j = 0; j < u_samples_list.size(); j++) {
        int valid_count = 0;
        std::vector<double> times;
        std::vector<double> lengths;
        for (size_t i = 0; i < 1; i ++) {
            MyKinoRRT kino_planner(200000, u_samples_list[j], 0.05, 0.25);
            // Time the planning
            amp::Timer timer("KinoRRT_Planning");
            double start_time = timer.now(TimeUnit::ms);
            KinoPath path = kino_planner.plan(prob, *agentFactory[prob.agent_type]());
            double end_time = timer.now(TimeUnit::ms);
            HW9::check(path, prob);
            if (path.valid) {
                valid_count++;
                times.push_back(end_time - start_time);
                lengths.push_back(kino_planner.getPathLength(path));
            }
            Visualizer::makeFigure(prob, path, false);
        }
        valid_solutions.push_back(valid_count);
        planning_times.push_back(times);
        path_lengths.push_back(lengths);
    }

    Visualizer::saveFigures(true, "hw9_figs");

    // Save results to .csv
    // // Write results to csv
    // std::cout << "cwd: " << std::filesystem::current_path() << std::endl;
    // // std::ofstream file_dec("benchmark_results_decentralized.csv");
    // std::ofstream file_dec("/workspaces/AMP_HW/AMP-Tools-public/file_dump/out/benchmark_results_decentralized.csv");
    // file_dec << "num_agents,tree_size,compute_time_ms\n";
    // for (const auto& result : results_dec) {
    //     file_dec << std::get<0>(result) << "," << std::get<1>(result) << "," << std::get<2>(result) << "\n";
    // }
    // file_dec.close();

    // Write to csv
    std::string filepath = std::string("/workspaces/AMP_HW/AMP-Tools-public/file_dump/out/hw9_kinorrt_results_") + std::to_string(select) + ".csv";
    std::ofstream file(filepath);
    file << "u_samples,valid_solutions,planning_time_ms,path_length\n";
    for (const auto& result : results) {
        file << std::get<0>(result) << "," << std::get<1>(result) << "," << std::get<2>(result) << "," << std::get<3>(result) << "\n";
    }

    //HW9::grade<MyKinoRRT, MySingleIntegrator, MyFirstOrderUnicycle, MySecondOrderUnicycle, MySimpleCar>("firstName.lastName@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple());
    return 0;
}