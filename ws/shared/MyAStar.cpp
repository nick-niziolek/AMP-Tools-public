#include "MyAStar.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>

MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    struct NodeCost {
        amp::Node node;
        double cost;
        
        bool operator>(const NodeCost& other) const {
            return cost > other.cost;
        }
    };

    // std::cout << "Starting A* Graph Search: Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;
    GraphSearchResult result = {false, {}, 0.0};

    // Use min-heap priority queue
    std::priority_queue<NodeCost, std::vector<NodeCost>, std::greater<NodeCost>> open_list;
    
    // Use hash sets for O(1) lookups
    std::unordered_set<amp::Node> closed_set;
    std::unordered_set<amp::Node> open_set;
    
    // Maps for costs and backpointers
    std::unordered_map<amp::Node, amp::Node> backpointers;
    std::unordered_map<amp::Node, double> g_costs;
    std::unordered_map<amp::Node, double> f_costs;

    // Initialize
    g_costs[problem.init_node] = 0.0;
    //double init_f = heuristic(problem.init_node);
    double init_f = 0;
    f_costs[problem.init_node] = init_f;
    open_list.push({problem.init_node, init_f});
    open_set.insert(problem.init_node);

    int iteration = 0;
    while (!open_list.empty()) {
        iteration++;
        
        // Get node with lowest f-cost
        NodeCost current = open_list.top();
        open_list.pop();
        open_set.erase(current.node);
        
        // Skip if we've already processed this node with a better cost
        if (closed_set.count(current.node) || 
           (f_costs.count(current.node) && current.cost > f_costs[current.node])) {
            continue;
        }
        
        // Add to closed set
        closed_set.insert(current.node);

        // Check if goal reached
        if (current.node == problem.goal_node) {
            result.success = true;
            break;
        }

        // Process neighbors
        std::vector<amp::Node> neighbors = problem.graph->children(current.node);
        std::vector<double> outgoing_edges = problem.graph->outgoingEdges(current.node);

        for (size_t i = 0; i < neighbors.size(); i++) {
            amp::Node neighbor = neighbors[i];
            double edge_cost = outgoing_edges[i];

            // Skip if already processed
            if (closed_set.count(neighbor)) {
                continue;
            }

            double tentative_g = g_costs[current.node] + edge_cost;
            
            // If this is a better path to neighbor
            if (!g_costs.count(neighbor) || tentative_g < g_costs[neighbor]) {
                backpointers[neighbor] = current.node;
                g_costs[neighbor] = tentative_g;
                double f_cost = tentative_g;// + heuristic(neighbor);
                f_costs[neighbor] = f_cost;
                
                // Add to open list if not already there
                if (!open_set.count(neighbor)) {
                    open_list.push({neighbor, f_cost});
                    open_set.insert(neighbor);
                } else {
                    // Re-add with updated cost (old entries will be skipped)
                    open_list.push({neighbor, f_cost});
                }
            }
        }
    }

    // Reconstruct path
    if (result.success) {
        //std::cout << "Path found in " << iteration << " iterations." << std::endl;
        amp::Node current = problem.goal_node;
        std::list<amp::Node> path;
        while (current != problem.init_node) {
            path.push_front(current);  // Build path in correct order
            current = backpointers[current];
        }
        path.push_front(problem.init_node);
        result.node_path = path;
        result.path_cost = g_costs[problem.goal_node];
    } else {
        //std::cout << "No path found from " << problem.init_node << " to " << problem.goal_node << std::endl;
    }

    // result.print();
    return result;
}