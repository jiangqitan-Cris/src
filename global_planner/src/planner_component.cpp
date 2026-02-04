#include "planner_component.hpp"
#include "algorithms/astar.hpp"
#include "algorithms/rrt_connect.hpp"
#include <iostream>
#include <algorithm>

namespace global_planner {

std::unique_ptr<BaseAlgorithm> PlannerComponent::create(PlannerType type) {
    switch (type) {
        case PlannerType::ASTAR:
            std::cout << "[PlannerComponent] Creating AStar Algorithm..." << std::endl;
            return std::make_unique<AStar>();

        case PlannerType::RRTCONNECT:
            std::cout << "[PlannerComponent] Creating RRTConnect Algorithm..." << std::endl;
            return std::make_unique<RRTConnect>();

        case PlannerType::DIJKSTRA:
            std::cout << "[PlannerComponent] Creating Dijkstra Algorithm..." << std::endl;
            return nullptr; // 暂时还没实现
        
        default:
            std::cerr << "[PlannerComponent] Error: Unknown Planner Type!" << std::endl;
            return nullptr;
    } 
}

PlannerType PlannerComponent::stringToType(const std::string& type_str) {
    std::string s = type_str;
    std::transform(s.begin(), s.end(), s.begin(), ::toupper);

    if (s == "ASTAR" || s == "A_STAR") return PlannerType::ASTAR;
    if (s == "RRTCONNECT" || s == "RRT_CONNECT") return PlannerType::RRTCONNECT;
    if (s == "DIJKSTRA") return PlannerType::DIJKSTRA;

    return PlannerType::UNKNOWN;
}

void PlannerComponent::printSupportList() const {
    std::cout << "Supported planners:" << std::endl;
    std::cout << "  - ASTAR" << std::endl;
    std::cout << "  - RRTCONNECT" << std::endl;
    std::cout << "  - DIJKSTRA (not implemented)" << std::endl;
}

} // namespace global_planner
