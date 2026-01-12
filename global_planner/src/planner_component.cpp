#include "planner_component.hpp"
#include "algorithms/astar.hpp"
#include <iostream>
#include <algorithm>

namespace global_planner {

unique_ptr<BaseAlgorithm> PlannerComponent::create(PlannerType type) {
    switch (type) {
        case PlannerType::ASTAR:
            cout << "[PlannerComponent] Creating AStar Algorithm..." << endl;
            return make_unique<AStar>();

        case PlannerType::RRTCONNECT:
            cout << "[PlannerComponent] Creating RRTConnect Algorithm..." << endl;
            return nullptr; // 暂时还没实现，先返回nullptr

        case PlannerType::DIJKSTRA:
            cout << "[PlannerComponent] Creating Dijkstra Algorithm..." << endl;
            return nullptr; // 暂时还没实现，先返回nullptr
        
        default:
            cerr << "[PlannerComponent] Error: Unknown Planner Type!" << endl;
            return nullptr;
    } 
}

PlannerType PlannerComponent::stringToType(const std::string& type_str) {
    std::string s = type_str;
    // 统一转大写，增强容错性
    std::transform(s.begin(), s.end(), s.begin(), ::toupper);

    if (s == "ASTAR" || s == "A_STAR") return PlannerType::ASTAR;
    if (s == "RRTCONNECT" || s == "RRT_CONNECT") return PlannerType::RRTCONNECT;
    if (s == "DIJKSTRA") return PlannerType::DIJKSTRA;

    return PlannerType::UNKNOWN;
}

} // namespace global_planner