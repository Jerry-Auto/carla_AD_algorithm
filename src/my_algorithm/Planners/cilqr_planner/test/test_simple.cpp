#include <iostream>
#include "cilqr_planner/cilqr_planner.h"

int main() {
    std::cout << "Creating cilqr planner..." << std::endl;
    AD_algorithm::planner::cilqrPlanner planner;
    std::cout << "Planner created successfully!" << std::endl;
    return 0;
}
