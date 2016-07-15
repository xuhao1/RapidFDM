#include <RapidFDM/aerodynamics/nodes/BaseNode.h>

#include <iostream>
#include "RapidFDM/aerodynamics/nodes/node_helper.h"
#include <RapidFDM/aerodynamics/joints/joint_helper.h>
#include <RapidFDM/aerodynamics/airdynamics_parser.h>

using namespace RapidFDM::Aerodynamics;

int main() {
    std::cout << "This is a simple test" << std::endl;
    parser parser1("/Users/xuhao/Develop/FixedwingProj/RapidFDM/sample_data/sample_aircraft");
    parser1.get_aircraft_node()->brief();
    std::cout << "Finish test " << std::endl;
}