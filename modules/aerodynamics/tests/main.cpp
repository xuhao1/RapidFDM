#include <RapidFDM/aerodynamics/nodes/Node.h>

#include <iostream>
#include "RapidFDM/aerodynamics/nodes/node_helper.h"
#include <RapidFDM/aerodynamics/joints/joint_helper.h>
#include <RapidFDM/aerodynamics/airdynamics_parser.h>

using namespace RapidFDM::Aerodynamics;

int main() {
    std::cout << "This is a simple test" << std::endl;
//    std::string data_root = "/Users/xuhao/Develop/FixedwingProj/RapidFDM/sample_data";
//    auto nodes = NodeHelper::scan_node_folder(data_root + "/nodes/");
//    for (auto k : nodes) {
//        k.second->brief();
//    }
//    auto joint = JointHelper::create_joint_from_file(
//            data_root + "/joints/fixed_joint.json", nodes);
//    joint->brief();
    parser parser1("/Users/xuhao/Develop/FixedwingProj/RapidFDM/sample_data/sample_aircraft");
    parser1.get_aircraft_node()->brief();
    std::cout << "Finish test " << std::endl;
}