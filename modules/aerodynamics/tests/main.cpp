#include <nodes/Node.h>

#include <iostream>
#include "nodes/node_helper.h"
#include <joints/joint_helper.h>
using namespace RapidFDM::Aerodynamics;

int main() {
    std::cout << "This is a simple test" << std::endl;
    auto nodes = NodeHelper::scan_node_folder("/cygdrive/c/Users/xuhao/Desktop/develop/RapidFDM/sample_data/nodes/");
    for (auto k : nodes) {
        k.second->brief();
    }
    auto joint = JointHelper::create_joint_from_file(
            "/cygdrive/c/Users/xuhao/Desktop/develop/RapidFDM/sample_data/joints/fixed_joint.json", nodes);
    joint->brief();
    std::cout << "Finish test " << std::endl;
}