#include <joints/FreeJoint.h>
#include <nodes/Node.h>
#include <iostream>
#include "nodes/node_helper.h"

using namespace RapidFDM::Aerodynamics;

int main() {
    std::cout << "This is a simple test" << std::endl;
    Node *sample_node = NodeHelper::create_node_from_file(
            "C:\\Users\\xuhao\\Desktop\\develop\\RapidFDM/sample_data/nodes/box_fuselage.json");
    sample_node->brief();
    std::cout << "Finish test " << std::endl;
}