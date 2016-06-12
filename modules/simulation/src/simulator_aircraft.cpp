//
// Created by Hao Xu on 16/6/11.
//

#include <RapidFDM/simulation/simulator_aircraft.h>


namespace RapidFDM
{
    namespace Simulation
    {
        void SimulatorAircraft::construct_rigid_dynamics_from_aircraft(PxScene *pxScene)
        {

        }

        void SimulatorAircraft::dfs_create_rigids(
                Aerodynamics::Node *root,
                std::vector<node_rigid *> &nodes,
                std::vector<joint_Joint *> &joints)
        {
            assert(root != nullptr);
            printf("Scanning node %s\n", root->getName().c_str());
            for (Aerodynamics::Joint *joint : root->get_linked_joints()) {
                auto child = joint->getChild();
                printf("Scan for joint :%s with node %s",
                       joint->getName().c_str(),
                       child->getName().c_str()
                );
            }
        }
    }
}


