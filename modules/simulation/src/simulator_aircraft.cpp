//
// Created by Hao Xu on 16/6/11.
//

#include <RapidFDM/simulation/simulator_aircraft.h>
#include <RapidFDM/simulation/utils.h>
#include <PxPhysicsAPI.h>
#include <cassert>
#include <iostream>

using namespace RapidFDM::Simulation::Utils;

namespace RapidFDM
{
    namespace Simulation
    {
        void SimulatorAircraft::construct_rigid_dynamics_from_aircraft()
        {
            assert(aircraftNode != nullptr);
            PxRigidBody * actor = construct_rigid(aircraftNode);
            dfs_create_rigids(
                    aircraftNode,nodes,joints,actor
            );
        }

        PxRigidBody *SimulatorAircraft::construct_rigid(Aerodynamics::Node *node)
        {
            assert(root != nullptr);
            assert(pxScene != nullptr);
            assert(mPhysics != nullptr);
            PxTransform trans = transform_e2p(node->get_body_transform());
            //TODO:
            //Material params
            PxMaterial *aMaterial = mPhysics->createMaterial(0.01f, 0.01f, 0.5);
            Eigen::Vector3d boundingbox = node->get_bounding_box();
            PxRigidBody *actor = PxCreateDynamic(*mPhysics,
                                                 trans,
                                                 PxBoxGeometry(boundingbox.x(), boundingbox.y(), boundingbox.z()),
                                                 *aMaterial, 1);
            actor->setMass(node->get_mass());
            actor->setMassSpaceInertiaTensor(vector_e2p(node->get_inertial()));
            actor->setCMassLocalPose(
                    PxTransform(
                            vector_e2p(node->get_mass_center()),
                            PxQuat(0, 0, 0, 1)
                    )
            );
            actor->setLinearVelocity(PxVec3(0, 0, 0));
            pxScene->addActor(*actor);
            return actor;
        }

        PxJoint *SimulatorAircraft::construct_joint(Aerodynamics::Node *root, Aerodynamics::Joint *joint,
                                                    PxRigidBody *root_rigid, PxRigidBody *child_rigid)
        {
            assert(root != nullptr);
            assert(joint != nullptr);
            assert(root_rigid != nullptr);
            assert(root_rigid != nullptr);
            assert(child_rigid != nullptr);

            switch (joint->getType()) {
                case AerodynamicsFixedJoint: {
                    PxTransform root_joint_frame = transform_e2p(joint->get_base_transform());
                    PxTransform child_joint_frame = transform_e2p(joint->get_child_transform());
                    return PxFixedJointCreate(
                            *mPhysics,
                            root_rigid, root_joint_frame,
                            child_rigid, child_joint_frame
                    );
                }
                default: {
                    std::cerr << "Failed on joint type\n";
                    abort();
                    return nullptr;
                }
            }
            return nullptr;
        }

        void SimulatorAircraft::dfs_create_rigids(
                Aerodynamics::Node *root,
                std::vector<node_rigid *> &nodes,
                std::vector<joint_Joint *> &joints,
                PxRigidBody *root_rigid
        )
        {
            assert(root != nullptr);
            assert(pxScene != nullptr);
            assert(mPhysics != nullptr);

            printf("Scanning node %s\n", root->getName().c_str());
            for (Aerodynamics::Joint *joint : root->get_linked_joints()) {
                auto child = joint->getChild();
                printf("Scan for joint :%s with node %s",
                       joint->getName().c_str(),
                       child->getName().c_str()
                );
                PxRigidBody * actor = construct_rigid(child);
                PxJoint *pxJoint = construct_joint(root, joint, root_rigid, actor);
                nodes.push_back(new node_rigid(
                        actor, child
                ));
                //Construct Joint
                joints.push_back(new joint_Joint(
                        pxJoint,
                        joint
                ));
                //DFS
                dfs_create_rigids(child, nodes, joints, actor);

            }
        }
    }
}


