//
// Created by Hao Xu on 16/6/11.
//

#include <RapidFDM/simulation/simulator_aircraft.h>
#include <RapidFDM/simulation/utils.h>
#include <RapidFDM/simulation/simulator_world.h>

#ifndef NDEBUG
#define NDEBUG
#endif

#include <PxPhysicsAPI.h>

#undef NDEBUG

#include <iostream>

using namespace RapidFDM::Simulation::Utils;

namespace RapidFDM
{
    namespace Simulation
    {
        SimulatorAircraft::SimulatorAircraft(
                Aerodynamics::AircraftNode *_aircraftNode,
                ControlSystem::BaseController *_baseController,
                SimulatorWorld *_simulator,
                PxTransform init_trans,
                double init_speed
        ) :
                aircraftNode(_aircraftNode), baseController(_baseController)
        {

            pxScene = _simulator->pxScene;
            mPhysics = _simulator->mPhysics;

            assert(pxScene != nullptr);
            assert(mPhysics != nullptr);
            assert(aircraftNode != nullptr);
            assert(baseController != nullptr);
            printf("Construct simulator aircraft %s \n", aircraftNode->getName().c_str());
            construct_rigid_dynamics_from_aircraft();
            printf("Construct simulator success %s \n", aircraftNode->getName().c_str());
            reset_aircraft(init_trans, init_speed);
        }

        void SimulatorAircraft::construct_rigid_dynamics_from_aircraft()
        {
            if (aircraftNode->is_rigid()) {
                //Create rigidbody aircraft
                nodes[aircraftNode] = construct_rigid_aircraft();
            }
            else {
                assert(true);
                //Create a multi rigidbody aircraft
                PxRigidBody *actor = construct_rigid(aircraftNode);
                assert(actor != nullptr);
                nodes[aircraftNode] = actor;
                dfs_create_rigids(
                        aircraftNode, nodes, joints, actor
                );
            }
        }

        PxRigidBody *SimulatorAircraft::construct_rigid_aircraft()
        {
            assert(aircraftNode != nullptr);
            printf("Construct rigidbody by aircraft %s \n", aircraftNode->getUniqueID().c_str());

            //TODO:
            //add gyroscope

            PxMaterial *aMaterial = mPhysics->createMaterial(0.01f, 0.01f, 0.5);
            assert(aMaterial != nullptr);
            Eigen::Vector3d boundingbox = aircraftNode->get_bounding_box();
            auto init_trans = PxTransform::createIdentity();
            init_trans.p.z = 10;
            printf("init trans pos %f %f %f\n",init_trans.p.x,init_trans.p.y,init_trans.p.z);
            PxRigidBody *actor = PxCreateDynamic(
                    *mPhysics,
                    init_trans,
                    PxBoxGeometry(boundingbox.x(), boundingbox.y(), boundingbox.z()),
                    *aMaterial, 1
            );
            assert(actor != nullptr);
            actor->setMass(aircraftNode->get_total_mass());
            actor->setMassSpaceInertiaTensor(vector_e2p(aircraftNode->get_total_inertial()));
            pxScene->addActor(*actor);
            actor->setLinearVelocity(PxVec3(0,0,0));
            //Debug !!!!

            aircraftNode->setSimulate(true);
            return actor;
        }

        PxRigidBody *SimulatorAircraft::construct_rigid(Aerodynamics::BaseNode *node)
        {
            assert(node != nullptr);
            printf("Construct rigidbody by node: %s\n", node->getName().c_str());
            PxTransform trans = transform_e2p(node->get_body_transform());
            //TODO:
            //Material params
            PxMaterial *aMaterial = mPhysics->createMaterial(0.01f, 0.0f, 0.5);
            assert(aMaterial != nullptr);
            Eigen::Vector3d boundingbox = node->get_bounding_box();
            PxRigidBody *actor = PxCreateDynamic(*mPhysics,
                                                 trans,
                                                 PxBoxGeometry(boundingbox.x(), boundingbox.y(), boundingbox.z()),
                                                 *aMaterial, 1);
            assert(actor != nullptr);
            actor->setMass(node->get_mass());
            actor->setMassSpaceInertiaTensor(vector_e2p(node->get_inertial()));
            pxScene->addActor(*actor);
            node->setSimulate(true);
            return actor;
        }

        PxJoint *SimulatorAircraft::construct_joint(Aerodynamics::BaseNode *root, Aerodynamics::BaseJoint *joint,
                                                    PxRigidBody *root_rigid, PxRigidBody *child_rigid)
        {
            assert(root != nullptr);
            assert(joint != nullptr);
            assert(root_rigid != nullptr);
            assert(root_rigid != nullptr);
            assert(child_rigid != nullptr);

            printf("Construct joint by joint: %s type %d \n",
                   joint->getName().c_str(),
                   joint->getType()
            );
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
                Aerodynamics::BaseNode *root,
                std::map<BaseNode *, PxRigidBody *> &nodes,
                std::map<BaseJoint *, PxJoint *> &joints,
                PxRigidBody *root_rigid
        )
        {
            assert(root != nullptr);

            printf("Scanning node %s\n", root->getName().c_str());
            for (Aerodynamics::BaseJoint *joint : root->get_linked_joints()) {
                auto child = joint->getChild();
                printf("Scan for joint :%s with node %s\n",
                       joint->getName().c_str(),
                       child->getName().c_str()
                );
                PxRigidBody *actor = construct_rigid(child);
                assert(actor != nullptr);
                PxJoint *pxJoint = construct_joint(root, joint, root_rigid, actor);
                assert(pxJoint != nullptr);
                nodes[child] = actor;
                joints[joint] = pxJoint;
                //DFS
                dfs_create_rigids(child, nodes, joints, actor);

            }
        }

        void SimulatorAircraft::fetch_forces_torques_from_aerodynamics()
        {
            assert(aircraftNode->is_rigid());
            PxRigidBody *rigidBody = nodes[aircraftNode];
            Eigen::Vector3d total_force = aircraftNode->get_total_force(count); //body coordinate
            total_force = aircraftNode->get_ground_transform().linear() * total_force;
            assert(this->aircraftNode->is_rigid());
                   
            Eigen::Vector3d total_torque = aircraftNode->get_total_torque(count);//body
            total_torque = aircraftNode->get_ground_transform().linear() * total_torque;
            rigidBody->addForce(vector_e2p(total_force));
            gAcc = total_force / aircraftNode->get_total_mass();
            rigidBody->addTorque(vector_e2p(total_torque));
        }

        void SimulatorAircraft::reset_aircraft(PxTransform init_trans, double init_speed)
        {
            assert(aircraftNode->is_rigid());
            PxVec3 init_speed_vec = PxVec3(-init_speed, 0, 0);
            init_speed_vec = init_trans.rotate(init_speed_vec);
            nodes[aircraftNode]->setGlobalPose(init_trans);
            nodes[aircraftNode]->setLinearVelocity(init_speed_vec);
            nodes[aircraftNode]->setAngularVelocity(PxVec3(0,0,0));
        }

        void SimulatorAircraft::update_states_from_physx()
        {
            if (aircraftNode->is_rigid()) {
                Eigen::Affine3d root_transform = transform_p2e(nodes[aircraftNode]->getGlobalPose());
                Aerodynamics::ComponentData data;

                assert(nodes.find(aircraftNode) != nodes.end());

                PxRigidBody *rigidBody = nodes[aircraftNode];
                data.transform = root_transform;
                data.angular_velocity = aircraftNode->get_ground_transform().linear().inverse() *
                                        vector_p2e(rigidBody->getAngularVelocity());
                data.ground_velocity = vector_p2e(rigidBody->getLinearVelocity());
                data.body_transform = aircraftNode->get_body_transform();
                aircraftNode->setStatefromsimulator(data,count);

            }
            else {
                Eigen::Affine3d root_transform = transform_p2e(nodes[aircraftNode]->getGlobalPose());
                for (auto pair : nodes) {
                    Aerodynamics::ComponentData data;
                    Aerodynamics::BaseNode *node = pair.first;
                    PxRigidBody *rigidBody = pair.second;
                    data.transform = transform_p2e(rigidBody->getGlobalPose());

                    //TODO:
                    //Confirm coordinate system
                    data.angular_velocity = vector_p2e(rigidBody->getAngularVelocity());
                    data.ground_velocity = vector_p2e(rigidBody->getLinearVelocity());

                    data.body_transform = root_transform.inverse() * data.transform;

                    node->setStatefromsimulator(data);
                }
            }
        }
    }
}


