#ifndef __RAPIDFDM_AERODYNAMIC_NODE_H__
#define __RAPIDFDM_AERODYNAMIC_NODE_H__
#include <RapidFDM/aerodynamics/nodes/Node.h>
#include <RapidFDM/aerodynamics/FlyingData.h>
#include <rapidjson/document.h>
#include <RapidFDM/aerodynamics/geometrys/geometry_helper.h>
#include <stdio.h>

namespace RapidFDM {
    namespace Aerodynamics {
        //! This node stand for the base of a aircraft
        class AircraftNode : public Node {
        public:
            AircraftNode(rapidjson::Value &_json, rapidjson::Document &document) :
                    Node(_json, document) {
                assert(_json.IsObject());
                if (document.HasMember("geometry") && document["geometry"].IsObject()) {
                    this->geometry = GeometryHelper::create_geometry_from_json(document["geometry"]);
                }
                else {
                    this->geometry = new BaseGeometry();
                }
                this->type_str = "aircraft_node";
                printf("Success parse aircraft_node\n");
                printf("Name %s type: %s geometry %s\n", this->name.c_str(), this->type_str.c_str(),
                       geometry->get_type().c_str());

            }

            AircraftNode(rapidjson::Document &document) :
                    Node(document) {
                assert(document.IsObject());

                if (document.HasMember("geometry") && document["geometry"].IsObject()) {
                    this->geometry = GeometryHelper::create_geometry_from_json(document["geometry"]);
                }
                else {
                    this->geometry = new BaseGeometry();
                }

                this->type_str = "aircraft_node";
                printf("Success parse aircraft_node\n");
                printf("Name %s type: %s geometry %s\n", this->name.c_str(), this->type_str.c_str(),
                       geometry->get_type().c_str());

            }

            AircraftNode():
                    Node()
            {
                this->type_str = "aircraft_node";
            }

            virtual Eigen::Affine3d get_body_transform() override
            {
                return Eigen::Affine3d::Identity();
            }

            virtual Eigen::Vector3d get_total_force()
            {

            }
            virtual Eigen::Vector3d get_total_aerodynamics_force()
            {

            }
            virtual Eigen::Vector3d get_total_torque()
            {

            }
            virtual Eigen::Vector3d get_total_aerodynamics_torque()
            {

            }

            virtual Node * instance() override
            {
                AircraftNode * node = new AircraftNode;
                memcpy(node,this, sizeof(AircraftNode));
                node->geometry = this->geometry->instance();
                return node;
            }
        };
    }
}


#endif