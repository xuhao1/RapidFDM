#ifndef __RAPIDFDM_AERODYNAMIC_BASE_COMPONENT_H__
#define __RAPIDFDM_AERODYNAMIC_BASE_COMPONENT_H__

#include <Eigen/Eigen>
#include <iostream>
#include <RapidFDM/utils.h>
#include <stdlib.h>
#include <stdio.h>
#include <map>
#include <string>
#include <rapidjson/document.h>
#include "flying_data_defines.h"

using namespace RapidFDM::Utils;

namespace RapidFDM
{
    namespace Aerodynamics
    {
        class BaseMoveableComponent
        {
        public:
            virtual Eigen::Vector3d get_ground_velocity() const
            {
                std::abort();
            }

            virtual Eigen::Vector3d get_angular_velocity() const
            {
                std::abort();
            }

            virtual Eigen::Affine3d get_ground_transform() const
            {
                std::abort();
            }


            virtual Eigen::Quaterniond get_ground_attitude() const
            {
                std::abort();
            }


            virtual Eigen::Affine3d get_body_transform() const
            {
                std::abort();
            }


        };

        class BaseAerodynamicsComponent
        {
        protected:
            ComponentData flying_states;
        public:
            virtual double getLift(ComponentData state, AirState airState) const
            {
                return 0;
            }

            virtual double getDrag(ComponentData state, AirState airState) const
            {
                return 0;
            }

            virtual double getSide(ComponentData state, AirState airState) const
            {
                return 0;
            }

            /*!
              \return The calucated realtime aerodynamics force
            */
            virtual Eigen::Vector3d get_aerodynamics_force(ComponentData data,AirState airState) const;

            //! Calucate aerodynamics torque of this node
            /*!
              \return The calucated realtime aerodynamics torque
            */
            virtual Eigen::Vector3d get_aerodynamics_torque(ComponentData data,AirState airState) const
            {
                return Eigen::Vector3d(0, 0, 0);
            }


        };

        class BaseControllableComponent
        {
        protected:
            std::map<std::string, double> internal_states;
            std::map<std::string, double> control_axis;
            float freq_cut = 3;
        public:
            virtual std::map<std::string, double> get_internal_states() const
            {
                return this->internal_states;
            };

            virtual std::vector<std::string> get_control_frame() const
            {
                std::vector<std::string> res;
                for (auto s : control_axis) {
                    res.push_back(s.first);
                }
                return res;
            }

            virtual std::map<std::string, double> get_control_axis() const
            {
                return control_axis;
            };

            virtual int set_control_value(std::string name, double v)
            {
                if (this->control_axis.find(name) != this->control_axis.end()) {
                    this->control_axis[name] = v;
                    return 0;
                }
                return -1;
            }

            virtual int set_internal_state(std::string name, double v)
            {
                if (this->internal_states.find(name) != this->internal_states.end()) {
                    this->internal_states[name] = v;
                    return 0;
                }
                return -1;
            }

            virtual void iter_internal_state(double deltatime)
            {
                for (auto pair : this->control_axis) {
                    this->internal_states[pair.first] = lowpass_filter(this->control_axis[pair.first], this->freq_cut,
                                                                   this->internal_states[pair.first], deltatime);
                }
            }

        };

        class BaseComponent : public BaseMoveableComponent , public BaseAerodynamicsComponent, public BaseControllableComponent
        {
        protected:
            std::string name;
            std::string unique_id;

            double time = 0;
//            std::vector<std::string> control_frame;
            rapidjson::Document source_document;
        public:
            BaseComponent()
            {
            }

            BaseComponent(const rapidjson::Value &v)
            {
                assert(v.IsObject());
                this->name = fast_string(v, "name");
                std::string _id = fast_string(v, "id");
                if (_id == "") {
                    char buffer[9] = {0};
                    sprintf(buffer, "%d", rand() % 100000000);
                    _id = std::string(buffer);
                }

                this->unique_id = this->name + "_" + _id;
                this->source_document.CopyFrom(v, source_document.GetAllocator());
            }

            virtual double get_internal_state(std::string name) const
            {
                return this->get_internal_states().find(getUniqueID()+"/" + name)->second;
            }


            double get_control_value(std::string name) const
            {
                return this->get_control_axis().find(getUniqueID()+"/" + name)->second;
            }

            virtual void brief()
            {
            }


            std::string getName() const
            {
                return this->name;
            }

            std::string getUniqueID() const
            {
                return this->unique_id;
            }

        public:
            virtual const rapidjson::Value &getJsonDefine()
            {
                return this->source_document;
            }

        };
    }
}

#endif