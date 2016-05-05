//
// Created by xuhao on 2016/5/4.
//


#ifndef RAPIDFDM_BASE_GEOMETRY_H
#define RAPIDFDM_BASE_GEOMETRY_H

#include <FlyingData.h>
#include <iostream>
#include <stdio.h>

namespace RapidFDM {
    namespace Aerodynamics {

        class BaseGeometry {
        protected:
            float Aero = 0;
            /*!< aera for aerodynamics calucation */
            std::string type = "base";
        public:
            BaseGeometry() { }

            std::string get_type() {
                return type;
            };
            //TODO:
            //Write codes
            virtual float getLift(ComponentData state, AirState airState) {
                std::cerr << "Code not wrote" << std::endl;
                abort();
                return 0;
            }

            virtual float getDrag(ComponentData state, AirState airState) {
                std::cerr << "Code not wrote" << std::endl;
                abort();
                return 0;
            }

            virtual float getSide(ComponentData state, AirState airState) {
                std::cerr << "Code not wrote" << std::endl;
                abort();
                return 0;
            }

            virtual Eigen::Vector3d getTorque(ComponentData state, AirState airState) {
                std::cerr << "Code not wrote" << std::endl;
                abort();
                return Eigen::Vector3d(0, 0, 0);
            }

            Eigen::Vector3d getForce(ComponentData state, AirState airState) {
                return Eigen::Vector3d(-getDrag(state, airState), -getSide(state, airState), -getLift(state, airState));
            }

            virtual void brief() {
                printf("Geometry %s\n", type.c_str());
            }
        };
    }

}

#endif //RAPIDFDM_BASE_GEOMETRY_H
