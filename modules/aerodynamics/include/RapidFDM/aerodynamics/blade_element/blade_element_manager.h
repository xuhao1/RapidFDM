//
// Created by Hao Xu on 16/7/18.
//

#ifndef RAPIDFDM_BLADE_MANAGER_H
#define RAPIDFDM_BLADE_MANAGER_H

#include <RapidFDM/aerodynamics/blade_element/base_blade_element.h>
#include <RapidFDM/aerodynamics/blade_element/blade_element_manager.h>
#include <vector>
#include <rapidjson/document.h>

namespace RapidFDM
{
    namespace Aerodynamics{
        class BladeElementManager {
            std::map < BaseBladeElement * ,AirState * > blade_elements;
        protected:
            Eigen::Vector3d forces;
            Eigen::Vector3d torques;
        public:
            BladeElementManager() {

            }
            void add_blade_element(BaseBladeElement * blade);

            void add_blade_elements(std::vector<BaseBladeElement *> blades);

            AirState get_blade_element_airstate(BaseBladeElement * element);

            //If deltatime < 0 ,then we will get stabilized result ...
            void calculate_washes(AirState airState,double delatime = -1);
            void calculate_forces_and_torques();
            
            Eigen::Vector3d get_total_force();
            Eigen::Vector3d get_total_torque();

            rapidjson::Document * get_blades_information();
        };
    }
}

#endif //RAPIDFDM_BLADE_MANAGER_H
