//
// Created by Hao Xu on 16/7/18.
//


#include <RapidFDM/aerodynamics/blade_element/blade_element_manager.h>

namespace RapidFDM
{
    namespace Aerodynamics{
        void BladeElementManager::calculate_washes(AirState airState)
        {
            //TODO:
            for (auto pair : blade_elements)
            {
                pair.second->rho = airState.rho;
                pair.second->ground_air_speed = airState.ground_air_speed;
            }
        }

        void BladeElementManager::add_blade_element(BaseBladeElement * blade)
        {
            blade_elements[blade]  = new AirState;
            blade_elements[blade]->ground_air_speed;
        }

        AirState BladeElementManager::get_blade_element_airstate(BaseBladeElement *element)
        {
            return *blade_elements[element];
        }
        void BladeElementManager::add_blade_elements(std::vector<BaseBladeElement *> blades)
        {
            for (auto blade : blades)
            {
                add_blade_element(blade);
            }
        }
    }
}