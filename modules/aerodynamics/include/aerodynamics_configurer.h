//
// Created by Hao Xu on 16/6/11.
//

#ifndef RAPIDFDM_AERODYNAMICS_CONFIGURER_H
#define RAPIDFDM_AERODYNAMICS_CONFIGURER_H
#include <FlyingData.h>
#include <airdynamics_components.h>
#include <vector>
#include <string>
#include <rapidjson/document.h>
/*
 * configure_server
 * chroot folder
 * load a model
 * save a model
 * update model from json defines
 * transfer model datas
 */

namespace Aerodynamics{
    class aerodynamics_configurer
    {
    protected:
        std::string root_path;
    public:
        aerodynamics_configurer();

        //! Chroot to a folder
        std::vector<std::string> chroot_folder(std::string path);

        //!Load model from root
        void load_model(std::string name);

        //!Save model to root
        void save_model(std::string name);

        //!update model from json defs
        void update_model(rapidjson::Value & v);
    };

}

#endif //RAPIDFDM_AERODYNAMICS_CONFIGURER_H
