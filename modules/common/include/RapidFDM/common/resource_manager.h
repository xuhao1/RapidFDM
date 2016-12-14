//
// Created by xuhao on 2016/12/13.
//

#ifndef RAPIDFDM_RESOURCE_MANAGER_H
#define RAPIDFDM_RESOURCE_MANAGER_H

#include <boost/filesystem.hpp>
#include <iostream>

namespace RapidFDM {
    namespace Common {
        class resource_manager {
            boost::filesystem::path ExePath;
        public:
            resource_manager(char **argv) {
                ExePath = boost::filesystem::system_complete(argv[0]).parent_path();
                std::cout << "Program is running at" << ExePath << std::endl;
            }

            std::string exe_path() {
                return ExePath.string();
            }

        };

        void init_resource_manager(char** argv);

        std::string get_exe_path();

        std::string get_data_path();

        extern resource_manager * resourceManager;
    }
}

#endif //RAPIDFDM_RESOURCE_MANAGER_H
