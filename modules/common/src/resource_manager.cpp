//
// Created by xuhao on 2016/12/13.
//

#include <RapidFDM/common/resource_manager.h>

namespace RapidFDM {
    namespace Common {
        resource_manager * resourceManager = nullptr;
        void init_resource_manager(char** argv)
        {
            resourceManager = new resource_manager(argv);
            return;
        }

        std::string get_exe_path()
        {
            if(resourceManager == nullptr)
            {
                std::cerr << "Resouce Manager not inited, please check!!!" <<std::endl;
                std::abort();
            }

            return resourceManager->exe_path();
        }

        std::string get_data_path()
        {
            if(resourceManager == nullptr)
            {
                std::cerr << "Resouce Manager not inited, please check!!!" <<std::endl;
                std::abort();
            }

#ifdef __WIN32__
            return resourceManager->exe_path() + "../data";
#else
            return resourceManager->exe_path() + "/data";
#endif
        }
    }
}

