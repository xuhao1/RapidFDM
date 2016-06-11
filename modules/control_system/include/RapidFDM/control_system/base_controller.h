//
// Created by Hao Xu on 16/6/11.
//

#ifndef RAPIDFDM_BASE_CONTROLLER_H
#define RAPIDFDM_BASE_CONTROLLER_H

#include <>

namespace RapidFDM
{
    namespace ControlSystem
    {
        class BaseController{
        public:

            BaseController();
            void control_step(float deltatime);
        };
    }
}

#endif //RAPIDFDM_BASE_CONTROLLER_H
