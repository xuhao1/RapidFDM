//
// Created by xuhao on 2016/5/2.
//

#include <airdynamics/include/nodes/Node.h>
#include "nodes/Node.h"
#include "joints/Joint.h"

namespace RapidFDM
{
    namespace Aerodynamics
    {
       Node::Node(Joint * _parent) {
           this->parent = _parent;
           if (_parent != nullptr)
           {
               this->states.attitude_ground = parent->getParent()->get_ground_attitude();
           }
       }
    }
}
