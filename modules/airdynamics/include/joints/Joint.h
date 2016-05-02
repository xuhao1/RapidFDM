//
// Created by xuhao on 2016/5/2.
//

#ifndef RAPIDFDM_AERODYNAMIC_JOINT_H
#define RAPIDFDM_AERODYNAMIC_JOINT_H


namespace RapidFDM
{
    namespace Aerodynamics
    {
        class Node;
        class Joint
        {
        protected:
            Node * parent;
            Node * child;
        public:
            Node * getParent();
            Node * getChild();
        };
    }
}

#endif
