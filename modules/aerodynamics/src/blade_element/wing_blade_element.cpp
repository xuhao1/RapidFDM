//
// Created by Hao Xu on 16/7/18.
//

#include <RapidFDM/aerodynamics/blade_element/wing_blade_element.h>
#include <RapidFDM/aerodynamics/geometrys/Geometrys.h>
#include <RapidFDM/aerodynamics/nodes/wings/wing_node.h>


namespace RapidFDM
{
    namespace Aerodynamics
    {
        WingBladeElement::WingBladeElement(BaseGeometry *geo, double inner_span_percent, double outer_span_percent) :
                BaseBladeElement(geo)
        {
            assert(fabs(outer_span_percent) > fabs(inner_span_percent));
            assert(fabs(outer_span_percent) <= 1);
            if (fabs(1 - outer_span_percent) < 1e-3) {
                this->on_taper = true;
            }
            else {
                this->on_taper = false;
            }

            WingGeometry *wingGeometry = dynamic_cast<WingGeometry * >(geo);
            assert(wingGeometry != nullptr);

            this->deflectAngle = wingGeometry->params.deflectAngle;
            this->MidChordSweep = wingGeometry->params.MidChordSweep;

            mid_span_length =
                    (outer_span_percent + inner_span_percent) * wingGeometry->params.b_2 / 2 / cos(this->deflectAngle);
            float chord_center_position = 0.25 * wingGeometry->params.root_chord_length // root chord center offset part
                                          + tan(wingGeometry->params.MidChordSweep) *
                                            fabs(outer_span_percent + inner_span_percent) / 2 *
                                            wingGeometry->params.b_2; // sweep part
            Eigen::Vector3d relative_pos = Eigen::Vector3d(
                    chord_center_position,
                    mid_span_length,
                    fabs(outer_span_percent + inner_span_percent) / 2 * tan(this->deflectAngle) * wingGeometry->params.b_2
            );

            Eigen::Vector3d rotate_axis = Eigen::Vector3d(1, 0, 0);
            if (mid_span_length < 0) {
                rotate_axis = -rotate_axis;
            }

            Eigen::Quaterniond element_attitude(Eigen::AngleAxisd(deflectAngle, rotate_axis));
            relative_transform.fromPositionOrientationScale(relative_pos, element_attitude, Eigen::Vector3d(1, 1, 1));

            float span_ratio = fabs(outer_span_percent + inner_span_percent) / 2;
            this->Mac = (1 - span_ratio) * wingGeometry->params.root_chord_length +
                        span_ratio * wingGeometry->params.taper_chord_length;
            printf("name %s span ratio %f root chord %f taper %f\n",
                   this->geometry->getName().c_str(),
                   span_ratio,
                   wingGeometry->params.root_chord_length,
                   wingGeometry->params.taper_chord_length
            );

            this->element_span_length = fabs(outer_span_percent - inner_span_percent) * wingGeometry->params.b_2;
        }

        float WingBladeElement::getLift(ComponentData state, AirState airState) const
        {

            double cl = get_cl(state, airState);
            double lift = cl * state.get_q_bar(airState) * this->Mac * this->element_span_length;
            return lift;
        }

        float WingBladeElement::getDrag(ComponentData state, AirState airState) const
        {
            double cd = get_cd(state, airState);
            return cd * state.get_q_bar(airState) * this->Mac * this->element_span_length;
        }

        float WingBladeElement::getSide(ComponentData state, AirState airState) const
        {
            //TODO:
            //real side force
            return 0;
        }

        float WingBladeElement::get_cl(ComponentData state, AirState airState) const
        {
            WingGeometry *wingGeometry = dynamic_cast<WingGeometry * >(geometry);
            double x = state.get_angle_of_attack(airState);


            double cl_by_control = 0;
            if (get_wing_node()->enableControl)
            {
                float internal = 0;
                if(wingGeometry->params.wingPart == 2) {
                    if (this->mid_span_length < 0) {
                        //LEFT SIDE
                        internal = get_wing_node()->get_internal_states().find("flap_0")->second;
                    }
                    else
                    {
                        //Right Side
                        internal = get_wing_node()->get_internal_states().find("flap_1")->second;
                    }
                }
                else {
                    internal = get_wing_node()->get_internal_states().find("flap")->second;
                }
                cl_by_control = 0.1 * internal;
            }
            double cl = 5.27966 * x + 0.812763 * x * x - 5.66835 * x * x * x - 13.7039 * x * x * x * x;
            cl = cl / 4.302 + cl_by_control;
            //Stall
            if (x * 180 / M_PI > 20 || x * 180 / M_PI < -20) {
                cl = 0;
                if (this->geometry->getName() == "vertical_wing_0")
                {
                    printf("stalll!!!!!!\n");
                }

            }
            return cl;
        }

        float WingBladeElement::get_cd(ComponentData state, AirState airState) const
        {
            double alpha = state.get_angle_of_attack(airState);
            double cl = get_cl(state, airState);
            double cd = 0.0109392 + 0.494631 * alpha * alpha;
            cd = cd / 4.302 + cl * cl;
            return cd;
        }

        float WingBladeElement::get_cm(ComponentData state, AirState airState) const
        {
            return 0;
        }

        Eigen::Vector3d WingBladeElement::get_aerodynamics_torque(ComponentData data, AirState airState) const
        {
            return Eigen::Vector3d(0, 0, 0);
        }

        Eigen::Affine3d WingBladeElement::get_relative_transform() const
        {
            return relative_transform;
        }

        WingNode* WingBladeElement::get_wing_node() const
        {
            return dynamic_cast<WingNode*>(geometry->_parent);
        }
    }
}