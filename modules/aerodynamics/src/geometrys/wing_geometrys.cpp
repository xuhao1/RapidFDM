//
// Created by Hao Xu on 16/7/14.
//


#include <RapidFDM/aerodynamics/geometrys/wing_geometrys.h>

namespace RapidFDM
{
    namespace Aerodynamics
    {
        float WingGeometry::getLift(ComponentData state, AirState airState)
        {
//                std::cerr << "Code not wrote" << std::endl;
//                abort();
            double velocity = state.get_airspeed_mag(airState);

            double cl = 2 * M_PI * state.get_angle_of_attack(airState);
            //Stall
            if (abs(state.get_angle_of_attack(airState) * 180 / M_PI) > 15)
                cl = 0;
            return cl * state.get_q_bar(airState) * this->aera;

        }

        float WingGeometry::getDrag(ComponentData state, AirState airState)
        {
//                std::cerr << "Code not wrote" << std::endl;
//                abort();
            return 0;
        }

        float WingGeometry::getSide(ComponentData state, AirState airState)
        {
//                std::cerr << "Code not wrote" << std::endl;
//                abort();
            return 0;
        }

        //!Build a wing
        /*flow KAR parameter
         * MODULE
            b_2 = 0.5               //distance from wing root to tip; semi-span
            MAC = 0.5               //Mean Aerodynamic Chord
            nonSideAttach = 0           //0 for canard-like / normal wing pieces, 1 for ctrlsurfaces attached to the back of other wing parts
            TaperRatio = 0.7            //Ratio of tip chord to root chord generally < 1, must be > 0
            MidChordSweep = 25          //Sweep angle in degrees; measured down the center of the span / midchord position
            maxdeflect = 15             //Default maximum deflection value; only used by FARControlableSurface
            controlSurfacePivot = 1, 0, 0;      //Local vector that obj_ctrlSrf pivots about; defaults to 1, 0, 0 (right)
            ctrlSurfFrac = 0.2          //Value from 0-1, percentage of the part that is a flap; only used by FARControlableSurface
            wing part: 0 control wing on negative part of y-axis, 1 on positive part, 3 both
        */
        WingGeometry::WingGeometry(const rapidjson::Value &v) :
                BaseGeometry(v)
        {
            params.b_2 = fast_value(v, "b_2", 0);
            params.Mac = fast_value(v, "Mac", 0);
            params.nonSideAttach = fast_value(v, "nonSideAttch", 1);
            params.TarperRatio = fast_value(v, "TaperRatio", 1);
            params.MidChordSweep = fast_value(v, "MidChordSweep", 0);
            params.maxdeflect = fast_value(v, "maxdeflect", 15);
            params.enableControl = fast_value(v, "enableControl", 0) == 1;
            if (params.enableControl)
                params.ctrlSurfFrac = fast_value(v, "ctrlSurfFrac", 0.2);
            params.wingPart = fast_value(v, "wingPart", 2);
            this->type = "WingGeometry";
            this->aera = params.b_2 * params.Mac;
            if (params.wingPart == 2) {
                this->aera = params.b_2 * params.Mac * 2;
            }
        }

        void WingGeometry::brief()
        {
            if (this->params.enableControl)
                printf("WindControlable\n");
            else
                printf("WingDiscontrolable\n");
            printf("b_2 %f\n", params.b_2);
            if (params.wingPart == 0)
                printf("wing on left side\n");
            else if (params.wingPart == 1)
                printf("Wing on right side\n");
            else
                printf("Wing on both side\n");

        }

        Eigen::Vector3d WingGeometry::get_aerodynamics_center()
        {
            return Eigen::Vector3d(params.Mac * 0.3,0,0);
        }
    }
}

