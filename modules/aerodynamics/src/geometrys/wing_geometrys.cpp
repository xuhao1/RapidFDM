//
// Created by Hao Xu on 16/7/14.
//


#include <RapidFDM/aerodynamics/geometrys/wing_geometrys.h>
#include <RapidFDM/aerodynamics/geometrys/Geometrys.h>

namespace RapidFDM
{
    namespace Aerodynamics
    {
        float WingGeometry::getLift(ComponentData state, AirState airState) const
        {
//                std::cerr << "Code not wrote" << std::endl;
//                abort();
            double velocity = state.get_airspeed_mag(airState);
            double x = state.get_angle_of_attack(airState);
            double cl = 0.25 + 5.27966 * x + 0.812763 * x * x - 5.66835  * x * x * x - 13.7039 * x * x * x  *x;
//            double cl = 2 * M_PI * x;
            cl = cl /4.302;
            //Stall
            if (x * 180 / M_PI > 15 || x * 180 / M_PI < -15)
                cl = 0;

            return cl * state.get_q_bar(airState) * this->aera;

        }

        float WingGeometry::getDrag(ComponentData state, AirState airState) const
        {
            double alpha = state.get_angle_of_attack(airState);
            double x = alpha;

            double cl = 0.25 + 5.27966 * x + 0.812763 * x * x - 5.66835  * x * x * x - 13.7039 * x * x * x  *x;
            cl = cl /4.302;

            double cd = 0.0109392 + 0.494631 * alpha * alpha + 0.04 * cl * cl;
            cd = cd /4.302;
            return cd * state.get_q_bar(airState) * this->aera;
        }

        float WingGeometry::getSide(ComponentData state, AirState airState) const
        {
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
            params.MidChordSweep = fast_value(v, "MidChordSweep", 0) * 180 / M_PI;
            params.maxdeflect = fast_value(v, "maxdeflect", 15);
            params.enableControl = fast_value(v, "enableControl", 0) == 1;
            if (params.enableControl)
                params.ctrlSurfFrac = fast_value(v, "ctrlSurfFrac", 0.2);
            params.wingPart = fast_value(v, "wingPart", 2);
            params.deflectAngle = fast_value(v,"deflectAngle")  * 180 / M_PI;
            this->type = "WingGeometry";
            this->aera = params.b_2 * params.Mac;
            if (params.wingPart == 2) {
                this->aera = params.b_2 * params.Mac * 2;
            }
            params.root_chord_length = params.Mac * 2 / (1+params.TarperRatio) ;
            params.taper_chord_length = params.Mac * 2 * params.TarperRatio / (1+params.TarperRatio) ;
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

    }
}

