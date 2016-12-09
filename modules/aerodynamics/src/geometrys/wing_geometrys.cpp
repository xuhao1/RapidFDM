//
// Created by Hao Xu on 16/7/14.
//


#include <RapidFDM/aerodynamics/geometrys/wing_geometrys.h>
#include <RapidFDM/aerodynamics/geometrys/Geometrys.h>
#include <RapidFDM/aerodynamics/blade_element/wing_blade_element.h>

namespace RapidFDM
{
    namespace Aerodynamics
    {

        Eigen::Vector3d WingGeometry::get_aerodynamics_force(ComponentData state, AirState airState) const
        {
            Eigen::Vector3d res = Eigen::Vector3d(0,0,0);
            for (auto blade : blades) {
                ComponentData data = blade->make_component_data_from_geometry(state);
                auto convert_coord = blade->get_relative_transform().linear();
                res += convert_coord *blade->get_aerodynamics_force(data,airState);
            }
            return res;
        }


        Eigen::Vector3d WingGeometry::get_aerodynamics_torque(ComponentData state, AirState airState) const
        {
            Eigen::Vector3d res = Eigen::Vector3d(0,0,0);
            for (auto blade : blades) {
                ComponentData data = blade->make_component_data_from_geometry(state);
                auto convert_coord = blade->get_relative_transform().linear();
                Eigen::Vector3d node_body_r = (Eigen::Vector3d) blade->get_relative_transform().translation();
                res += convert_coord * blade->get_aerodynamics_torque(data,airState) +
                       node_body_r.cross(convert_coord * blade->get_aerodynamics_force(data,airState));

            }
            return res;
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
            params.MidChordSweep = fast_value(v, "MidChordSweep", 0) * M_PI / 180;
            params.maxdeflect = fast_value(v, "maxdeflect", 15);
            params.enableControl = fast_value(v, "enableControl", 0) == 1;
            params.cl0 = fast_value(v,"cl0",0.1f);
            params.cl_by_deg = fast_value(v,"cl_by_deg",0.1f);
            params.cd0 = fast_value(v,"cd0",0.01);
            params.cd_by_deg2 = fast_value(v,"cd_by_deg",6e-5);
            params.stall_angle = fast_value(v,"stall_angle",20);
            int pieces = fast_value(v,"pieces",5);

            params.ctrlSurfFrac = fast_value(v, "ctrlSurfFrac", 0.2);

            params.wingPart = fast_value(v, "wingPart", 2);
            params.deflectAngle = fast_value(v, "deflectAngle") *  M_PI / 180;
            this->type = "WingGeometry";
            this->aera = params.b_2 * params.Mac;
            if (params.wingPart == 2) {
                this->aera = params.b_2 * params.Mac * 2;
            }
            params.root_chord_length = params.Mac * 2 / (1 + params.TarperRatio);
            params.taper_chord_length = params.Mac * 2 * params.TarperRatio / (1 + params.TarperRatio);
            for (int i = 0; i < pieces; i ++) {
                double k = ((double)i) / ((double) pieces);
                if (params.wingPart == 1 || params.wingPart == 2)
                    this->blades.push_back(
                            new WingBladeElement(this, k, k + 1/((double)pieces))
                    );
                if (params.wingPart == 0 || params.wingPart == 2)
                    this->blades.push_back(
                            new WingBladeElement(this, -k, -k - 1/((double)pieces))
                    );
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
        
        void WingGeometry::set_flying_state(const ComponentData &data)
        {
            for (auto blade : blades) {
                blade->update_component_data_from_geometry(data);
            }
        }

    }
}

