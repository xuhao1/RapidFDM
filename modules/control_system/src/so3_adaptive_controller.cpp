//
// Created by xuhao on 2017/3/23.
//
#include <RapidFDM/control_system/so3_adaptive_controller.h>
#include <ctime>
#include <iomanip>
#include <RapidFDM/utils.h>
#include <L1AircraftControl_types.h>
#include <L1AircraftControl.h>
#include <random>

using namespace RapidFDM::Utils;

namespace RapidFDM {
    namespace ControlSystem {

        void copy_v3d_into_array(const Eigen::Vector3d v, float *arr) {
            arr[0] = (float) v.x();
            arr[1] = (float) v.y();
            arr[2] = (float) v.z();
        }

        void copy_v3d_into_array(const Eigen::Vector3d v, double *arr) {
            arr[0] = v.x();
            arr[1] = v.y();
            arr[2] = v.z();
        }


        so3_adaptive_controller::so3_adaptive_controller(Aerodynamics::AircraftNode *_aircraftNode) :
                BaseController(_aircraftNode),ang_vel_dist(0,0.02) {
            roll_sp = 0;
            pitch_sp = 0;
            init_attitude_controller(1, 6, &ctrlAttitude);
            double lag_fc = 12;
            double lag_alpha = 2;
            double p_actuator = 0.0;
            double ekf_p_noise = 0.05;

            double gamma = 120;
            L1ControllerUpdateParams(&(ctrlAttitude.RollCtrl), 0.6, 0.15, 64, gamma, lag_fc, lag_alpha, p_actuator,ekf_p_noise);
            L1ControllerUpdateParams(&(ctrlAttitude.PitchCtrl),0.6, 0.15, 64, gamma, lag_fc, lag_alpha, p_actuator,ekf_p_noise);
            L1ControllerUpdateParams(&(ctrlAttitude.YawCtrl), 0.6, 0.1, 20, gamma, lag_fc, lag_alpha, p_actuator,ekf_p_noise);
            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);
            std::ostringstream oss;
            oss << std::put_time(&tm, "/var/log/rapidfdm/so3_log_%Y_%m_%d_%H_%M_%S.mat");
            auto file = oss.str();
            printf("Creating file %s...\n\n", file.c_str());
            pmat = matOpen(file.c_str(), "w");

            controller_type = fast_string(aircraftNode->getJsonDefine(), "controller_type");
        }

        void convert_euler_to_quat_array(float quat[], double roll, double pitch, double yaw) {
            Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond quat_sp = yawAngle * pitchAngle * rollAngle;
            quat[0] = (float) quat_sp.w();
            quat[1] = (float) quat_sp.x();
            quat[2] = (float) quat_sp.y();
            quat[3] = (float) quat_sp.z();
        }

        void convert_euler_to_quat_array(double quat[], double roll, double pitch, double yaw) {
            Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond quat_sp = yawAngle * pitchAngle * rollAngle;
            quat[0] = quat_sp.w();
            quat[1] = quat_sp.x();
            quat[2] = quat_sp.y();
            quat[3] = quat_sp.z();
        }

        void so3_adaptive_controller::control_combinevtol(float deltatime) {
            Eigen::Vector3d eul = quat2eulers(quat);
            QuatControlSetpoint quatControlSetpoint;
            static double yaw_angle_sp = 0;
            if (fabs(yaw_sp) > 0.05 || aux1_sp > 0.1) {
                yaw_angle_sp = eul.z();
                quatControlSetpoint.yaw_rate = yaw_sp * M_PI;
                if (airspeed > 3.0f)
                {
                    quatControlSetpoint.yaw_rate += tan(eul.x())*cos(eul.y())*9.81f/airspeed;
                }
                quatControlSetpoint.yaw_sp_is_rate = 1;
            } else {
                quatControlSetpoint.yaw_sp_is_rate = 0;
            }



            float roll_act_real =(float) this->aircraftNode->get_internal_state("main_wing_0/flap_0");

            convert_euler_to_quat_array(quatControlSetpoint.quat, roll_sp * M_PI / 6, pitch_sp * M_PI / 6,
                                        yaw_angle_sp);

            L1ControlAttitude(&ctrlAttitude, deltatime, &quatControlSetpoint, &sys);

            sys.quat[0] = eul.x();
            sys.quat[1] = roll_sp * M_PI /6;

            Eigen::Vector3d u = Eigen::Vector3d(0, 0, 0);

            u.x() = ctrlAttitude.u[0];
            u.y() = ctrlAttitude.u[1];
            u.z() = ctrlAttitude.u[2];

            float combine_vtol_torque_ratio  = 1.0;

            pwm[0] = (float) float_constrain((-u.x() + u.y() + u.z())*combine_vtol_torque_ratio + throttle_sp, 0, 1);
            pwm[1] = (float) float_constrain((u.x() + u.y() - u.z())*combine_vtol_torque_ratio + throttle_sp, 0, 1);
            pwm[2] = (float) float_constrain((u.x() - u.y() + u.z())*combine_vtol_torque_ratio + throttle_sp, 0, 1);
            pwm[3] = (float) float_constrain((-u.x() - u.y() - u.z())*combine_vtol_torque_ratio + throttle_sp, 0, 1);

            pwm[4] = (float) float_constrain(aux1_sp, -1, 1);
            pwm[5] = (float) float_constrain(u.x(), -1, 1);
            pwm[6] = (float) float_constrain(u.x(), -1, 1);
            pwm[7] = (float) float_constrain(u.y(), -1, 1);
            pwm[8] = (float) float_constrain(u.z(), -1, 1);

            sys.acc[0] = roll_act_real;
        }

        void so3_adaptive_controller::control_multirotor(float deltatime) {
            Eigen::Vector3d eul = quat2eulers(quat);
            QuatControlSetpoint quatControlSetpoint;
            static double yaw_angle_sp = 0;
            if (fabs(yaw_sp) > 0.05) {
                yaw_angle_sp = eul.z();
                quatControlSetpoint.yaw_rate = yaw_sp * M_PI;
                quatControlSetpoint.yaw_sp_is_rate = 1;
            } else {
                quatControlSetpoint.yaw_sp_is_rate = 0;
            }

            float th0 = (float) this->aircraftNode->get_internal_state("main_engine_0/thrust");
            float th1 = (float) this->aircraftNode->get_internal_state("main_engine_1/thrust");
            float th2 = (float) this->aircraftNode->get_internal_state("main_engine_2/thrust");
            float th3 = (float) this->aircraftNode->get_internal_state("main_engine_3/thrust");


            float roll_act_real = (th1 + th2 - th0 - th3) / 2;
            float pitch_act_real = (th1 + th0 - th2 - th3) / 2;
            float yaw_act_real = (th0 + th2 - th1 - th3) / 2;

            convert_euler_to_quat_array(quatControlSetpoint.quat, roll_sp * M_PI / 6, pitch_sp * M_PI / 6,
                                        yaw_angle_sp);
            double angular_vel_sp[3] = {0};
            angular_vel_sp[0] = roll_sp*M_PI;
            angular_vel_sp[1] = pitch_sp*M_PI/2;
            angular_vel_sp[2] = yaw_sp * M_PI/2;
//            L1ControlAngularVelocity(&ctrlAttitude, deltatime, angular_vel_sp, &sys);
            L1ControlAttitude(&ctrlAttitude, deltatime, &quatControlSetpoint, &sys);

            sys.quat[0] = eul.x();
            sys.quat[1] = roll_sp * M_PI /6;

            Eigen::AngleAxisd rot_u(0 * M_PI / 180, Eigen::Vector3d::UnitZ());
            Eigen::Vector3d u = Eigen::Vector3d(0, 0, 0);
            Eigen::Vector3d u_last = Eigen::Vector3d(0, 0, 0);


            double p_act = 0.0;
            u.x() = ctrlAttitude.u[0];
            u.y() = ctrlAttitude.u[1];
            u.z() = ctrlAttitude.u[2];

            pwm[0] = (float) float_constrain((-u.x() + u.y() + u.z()) + throttle_sp, 0, 1);
            pwm[1] = (float) float_constrain((u.x() + u.y() - u.z()) + throttle_sp, 0, 1);
            pwm[2] = (float) float_constrain((u.x() - u.y() + u.z()) + throttle_sp, 0, 1);
            pwm[3] = (float) float_constrain((-u.x() - u.y() - u.z()) + throttle_sp, 0, 1);

            sys.acc[0] = roll_act_real;
        }

        void so3_adaptive_controller::control_fixedwing(float deltatime) {
            static int count = 0;
            count++;
            static double us_count = 0;

            Eigen::Vector3d eul = quat2eulers(quat);
            QuatControlSetpoint quatControlSetpoint;
            static double yaw_angle_sp = 0;
            yaw_angle_sp = eul.z();
            quatControlSetpoint.yaw_rate = yaw_sp / 10 * M_PI;
            quatControlSetpoint.yaw_sp_is_rate = 1;
            quatControlSetpoint.yaw_rate += tan(eul.x())*cos(eul.y())*9.81f/airspeed;

            convert_euler_to_quat_array(quatControlSetpoint.quat, roll_sp * M_PI / 3, pitch_sp * M_PI / 3,
                                        yaw_angle_sp);

            long us0 = current_timestamp_us();
            double angular_vel_sp[3] = {0};
            angular_vel_sp[0] = roll_sp*M_PI;
            angular_vel_sp[1] = pitch_sp*M_PI/2;
            angular_vel_sp[2] = yaw_sp * M_PI/6;
            L1ControlAttitude(&ctrlAttitude, deltatime, &quatControlSetpoint, &sys);
            sys.quat[0] = eul.x();
            sys.quat[1] = roll_sp * M_PI/3;
//            L1ControlAngularVelocity(&ctrlAttitude, deltatime, angular_vel_sp, &sys);
            long used = current_timestamp_us() - us0;
            us_count += used;

            ctrlAttitude.debug_time_used = used;

            if (count % 200 == 0) {
                printf("Controller average us used %f\n", us_count / 200);
                us_count = 0;
            }

            Eigen::Vector3d u = Eigen::Vector3d(0, 0, 0);

            u.x() = ctrlAttitude.u[0];
            u.y() = ctrlAttitude.u[1];
            u.z() = ctrlAttitude.u[2];

            pwm[0] = (float) float_constrain(u.x(), -1, 1);
            pwm[1] = (float) float_constrain(u.y(), -1, 1);
            pwm[2] = (float) float_constrain(throttle_sp, 0, 1);
            pwm[3] = (float) float_constrain(u.z(), -1, 1);
        }

        void so3_adaptive_controller::control_step(float deltatime) {
            static int count = 0;
            count++;
            deltatime = deltatime / 1000;
            copy_v3d_into_array(angular_rate, sys.angular_rate);
            sys.angular_rate[0] = sys.angular_rate[0] + ang_vel_dist(generator);
            sys.angular_rate[1] = sys.angular_rate[1] + ang_vel_dist(generator);
            sys.angular_rate[2] = sys.angular_rate[2] + ang_vel_dist(generator);

            sys.quat[0] = quat.w();
            sys.quat[1] = quat.x();
            sys.quat[2] = quat.y();
            sys.quat[3] = quat.z();

            sys.acc[1] = angular_rate.x();

            if (controller_type == "fixedwing") {
                control_fixedwing(deltatime);
            } else if (controller_type == "multirotor") {
                control_multirotor(deltatime);
            } else if (controller_type == "combinevtol") {
                control_combinevtol(deltatime);
            }

            aircraftNode->set_control_from_channels(pwm, 16);

            ctrl_log.push_back(ctrlAttitude.RollCtrl);
            sys_log.push_back(sys);
            att_con_log.push_back(ctrlAttitude);

            float roll_act_real = (float) this->aircraftNode->get_internal_state("main_wing_0/flap_0");
            float pitch_act_real =(float) this->aircraftNode->get_internal_state("horizon_wing_0/flap_0");
            sys.acc[0] = roll_act_real;

            if (count % 200 == 0) {
                save_data_file();
            }
        }

        void set_value_mx_array(mxArray *arr, int i, int j, double data) {
            size_t mi = mxGetM(arr);
            size_t ni = mxGetN(arr);//4
            *(mxGetPr(arr) + i + j * mi) = data;
        }

        void so3_adaptive_controller::save_data_file() {
            static int log_number = 0;
            mxArray *pa1, *pa2;
            //t 1 x 6 err 2 u 1 eta 1

            int cols = 32;
            pa1 = mxCreateDoubleMatrix(ctrl_log.size(), cols, mxREAL);
            for (int i = 0; i < ctrl_log.size(); i++) {
                const AdaptiveCtrlT &ctrlT = ctrl_log[i];
                set_value_mx_array(pa1, i, 0, ctrlT.t);
                for (int j = 0; j < 7; j++) {
                    set_value_mx_array(pa1, i, j + 1, ctrlT.x[j]);
                }
                set_value_mx_array(pa1, i, 8, ctrlT.err[0]);
                set_value_mx_array(pa1, i, 9, ctrlT.err[1]);
                set_value_mx_array(pa1, i, 10, ctrlT.u);
                set_value_mx_array(pa1, i, 11, ctrlT.eta);
                set_value_mx_array(pa1, i, 12, ctrlT.rdot);
                set_value_mx_array(pa1, i, 13, ctrlT.g[0]);
                for (int j = 0; j < 4; j++) {
                    set_value_mx_array(pa1, i, j + 14, att_con_log[i].quat[j]);
                }
                for (int j = 0; j < 4; j++) {
                    set_value_mx_array(pa1, i, j + 18, att_con_log[i].quat_sp[j]);
                }
                set_value_mx_array(pa1, i, 22, att_con_log[i].debug_time_used);
                set_value_mx_array(pa1, i, 23, att_con_log[i].u[0]);
                set_value_mx_array(pa1, i, 24, ctrlT.x_real[1]);
                set_value_mx_array(pa1, i, 25, sys_log[i].acc[0]);
                set_value_mx_array(pa1, i, 26, ctrlT.actuator_estimator.actuator_real);
                set_value_mx_array(pa1, i, 27, sys_log[i].angular_rate[0]);
                set_value_mx_array(pa1, i, 28, sys_log[i].acc[1]);
                set_value_mx_array(pa1, i, 29, sys_log[i].quat[0]);
                set_value_mx_array(pa1, i, 30, sys_log[i].quat[1]);
                set_value_mx_array(pa1, i, 31, ctrlT.actuator_estimator.P[6 + 1]);
            }
            char matname[100] = {0};
            sprintf(matname, "AdaptiveCtrlT_%d", log_number);
            matPutVariable(pmat, matname, pa1);
            mxDestroyArray(pa1);
            ctrl_log.clear();
            sys_log.clear();
            att_con_log.clear();
            log_number++;
        }
    }
}
