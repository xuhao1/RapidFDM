var AircraftView = function (config) {
    var driver = config.driver;
    this.driver = driver;
    this.dis = config.dis;
    this.scale = config.scale;
    this.component_mesh_list = {};
    this.config = config;
    let obj = this;

    this.position = new THREE.Vector3(0, 0, 0);
    this.attitude = new THREE.Quaternion(0, 0, 0, 1);
    console.log("set target");
    config.camera_controller.set_target(this);

    driver.load_model(config.name, function (data) {
        obj.mass = data.mass;
        obj.mass_center = new THREE.Vector3(
            data.mass_center[0],
            data.mass_center[1],
            data.mass_center[2]);

        obj.mass_center_body = new THREE.Vector3(
            data.mass_center[0],
            data.mass_center[1],
            data.mass_center[2]);

        log("load aircraft");
        obj.component_mesh_list = LoadAircraftGeometry(data, obj.dis);


        // obj.set_value_test();

        obj.set_wind_speed(15, 0,0);
        
        obj.a3_online = false;


        setTimeout(function () {
                obj.update_view()
            }, 100
        );

    });

    this.rho = 1.29;
    this.engine_force_arrow = 0;
    this.aerodynamics_force_arrow = 0;
    this.gravity_force_arrow = 0;
    this.wind_arrows = 0;
    this.in_realtime_simulator = false;
    this.blade_force_arrows = 0;
};


AircraftView.prototype.constructor = AircraftView;

AircraftView.prototype.update_view = function () {
    let obj = this;
    setTimeout(function () {
            obj.update_view()
        }, 1000
    );
    if (!this.in_realtime_simulator) {
        this.receive_simulator_data();
    }
};

AircraftView.prototype.get_internal_states_list = function () {
    let obj = this;
    this.driver.query_state(function (data) {
        log(JSON.stringify(data));
    });
};

AircraftView.prototype.get_control_axis_list = function () {
    let obj = this;
    this.driver.query_control_axis(function (data) {
        // log(JSON.stringify(data));
    });
};
function force_pos_from_force_torque(force, torque) {
    //Assume force and pos is orth
    var tmp = force.clone().cross(torque);
    tmp.multiplyScalar(1 / force.length() / force.length());
    return tmp;
}
AircraftView.prototype.start_simulation = function (config) {
    log("Tring to start simulation");
    this.ws_simulator.send(JSON.stringify(
        {
            opcode: "start",
            data: {
                init_transform: config.transform,
                init_speed: config.init_speed
            }
        }
    ));
    log(JSON.stringify(
        {
            opcode: "start",
            data: {
                init_transform: config.transform,
                init_speed: config.init_speed
            }
        }));
};
AircraftView.prototype.receive_simulator_data = function () {
    let obj = this;
    try {
        this.ws_simulator = new WebSocket("ws://127.0.0.1:9093/output");
        this.ws_simulator.onmessage = function (event) {
            var data = eval(`(${event.data})`);
            
            if (data.forces_torques !== undefined)
                this.forces_torques = data.forces_torques;
            if (this.forces_torques !== undefined)
                obj.forces_torques_callback(this.forces_torques);
            
            var airstate = data.airstate;

            $("#air_speed").text(airstate.airspeed);
            $("#angle_of_attack").text(airstate.angle_of_attack * 180/3.14);
            $("#sideslip").text(airstate.sideslip * 180 / 3.14);

            obj.set_transform(data.transform);
            $("#angular_velocity").text(
                `${(data.angular_velocity[0] * 180 / 3.1415926).toFixed(1)}\
                 ${(data.angular_velocity[1] * 180 / 3.1415926).toFixed(1)}\
                 ${(data.angular_velocity[2] * 180 / 3.1415926).toFixed(1)}`
            );
            $("#attitude").text(
                `roll:${(data.euler[0]).toFixed(1)}\
                 pitch:${(data.euler[1]).toFixed(1)}\
                 yaw:${(data.euler[2]).toFixed(1)}`
            )

            $("#velocity").text(
                `X:${(data.vel[0]).toFixed(1)}\
                 Y:${(data.vel[1]).toFixed(1)}\
                 Z:${(data.vel[2]).toFixed(1)}`
            );

            if (data.a3_sim_status !== undefined)
            {
                obj.input.use_a3 = true;
                for (var i = 0 ;i < 8 ;i ++)
                {
                    data.a3_sim_status.PWM[i] = Math.floor(data.a3_sim_status.PWM[i]);
                }
                $("#pwm").text(JSON.stringify(data.a3_sim_status.PWM));
                
               obj.a3_online = data.a3_sim_status.online > 0;
                obj.input.a3_online = obj.a3_online;
                
                if (obj.input.a3_online) {
                    obj.input.throttle = data.a3_sim_status.RcT / 2 + 5000;
                    obj.input.aileron = data.a3_sim_status.RcA;
                    obj.input.rudder = data.a3_sim_status.RcR;
                    obj.input.elevator = data.a3_sim_status.RcE;
                }
                
            }
                
        };
        this.ws_simulator.onopen = function (event) {
            obj.in_realtime_simulator = true;
            "use strict";
            log("open ws from simulator");
            obj.start_simulation({
                transform: {
                    attitude: [0, 0, 0],
                    vector: [10,-30, 300]
                },
                init_speed:0
            });
        };
        this.ws_simulator.onclose = function (event) {
            obj.in_realtime_simulator = false;
            // log("ws from simulator closed");
        };
        this.ws_simulator.onerror = function (event) {
            obj.in_realtime_simulator = false;
        };
    }
    catch (e)
    {
        
    }
};

AircraftView.prototype.set_transform = function (transform, disable_position = false) {
    var root_attitude = new THREE.Quaternion(
        transform.attitude[1],
        transform.attitude[2],
        transform.attitude[3],
        transform.attitude[0]
    );

    this.attitude = root_attitude;

    var root_position = new THREE.Vector3(
        transform.vector[0],
        transform.vector[1],
        transform.vector[2]
    );
    if (disable_position) {
        root_position = new THREE.Vector3(30, 30, 30);
    }

    this.position = root_position.clone();
    this.mass_center = this.mass_center_body.clone().applyQuaternion(root_attitude).add(root_position);

    for (var name in this.component_mesh_list) {
        var mesh = this.component_mesh_list[name].mesh;
        var transform_relative_body = this.component_mesh_list[name].body_transform;
        var attitude_relative_body = new THREE.Quaternion(
            transform_relative_body.attitude[1],
            transform_relative_body.attitude[2],
            transform_relative_body.attitude[3],
            transform_relative_body.attitude[0]
        );

        var position_relative_mass_center = new THREE.Vector3(
            transform_relative_body.vector[0] - this.mass_center_body.x,
            transform_relative_body.vector[1] - this.mass_center_body.y,
            transform_relative_body.vector[2] - this.mass_center_body.z
        );


        var quaternion_tmp = root_attitude.clone().multiply(attitude_relative_body);

        position_relative_mass_center.applyQuaternion(root_attitude);

        mesh.quaternion.w = quaternion_tmp.w;
        mesh.quaternion.x = quaternion_tmp.x;
        mesh.quaternion.y = quaternion_tmp.y;
        mesh.quaternion.z = quaternion_tmp.z;

        var position_tmp = this.mass_center.clone().add(position_relative_mass_center);
        mesh.position.x = position_tmp.x;
        mesh.position.y = position_tmp.y;
        mesh.position.z = position_tmp.z;
        // console.log(position_tmp);
    }

};

AircraftView.prototype.blade_force_callback = function (data) {
    // console.log(data);
    if (this.blade_force_arrows != 0) {
        for (arrow_index in this.blade_force_arrows) {
            this.dis.scene.remove(this.blade_force_arrows[arrow_index]);
        }
    }
    this.blade_force_arrows = [];
    for (var index in data) {
        let force = data[index].force;
        let location = data[index].location;
        var force_vec = new THREE.Vector3(
            force[0],
            force[1],
            force[2]
        );
        var origin = new THREE.Vector3(
            location[0],
            location[1],
            location[2]
        );
        origin.applyQuaternion(this.attitude.clone()).add(this.mass_center);
        var length = force_vec.length() / this.mass / 9.8 * this.scale * 2;
        var hex = 0xffffff;
        var arrow = new
            THREE.ArrowHelper(force_vec.normalize(), origin, length, hex);
        this.dis.addObject(arrow);
        this.blade_force_arrows.push(arrow);

    }
};

AircraftView.prototype.forces_torques_callback = function (data) {
    var dis = this.dis;
    if (data.blades!==undefined)
        this.blade_force_callback(data.blades);
    var engine_force = new THREE.Vector3(
        data.total_engine_force[0],
        data.total_engine_force[1],
        data.total_engine_force[2]
    );
    $("#engine_force").text(engine_force.length());

    var engine_torque = new THREE.Vector3(
        data.total_engine_torque[0],
        data.total_engine_torque[1],
        data.total_engine_torque[2]
    );
    var aerodynamics_force = new THREE.Vector3(
        data.total_airdynamics_force[0],
        data.total_airdynamics_force[1],
        data.total_airdynamics_force[2]
    );



    var aerodynamics_torque = new THREE.Vector3(
        data.total_airdynamics_torque[0],
        data.total_airdynamics_torque[1],
        data.total_airdynamics_torque[2]
    );
    var aerodynamics_torque_local = aerodynamics_torque.clone().applyQuaternion(
        this.attitude.clone().inverse()
    );

    $("#aerodynamics_torque").text(`${aerodynamics_torque_local.x.toFixed(3)} ${aerodynamics_torque_local.y.toFixed(3)} ${aerodynamics_torque_local.z.toFixed(3)}`);
    $("#aerodynamics_force").text(`${aerodynamics_force.x.toFixed(3)} ${aerodynamics_force.y.toFixed(3)} ${aerodynamics_force.z.toFixed(3)}`);
    if (this.in_realtime_simulator)
        return;

    var aero_origin = force_pos_from_force_torque(aerodynamics_force, aerodynamics_torque)
        .add(this.mass_center);

    if (engine_force.length() > 0.01) {
        var dir = engine_force.clone();
        dir.normalize();
        var origin = force_pos_from_force_torque(engine_force, engine_torque).add(this.mass_center);
        var length = engine_force.length() / this.mass / 9.8 * this.scale;
        var hex = 0xff0000;

        if (this.engine_force_arrow != 0) {
            dis.scene.remove(this.engine_force_arrow);
        }

        this.engine_force_arrow = new THREE.ArrowHelper(dir, origin, length, hex);
        dis.addObject(this.engine_force_arrow);
    }

    if (aerodynamics_force.length() > 0.01) {

        var aero_hex = 0x00ffff;

        var aerodynamics_length = aerodynamics_force.length() / this.mass / 9.8 * this.scale;
        var aerodynamics_dir = aerodynamics_force.clone().normalize();

        if (this.aerodynamics_force_arrow != 0) {
            dis.scene.remove(this.aerodynamics_force_arrow);
        }

        this.aerodynamics_force_arrow = new THREE.ArrowHelper(aerodynamics_dir, aero_origin, aerodynamics_length, aero_hex);
        dis.addObject(this.aerodynamics_force_arrow);
    }
    {
        if (this.gravity_force_arrow != 0)
            dis.scene.remove(this.gravity_force_arrow);
        var dir = new THREE.Vector3(0, 0, -1);

        var length = this.scale;

        var hex = 0xffff00;

        this.gravity_force_arrow = new THREE.ArrowHelper(dir, this.mass_center, length, hex);
        this.dis.addObject(this.gravity_force_arrow);
    }


};

AircraftView.prototype.get_total_forces_torques = function () {
    let obj = this;
    this.driver.get_total_forces_torques(function (data) {
        obj.forces_torques_callback(data);
    });
};

AircraftView.prototype.set_control_axis_value = function (name, v) {
    let obj = this;
    this.driver.set_control_axis_value(name, v, function (data) {
        log(`set control value ${JSON.stringify(data)}`);
        obj.get_total_forces_torques();
    });
};

AircraftView.prototype.set_internal_state_value = function (name, v) {
    let obj = this;
    this.driver.set_internal_state_value(name, v, function (data) {
        log(`set internal value ${JSON.stringify(data)}`);
        obj.get_total_forces_torques();
    });
};

AircraftView.prototype.set_value_test = function () {
    let obj = this;
    this.set_internal_state_value("main_wing_0/flap_0", 1);
    this.set_internal_state_value("main_wing_0/flap_1", 1);
    this.get_internal_states_list();
};

AircraftView.prototype.draw_wind = function (speed, alpha, sideslip) {
    $("#air_speed").text(speed);
    $("#angle_of_attack").text(alpha);
    $("#sideslip").text(sideslip);

    alpha = alpha / 180 * Math.PI;
    sideslip = sideslip / 180 * Math.PI;

    let wind_speed = {
        x: speed * Math.cos(alpha) * Math.cos(-sideslip),
        y: speed * Math.sin(-sideslip),
        z: speed * Math.sin(alpha)
    };

    if (this.wind_arrows != 0) {
        for (arrow_index in this.wind_arrows) {
            this.dis.scene.remove(this.wind_arrows[arrow_index]);
        }
    }
    this.wind_arrows = [];

    var dir = new THREE.Vector3(wind_speed.x, wind_speed.y, wind_speed.z);
    dir.normalize();
    dir.applyQuaternion(this.attitude.clone())
    // console.log("draw wind");
    var base_position = this.mass_center;
    for (var i = -2; i < 3; i++)
        for (var j = -2; j < 3; j++) {
            var origin = new THREE.Vector3(
                -this.scale/3,
                i * this.scale/3,
                j * this.scale/3
            );
            origin.applyQuaternion(this.attitude.clone());
            origin.add(base_position);
            var length = speed * this.scale / 30;
            var hex = 0x0000ff;


            var arrow = new THREE.ArrowHelper(dir, origin, length, hex);
            this.dis.addObject(arrow);
            this.wind_arrows.push(arrow);
        }
};

AircraftView.prototype.set_wind_speed = function (speed, alpha = 0, sideslip = 0) {
    this.draw_wind(speed, alpha, sideslip);

    alpha = alpha / 180 * Math.PI;
    sideslip = sideslip / 180 * Math.PI;
    let wind_speed = {
        x: speed * Math.cos(alpha) * Math.cos(-sideslip),
        y: speed * Math.sin(-sideslip),
        z: speed * Math.sin(alpha)
    };


    let obj = this;
    this.driver.set_air_state(this.rho, [wind_speed.x, wind_speed.y, wind_speed.z], function (data) {
        obj.get_total_forces_torques();
    });

};

AircraftView.prototype.set_flying_state = function (angular_velocity,attitude) {
    let obj = this;
        this.driver.set_flying_state(angular_velocity,[0,0,0],{
            vector:[0,0,0],
            attitude:attitude,
        },function (data) {
            log(`set flying state ${data}`);
            // obj.set_transform(data.transform);
            obj.draw_wind(data.airstate.airspeed,
                data.airstate.angle_of_attack,data.airstate.sideslip);
            obj.get_total_forces_torques();
        });
};
