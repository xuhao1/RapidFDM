/**
 * Created by xuhao on 2016/10/3.
 */
var con = 0;
function configurer_view_init(_con) {
    con = _con;
    setInterval("configurer_view_update()", 500);
    setTimeout(function () {
        con.query_state(display_state)
    }, 1000);
}

function display_state(data) {
    $("#internal_state_table").html("<thead><tr><th>#</th> <th>name</th> <th>value</th> <th>setvalue</th></tr></thead>");
    var count = 0;
    for (var name in data) {
        count++;
        var tr = $("<tr>").html(`<th>${count}</th>`);
        var th0 = $("<th>").html(`${name}`);
        var th1 = $("<th>").html(`${data[name]}`);
        var th2 = $("<th>");


        var form = $("<form>", {
            class: "form-inline",
            role: "form"
        });

        let value_input = $("<input>", {
            type: "number",
            class: "form-control",
            value: data[name]
        });

        form.append(value_input);

        var button = $("<button>", {
            type: "button",
            class: "btn btn-default"
        }).html("submit");

        form.append(button);
        let state_name = name;
        button.click(function () {
            console.log(value_input);
            // alert(`Set internal ${state_name} to ${value_input[0].value}`);
            con.set_internal_state_value(state_name, parseFloat(value_input[0].value), function () {
                con.query_state(display_state)
            });
        });
        th2.append(form);

        tr.append(th0);
        tr.append(th1);
        tr.append(th2);
        $("#internal_state_table").append(tr);
    }
}
function get_total_forces_torques() {
    console.log("query forces");
    con.get_total_forces_torques(function (data) {
        var dis = aircraftview.dis;
        // if (data.blades !== undefined)
        //     aircraftview.blade_force_callback(data.blades);
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
        console.log(data.total_airdynamics_torque);
        var aerodynamics_torque_local = aerodynamics_torque.clone().applyQuaternion(
            aircraftview.attitude.clone().inverse()
        );

        $("#aerodynamics_torque").text(`${aerodynamics_torque_local.x.toFixed(3)} ${aerodynamics_torque_local.y.toFixed(3)} ${aerodynamics_torque_local.z.toFixed(3)}`);
        $("#aerodynamics_force").text(`${aerodynamics_force.x.toFixed(3)} ${aerodynamics_force.y.toFixed(3)} ${aerodynamics_force.z.toFixed(3)}`);

        var aero_origin = force_pos_from_force_torque(aerodynamics_force, aerodynamics_torque)
            .add(aircraftview.mass_center);

        if (engine_force.length() > 0.01) {
            var dir = engine_force.clone();
            dir.normalize();
            var origin = force_pos_from_force_torque(engine_force, engine_torque).add(aircraftview.mass_center);
            var length = engine_force.length() / aircraftview.mass / 9.8 * aircraftview.scale;
            var hex = 0xff0000;

            if (aircraftview.engine_force_arrow != 0) {
                dis.scene.remove(aircraftview.engine_force_arrow);
            }

            aircraftview.engine_force_arrow = new THREE.ArrowHelper(dir, origin, length, hex);
            dis.addObject(aircraftview.engine_force_arrow);
        }

        if (aerodynamics_force.length() > 0.01) {

            var aero_hex = 0x00ffff;

            var aerodynamics_length = aerodynamics_force.length() / aircraftview.mass / 9.8 * aircraftview.scale;
            var aerodynamics_dir = aerodynamics_force.clone().normalize();

            if (aircraftview.aerodynamics_force_arrow != 0) {
                dis.scene.remove(aircraftview.aerodynamics_force_arrow);
            }

            aircraftview.aerodynamics_force_arrow = new THREE.ArrowHelper(aerodynamics_dir, aero_origin, aerodynamics_length, aero_hex);
            dis.addObject(aircraftview.aerodynamics_force_arrow);
        }
        {
            if (aircraftview.gravity_force_arrow != 0)
                dis.scene.remove(aircraftview.gravity_force_arrow);
            var dir = new THREE.Vector3(0, 0, -1);

            var length = aircraftview.scale;

            var hex = 0xffff00;

            aircraftview.gravity_force_arrow = new THREE.ArrowHelper(dir, aircraftview.mass_center, length, hex);
            aircraftview.dis.addObject(aircraftview.gravity_force_arrow);
        }
    });
};
function configurer_view_update() {
    get_total_forces_torques();
}