/**
 * Created by xuhao on 16/7/19.
 */


var AircraftInput = function (aircraftview) {
    "use strict";
    let obj = this;
    this.gamepad_connected = false;
    this.gamepad_index = 0;
    if (navigator.getGamepads()[0] !== undefined) {
        onconnect(0);
    }
    window.addEventListener("gamepadconnected", function (e) {
        console.log("On connected gamepad");
        obj.on_connect(e.gamepad.index);
    });
    window.addEventListener("gamepaddisconnected", function (e) {
        obj.dis_connect(e);
    });
    this.throttle = 0;
    this.aileron = 0;
    this.rudder = 0;
    this.elevator = 0;
    this.aux1 = -10000;
    this.delatatime = 0.02;
    this.throttle_ratio = 1;

    this.d_elevator = 0;
    this.d_rudder = 0;
    this.d_ailreon = 0;
    this.d_throttle = 0;
    this.d_aux1 = 0;

    this.d_axis_keyboard_ratio = 0.97;
    this.axis_keyboard_back_ratio = 3;

    this.use_hil = false;
    this.hil_online = false;

    obj.update();
    this.aircraftview = aircraftview;
    document.addEventListener('keydown', function (event) {
        switch (event.keyCode) {
            case 87://W
                obj.d_elevator = 500;
                break;
            case 83://S
                obj.d_elevator = -500;
                break;
            case 65://A
                obj.d_rudder = -500;
                break;
            case 68://D
                obj.d_rudder = 500;
                break;
            case 39://right
                obj.d_ailreon = 500;
                break;
            case 37://left
                obj.d_ailreon = -500;
                break;
            case 38://Up
                obj.d_throttle = 3000;
                break;
            case 40://Down
                obj.d_throttle = -3000;
                break;
            default:
                // log(event.keyCode);
                break;
        }
    }, false);

};


AircraftInput.prototype.on_connect = function (gamepad_index) {
    "use strict";
    this.gamepad_connected = true;
    $("#gamepad_connection").text("Gamepad connected :" + gamepad_index);
    $("#gamepad_connection").css("color", "green");
    this.gamepad_index = gamepad_index;
};

AircraftInput.prototype.get_gamepad = function () {
    return navigator.getGamepads()[this.gamepad_index];
};

AircraftInput.prototype.dis_connect = function (e) {
    "use strict";
    this.gamepad_connected = false;
};
function float_constrain(v,l,u)
{
    if(v > u)
        return u;
    if (v<l)
        return l;
    return v;
}
var fuck = 0;
AircraftInput.prototype.update = function () {
    if (!this.use_hil) {
        var dt = this.delatatime;
        var gamepad = this.get_gamepad();
        if (gamepad !== undefined) {
            this.elevator = 0;
            console.log(gamepad.axes);
            this.d_aux1 = (gamepad.axes[5] - gamepad.axes[2])*10000/2;
            this.aileron = gamepad.axes[3] * 10000;
            this.elevator = - gamepad.axes[4] * 10000;
            this.rudder = gamepad.axes[0] * 10000;
            this.d_throttle = - gamepad.axes[1] * 10000;
        }
        else {
            this.d_elevator = this.d_elevator * this.d_axis_keyboard_ratio;
            this.elevator = this.elevator + this.d_elevator ;//- this.elevator * this.axis_keyboard_back_ratio * this.delatatime;
            this.elevator = float_constrain(this.elevator,-10000,10000);

            this.d_ailreon = this.d_ailreon * this.d_axis_keyboard_ratio;
            this.aileron = this.aileron + this.d_ailreon;// - this.aileron * this.axis_keyboard_back_ratio * this.delatatime;
            this.aileron = float_constrain(this.aileron,-10000,10000);

            this.d_rudder = this.d_rudder * this.d_axis_keyboard_ratio;
            this.rudder = this.rudder + this.d_rudder - this.rudder * this.axis_keyboard_back_ratio * this.delatatime;

            this.d_throttle = this.d_axis_keyboard_ratio * this.d_throttle;

        }

        if (Math.abs(this.d_throttle) > 500) {
            this.throttle = this.throttle + this.d_throttle * dt * this.throttle_ratio;
            if (this.throttle > 10000) {
                this.throttle = 10000;
            }
            else if (this.throttle < -10000) {
                this.throttle = -10000;
            }
        }
         if (Math.abs(this.d_aux1) > 500) {
            this.aux1 = this.aux1 + this.d_aux1 * dt * this.throttle_ratio;
            if (this.aux1 > 10000) {
                this.aux1 = 10000;
            }
            else if (this.aux1 < -10000) {
                this.aux1 = -10000;
            }
        }

        try {
            if (this.aircraftview.in_realtime_simulator) {
                this.aircraftview.ws_simulator.send(JSON.stringify({
                    opcode: "set_channel_value",
                    data: [
                        this.aileron / 10000,
                        this.elevator / 10000,
                        this.throttle / 10000,
                        this.rudder / 10000,
                        this.aux1 / 10000,
                    ]
                }));

            }
        }
        catch (e) {
        }
    }
    else {
        if (this.hil_online) {
            $("#gamepad_connection").text("HIL connected");
            $("#gamepad_connection").css("color", "green");
        }
        else {
            $("#gamepad_connection").text("HIL disconnected");
            $("#gamepad_connection").css("color", "red");
        }
    }

    try {

        $("#left_game_bar").css("left", this.rudder / 4000 + 2.35 + "em");
        $("#left_game_bar").css("bottom", this.throttle / 4000 + 2 + "em");

        $("#right_game_bar").css("left", this.aileron / 4000 + 2.35 + "em");
        $("#right_game_bar").css("bottom", this.elevator / 4000 + 2 + "em");


    }
    catch (e) {
        console.log(e);
    }
    let obj = this;
    setTimeout(function () {
        obj.update();
    }, dt * 20);

};
