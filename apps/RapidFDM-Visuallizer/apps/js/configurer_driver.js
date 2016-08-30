/**
 * Created by xuhao on 16/7/7.
 * This will build up a bridge between backend & frontend
 */

var ConfigurerDriver = function (onopencb) {
    this.ws_configurer = new WebSocket("ws://127.0.0.1:9091/query");
    let self = this;
    this.ws_configurer.onmessage = function (msg) {
        self.onmessage(msg, self);
    };
    this.ws_configurer.onopen = function () {
        log("Successful connected");
        onopencb();
    };
    this.ws_configurer.onclose = function () {
        log("Connetion closed");
    };
    this.ws_configurer.onerror = function () {
        log("connection error");
    };
    this.query_configurer_callback_table = [];
    this.query_model_callback_table = [];
};
ConfigurerDriver.prototype.onmessage = function (msg, self) {
    var obj = eval("(" + msg.data + ")");
    if (obj.opcode == "response_query_configurer") {
        // console.log(self.query_configurer_callback_table);
        self.query_configurer_callback_table[obj.type](obj.data);
    }
    else if (obj.opcode == "response_query") {
        self.query_model_callback_table[obj.type](obj.data);
    }
};

ConfigurerDriver.prototype.list_model = function (callback) {
    this.ws_configurer.send(
        JSON.stringify({
            "opcode": "query_configurer",
            "data": {
                "opcode": "list_model"
            }
        }));
    this.query_configurer_callback_table["list_model"] = callback;
};

ConfigurerDriver.prototype.load_model = function (name,callback) {
    this.ws_configurer.send(
        JSON.stringify({
            "opcode": "query_configurer",
            "data": {
                "opcode": "load_model",
                "name": name
            }
        }));
    this.query_configurer_callback_table["load_model"] = callback;
};

ConfigurerDriver.prototype.query_state = function (callback) {
    this.ws_configurer.send(
        JSON.stringify({
            "opcode": "query_model",
            "data": {
                "opcode": "get_internal_states_list"
            }
        }));
    this.query_model_callback_table["get_internal_states_list"] = callback;
};

ConfigurerDriver.prototype.query_control_axis = function (callback) {
    this.ws_configurer.send(
        JSON.stringify({
            "opcode": "query_model",
            "data": {
                "opcode": "get_control_axis_list"
            }
        }));
    this.query_model_callback_table["get_control_axis_list"] = callback;
};

ConfigurerDriver.prototype.get_total_forces_torques = function (callback) {
    this.ws_configurer.send(
        JSON.stringify({
            "opcode": "query_model",
            "data": {
                "opcode": "get_total_forces_torques"
            }
        }));
    this.query_model_callback_table["get_total_forces_torques"] = callback;
};

ConfigurerDriver.prototype.set_internal_state_value = function (name,value,callback) {
    "use strict";
     this.ws_configurer.send(
        JSON.stringify({
            "opcode": "query_model",
            "data": {
                "opcode": "set_internal_state_value",
                "internal_state_name": name,
                "value":value
            }
        }));
    this.query_model_callback_table["set_internal_state_value"] = callback;
};

ConfigurerDriver.prototype.set_control_axis_value = function (name,value,callback) {
    "use strict";
     this.ws_configurer.send(
        JSON.stringify({
            "opcode": "query_model",
            "data": {
                "opcode": "set_control_axis_value",
                "control_axis_name": name,
                "value":value
            }
        }));
    this.query_model_callback_table["set_control_axis_value"] = callback;
};

ConfigurerDriver.prototype.set_air_state = function (rho,ground_air_speed,callback) {
    "use strict";
     this.ws_configurer.send(
        JSON.stringify({
            "opcode": "query_model",
            "data": {
                "opcode": "set_air_state",
                "rho": rho,
                "ground_air_speed":ground_air_speed
            }
        }));
    this.query_model_callback_table["set_air_state"] = callback;
};

ConfigurerDriver.prototype.set_flying_state = function (angular_velocity,ground_speed,transform,callback) {
    "use strict";
     this.ws_configurer.send(
        JSON.stringify({
            "opcode": "query_model",
            "data": {
                "opcode" : "set_flying_state",
                "angular_velocity" : angular_velocity,
                "ground_velocity" : ground_speed,
                "transform" : transform
            }
        }));
    this.query_model_callback_table["set_flying_state"] = callback;
};
