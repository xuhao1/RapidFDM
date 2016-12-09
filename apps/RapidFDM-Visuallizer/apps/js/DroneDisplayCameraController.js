"use strict";
var DroneDisplayCameraController = function (camera) {
    this.camera = camera;
    camera.position.x = 0;
    camera.position.y = 0;
    camera.position.z = 1;


    this.camera.up.x = 0;
    this.camera.up.y = 0;
    this.camera.up.z = 1;

    this.radius = 2;
    this.camera.lookAt(new THREE.Vector3(0, 0, 0));
    this.target = 0;

    this.mode = "third_person_follow";

    this.theta = 0;
    this.phi = Math.PI * 20 / 180;
    this.height = 3;
    this.wx = 0;
    this.wy = 0;
    this.min_cam_height = 1;
    this.max_cam_height = 10;
    
    this.relative_center_point = new THREE.Vector3(0,0,0.0);

    let obj = this;
    document.addEventListener('keydown', function (event) {
        switch (event.keyCode) {
            // case 187:
            //     obj.zoom(-1);
            //     break;
            // case 189:
            //     obj.zoom(1);
            //     break;
            case 191:
                obj.leftright(1);
                break;
            case 188:
                obj.leftright(-1);
                break;
            case 76:
                obj.updown(1);
                break;
            case 190:
                obj.updown(-1);
                break;
        }
    }, false);


};

DroneDisplayCameraController.prototype.set_target = function (target) {
    this.target = target;
};

DroneDisplayCameraController.prototype.constructor = DroneDisplayCameraController;
DroneDisplayCameraController.prototype.update = function () {
    let camera = this.camera;
    this.theta += this.wx;
    this.phi += this.wy;
    if (this.phi > 85 / 180 * Math.PI) {
        this.phi = 85 / 180 * Math.PI;
    }
    if (this.phi < -85 / 180 * Math.PI) {
        this.phi = -85 / 180 * Math.PI;
    }
    var rad = this.height;

    this.camera.up.x = 0;
    this.camera.up.y = 0;
    this.camera.up.z = 1;
    this.wx = this.wx * 0.93;
    this.wy = this.wy * 0.93;

    // console.log(this.target);
    if (this.target != 0) {

        var relative_position = new THREE.Vector3(
            rad * Math.cos(this.theta) * Math.cos(this.phi),
            rad * Math.sin(this.theta) * Math.cos(this.phi) ,
            rad * Math.sin(this.phi)
        );
        relative_position.applyQuaternion(this.target.attitude);
        this.camera.position.x = relative_position.x + this.target.position.x;
        this.camera.position.y = relative_position.y + this.target.position.y;
        this.camera.position.z = relative_position.z + this.target.position.z;


        //Lookat must be after position
        var target = new THREE.Vector3(
            this.target.position.x,
            this.target.position.y,
            this.target.position.z
        );
        target.add(this.relative_center_point.clone().applyQuaternion(this.target.attitude));
        this.camera.lookAt(target);
       
    }
};

DroneDisplayCameraController.prototype.zoom = function (k) {
    this.height = this.height * (1 + 0.1 * k);
    //console.log(this.height);
    if (this.height < this.min_cam_height) {
        this.height = this.min_cam_height;
    }
    if (this.height > this.max_cam_height) {
        this.height = this.max_cam_height;
    }
};

DroneDisplayCameraController.prototype.leftright = function (k) {
    this.wx += 0.05 * k * this.height / 10;
};

DroneDisplayCameraController.prototype.updown = function (k) {
    this.wy += 0.05 * k * this.height / 10;
};


//module.exports = DroneDisplayCameraController;