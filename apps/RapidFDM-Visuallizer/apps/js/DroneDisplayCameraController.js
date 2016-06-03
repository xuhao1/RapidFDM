"use strict";
var DroneDisplayCameraController = function(camera)
{
    this.camera = camera;
    camera.position.x = 2;
    camera.position.y = 0;
    camera.position.z = 0.3;


    this.camera.up.x = 0;
    this.camera.up.y = 0;
    this.camera.up.z = 1;

    this.camera.lookAt(new THREE.Vector3(0, 0, 0));

};

DroneDisplayCameraController.prototype.constructor = DroneDisplayCameraController;
var total_time = 0;
var rotate_time = 2;
DroneDisplayCameraController.prototype.update = function () {
    let camera = this.camera;
    total_time = total_time + 0.015;
    camera.position.x = Math.cos(total_time/rotate_time);
    camera.position.y = Math.sin(total_time/rotate_time);
    //camera.position.z = 2 * Math.cos(total_time/rotate_time/2);
    this.camera.lookAt(new THREE.Vector3(0, 0, 0));
};


//module.exports = DroneDisplayCameraController;