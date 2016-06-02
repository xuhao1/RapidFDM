"use strict";
var DroneDisplayCameraController = function(camera)
{
    this.camera = camera;
    camera.position.x = 0;
    camera.position.y = 0;
    camera.position.z = 2;


    this.camera.up.x = 0;
    this.camera.up.y = 0;
    this.camera.up.z = 1;

    this.camera.lookAt(new THREE.Vector3(0, 0, 0));

};

DroneDisplayCameraController.prototype.constructor = DroneDisplayCameraController;


//module.exports = DroneDisplayCameraController;