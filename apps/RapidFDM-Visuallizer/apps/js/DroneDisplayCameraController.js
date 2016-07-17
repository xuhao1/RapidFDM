"use strict";
var DroneDisplayCameraController = function (camera) {
    this.camera = camera;
    camera.position.x = 0;
    camera.position.y = 0;
    camera.position.z = 1;


    this.camera.up.x = 0;
    this.camera.up.y = 0;
    this.camera.up.z = 1;

    this.total_time = 2.5;
    this.rotate_time = 8;
    this.radius = 2;
    this.camera.lookAt(new THREE.Vector3(0, 0, 0));
    this.target = 0;
    
    this.mode = "third_person_follow";


};

DroneDisplayCameraController.prototype.set_target = function (target) {
    this.target = target;
};

DroneDisplayCameraController.prototype.constructor = DroneDisplayCameraController;
DroneDisplayCameraController.prototype.update = function () {
    let camera = this.camera;
    if (this.target != 0) {
        var relative_position = new THREE.Vector3(3,0,1);
        relative_position.applyQuaternion(this.target.attitude);
        relative_position.add(this.target.position);
        
        camera.position.x = relative_position.x;
        camera.position.y = relative_position.y;
        camera.position.z = relative_position.z;
        
         
        this.camera.lookAt(new THREE.Vector3(
            this.target.position.x,
            this.target.position.y,
            this.target.position.z
            )
        );
        
    }
    else {
        // this.total_time = this.total_time + 0.015;
        camera.position.x = Math.cos(this.total_time / this.rotate_time * 6.28) * this.radius;
        camera.position.y = Math.sin(this.total_time / this.rotate_time * 6.28) * this.radius;
        // console.log(camera);
        //camera.position.z = 2 * Math.cos(total_time/rotate_time/2);
    }
};


//module.exports = DroneDisplayCameraController;