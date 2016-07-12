"use strict";
//DroneDisplayCameraController = require("./DroneCameraController.js");

var DroneDisplayEngine = function (container,w,h)
{
    this.camera = new THREE.PerspectiveCamera(60, w / h, 0.01, 10000);


    this.controller = new DroneDisplayCameraController(this.camera);

    let scene = this.scene = new THREE.Scene();


    scene.add( new THREE.AmbientLight( 0x222222 ) );
    this.light = new THREE.PointLight( 0xffffff );
    this.light.position.copy( this.camera.position );
    scene.add( this.light );

    let renderer = this.renderer = new THREE.WebGLRenderer({
        antialias:true,
        depth:true
    });
    renderer.setClearColor(0xbfd1e5);
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(w, h);
    container.innerHTML = "";
    container.appendChild(renderer.domElement);

    let stats = this.stats = new Stats();
    stats.domElement.style.position = 'absolute';
    stats.domElement.style.top = '0px';
    container.appendChild(stats.domElement);

    var axiscale = 1000;
    var axisHelper = new THREE.AxisHelper(axiscale);
    scene.add(axisHelper);
    scene.add(axisHelper);
    var size = 1000;
    var step = 10;


};

DroneDisplayEngine.prototype.constructor = DroneDisplayEngine;

DroneDisplayEngine.prototype.render = function () {
    this.light.position.copy( this.camera.position );
    this.controller.update();
    this.renderer.render(this.scene,this.camera);
};

DroneDisplayEngine.animate = function(engine)
{
    requestAnimationFrame(function () {
        DroneDisplayEngine.animate(engine);
    });
    engine.render();
    engine.stats.update();

};

DroneDisplayEngine.prototype.addObject = function(obj)
{
    this.scene.add(obj);
};

window.DroneDisplayEngine = DroneDisplayEngine;
//
//module.exports = DroneDisplayEngine;