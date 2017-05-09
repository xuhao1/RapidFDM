"use strict";
//DroneDisplayCameraController = require("./DroneCameraController.js");

var DroneDisplayEngine = function (container,w,h)
{
    this.camera = new THREE.PerspectiveCamera(60, w / h, 0.01, 500);

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
    renderer.gammaInput = true;
    renderer.gammaOutput = true;
    this.effect = new THREE.OutlineEffect( renderer );
    let reflectionCube = new THREE.CubeTextureLoader()
        .setPath( '../static/textures/cube/skybox/' )
        .load( [ 'pz.jpg', 'nz.jpg', 'px.jpg', 'nx.jpg', 'py.jpg', 'ny.jpg' ] );
    reflectionCube.format = THREE.RGBFormat;
    scene.background = reflectionCube;

    container.innerHTML = "";
    container.appendChild(renderer.domElement);

    let stats = this.stats = new Stats();
    stats.domElement.style.position = 'fixed';
    stats.domElement.style.bottom = '0px';
    stats.domElement.style.left = '0px';
    container.appendChild(stats.domElement);



    this.composer = new THREE.EffectComposer( renderer );
    this.composer.addPass( new THREE.RenderPass( scene, this.camera ) );

    this.outlinePass = new THREE.OutlinePass( new THREE.Vector2(window.innerWidth, window.innerHeight), scene, this.camera);
    this.outlinePass.edgeThickness = 1.0;
    this.outlinePass.edgeStrength = 1.0;
    // this.outlinePass.renderTo
    this.composer.addPass(this.outlinePass);


    var axiscale = 1000;
    var axisHelper = new THREE.AxisHelper(axiscale);
    scene.add(axisHelper);
    this.axisHelpe = axisHelper;
    var size = 1000;
    var step = 10;
    var loader = new THREE.TextureLoader();
    loader.load('../static/textures/grass_texture236.jpg', function ( texture ) {
        var geometry = new THREE.PlaneGeometry( 10000, 10000, 1,1 );
        texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
        texture.repeat.set(1000,1000);
        var material = new THREE.MeshBasicMaterial({map: texture});
        var mesh = new THREE.Mesh(geometry, material);
        mesh.position.z = -1;
        // scene.add(mesh);
    });

    let collada_loader = new THREE.ColladaLoader();

    collada_loader.load(
        '../assets/psg/model.dae',
        function ( collada ) {
            let object = collada.scene;
            object.position.set( -250, -150, -17.9);
            scene.add( object );
        },
        function ( xhr ) {
            console.log( (xhr.loaded / xhr.total * 100) + '% loaded' );
        }
    );
    //
    // collada_loader.load(
    //     '../assets/tileable-informal-city-solid.dae',
    //     function ( collada ) {
    //         let object = collada.scene;
    //         object.position.set( -250, -150, 0);
    //         scene.add( object );
    //     },
    //     function ( xhr ) {
    //         console.log( (xhr.loaded / xhr.total * 100) + '% loaded' );
    //     }
    // );
    //     collada_loader.load(
    //     '../assets/CityV5/model.dae',
    //     function ( collada ) {
    //         let object = collada.scene;
    //         object.position.set( 1350, -600, 0);
    //         scene.add( object );
    //     },
    //     function ( xhr ) {
    //         console.log( (xhr.loaded / xhr.total * 100) + '% loaded' );
    //     }
    // );



};


DroneDisplayEngine.prototype.constructor = DroneDisplayEngine;

DroneDisplayEngine.prototype.render = function () {
    this.light.position.copy( this.camera.position );
    this.controller.update();
    // this.effect.render(this.scene,this.camera);
    this.renderer.render(this.scene,this.camera);
    // this.renderer.autoClear = true;
    // this.renderer.setClearColor( 0xfff0f0 );
    // this.renderer.setClearAlpha( 0.0 );
    // this.composer.render();
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