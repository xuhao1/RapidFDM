/**
 * Created by xuhao on 16/7/7.
 */

function SetTransform(mesh, trans) {
    "use strict";
    var vector = trans.vector;
    var quat = trans.attitude;
    // log(JSON.stringify(trans));
    mesh.position.x = vector[0];
    mesh.position.y = vector[1];
    mesh.position.z = vector[2];
    mesh.quaternion.w = quat[0];
    mesh.quaternion.x = quat[1];
    mesh.quaternion.y = quat[2];
    mesh.quaternion.z = quat[3];
}

function BoxGeometryLoader(box_config, node_json) {
    "use strict";
    var scale = box_config.scale;
    var geometry = new THREE.BoxGeometry(
        scale[0], scale[1], scale[2]
    );
    var material = new THREE.MeshPhongMaterial({
        color: 0x156289,
        emissive: 0x072534,
        side: THREE.DoubleSide,
        shading: THREE.FlatShading
    });
    var cube = new THREE.Mesh(geometry, material);
    SetTransform(cube, node_json.transform);
    return cube;
}

function PropellerGeometryLoader(engine,propeller_config, node_json) {
    "use strict";
    let D = propeller_config.D;
    let geometry = new THREE.TorusGeometry(D / 2, D / 15, 16, 16);

    let  beta = 0.2;
    let alpha = Math.random();
    let gamma = 0.9;

    let diffuseColor = new THREE.Color().setHSL( alpha, 0.5, gamma * 0.5 ).multiplyScalar( 1 - beta * 0.2 );
    let specularColor = new THREE.Color( beta * 0.2, beta * 0.2, beta * 0.2 );
    let specularShininess = Math.pow( 2, alpha * 10 );

    let material = new THREE.MeshToonMaterial( {
        // map: imgTexture,
        // bumpMap: imgTexture,
        bumpScale: 1.0,
        color: diffuseColor,
        specular: specularColor,
        reflectivity: beta,
        shininess: specularShininess,
        shading: THREE.SmoothShading,
        envMap: engine.scene.background,
        side: THREE.DoubleSide
    } );

    var torus = new THREE.Mesh(geometry, material);
    var quaternion = new THREE.Quaternion();
    quaternion.setFromAxisAngle( new THREE.Vector3( 0, 1, 0 ), Math.PI / 2 );
    SetTransform(torus, node_json.transform);
    torus.setRotationFromQuaternion(quaternion.multiply(torus.quaternion));
    node_json.transform.attitude[0] = torus.quaternion.w;
    node_json.transform.attitude[1] = torus.quaternion.x;
    node_json.transform.attitude[2] = torus.quaternion.y;
    node_json.transform.attitude[3] = torus.quaternion.z;

    return torus;

}

function NodeGeometryLoader(node_json,engine,mesh_list) {
    "use strict";
    if (node_json.type == "wing") {
        log("Trying to load wing");
        // console.log(node_json);
        load_wing_from_uiucdb(engine,node_json,function (mesh) {
            // log(`Loaded wing from UIUC ${JSON.stringify(node_json)}`);
            SetTransform(mesh,node_json.transform);
            mesh_list[node_json.name + "_" + node_json.id] = {
                mesh : mesh,
                body_transform : node_json.transform
            };
            engine.outlinePass.selectedObjects.push(mesh);
            engine.addObject(mesh);
        });
    }
    else {
        var geometry = node_json.geometry;
        var mesh = 0;
        switch (geometry.type) {
            case "box":
                log("Trying to load box geometry");
                mesh = BoxGeometryLoader(geometry, node_json);
                break;
            case "propeller":
                log("Trying to load propeller");
                mesh = PropellerGeometryLoader(engine,geometry, node_json);
                break;
            default:
                log(`Unknow geometry ${JSON.stringify(geometry)}`);
                break;
        }

        mesh_list[node_json.name + "_" + node_json.id] = {
            mesh : mesh,
            body_transform : node_json.transform
        };

        engine.outlinePass.selectedObjects.push(mesh);
        console.log(engine.outlinePass);
        engine.addObject(mesh);
    }
}

function LoadAircraftGeometry(aircraft, engine) {
    var component_mesh_list = {};
    NodeGeometryLoader(aircraft,engine,component_mesh_list);
    for (var node_id in aircraft.nodes) {
        var node = aircraft.nodes[node_id];
        NodeGeometryLoader(node,engine,component_mesh_list);
    }
    return component_mesh_list;
}
