"use strict";

if (typeof XMLHttpRequest == "undefined") {
    var XMLHttpRequest = require('xhr2');
}

function gen_airfoil_url(airfoil_name) {
    //return `../../data/airfoils/${airfoil_name}.dat`;
    return `http://m-selig.ae.illinois.edu/ads/coord/${airfoil_name}.dat`;
}

function load_airfoil_from_dat(dat_file) {
    let lines = dat_file.split("\n");
    lines.shift();
    for (var i in lines) {
        var xypoint = lines[i].split(" ");
        lines[i] = [parseFloat(xypoint[0]),
            parseFloat(xypoint[xypoint.length - 1])];
    }
    lines.pop();
    return lines;
}

function load_airfoild_from_uiucdb(airfoil_name, onLoad) {
    console.log("loading airfoil");
    var oReq = new XMLHttpRequest();
    var url = gen_airfoil_url(airfoil_name);
    oReq.open("GET", url, true);
    oReq.responseType = "text";
    var obj = this;
    oReq.onload = function (evt) {
        var data = oReq.response;
        console.log(`loaded airfoil ${airfoil_name}`);
        onLoad(construct_wing_from_dat(data));
    };
    oReq.send(null);
}
/*
 * MODULE
 b_2 = 0.5               //distance from wing root to tip; semi-span
 MAC = 0.5               //Mean Aerodynamic Chord
 nonSideAttach = 0           //0 for canard-like / normal wing pieces, 1 for ctrlsurfaces attached to the back of other wing parts
 TaperRatio = 0.7            //Ratio of tip chord to root chord generally < 1, must be > 0
 MidChordSweep = 25          //Sweep angle in degrees; measured down the center of the span / midchord position
 maxdeflect = 15             //Default maximum deflection value; only used by FARControlableSurface
 controlSurfacePivot = 1, 0, 0;      //Local vector that obj_ctrlSrf pivots about; defaults to 1, 0, 0 (right)
 ctrlSurfFrac = 0.2          //Value from 0-1, percentage of the part that is a flap; only used by FARControlableSurface
 wing_part: 0 control wing on negative part of y-axis, 1 on positive part, 2 both
 center_point_chord : 0.3
 */
let sample_wing_config = {
    b_2: 1,
    Mac: 0.5,
    TaperRatio: 0.7,
    MidChordSweep: 25,
    wing_part: 2,
    center_point_chord: 0.3
};
function construct_wing_geometry_from_data(dat_file, wing_config) {
    let lines = load_airfoil_from_dat(dat_file);
    var segments = 10;
    if ("segments" in wing_config)
        segments = wing_config.segments;
    let b_2 = wing_config.b_2;
    let TaperRatio = wing_config.TaperRatio;
    let sweep_angle = wing_config.MidChordSweep * Math.PI / 180.0;
    let wing_part = wing_config.wing_part;
    let center_point_chord = wing_config.center_point_chord;
    let Mac = wing_config.Mac;


    //root_chord (1+TaperRatio) / 2 = Mac
    let root_chord = Mac * 2 / (1 + TaperRatio);
    let taper_chord = Mac * 2 * TaperRatio / (1 + TaperRatio);
    //offset on X
    let root_offset = -center_point_chord * root_chord;

    var vertices;
    if (wing_part == 1 || wing_part == 0)
        vertices = new Float32Array(segments * lines.length * 3);
    else
        vertices = new Float32Array(segments * lines.length * 3 * 2);

    if (wing_part == 0 || wing_part == 2) {
        //left side
        for (var k = 0; k < segments; k++) {
            //construct from taper to root
            // b_ratio
            // 1----0
            let b_ratio = 1 - k / ( segments - 1 );
            let chord_length = taper_chord * b_ratio + (1 - b_ratio) * root_chord;
            let b = -b_ratio * b_2;
            let sweep_offset = b * Math.tan(-sweep_angle) + (center_point_chord) * (root_chord - chord_length);
            for (var i in lines) {
                //x cooridnate
                vertices[k * 3 * lines.length + i * 3] = lines[i][0] * chord_length + sweep_offset + root_offset;
                console.log(lines[i][0] * chord_length + sweep_offset + root_offset);
                //y
                vertices[k * 3 * lines.length + i * 3 + 1] = b;
                //z
                vertices[k * 3 * lines.length + i * 3 + 2] = lines[i][1] * chord_length;
            }
        }
    }
    if (wing_part == 1 || wing_part == 2) {
        //right side
        for (var k = 0; k < segments; k++) {
            //construct from taper to root
            //b_ratio
            //0----1
            let b_ratio = k / ( segments - 1 );
            let chord_length = taper_chord * b_ratio + (1 - b_ratio) * root_chord;
            let b = b_ratio * b_2;
            let sweep_offset = b * Math.tan(sweep_angle) + (center_point_chord) * (root_chord - chord_length);
            for (var i in lines) {
                //x cooridnate
                vertices[segments * lines.length * 3 + k * 3 * lines.length + i * 3] = lines[i][0] * chord_length + sweep_offset + root_offset;
                //y
                vertices[segments * lines.length * 3 + k * 3 * lines.length + i * 3 + 1] = b;
                //z
                vertices[segments * lines.length * 3 + k * 3 * lines.length + i * 3 + 2] = lines[i][1] * chord_length;
            }
        }
    }

    //console.log(vertices);
    var geometry = new THREE.BufferGeometry();
    geometry.addAttribute('position', new THREE.BufferAttribute(vertices, 3));
    console.log(geometry);
    return geometry;
}

function construct_wing_from_dat(dat_file) {
    var pts = [];
    let lines = load_airfoil_from_dat(dat_file);
    for (var i in lines) {
        pts.push(new THREE.Vector2(lines[i][0] * 0.5, lines[i][1] * 0.5));
    }
    var shape = new THREE.Shape(pts);
    var extrudeSettings = {
        steps: 10,
        bevelEnabled: false,
        amount: 1
    };
    //var geometry = new THREE.ExtrudeGeometry(shape, extrudeSettings);
    var geometry = construct_wing_geometry_from_data(dat_file, sample_wing_config);
    var material = new THREE.MeshLambertMaterial({
        color: 0x0000ff,
        wireframe: true
    });
    return new THREE.Mesh(geometry, material);
}
