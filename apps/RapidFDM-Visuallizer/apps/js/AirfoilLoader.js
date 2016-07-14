"use strict";
let sample_wing_config = {
    b_2: 0.5,
    Mac: 0.2,
    TaperRatio: 0.7,
    MidChordSweep: 0,
    wing_part: 2,
    center_point_chord: 0.3,
    deflectAngle: 8,
    airfoil_name: "naca0015"
};
if (typeof XMLHttpRequest == "undefined") {
    var XMLHttpRequest = require('xhr2');
}

function gen_airfoil_url(airfoil_name) {
    //return `../../data/airfoils/${airfoil_name}.dat`;
    return `http://m-selig.ae.illinois.edu/ads/coord/${airfoil_name}.dat`;
}

function load_airfoil_from_dat(dat_file) {
    let lines = dat_file.split("\n");
    let spline = [];
    lines.shift();
    for (var i in lines) {
        var xypoint = lines[i].split(" ");
        let x = parseFloat(xypoint[0]);
        let y = parseFloat(xypoint[xypoint.length - 1]);
        if (!isNaN(x) && !isNaN(y)) {
            spline.push([x, y]);
            //console.log(spline[spline.length - 1]);
        }

    }
    return spline;
}

function load_wing_from_uiucdb(wing_config, onLoad) {
    console.log("loading airfoil");
    var oReq = new XMLHttpRequest();
    var url = gen_airfoil_url(wing_config.airfoil_name);
    oReq.open("GET", url, true);
    oReq.responseType = "text";
    var obj = this;
    oReq.onload = function (evt) {
        var data = oReq.response;
        console.log(`loaded airfoil ${wing_config.airfoil_name}`);
        onLoad(construct_wing_from_dat(data, wing_config));
    };
    oReq.send(null);
}
/*
 * MODULE
 b_2 = 0.5               //distance from wing root to tip; semi-span
 Mac = 0.5               //Mean Aerodynamic Chord
 nonSideAttach = 0           //0 for canard-like / normal wing pieces, 1 for ctrlsurfaces attached to the back of other wing parts
 TaperRatio = 0.7            //Ratio of tip chord to root chord generally < 1, must be > 0
 MidChordSweep = 25          //Sweep angle in degrees; measured down the center of the span / midchord position
 maxdeflect = 15             //Default maximum deflection value; only used by FARControlableSurface
 controlSurfacePivot = 1, 0, 0;      //Local vector that obj_ctrlSrf pivots about; defaults to 1, 0, 0 (right)
 ctrlSurfFrac = 0.2          //Value from 0-1, percentage of the part that is a flap; only used by FARControlableSurface
 wing_part: 0 control wing on negative part of y-axis, 1 on positive part, 2 both
 center_point_chord : 0.0
 deflectAngle = 15
 */

function construct_wing_geometry_from_data(dat_file, wing_config) {
    let airfoil_spline = load_airfoil_from_dat(dat_file);
    var segments = 10;
    if ("segments" in wing_config)
        segments = wing_config.segments;

    let b_2 = wing_config.b_2;

    if (b_2 === undefined) {
        console.log("b2 undefined");
    }

    var TaperRatio = 1.0;
    if ("TaperRatio" in wing_config)
        TaperRatio = wing_config.TaperRatio;

    var sweep_angle = 0;
    if ("MidChordSweep" in wing_config)
        sweep_angle = wing_config.MidChordSweep * Math.PI / 180.0;

    var wing_part = 2;
    if ("wing_part" in wing_config)
        wing_part = wing_config.wing_part;

    let center_point_chord = wing_config.center_point_chord;

    if (center_point_chord === undefined) {
        console.log("center point chord undefined");
        center_point_chord = 0;
    }

    let Mac = wing_config.Mac;

    if (Mac === undefined) {
        console.log("Mac undefined");
    }

    var deflectAngle = 0;
    if ("deflectAngle" in wing_config)
        deflectAngle = wing_config.deflectAngle * Math.PI / 180.0;


    //root_chord (1+TaperRatio) / 2 = Mac
    let root_chord = Mac * 2 / (1 + TaperRatio);
    let taper_chord = Mac * 2 * TaperRatio / (1 + TaperRatio);
    //offset on X
    let root_offset = -center_point_chord * root_chord;
    // let root_offset = 0;

    var positions, normals, uvs, vertices = [];
    if (wing_part == 1 || wing_part == 0) {
        positions = new THREE.BufferAttribute(new Float32Array(segments * airfoil_spline.length * 3), 3);
        normals = new THREE.BufferAttribute(new Float32Array(segments * airfoil_spline.length * 3), 3);
        uvs = new THREE.BufferAttribute(new Float32Array(segments * airfoil_spline.length * 3), 3);

    }
    else {
        positions = new THREE.BufferAttribute(new Float32Array(segments * airfoil_spline.length * 3 * 2), 3);
        normals = new THREE.BufferAttribute(new Float32Array(segments * airfoil_spline.length * 3 * 2), 3);
        uvs = new THREE.BufferAttribute(new Float32Array(segments * airfoil_spline.length * 3 * 2), 3);
    }
    var index = 0;
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
            var verticesRow = [];
            for (var count = 0; count < airfoil_spline.length; count++) {

                //k * 3 * airfoil_spline.length + i * 3]
                //x cooridnate
                let px = airfoil_spline[count][0] * chord_length + sweep_offset + root_offset;
                //y
                let py = b;
                //z
                let pz = airfoil_spline[count][1] * chord_length + Math.tan(deflectAngle) * Math.abs(b);

                positions.setXYZ(index, px, py, pz);

                //TODO:
                //Use correct normals
                if (count == 0 || count == airfoil_spline.length - 1) {
                    if (count == 0) {
                        let dx = airfoil_spline[count + 1][0] - airfoil_spline[count][0];
                        let dy = airfoil_spline[count + 1][1] - airfoil_spline[count][1];
                        normals.setXYZ(index, dy, 0, -dx);
                    }
                    else {
                        let dx = airfoil_spline[count ][0] - airfoil_spline[count-1][0];
                        let dy = airfoil_spline[count ][1] - airfoil_spline[count-1][1];
                        normals.setXYZ(index, dy, 0, -dx);
                    }
                }
                else {
                    let dx = airfoil_spline[count + 1][0] - airfoil_spline[count - 1][0];
                    let dy = airfoil_spline[count + 1][1] - airfoil_spline[count - 1][1];
                    var normal = new THREE.Vector3(dy, 0, -dx);
                    // normal from ratio and deleft angle

                    normal.normalize();
                    var alpha = Math.atan2((1 - TaperRatio) * (airfoil_spline[count][1] * chord_length) * root_chord
                        + Math.tan(deflectAngle) * Math.abs(b)
                        , b_2);
                    //x cos f - y sin f
                    //y cos f + x sin f
                    let z1 = normal.z * Math.cos(alpha) + normal.y * Math.sin(alpha);
                    let y1 = normal.y * Math.cos(alpha) - normal.z * Math.sin(alpha);
                    normal.z = z1;
                    normal.y = y1;

                    normals.setXYZ(index, normal.x, normal.y, normal.z);
                }

                verticesRow.push(index);
                index++;

            }
            vertices.push(verticesRow);

        }
    }
    if (wing_part == 1 || wing_part == 2) {
        //right side
        for (var k = 0; k < segments; k++) {
            //construct from taper to root
            //b_ratio
            //0----1
            let b_ratio = (k + 1) / segments;
            let chord_length = taper_chord * b_ratio + (1 - b_ratio) * root_chord;
            let b = b_ratio * b_2;
            let sweep_offset = b * Math.tan(sweep_angle) + (center_point_chord) * (root_chord - chord_length);
            var verticesRow = [];
            for (var count = 0; count < airfoil_spline.length; count++) {
                //segments * airfoil_spline.length * 3 + k * 3 * airfoil_spline.length + i * 3]
                //x cooridnate
                let px = airfoil_spline[count][0] * chord_length + sweep_offset + root_offset;
                //y
                let py = b;
                //z
                let pz = airfoil_spline[count][1] * chord_length + Math.tan(deflectAngle) * Math.abs(b);
                positions.setXYZ(index, px, py, pz);

                if (count == 0 || count == airfoil_spline.length - 1) {
                    if (count == 0) {
                        let dx = airfoil_spline[count + 1][0] - airfoil_spline[count][0];
                        let dy = airfoil_spline[count + 1][1] - airfoil_spline[count][1];
                        normals.setXYZ(index, dy, 0, -dx);
                    }
                    else {
                        let dx = airfoil_spline[count ][0] - airfoil_spline[count-1][0];
                        let dy = airfoil_spline[count ][1] - airfoil_spline[count-1][1];
                        normals.setXYZ(index, dy, 0, -dx);
                    }
                }
                else {
                    let dx = airfoil_spline[count + 1][0] - airfoil_spline[count - 1][0];
                    let dy = airfoil_spline[count + 1][1] - airfoil_spline[count - 1][1];
                    var normal = new THREE.Vector3(dy, 0, -dx);
                    // normal from ratio and deleft angle

                    normal.normalize();
                    var alpha = -Math.atan2((1 - TaperRatio) * (airfoil_spline[count][1] * chord_length) * root_chord
                        + Math.tan(deflectAngle) * Math.abs(b)
                        , b_2);
                    //x cos f - y sin f
                    //y cos f + x sin f
                    let z1 = normal.z * Math.cos(alpha) + normal.y * Math.sin(alpha);
                    let y1 = normal.y * Math.cos(alpha) - normal.z * Math.sin(alpha);
                    normal.z = z1;
                    normal.y = y1;

                    normals.setXYZ(index, normal.x, normal.y, normal.z);
                }
                verticesRow.push(index);
                index++;
            }
            vertices.push(verticesRow);
        }
    }
    var indices = [];
    var b_segments = segments;
    if (wing_part == 2) {
        b_segments = 2 * segments;
    }
    //console.log(`position length : ${positions.count}`);

    for (var y = 0; y < b_segments; y++) {

        for (var x = 0; x < airfoil_spline.length; x++) {

            if (y == 0 && x != 0) {

                if (x == airfoil_spline.length - 1)
                    continue;

                //console.log(`v123 x ${x} y ${y}  x   ycoor :${positions.getY(y*airfoil_spline.length + x)}`);
                //console.log(`v123 x ${x} y ${y}  x+1 ycoor :${positions.getY(y*airfoil_spline.length + x + 1)}`);
                //console.log(`v123 x ${airfoil_spline.length - x} y ${y}  l-x ycoor :${positions.getY(y*airfoil_spline.length + airfoil_spline.length - x)}`);

                var v1 = vertices[y][airfoil_spline.length - x];
                var v2 = vertices[y][x];
                var v3 = vertices[y][x + 1];


                indices.push(v1, v2, v3);
            }
            if (y == b_segments - 1) {

                if (x == airfoil_spline.length - 1 || x == 0)
                    continue;

                //console.log(`v123 x ${x} y ${y}  x   ycoor :${positions.getY(y*airfoil_spline.length + x)}`);
                //console.log(`v123 x ${x} y ${y}  x+1 ycoor :${positions.getY(y*airfoil_spline.length + x + 1)}`);
                //console.log(`v123 x ${airfoil_spline.length - x} y ${y}  l-x ycoor :${positions.getY(y*airfoil_spline.length + airfoil_spline.length - x)}`);

                var v1 = vertices[y][airfoil_spline.length - x];
                var v2 = vertices[y][x];
                var v3 = vertices[y][x + 1];
                //var
                indices.push(v3, v2, v1);
                continue;
            }
            if (x == airfoil_spline.length - 1) {
                var v1 = vertices[y][0];
                var v2 = vertices[y][x];
                var v3 = vertices[y + 1][x];
                var v4 = vertices[y + 1][0];
                indices.push(v1, v2, v3);
                indices.push(v1, v3, v4);
                continue;
            }

            var v1 = vertices[y][x + 1];
            var v2 = vertices[y][x];
            var v3 = vertices[y + 1][x];
            var v4 = vertices[y + 1][x + 1];
            indices.push(v1, v2, v3);
            indices.push(v1, v3, v4);


        }

    }

    //console.log(vertices);
    var geometry = new THREE.BufferGeometry();
    geometry.addAttribute('position', positions);
    geometry.addAttribute('normal', normals);
    geometry.setIndex(new THREE.Uint32Attribute(indices, 1));
    // console.log(geometry);
    return geometry;
}

function construct_wing_from_dat(dat_file, wing_config) {
    var pts = [];
    let lines = load_airfoil_from_dat(dat_file);
    for (var i in lines) {
        pts.push(new THREE.Vector2(lines[i][0] * 0.5, lines[i][1] * 0.5));
    }
    var shape = new THREE.Shape(pts);
    var geometry = construct_wing_geometry_from_data(dat_file, wing_config);
    var material = new THREE.MeshPhongMaterial({
        wireframe: false,
        side: THREE.DoubleSide,
        shading: THREE.SmoothShading,
        color: 0x898989
    });
    return new THREE.Mesh(geometry, material);
}
