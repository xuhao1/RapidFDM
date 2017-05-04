const electron = require('electron');
const {app} = electron;
const {BrowserWindow} = electron;
const os = require('os');

let win;

function createWindow() {
    let app_path = app.getAppPath();
    let aircraft = process.argv[2];
    let platform = process.argv[3];
    let aircraft_path = `${app_path}/../aircrafts`;
    win = new BrowserWindow({width: 1024, height: 768});
    global.sharedObj = {aircraftname: aircraft};


    win.on('closed', () => {
        win = null;
    });

    const spawn = require('child_process').spawn;

    var configure_server_path = `${app_path}/../bin/rapidfdm_aerodynamics_configure_server`;
    console.log(app.getAppPath());
    console.log(configure_server_path);

    let rconfig = spawn(configure_server_path, [aircraft_path]);
    rconfig.stdout.on('data', (data) => {
        console.log(`stdout: ${data}`);
    });
    console.log("launching simulator");

    var simulator_path = `${app_path}/../bin/rapidfdm_simulator_sitl`;
    switch (platform) {
        case "px4":
            simulator_path = `${app_path}/../bin/rapidfdm_simulator_pixhawk`;
            // r_mavproxy = spawn("/usr/local/bin/mavproxy.py", ["--master=/dev/tty.usbmodem1","--mav20", "--out=udp:127.0.0.1:14555","--mav20"]);
            // r_mavproxy.stdout.on('data', (data) => {
            //     console.log(`stdout: ${data}`);
            // });
            //
            // r_mavproxy.stderr.on('data', (data) => {
            //     console.log(`stdout: ${data}`);
            // });

            break;
        case "a3":
            simulator_path = `${app_path}/../bin/rapidfdm_simulator_dji`;
            break;
        case "sitl":
        default:
            simulator_path = `${app_path}/../bin/rapidfdm_simulator_sitl`;
            break;
    }
    r_sim = spawn(simulator_path, [aircraft_path + "/" + aircraft]);

    r_sim.stdout.on('data', (data) => {
        console.log(`stdout: ${data}`);
    });

    r_sim.stderr.on('data', (data) => {
        console.log(`stdout: ${data}`);
    });

    r_sim.on('close', (code) => {
        console.log(`child process exited with code ${code}`);
    });
    win.loadURL(`file://${__dirname}/apps/gui-test.html`);
}


app.on('ready', createWindow);


app.on('window-all-closed', () => {

    if (process.platform !== 'darwin') {
        app.quit();
    }
});

app.on('activate', () => {

    if (win === null) {
        createWindow();
    }
});
