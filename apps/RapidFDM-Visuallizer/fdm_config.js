const electron = require('electron');
const {app} = electron;
const {BrowserWindow} = electron;
const os = require('os');

let win;

function createWindow() {
    let app_path = app.getAppPath();
    let aircraft = process.argv[2];
    let aircraft_path = `${app_path}/../../build/release/aircrafts`;
    win = new BrowserWindow({width: 1024, height: 768});
    global.sharedObj = {aircraftname: aircraft};


    win.on('closed', () => {
        win = null;
    });

    const spawn = require('child_process').spawn;
    var configure_server_path = `${app_path}/../../build/release/bin/rapidfdm_aerodynamics_configure_server`;
    if (os.type() == "Windows_NT")
        configure_server_path = `${app_path}/../../build/bin/Release/rapidfdm_aerodynamics_configure_server`;

    let rconfig = spawn(configure_server_path, [aircraft_path]);
    rconfig.stdout.on('data', (data) => {
        console.log(`stdout: ${data}`);
    });
    win.loadURL(`file://${__dirname}/apps/configurer.html`)
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
