const electron = require('electron');
const {app} = electron;
const {BrowserWindow} = electron;
const os = require('os');

let win;

function createWindow() {
    let app_path = app.getAppPath();
    let aircraft = process.argv[2];
    let aircraft_path = `${app_path}/../../sample_data/aircrafts`;
    win = new BrowserWindow({width: 1024, height: 768});
    global.sharedObj = {aircraftname : aircraft};
   

    win.on('closed', () => {
        win = null;
    });

    const spawn = require('child_process').spawn;
    var configure_server_path = `${app_path}/../../build/bin/rapidfdm_aerodynamics_configure_server`;
    if(os.type() == "Windows_NT")
        configure_server_path = `${app_path}/../../build/bin/Release/rapidfdm_aerodynamics_configure_server`;
    
    let rconfig = spawn(configure_server_path, [aircraft_path]);
    rconfig.stdout.on('data', (data) => {
        console.log(`stdout: ${data}`);
    });
    if (process.argv[3] == "--enable_dynamics")
    {
        console.log("launching simulator");

        var simulator_path = `${app_path}/../../build/bin/rapidfdm_simulator_ws`;
        if(os.type() == "Windows_NT")
            var simulator_path = `${app_path}/../../build/bin/Release/rapidfdm_simulator_ws`;
        r_sim = spawn(simulator_path, [aircraft_path+"/"+aircraft]);
        
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
    else
    {
        win.loadURL(`file://${__dirname}/apps/configurer.html`)
    }
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
