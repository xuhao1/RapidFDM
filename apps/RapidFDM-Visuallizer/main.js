const electron = require('electron');
const {app} = electron;
const {BrowserWindow} = electron;

let win;

function createWindow() {
    global.sharedObj = {aircraftname : process.argv[2]};
    console.log(process.argv[2]);

    win = new BrowserWindow({width: 1024, height: 768});


    win.on('closed', () => {
        win = null;
    });

    const spawn = require('child_process').spawn;
    let rconfig = spawn('/Users/xuhao/Develop/FixedwingProj/RapidFDM/build/bin/rapidfdm_aerodynamics_configure_server', [process.argv[2]]);
    rconfig.stdout.on('data', (data) => {
        console.log(`stdout: ${data}`);
    });

    if (process.argv[3] == "--enable_dynamics")
    {
        console.log("launching simulator");
        r_sim = spawn('/Users/xuhao/Develop/FixedwingProj/RapidFDM/build/modules/simulation/rapidfdm_simulator_ws', [process.argv[2]]);
        r_sim.stdout.on('data', (data) => {
            console.log(`stdout: ${data}`);
        });

        r_sim.stderr.on('data', (data) => {
            console.log(`stdout: ${data}`);
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
