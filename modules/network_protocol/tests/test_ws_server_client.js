/**
 * Created by xuhao on 16/7/2.
 */

var WebSocket = require('ws')
var ws = new WebSocket("ws://127.0.0.1:9091/query");
ws.on("open", function () {
    ws.send(JSON.stringify({
        "opcode": "query_configurer",
        "data": {
            "opcode": "list_model"
        }
    }));
    ws.send(JSON.stringify({
        "opcode": "query_configurer",
        "data": {
            "opcode": "load_model",
            "name": "sample_aircraft"
        }
    }));
});
ws.on("message", function (msg, flag) {
    console.log(msg);
});
