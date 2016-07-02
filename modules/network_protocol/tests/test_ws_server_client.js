/**
 * Created by xuhao on 16/7/2.
 */

var WebSocket = require('ws')
var ws = new WebSocket("ws://127.0.0.1:9091/channel");
ws.on("open",function () {
   ws.send("hello,world");
});
ws.on("message",function (msg,flag) {
    console.log(msg);
});
