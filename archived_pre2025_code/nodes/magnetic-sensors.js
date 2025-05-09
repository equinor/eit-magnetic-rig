const net = require('net');

const homing = require('./homing');


module.exports = function(RED) {
    function MagneticSensorsNode(config) {
        RED.nodes.createNode(this,config);
        var node = this;
        homing("192.168.1.254", 2468, (payload) => {
            payload.timestamp = Date.now();
            let msg = {"payload": payload};
            node.send(msg);
        });
    }
    RED.nodes.registerType("magnetic-sensors",MagneticSensorsNode);
}