const net = require('net');
const { send } = require('process');

const homing = require('homing');

const crc16 = homing.crc16;

module.exports = function(RED) {
    function MagneticSensorsNode(config) {
        RED.nodes.createNode(this,config);
        var node = this;
        let socket = new net.Socket();
        try {
            socket.connect(5000, "192.168.2.61");
        } catch (error) {
            node.error("Was not able to connect to sec side.",hasError);
        }
        node.status({fill:"red",shape:"ring",text:"disconnected"});

        socket.on("close", (hasError) => {
            node.status({fill:"red",shape:"ring",text:"disconnected"});
            if(hasError) {
                node.error(hasError);
            }
        });
    
        socket.on("connect", () => {
            node.status({fill:"green",shape:"dot",text:"connected"});
        });
    
        socket.on("error", error => {
            node.status({fill:"red",shape:"ring",text:"disconnected"});
            node.error(error);
        });
    
        socket.on("data", (data) => {
            let time = Date.now();
            //data <Buffer 02 00 68 73 09 01 03 18 01 de 01 48 00 0d 5f 80 00>
            let view = new DataView(data.buffer);
            let status = view.getUint8(5);
            let a = view.getUint16(6);
            let b = view.getUint16(8);
            let c = view.getUint16(10);
            let z = view.getUint16(12);
            let sensor = ((status>>0) % 2 != 0);
            let connected = ((status>>1) % 2 != 0);

            let msg = {"payload": {}};
            msg.payload.sensor = sensor;
            msg.payload.connected = connected;
            msg.payload.a = a == 0xFFFF ? -1 : a;
            msg.payload.b = b == 0xFFFF ? -1 : b;
            msg.payload.c = c == 0xFFFF ? -1 : c;
            msg.payload.z = z == 0xFFFF ? -1 : z;
            msg.payload.timestamp = time;

            node.send(msg);
        });
    

        node.on('input', function(msg, send, done) {
            var buffer = new ArrayBuffer(9);
            let view = new DataView(buffer);
            view.setUint8(0, 0x02); // Start of message
            view.setUint8(1, 0x00); // Address
            view.setUint8(2, 'H'.charCodeAt(0)); // Command identification
            view.setUint8(3, 'S'.charCodeAt(0));
            view.setUint8(4, 0x02); // Data Length
            view.setUint8(5, msg.payload ? 0x01 : 0x00);
            view.setUint8(6, msg.payload);
            view.setUint16(7, crc16(new Uint8Array(buffer.slice(0,7))));

            socket.write(new Uint8Array(buffer));
            done();
        });
    }
    RED.nodes.registerType("magnetic-sensors",MagneticSensorsNode);
}