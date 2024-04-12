const net = require('net');

module.exports = function homing(ip, port, callback) {
    messageStream(ip, port, (data) => {
        let sensor = parseMessage(data);
        callback(sensor)
    });
}

const ON = Buffer.from([0x01, 0x01, 0x02, 0x01, 0x05, 0x10, 0x03, 0x10, 0x01, 0x01, 0x0C, 0xBD ]);
const OFF = Buffer.from([0x01, 0x01, 0x02, 0x01, 0x05, 0x10, 0x03, 0x10, 0x01, 0x00, 0xCC, 0x7C ]);
const A = Buffer.from([0x01, 0x01, 0x01, 0x01, 0x04, 0x10, 0x03, 0x10, 0x03, 0x7C, 0x9F ]);
const B = Buffer.from([0x01, 0x01, 0x01, 0x01, 0x04, 0x10, 0x03, 0x10, 0x04, 0xBE, 0xDE ]);
const C = Buffer.from([0x01, 0x01, 0x01, 0x01, 0x04, 0x10, 0x03, 0x10, 0x05, 0x7E, 0x1F ]);
const Z = Buffer.from([0x01, 0x01, 0x01, 0x01, 0x04, 0x10, 0x03, 0x10, 0x06, 0x7F, 0x5F ]);

function* requests() {
    yield ON;
    while (true) {
        yield A;
        yield B;
        yield C;
        yield Z;
    }
}

function messageStream(ip, port, callback) {
    let messages = requests();
    let socket = net.createConnection(port, ip)
    socket.on("connect", () => {
        let msg = messages.next().value;
        socket.write(new Uint8Array(msg));
    });

    socket.on("data", (data) => {
        callback(data);
        setTimeout(() => {
            let msg = messages.next().value;
            socket.write(new Uint8Array(msg));
        }, 10);
    });
} 

function parseMessage(buffer) {
    let device = buffer.readUInt8(8);
    let value = buffer.readUInt16BE(9);

    let sensor = "U";
    if (device == 3) sensor = "A";
    if (device == 4) sensor = "B";
    if (device == 5) sensor = "C";
    if (device == 6) sensor = "Z";
    return [sensor,value];
}