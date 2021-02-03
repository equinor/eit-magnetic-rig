const net = require('net');
const homing = require('homing');

const crc16 = homing.crc16;

function test() {
    var buffer = new ArrayBuffer(9);
        let view = new DataView(buffer);
        view.setUint8(0, 0x02); // Start of message
        view.setUint8(1, 0x00); // Address
        view.setUint8(2, 0x48); // Command identification
        view.setUint8(3, 0x53);
        view.setUint8(4, 0x02); // Data Length
        view.setUint8(5, 0x01);
        view.setUint8(6, 0x02);
        console.log(crc16(new Uint8Array(buffer.slice(0,7))));

        view.setUint16(7, crc16(new Uint8Array(buffer.slice(0,7))));

        console.log(buffer);
}

test();

function main() {
    let socket = net.createConnection(5000, "192.168.1.253")
    socket.on("close", (hasError) => {
        console.log("close hasError:", hasError);
    });

    socket.on("connect", () => {
        console.log("connected");

        var buffer = new ArrayBuffer(9);
        let view = new DataView(buffer);
        view.setUint8(0, 0x02); // Start of message
        view.setUint8(1, 0x00); // Address
        view.setUint8(2, 0x48); // Command identification
        view.setUint8(3, 0x53);
        view.setUint8(4, 0x02); // Data Length
        view.setUint8(5, 0x01);
        view.setUint8(6, 0x02);
        console.log(crc16(buffer.slice(0,7)));

        let buff2 = buffer.slice(0,7);
        new Uint8Array(buff2).reverse();
        view.setUint16(7, polycrc.crc16(buffer.slice(0,7)));

        console.log(buffer);

        //console.log(socket.write(new Uint8Array(buffer)));
    });

    socket.on("data", (data) => {
        console.log("data", data);
    });

    socket.on("drain", () => {
        console.log("drain");
    });

    socket.on("end", () => {
        console.log("end");
    });

    socket.on("error", (error) => {
        console.log("error: ", error);
    });

    socket.on("lookup", (err, address, family, host) => {
        console.log("lookup: ", err, address, family, host);
    });

    socket.on("timeout", () => {
        console.log("timeout: ");
    });
}


