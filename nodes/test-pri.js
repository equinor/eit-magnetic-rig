const net = require('net');
const homing = require('homing');

const crc16 = homing.crc16;

function send(ip, message) {
    let socket = net.createConnection(5000, ip)
    socket.on("close", (hasError) => {
        console.log("close hasError:", hasError);
    });

    socket.on("connect", () => {
        console.log("connected");

        console.log(socket.write(new Uint8Array(message), function(e) {
            console.log("sent", e)
        }));
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

function primaryTest() {
    var buffer = new ArrayBuffer(8);
    let view = new DataView(buffer);
    view.setUint8(0, 0x02); // Start of message
    view.setUint8(1, 0x00); // Address
    view.setUint8(2, 'H'.charCodeAt(0)); // Command identification
    view.setUint8(3, 'P'.charCodeAt(0));
    view.setUint8(4, 0x01); // Data Length
    view.setUint8(5, 0x01);
    console.log(buffer.slice(0,6));
    console.log(crc16(new Uint8Array(buffer.slice(0,6))));

    view.setUint16(6, crc16(new Uint8Array(buffer.slice(0,6))));

    console.log(buffer);

    send("192.168.1.253", buffer);
}

function primaryTest2() {
    var buffer = new ArrayBuffer(7);
    let view = new DataView(buffer);
    view.setUint8(0, 0x02); // Start of message
    view.setUint8(1, 0x00); // Address
    view.setUint8(2, 0x52); // Command identification
    view.setUint8(3, 0x49);
    view.setUint8(4, 0x00); // Data Length
    console.log(buffer.slice(0,5));
    console.log(crc16(new Uint8Array(buffer.slice(0,5))));

    view.setUint16(5, crc16(new Uint8Array(buffer.slice(0,5))));

    console.log(buffer);

    send("192.168.1.253", buffer);
}

function secondaryTest() {
    var buffer = new ArrayBuffer(9);
    let view = new DataView(buffer);
    view.setUint8(0, 0x02); // Start of message
    view.setUint8(1, 0x00); // Address
    view.setUint8(2, 'H'.charCodeAt(0)); // Command identification
    view.setUint8(3, 'S'.charCodeAt(0));
    view.setUint8(4, 0x02); // Data Length
    view.setUint8(5, 0x01);
    view.setUint8(6, 0x02);
    console.log(buffer.slice(0,7));
    console.log(crc16(new Uint8Array(buffer.slice(0,7))));

    view.setUint16(7, crc16(new Uint8Array(buffer.slice(0,7))));

    console.log(buffer);

    send("192.168.2.61", buffer);
}

//primaryTest2();
//data <Buffer 02 00 68 73 09 01 03 18 01 de 01 48 00 0d 5f 80 00>
secondaryTest();