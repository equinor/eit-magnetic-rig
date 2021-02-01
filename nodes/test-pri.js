const net = require('net');
const polycrc = require('polycrc');

const crc16 = polycrc.crc(16, 0xa001, 0xffff, 0x0000, false);


function main() {
    let socket = net.createConnection(5000, "192.168.1.253")
    socket.on("close", (hasError) => {
        console.log("close hasError:", hasError);
    });

    socket.on("connect", () => {
        console.log("connected");

        var buffer = new ArrayBuffer(8);
        let view = new DataView(buffer);
        view.setUint8(0, 0x02); // Start of message
        view.setUint8(1, 0x00); // Address
        view.setUint8(2, "H".charCodeAt(0)); // Command identification
        view.setUint8(3, "P".charCodeAt(0));
        view.setUint8(4, 0x01); // Data Length
        view.setUint8(5, 0x00);
        console.log(crc16(buffer.slice(0,6)));
        view.setUint16(6, crc16(buffer.slice(0,6)));

        console.log(buffer);

        console.log(socket.write(new Uint8Array(buffer)));
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

main();
