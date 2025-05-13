module.exports = function(RED) {
    function ParseStatusNode(config) {
        RED.nodes.createNode(this,config);
        var node = this;
        node.on('input', function(msg, send, done) {
            let time = Date.now();
            let reg = /<(.*)>/g;
            for (const match of msg.payload.matchAll(reg))  {
                let body = match[1].split("|");
                let msg = {payload: {}};
                msg.timestamp = time;
                
                let status = body.shift();
                msg.payload.status = status;
                for (const item of body) {
                    let pair = item.split(":");
                    msg.payload[pair[0]] = pair[1].split(",").map(v => {
                        const num = Number(v);
                        if(isNaN(num)) {
                            return v;
                        }else {
                            return num;
                        }
                    });
                }
                send(msg)
            }

            done();
        });
    }
    RED.nodes.registerType("parse-status",ParseStatusNode);
}