module.exports = function(RED) {
    function ParseStatusNode(config) {
        RED.nodes.createNode(this,config);
        var node = this;
        node.on('input', function(msg, send, done) {

            send([null, msg]);
            done();
        });
    }
    RED.nodes.registerType("parse-status",ParseStatusNode);
}