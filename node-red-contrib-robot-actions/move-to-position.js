module.exports = function(RED) {
    function Node(config) {
        RED.nodes.createNode(this, config);
        const node = this;
        node.on('input', function(msg) {
            const flow = node.context().flow;
            const actions = flow.get('actions') || [];
            const action = {"action":"move_to_position","position":{"x_coord":config.x_coord,"y_coord":config.y_coord,"z_coord":config.z_coord},"duration":config.duration, "name": "move_to_position"};
            actions.push(action);
            flow.set('actions', actions);
            msg.payload = actions;
            console.log(actions);
            node.send(msg);
        });
    }
    RED.nodes.registerType("move to position", Node);
};
