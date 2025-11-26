module.exports = function(RED) {
    function Node(config) {
        RED.nodes.createNode(this, config);
        const node = this;
        node.on('input', function(msg) {
            const flow = node.context().flow;
            const actions = flow.get('actions') || [];
            const action = {"action":"grasp", "name": "grasp"};
            actions.push(action);
            flow.set('actions', actions);
            msg.payload = actions;
            console.log(actions);
            node.send(msg);
        });
    }
    RED.nodes.registerType("grasp", Node);
};
