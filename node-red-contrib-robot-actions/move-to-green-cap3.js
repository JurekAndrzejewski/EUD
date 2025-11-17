module.exports = function(RED) {
    function Node(config) {
        RED.nodes.createNode(this, config);
        const node = this;
        node.on('input', function(msg) {
            const flow = node.context().flow;
            const actions = flow.get('actions') || [];
            const action3 = {"action":"move_to_position","position":{"x_coord": 0,"y_coord": 0.7,"z_coord": 0.85},"duration": 2, "name": "middle"};
            actions.push(action3);
            const action = {"action":"move_to_position","position":{"x_coord": 0.4125,"y_coord": -0.069,"z_coord": 0.665},"duration": 2, "name": "green_cap3_1"};
            actions.push(action);
            const action2 = {"action":"move_to_position","position":{"x_coord": 0.4125,"y_coord": -0.069,"z_coord": 0.625},"duration": 1, "name": "green_cap3_2"};
            actions.push(action2);
            flow.set('actions', actions);
            msg.payload = actions;
            console.log(actions);
            node.send(msg);
        });
    }
    RED.nodes.registerType("move to green cap 3", Node);
};
