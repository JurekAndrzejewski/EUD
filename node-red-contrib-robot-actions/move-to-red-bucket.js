module.exports = function(RED) {
    function Node(config) {
        RED.nodes.createNode(this, config);
        const node = this;
        node.on('input', function(msg) {
            const flow = node.context().flow;
            const actions = flow.get('actions') || [];
            const action3 = {"action":"move_to_position","position":{"x_coord": 0,"y_coord": 0.7,"z_coord": 0.85},"duration": 2, "name": "red_middle"};
            actions.push(action3);
            if (actions.length >= 2) {
                const oneToLast = actions[actions.length - 2]; // second last action

                if (oneToLast.action === "move_to_position" && oneToLast.name !== "red_cap" && oneToLast.name !== "green_cap") {
                    const x = parseFloat(oneToLast.position.x_coord);
                    const y = parseFloat(oneToLast.position.y_coord);
                    const z = parseFloat(oneToLast.position.z_coord);
                    const name = oneToLast.name;

                    // New action with z + 0.04
                    const newAction = {
                        action: "move_to_position",
                        position: {
                            x_coord: x.toString(),
                            y_coord: y.toString(),
                            z_coord: (z + 0.04).toString()
                        },
                        duration: oneToLast.duration,
                        name: name
                    };

                    // Push AFTER the last action
                    actions.push(newAction);

                    flow.set("actions", actions);
                }
            }
            const action = {"action":"move_to_position","position":{"x_coord": 0.7,"y_coord": 0.45,"z_coord": 0.85},"duration": 2, "name": "red_bucket"};
            actions.push(action);
            flow.set('actions', actions);
            msg.payload = actions;
            console.log(actions);
            node.send(msg);
        });
    }
    RED.nodes.registerType("move to red bucket", Node);
};
