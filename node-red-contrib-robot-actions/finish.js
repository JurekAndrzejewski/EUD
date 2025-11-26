module.exports = function(RED) {
    function MissionBuilderNode(config) {
        RED.nodes.createNode(this, config);
        const node = this;

        node.on('input', async function(msg) {
            const flow = node.context().flow;
            const actions = flow.get('actions') || [];

            const flowName = config.flowName || "default_flow";

            // Convert position fields to simple x, y, z numbers
            const normalizedActions = actions.map(a => {
                if (a.position) {
                    return {
                        ...a,
                        position: {
                            x: Number(a.position.x_coord ?? a.position.x),
                            y: Number(a.position.y_coord ?? a.position.y),
                            z: Number(a.position.z_coord ?? a.position.z)
                        },
                        duration: a.duration ? Number(a.duration) : undefined
                    };
                }
                return a;
            });

            const mission = { missions: [{ mission_id: flowName, actions: normalizedActions }] };

            console.log("Mission JSON being sent:");
            console.log(JSON.stringify(mission, null, 2));

            try {
                const fetch = require("node-fetch");  // ensure node-fetch is installed
                const res = await fetch("http://localhost:8000/missions", {
                    method: "POST",
                    headers: { "Content-Type": "application/json" },
                    body: JSON.stringify(mission)
                });
                const result = await res.json();
                msg.payload = result;

                // Clear actions after sending
                flow.set('actions', []);
                node.send(msg);
            } catch (err) {
                node.error(err.message, msg);
            }
        });
    }

    RED.nodes.registerType("finish", MissionBuilderNode);
};
