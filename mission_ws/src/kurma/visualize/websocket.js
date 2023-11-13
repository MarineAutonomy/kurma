
let chartOptions = {
    debug: true,
    type: 'line',
    targetElement: 'orientChart',
    // defaultPoint: {        
    //     marker: {
    //       type: 'circle',
    //       size: 11,
    //       outline: { color: 'white', width: 2 }
    //     }
    //   },
    series: [
        {
            name: 'Roll (deg)',
            points: []
        },
        {
            name: 'Pitch (deg)',
            points: []
        },
        {
            name: 'Yaw (deg)',
            points: []
        }
    ],
    axes: {
        x: {
            scale: { type: 'time' }
        },
        y: {
            defaultTick: {
                label: {
                    text: '%value'
                }
            }
        }
    }    
};

var orientChart = JSC.Chart(chartOptions);
var t0 = new Date();
var samplingRate = 1000;

const ws = new WebSocket('ws://localhost:9090');

ws.onopen = function() {
    console.log('Connected to rosbridge_server.');

    // Subscribe to IMU topic
    const imuSubscribeMessage = JSON.stringify({
        op: 'subscribe',
        topic: '/imu_data',
        type: 'sensor_msgs/Imu'
    });

    ws.send(imuSubscribeMessage);
};

ws.onmessage = function(event) {
    const message = JSON.parse(event.data);

    if (message.op === 'publish' && message.topic === '/imu_data') {
        setTimeout(function() {
            const imuData = message.msg;
            const t = Math.round(new Date() - t0)/1000

            // Process IMU data here
            const quaternion = { x: imuData.orientation.x, y: imuData.orientation.y, z: imuData.orientation.z, w: imuData.orientation.w };
            const euler = quaternionToEuler(quaternion);

            // Update chart data here
            if (t >  10) {
                var shift_val = true;
            } else {
                var shift_val = false;
            }
            
            orientChart.series(0).points.add({ x: t, y: euler.roll * 180.0 / Math.PI }, { shift: shift_val });
            orientChart.series(1).points.add({ x: t, y: euler.pitch * 180.0 / Math.PI }, { shift: shift_val });
            orientChart.series(2).points.add({ x: t, y: euler.yaw * 180.0 / Math.PI }, { shift: shift_val });
        }, samplingRate);
    }
};

ws.onerror = function(error) {
    console.error('WebSocket Error: ' + error);
};

function quaternionToEuler(quaternion) {
    var { x, y, z, w } = quaternion;

    const norm = Math.sqrt(x * x + y * y + z * z + w * w);
    var x = x / norm;
    var y = y / norm;
    var z = z / norm;
    var w = w / norm;

    // Roll (x-axis rotation)
    const sinr_cosp = 2 * (w * x + y * z);
    const cosr_cosp = 1 - 2 * (x * x + y * y);
    const roll = Math.atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    const sinp = Math.sqrt(1 + 2 * (w * y - z * x));
    const cosp = Math.sqrt(1 - 2 * (w * y - x * z));
    const pitch = 2 * Math.atan2(sinp, cosp) - Math.PI / 2;

    // Yaw (z-axis rotation)
    const siny_cosp = 2 * (w * z + x * y);
    const cosy_cosp = 1 - 2 * (y * y + z * z);
    const yaw = Math.atan2(siny_cosp, cosy_cosp);

    return { roll, pitch, yaw };
}