// WebSocket connection (Socket.IO)
const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
const socket = io({ secure: protocol === 'wss:' });

// State Management
const state = {
    connected: false,
    velocityHistory: [],
    resourcesHistory: [],
    maxHistoryPoints: 20,
    sensors: {},
    logs: []
};

let velocityChart, resourcesChart;

function initCharts() {
    const ctx1 = document.getElementById('velocity-chart');
    const ctx2 = document.getElementById('resources-chart');
    if (!ctx1 || !ctx2) return;

    velocityChart = new Chart(ctx1, {
        type: 'line',
        data: {
            labels: [],
            datasets: [{
                label: 'Linear Velocity (m/s)',
                data: [],
                borderColor: '#0ea5e9',
                backgroundColor: 'rgba(14, 165, 233, 0.1)',
                tension: 0.4,
                fill: true,
                borderWidth: 2
            }]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            plugins: {
                legend: { labels: { color: '#e2e8f0' } }
            },
            scales: {
                y: {
                    ticks: { color: '#94a3b8' },
                    grid: { color: 'rgba(51, 65, 85, 0.2)' }
                },
                x: {
                    ticks: { color: '#94a3b8' },
                    grid: { color: 'rgba(51, 65, 85, 0.2)' }
                }
            }
        }
    });

    resourcesChart = new Chart(ctx2, {
        type: 'line',
        data: {
            labels: [],
            datasets: [
                {
                    label: 'CPU (%)',
                    data: [],
                    borderColor: '#0ea5e9',
                    backgroundColor: 'rgba(14, 165, 233, 0.1)',
                    tension: 0.4,
                    borderWidth: 2
                },
                {
                    label: 'Memory (%)',
                    data: [],
                    borderColor: '#10b981',
                    backgroundColor: 'rgba(16, 185, 129, 0.1)',
                    tension: 0.4,
                    borderWidth: 2
                }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            plugins: {
                legend: { labels: { color: '#e2e8f0' } }
            },
            scales: {
                y: {
                    ticks: { color: '#94a3b8' },
                    grid: { color: 'rgba(51, 65, 85, 0.2)' },
                    max: 100
                },
                x: {
                    ticks: { color: '#94a3b8' },
                    grid: { color: 'rgba(51, 65, 85, 0.2)' }
                }
            }
        }
    });
}

// Socket events
socket.on('connect', () => {
    state.connected = true;
    updateConnectionStatus('Connected', true);
    addLog('✓ WebSocket connected');
});

socket.on('disconnect', () => {
    state.connected = false;
    updateConnectionStatus('Disconnected', false);
    addLog('✗ WebSocket disconnected');
});

socket.on('ros2_topic_update', (data) => {
    updateTopicData(data);
});

socket.on('system_stats', (data) => {
    updateSystemStats(data);
});

socket.on('sensor_data', (data) => {
    updateSensorData(data);
});

socket.on('camera_feed', (data) => {
    updateCameraFeed(data);
});

// Update functions
function updateConnectionStatus(status, isConnected) {
    const badge = document.getElementById('connection-status');
    const dot = badge ? badge.previousElementSibling : null;
    if (badge) {
        badge.textContent = status;
        badge.style.color = isConnected ? '#10b981' : '#ef4444';
        if (dot) {
            dot.className = `pulse-dot ${isConnected ? 'online' : 'offline'}`;
        }
    }
}

function updateTopicData(data) {
    // Odometry
    if (data.odometry) {
        const odom = data.odometry;
        document.getElementById('pos-x').textContent = odom.x?.toFixed(2) || '0.00';
        document.getElementById('pos-y').textContent = odom.y?.toFixed(2) || '0.00';
        document.getElementById('heading').textContent = (odom.theta?.toFixed(1) || 0) + '°';
        document.getElementById('velocity').textContent = odom.velocity?.toFixed(2) || '0.00';

        if (velocityChart && typeof odom.velocity === 'number') {
            state.velocityHistory.push(odom.velocity);
            if (state.velocityHistory.length > state.maxHistoryPoints) {
                state.velocityHistory.shift();
            }
            updateVelocityChart();
        }
    }

    // GPS
    if (data.gps) {
        document.getElementById('gps-lat').textContent = data.gps.latitude?.toFixed(4) || '0.0000';
        document.getElementById('gps-lon').textContent = data.gps.longitude?.toFixed(4) || '0.0000';
    }

    // Simple sensors
    if (data.sensors) {
        document.getElementById('temp').textContent = (data.sensors.temperature || 0).toFixed(1) + '°C';
        document.getElementById('battery').textContent = (data.sensors.battery_voltage || 0).toFixed(2) + 'V';
    }

    updateTopicsList(data);
}

function updateSystemStats(data) {
    document.getElementById('cpu-usage').textContent = Math.round(data.cpu) + '%';
    document.getElementById('memory-usage').textContent = Math.round(data.memory) + '%';
    document.getElementById('cpu-bar').style.width = data.cpu + '%';
    document.getElementById('memory-bar').style.width = data.memory + '%';
    document.getElementById('latency').textContent = data.latency + 'ms';
    document.getElementById('nodes-count').textContent = data.active_nodes || 0;

    if (resourcesChart) {
        state.resourcesHistory.push({ cpu: data.cpu, memory: data.memory });
        if (state.resourcesHistory.length > state.maxHistoryPoints) {
            state.resourcesHistory.shift();
        }
        updateResourcesChart();
    }
}

function updateSensorData(data) {
    state.sensors = data;
    const list = document.getElementById('sensors-list');
    if (!list) return;

    list.innerHTML = Object.entries(data).map(([key, value]) => {
        let status = 'text-cyan-400';
        if (value > 50) status = 'text-yellow-400';
        if (value > 80) status = 'text-red-400';

        return `
            <div class="topic-item">
                <div class="flex justify-between items-center">
                    <span class="text-sm font-semibold">${key.replace(/_/g, ' ').toUpperCase()}</span>
                    <span class="metric-value text-base ${status}">${typeof value === 'number' ? value.toFixed(2) : value}</span>
                </div>
            </div>
        `;
    }).join('');
}

function updateCameraFeed(data) {
    if (data.camera_id === 1) {
        document.getElementById('cam1-fps').textContent = data.fps || 0;
    } else if (data.camera_id === 2) {
        document.getElementById('cam2-fps').textContent = data.fps || 0;
    } else if (data.camera_id === 3) {
        document.getElementById('cam3-fps').textContent = data.fps || 0;
    } else if (data.camera_id === 4) {
        document.getElementById('cam4-fps').textContent = data.fps || 0;
    }
}

function updateTopicsList(data) {
    const list = document.getElementById('topics-list');
    if (!list) return;

    const topics = Object.keys(data).filter(key => key !== 'timestamp');
    list.innerHTML = topics.map(topic => `
        <div class="topic-item" title="${topic}">
            <div class="text-xs truncate">
                <span class="text-gray-400">/</span><span class="text-cyan-400">${topic}</span>
            </div>
        </div>
    `).join('');
}

function updateVelocityChart() {
    if (!velocityChart) return;
    velocityChart.data.labels = Array.from({ length: state.velocityHistory.length }, (_, i) => i.toString());
    velocityChart.data.datasets[0].data = state.velocityHistory;
    velocityChart.update('none');
}

function updateResourcesChart() {
    if (!resourcesChart) return;
    const labels = Array.from({ length: state.resourcesHistory.length }, (_, i) => i.toString());
    resourcesChart.data.labels = labels;
    resourcesChart.data.datasets[0].data = state.resourcesHistory.map(d => d.cpu);
    resourcesChart.data.datasets[1].data = state.resourcesHistory.map(d => d.memory);
    resourcesChart.update('none');
}

function addLog(message) {
    const container = document.getElementById('logs-container');
    if (!container) return;

    const timestamp = new Date().toLocaleTimeString();
    const logElement = document.createElement('div');
    logElement.innerHTML = `<span class="text-gray-500">[${timestamp}]</span> ${message}`;
    container.insertBefore(logElement, container.firstChild);

    while (container.children.length > 100) {
        container.removeChild(container.lastChild);
    }

    state.logs.push(message);
}

// UI helpers
function switchTab(ev, tabName) {
    document.querySelectorAll('.tab-content').forEach(el => el.classList.add('hidden'));
    const tab = document.getElementById(tabName + '-tab');
    if (tab) tab.classList.remove('hidden');

    document.querySelectorAll('.sidebar-item').forEach(el => el.classList.remove('active'));
    if (ev && ev.target) {
        ev.target.classList.add('active');
    }
}

function sendCommand(command) {
    socket.emit('send_command', {
        command: command,
        velocity: parseFloat(document.getElementById('velocity-input').value),
        angular_velocity: parseFloat(document.getElementById('angular-input').value)
    });
    addLog(`📤 Command sent: ${command}`);
}

function toggleSettings() {
    alert('Settings panel coming soon!\n\nYou can configure:\n- Topic subscriptions\n- Update rates\n- Thresholds\n- Data recording');
}

// Sliders
document.addEventListener('DOMContentLoaded', () => {
    const velInput = document.getElementById('velocity-input');
    const angInput = document.getElementById('angular-input');
    if (velInput) {
        velInput.addEventListener('input', (e) => {
            document.getElementById('velocity-display').textContent = e.target.value;
        });
    }
    if (angInput) {
        angInput.addEventListener('input', (e) => {
            document.getElementById('angular-display').textContent = e.target.value;
        });
    }

    // Timestamp
    setInterval(() => {
        const now = new Date();
        const el = document.getElementById('timestamp');
        if (el) el.textContent = now.toLocaleTimeString();
    }, 1000);

    initCharts();
    addLog('🚀 Dashboard initialized');
    addLog('⏳ Waiting for ROS2 connection...');
});
