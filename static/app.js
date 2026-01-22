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
                backgroundColor: 'rgba(14,165,233,0.1)',
                tension: 0.4,
                fill: true,
                borderWidth: 2
            }]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            plugins: { legend: { labels: { color: '#e2e8f0' } } },
            scales: {
                y: { ticks: { color: '#94a3b8' }, grid: { color: 'rgba(51,65,85,0.2)' } },
                x: { ticks: { color: '#94a3b8' }, grid: { color: 'rgba(51,65,85,0.2)' } }
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
                    backgroundColor: 'rgba(14,165,233,0.1)',
                    tension: 0.4,
                    borderWidth: 2
                },
                {
                    label: 'Memory (%)',
                    data: [],
                    borderColor: '#10b981',
                    backgroundColor: 'rgba(16,185,129,0.1)',
                    tension: 0.4,
                    borderWidth: 2
                }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            plugins: { legend: { labels: { color: '#e2e8f0' } } },
            scales: {
                y: {
                    ticks: { color: '#94a3b8' },
                    grid: { color: 'rgba(51,65,85,0.2)' },
                    max: 100
                },
                x: { ticks: { color: '#94a3b8' }, grid: { color: 'rgba(51,65,85,0.2)' } }
            }
        }
    });
}

function updateResourcesChart() {
    if (!resourcesChart) return;

    const labels = state.resourcesHistory.map((_, i) => i);
    resourcesChart.data.labels = labels;
    resourcesChart.data.datasets[0].data = state.resourcesHistory.map(d => d.cpu);
    resourcesChart.data.datasets[1].data = state.resourcesHistory.map(d => d.memory);
    resourcesChart.update('none');
}

function addLog(message) {
    const container = document.getElementById('logs-container');
    if (!container) return;
    const timestamp = new Date().toLocaleTimeString();
    const el = document.createElement('div');
    el.innerHTML = `<span class="text-gray-500">[${timestamp}]</span> ${message}`;
    container.insertBefore(el, container.firstChild);
    while (container.children.length > 100) container.removeChild(container.lastChild);
}

function switchTab(ev, tabName) {
    document.querySelectorAll('.tab-content').forEach(el => el.classList.add('hidden'));
    document.getElementById(tabName + '-tab')?.classList.remove('hidden');
    document.querySelectorAll('.sidebar-item').forEach(el => el.classList.remove('active'));
    ev?.target?.classList.add('active');
}

async function fetchSystemStats() {
    try {
        const r = await fetch('/system_stats');
        const d = await r.json();

        document.getElementById('cpu-usage').textContent = Math.round(d.cpu) + '%';
        document.getElementById('memory-usage').textContent = Math.round(d.memory) + '%';
        document.getElementById('temp').textContent = d.temperature.toFixed(1) + '°C';

        document.getElementById('cpu-bar').style.width = d.cpu + '%';
        document.getElementById('memory-bar').style.width = d.memory + '%';

        state.resourcesHistory.push({
            cpu: d.cpu,
            memory: d.memory
        });

        if (state.resourcesHistory.length > state.maxHistoryPoints) {
            state.resourcesHistory.shift();
        }

        updateResourcesChart();
    } catch (e) {}
}

document.addEventListener('DOMContentLoaded', () => {
    setInterval(() => {
        const el = document.getElementById('timestamp');
        if (el) el.textContent = new Date().toLocaleTimeString();
    }, 1000);

    setInterval(fetchSystemStats, 1000);

    initCharts();
    addLog('🚀 Dashboard initialized');
});
