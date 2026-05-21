const socket = io();
const ros = new ROSLIB.Ros({url: 'ws://localhost:9091'});

const camera2Img = document.getElementById('camera2-image');
const routeImg = document.getElementById('route-image');
const routePlaceholder = document.getElementById('routePlaceholder')
let checkInterval = null;

// Камера 2 всегда активна
camera2Img.src = 'http://localhost:8080/stream?topic=/camera2/image_raw';



function togglePause() {
    socket.emit('toggle_pause');
}

socket.on('pause_status', function(data) {
    const btn = document.getElementById('pauseBtn');
    if (data.paused) {
        btn.textContent = 'Продолжить';
        btn.style.color = '#ffaa00';       
        btn.style.borderColor = '#ffaa00';
    } else {
        btn.textContent = 'Пауза';
        btn.style.color = '#888888';
        btn.style.borderColor = '#888888';
    }
});

// Функция проверки доступности изображения маршрута
function checkRouteImage() {
    const timestamp = Date.now();
    const testImg = new Image();
    testImg.onload = function() {
        routeImg.style.display = 'block';
        routePlaceholder.style.display = 'none';
        routeImg.src = `/newroutes/scan_route_recent.png?t=${timestamp}`;
        if (checkInterval) {
            clearInterval(checkInterval);
            checkInterval = null;
        }
    };
    testImg.onerror = function() {
        routeImg.style.display = 'none';
        routePlaceholder.style.display = 'flex'; 
    };
    testImg.src = `/newroutes/scan_route_recent.png?t=${timestamp}`;
}

// Управление миссией
function startMission() {
    socket.emit('start_mission');
    addLog('Mission started');
    
    routeImg.style.display = 'none';
    routePlaceholder.style.display = 'flex';   
    if (checkInterval) clearInterval(checkInterval);
    checkInterval = setInterval(checkRouteImage, 5000);
    checkRouteImage(); 
}

function emergencyReturn() {
    if(confirm('Confirm emergency return to home position?')) {
        socket.emit('emergency_home');
        addLog('Emergency return initiated');
    }
}

// Логирование
const colors = {
    'INFO': '#00ff00',
    'WARN': '#ffff00',
    'ERROR': '#ff0000',
    'CRITICAL': '#ff00ff'
};

function addLog(message) {
    const logContainer = document.getElementById('logContainer');
    const entry = document.createElement('div');
    entry.className = 'log-entry';
    
    const parts = message.split(']');
    const prefix = parts[0].replace('[', '');
    const content = parts.slice(1).join(']').trim();
    
    entry.innerHTML = `
        <span style="color: ${colors[prefix] || '#ffffff'}">[${prefix}]</span>
        ${content}
    `;
    
    logContainer.appendChild(entry);
    logContainer.scrollTop = logContainer.scrollHeight;
}

// Обработчики событий
socket.on('log', data => addLog(data.data));
socket.on('trajectory', data => console.log('Trajectory updated'));

ros.on('connection', () => addLog('ROS connected'));
ros.on('error', error => addLog(`ROS error: ${error}`));
ros.on('close', () => addLog('ROS connection closed'));

window.addEventListener('load', () => {
    checkRouteImage();
});