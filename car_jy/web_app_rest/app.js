// AMR Robot Control Web Dashboard (Pure REST API Client)
document.addEventListener('DOMContentLoaded', () => {
    const statusBadge = document.getElementById('statusBadge');
    const statusText = document.getElementById('statusText');
    const waypointsList = document.getElementById('waypointsList');
    const refreshWpBtn = document.getElementById('refreshWpBtn');
    const quickStopBtn = document.getElementById('quickStopBtn');
    const quickHomeBtn = document.getElementById('quickHomeBtn');
    const quickPatrolBtn = document.getElementById('quickPatrolBtn');
    const quickRoundTripBtn = document.getElementById('quickRoundTripBtn');
    const coordsForm = document.getElementById('coordsForm');
    const coordX = document.getElementById('coordX');
    const coordY = document.getElementById('coordY');
    const coordYaw = document.getElementById('coordYaw');
    const logBox = document.getElementById('logBox');
    const clearLogBtn = document.getElementById('clearLogBtn');

    // 1. 실시간 로봇 상태 폴링 (1.5초 주기)
    async function updateRobotStatus() {
        try {
            const resp = await fetch('/api/robot/status');
            if (resp.ok) {
                const data = await resp.json();
                const state = data.state || 'idle';
                const msg = data.message || '온라인 대기 중';

                statusBadge.className = `status-badge ${state}`;
                statusText.textContent = msg;
            }
        } catch (e) {
            statusBadge.className = 'status-badge stopped';
            statusText.textContent = '서버 통신 단절';
        }
    }

    // 2. 웨이포인트 목록 로드
    async function loadWaypoints() {
        waypointsList.innerHTML = '<div class="loading-state">장소 목록을 불러오는 중...</div>';
        try {
            const resp = await fetch('/api/waypoints');
            const data = await resp.json();

            waypointsList.innerHTML = '';
            if (Object.keys(data).length === 0) {
                waypointsList.innerHTML = '<div class="loading-state">등록된 장소가 없습니다.</div>';
                return;
            }

            for (const [wpId, info] of Object.entries(data)) {
                const item = document.createElement('div');
                item.className = 'wp-card';
                item.innerHTML = `
                    <div class="wp-info">
                        <span class="wp-name">📍 ${info.name}</span>
                        <span class="wp-coords">${wpId} (X: ${info.x.toFixed(2)}, Y: ${info.y.toFixed(2)}, Yaw: ${info.yaw || 0}°)</span>
                    </div>
                    <div class="wp-btn">
                        <span>출발</span>
                        <span>➔</span>
                    </div>
                `;
                item.addEventListener('click', () => sendMoveTarget(wpId, info.name));
                waypointsList.appendChild(item);
            }
            addLog('info', `웨이포인트 목록 갱신 완료 (총 ${Object.keys(data).length}개)`);
        } catch (e) {
            waypointsList.innerHTML = '<div class="loading-state" style="color:#ff3366;">웨이포인트 로드 실패</div>';
            addLog('error', `웨이포인트 로드 에러: ${e.message}`);
        }
    }

    // 3. 단일 웨이포인트 이동 명령 전송 (POST /api/robot/move)
    async function sendMoveTarget(wpId, placeName) {
        addLog('info', `🚀 [${placeName}] (${wpId}) 지점으로 이동 명령 전송...`);
        try {
            const resp = await fetch('/api/robot/move', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ target: wpId })
            });
            const data = await resp.json();
            if (resp.ok && data.success) {
                addLog('success', `✅ 주행 시작: ${data.message || placeName}`);
            } else {
                addLog('error', `❌ 주행 실패: ${data.error || '알 수 없는 오류'}`);
            }
        } catch (e) {
            addLog('error', `통신 오류: ${e.message}`);
        }
    }

    // 4. 임의 좌표 주행 명령 전송 (POST /api/robot/move)
    coordsForm.addEventListener('submit', async (e) => {
        e.preventDefault();
        const x = parseFloat(coordX.value);
        const y = parseFloat(coordY.value);
        const yaw = parseFloat(coordYaw.value) || 0.0;

        addLog('info', `📍 임의 좌표 (X: ${x}, Y: ${y}, Yaw: ${yaw}°) 주행 명령 전송...`);
        try {
            const resp = await fetch('/api/robot/move', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ x: x, y: y, yaw: yaw })
            });
            const data = await resp.json();
            if (resp.ok && data.success) {
                addLog('success', `✅ 좌표 주행 시작: ${data.message}`);
            } else {
                addLog('error', `❌ 주행 실패: ${data.error}`);
            }
        } catch (err) {
            addLog('error', `통신 오류: ${err.message}`);
        }
    });

    // 5. 긴급 정지 (POST /api/robot/stop)
    async function sendEmergencyStop() {
        addLog('error', '🛑 긴급 정지 명령 전송!');
        try {
            const resp = await fetch('/api/robot/stop', { method: 'POST' });
            const data = await resp.json();
            if (data.success) {
                addLog('warn', '🛑 로봇 정지 신호가 전송되었습니다.');
            }
        } catch (e) {
            addLog('error', `정지 명령 실패: ${e.message}`);
        }
    }

    // 6. 퀵 액션 버튼 핸들러
    quickStopBtn.addEventListener('click', sendEmergencyStop);
    quickHomeBtn.addEventListener('click', () => sendMoveTarget('home', '홈 스테이션'));
    
    quickPatrolBtn.addEventListener('click', async () => {
        addLog('info', '🔄 전체 순찰 미션 전송...');
        try {
            const resp = await fetch('/api/robot/patrol', { method: 'POST' });
            const data = await resp.json();
            if (data.success) addLog('success', '✅ 전체 순찰 주행을 시작합니다.');
            else addLog('error', `순찰 실패: ${data.error}`);
        } catch (e) {
            addLog('error', `통신 오류: ${e.message}`);
        }
    });

    quickRoundTripBtn.addEventListener('click', async () => {
        addLog('info', '🧪 1초 직진 왕복 테스트 미션 전송...');
        try {
            const resp = await fetch('/api/robot/round_trip', { method: 'POST' });
            const data = await resp.json();
            if (data.success) addLog('success', '✅ 왕복 테스트를 시작합니다.');
            else addLog('error', `테스트 실패: ${data.error}`);
        } catch (e) {
            addLog('error', `통신 오류: ${e.message}`);
        }
    });

    refreshWpBtn.addEventListener('click', loadWaypoints);
    clearLogBtn.addEventListener('click', () => { logBox.innerHTML = ''; });

    function addLog(type, text) {
        const item = document.createElement('div');
        item.className = `log-item ${type}`;
        const timeStr = new Date().toLocaleTimeString('ko-KR', { hour12: false });
        item.textContent = `[${timeStr}] ${text}`;
        logBox.appendChild(item);
        logBox.scrollTop = logBox.scrollHeight;
    }

    // 초기 구동
    loadWaypoints();
    setInterval(updateRobotStatus, 1500);
    addLog('info', 'AMR 웹 관제 대시보드가 준비되었습니다.');
});
