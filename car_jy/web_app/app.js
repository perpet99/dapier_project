// AMR Unified Dashboard Core JavaScript
document.addEventListener('DOMContentLoaded', () => {
    // DOM Elements
    const statusBadge = document.getElementById('statusBadge');
    const statusText = document.getElementById('statusText');
    const aiReplyText = document.getElementById('aiReplyText');
    const waypointsList = document.getElementById('waypointsList');
    const logBox = document.getElementById('logBox');

    const lblX = document.getElementById('lblX');
    const lblY = document.getElementById('lblY');
    const lblYaw = document.getElementById('lblYaw');
    const dispLinear = document.getElementById('dispLinear');
    const dispAngular = document.getElementById('dispAngular');

    const btnModeNav = document.getElementById('btnModeNav');
    const btnModeInit = document.getElementById('btnModeInit');
    const btnRotLeft = document.getElementById('btnRotLeft');
    const btnRotRight = document.getElementById('btnRotRight');
    const btnRot180 = document.getElementById('btnRot180');

    let clickMode = 'nav'; // 'nav' or 'init'
    let maxSpeed = 0.20;
    let maxAngular = 0.60;
    let robotPose = { x: 2.7, y: 2.0, yaw: -2.033 };
    let mapMeta = { resolution: 0.05, origin: [-7.817, -8.913, 0], width: 374, height: 413 };
    let mapImg = new Image();
    let mapLoaded = false;

    // --- 1. Map Canvas Engine ---
    const canvas = document.getElementById('mapCanvas');
    const ctx = canvas.getContext('2d');

    function resizeCanvas() {
        const container = canvas.parentElement;
        if (!container) return;
        canvas.width = container.clientWidth;
        canvas.height = container.clientHeight;
        drawMap();
    }
    window.addEventListener('resize', resizeCanvas);
    setTimeout(resizeCanvas, 150);

    mapImg.src = '/api/map_image?' + Date.now();
    mapImg.onload = () => {
        mapLoaded = true;
        mapMeta.width = mapImg.width;
        mapMeta.height = mapImg.height;
        drawMap();
    };

    const waypoints = [
        { id: 'home', name: '홈 (충전)', x: 2.7, y: 2.0, yaw: -116.5, color: '#4ade80' },
        { id: 'point_a', name: '지점 A', x: 1.0, y: 1.0, yaw: 0.0, color: '#38bdf8' },
        { id: 'point_b', name: '지점 B', x: 0.0, y: 2.0, yaw: 90.0, color: '#f59e0b' },
        { id: 'point_c', name: '지점 C', x: -2.0, y: 3.5, yaw: 0.0, color: '#ec4899' }
    ];

    function worldToPixel(wx, wy) {
        const px = (wx - mapMeta.origin[0]) / mapMeta.resolution;
        const py = mapMeta.height - ((wy - mapMeta.origin[1]) / mapMeta.resolution);
        return { x: px, y: py };
    }

    function pixelToWorld(px, py) {
        const wx = mapMeta.origin[0] + (px * mapMeta.resolution);
        const wy = mapMeta.origin[1] + ((mapMeta.height - py) * mapMeta.resolution);
        return { x: wx, y: wy };
    }

    function drawMap() {
        if (!mapLoaded) return;
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        const scale = Math.min(canvas.width / mapImg.width, canvas.height / mapImg.height) * 0.95;
        const offsetX = (canvas.width - mapImg.width * scale) / 2;
        const offsetY = (canvas.height - mapImg.height * scale) / 2;

        ctx.save();
        ctx.translate(offsetX, offsetY);
        ctx.scale(scale, scale);

        // Map Image
        ctx.drawImage(mapImg, 0, 0);

        // Waypoints Pins
        waypoints.forEach(wp => {
            const pt = worldToPixel(wp.x, wp.y);
            ctx.beginPath();
            ctx.arc(pt.x, pt.y, 7 / scale, 0, 2 * Math.PI);
            ctx.fillStyle = wp.color;
            ctx.fill();
            ctx.strokeStyle = '#ffffff';
            ctx.lineWidth = 1.5 / scale;
            ctx.stroke();

            ctx.fillStyle = '#ffffff';
            ctx.font = `bold ${Math.max(10, 12 / scale)}px sans-serif`;
            ctx.fillText(wp.name, pt.x + 9 / scale, pt.y + 4 / scale);
        });

        // Robot Chassis & Heading
        const rPt = worldToPixel(robotPose.x, robotPose.y);
        ctx.save();
        ctx.translate(rPt.x, rPt.y);
        ctx.rotate(-robotPose.yaw);

        // Chassis
        ctx.beginPath();
        ctx.roundRect(-10 / scale, -8 / scale, 20 / scale, 16 / scale, 4 / scale);
        ctx.fillStyle = '#0284c7';
        ctx.fill();
        ctx.strokeStyle = '#38bdf8';
        ctx.lineWidth = 2.5 / scale;
        ctx.stroke();

        // Bold Heading Arrow
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(24 / scale, 0);
        ctx.strokeStyle = '#f43f5e';
        ctx.lineWidth = 3.5 / scale;
        ctx.stroke();

        ctx.beginPath();
        ctx.moveTo(24 / scale, 0);
        ctx.lineTo(16 / scale, -6 / scale);
        ctx.lineTo(18 / scale, 0);
        ctx.lineTo(16 / scale, 6 / scale);
        ctx.closePath();
        ctx.fillStyle = '#f43f5e';
        ctx.fill();

        ctx.restore(); // Robot transform restored

        // Drag Arrow
        if (isMapDragging && dragStart && dragCurrent) {
            ctx.beginPath();
            ctx.moveTo(dragStart.imgX, dragStart.imgY);
            ctx.lineTo(dragCurrent.imgX, dragCurrent.imgY);
            ctx.strokeStyle = clickMode === 'init' ? '#f59e0b' : '#38bdf8';
            ctx.lineWidth = 4 / scale;
            ctx.stroke();

            const angle = Math.atan2(dragCurrent.imgY - dragStart.imgY, dragCurrent.imgX - dragStart.imgX);
            ctx.beginPath();
            ctx.moveTo(dragCurrent.imgX, dragCurrent.imgY);
            ctx.lineTo(dragCurrent.imgX - (16 / scale) * Math.cos(angle - Math.PI / 6), dragCurrent.imgY - (16 / scale) * Math.sin(angle - Math.PI / 6));
            ctx.lineTo(dragCurrent.imgX - (16 / scale) * Math.cos(angle + Math.PI / 6), dragCurrent.imgY - (16 / scale) * Math.sin(angle + Math.PI / 6));
            ctx.closePath();
            ctx.fillStyle = clickMode === 'init' ? '#f59e0b' : '#38bdf8';
            ctx.fill();
        }

        ctx.restore(); // Map transform restored
    }

    // Interactive Map Pointer Events
    let isMapDragging = false;
    let dragStart = null;
    let dragCurrent = null;

    function getCanvasCoords(e) {
        const rect = canvas.getBoundingClientRect();
        const clickX = e.clientX - rect.left;
        const clickY = e.clientY - rect.top;

        const scale = Math.min(canvas.width / mapImg.width, canvas.height / mapImg.height) * 0.95;
        const offsetX = (canvas.width - mapImg.width * scale) / 2;
        const offsetY = (canvas.height - mapImg.height * scale) / 2;

        const imgX = (clickX - offsetX) / scale;
        const imgY = (clickY - offsetY) / scale;
        return { imgX, imgY, valid: imgX >= 0 && imgX <= mapImg.width && imgY >= 0 && imgY <= mapImg.height };
    }

    canvas.addEventListener('pointerdown', (e) => {
        if (!mapLoaded) return;
        const pt = getCanvasCoords(e);
        if (!pt.valid) return;
        canvas.setPointerCapture(e.pointerId);
        isMapDragging = true;
        dragStart = pt;
        dragCurrent = pt;
        drawMap();
    });

    canvas.addEventListener('pointermove', (e) => {
        if (!isMapDragging) return;
        const pt = getCanvasCoords(e);
        if (pt.valid) {
            dragCurrent = pt;
            drawMap();
        }
    });

    canvas.addEventListener('pointerup', (e) => {
        if (!isMapDragging || !dragStart) return;
        canvas.releasePointerCapture(e.pointerId);
        isMapDragging = false;

        const startWorld = pixelToWorld(dragStart.imgX, dragStart.imgY);
        let targetYaw = robotPose.yaw;

        if (dragCurrent) {
            const dImgX = dragCurrent.imgX - dragStart.imgX;
            const dImgY = dragCurrent.imgY - dragStart.imgY;
            const dist = Math.hypot(dImgX, dImgY);
            if (dist > 6) {
                targetYaw = Math.atan2(-dImgY, dImgX);
            }
        }

        if (clickMode === 'init') {
            sendInitialPose(startWorld.x, startWorld.y, targetYaw);
            setClickMode('nav');
        } else {
            sendGoalPose(startWorld.x, startWorld.y, targetYaw);
        }

        dragStart = null;
        dragCurrent = null;
        drawMap();
    });

    function setClickMode(mode) {
        clickMode = mode;
        btnModeNav.classList.toggle('active', mode === 'nav');
        btnModeInit.classList.toggle('active', mode === 'init');
        addLog('info', mode === 'init' ? '📍 위치 맞추기 모드 활성화 (지도를 드래그하여 로봇 위치와 각도를 지정하세요)' : '🎯 목표 이동 모드 활성화 (지도를 클릭하여 목적지로 이동하세요)');
    }

    btnModeNav.addEventListener('click', () => setClickMode('nav'));
    btnModeInit.addEventListener('click', () => setClickMode('init'));

    function rotateRobotHeading(deltaDeg) {
        const deltaRad = (deltaDeg * Math.PI) / 180.0;
        let newYaw = robotPose.yaw + deltaRad;
        while (newYaw > Math.PI) newYaw -= 2 * Math.PI;
        while (newYaw < -Math.PI) newYaw += 2 * Math.PI;
        sendInitialPose(robotPose.x, robotPose.y, newYaw);
    }

    btnRotLeft.addEventListener('click', () => rotateRobotHeading(-45));
    btnRotRight.addEventListener('click', () => rotateRobotHeading(45));
    btnRot180.addEventListener('click', () => rotateRobotHeading(180));

    // --- 2. Teleop Joystick Engine (Optimized Non-blocking 20Hz Loop) ---
    const joyArea = document.getElementById('joystickArea');
    const joyNipple = document.getElementById('joystickNipple');
    let isJoyDragging = false;
    let joyRect = null;
    let targetLinear = 0.0;
    let targetAngular = 0.0;
    let lastSentLinear = 0.0;
    let lastSentAngular = 0.0;
    let isCmdInFlight = false;

    async function sendCmdImmediate(lin, ang) {
        dispLinear.innerHTML = `${lin.toFixed(2)} <small>m/s</small>`;
        dispAngular.innerHTML = `${ang.toFixed(2)} <small>rad/s</small>`;
        lastSentLinear = lin;
        lastSentAngular = ang;
        try {
            await fetch('/cmd', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ linear: lin, angular: ang })
            });
        } catch (e) {}
    }

    // 20Hz Teleop Dispatcher Loop (50ms interval)
    setInterval(() => {
        const hasChange = Math.abs(targetLinear - lastSentLinear) > 0.01 || Math.abs(targetAngular - lastSentAngular) > 0.01;
        const isMoving = targetLinear !== 0.0 || targetAngular !== 0.0;

        if ((hasChange || isMoving) && !isCmdInFlight) {
            isCmdInFlight = true;
            dispLinear.innerHTML = `${targetLinear.toFixed(2)} <small>m/s</small>`;
            dispAngular.innerHTML = `${targetAngular.toFixed(2)} <small>rad/s</small>`;
            lastSentLinear = targetLinear;
            lastSentAngular = targetAngular;

            fetch('/cmd', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ linear: targetLinear, angular: targetAngular })
            }).catch(() => {}).finally(() => {
                isCmdInFlight = false;
            });
        }
    }, 50);

    function resetJoy() {
        joyNipple.style.transform = 'translate(0px, 0px)';
        targetLinear = 0.0;
        targetAngular = 0.0;
        sendCmdImmediate(0.0, 0.0);
    }

    function handleJoy(e) {
        if (!joyRect) joyRect = joyArea.getBoundingClientRect();
        const clientX = e.touches ? e.touches[0].clientX : e.clientX;
        const clientY = e.touches ? e.touches[0].clientY : e.clientY;
        const centerX = joyRect.left + joyRect.width / 2;
        const centerY = joyRect.top + joyRect.height / 2;

        let dx = clientX - centerX;
        let dy = clientY - centerY;
        const maxR = joyRect.width / 2 - 20;
        const dist = Math.hypot(dx, dy);

        if (dist > maxR) {
            dx = (dx / dist) * maxR;
            dy = (dy / dist) * maxR;
        }

        joyNipple.style.transform = `translate(${dx}px, ${dy}px)`;
        targetLinear = (-dy / maxR) * maxSpeed;
        targetAngular = (-dx / maxR) * maxAngular;
    }

    joyArea.addEventListener('pointerdown', (e) => {
        isJoyDragging = true;
        joyRect = joyArea.getBoundingClientRect();
        joyArea.setPointerCapture(e.pointerId);
        handleJoy(e);
    });

    window.addEventListener('pointermove', (e) => {
        if (isJoyDragging) handleJoy(e);
    });

    window.addEventListener('pointerup', (e) => {
        if (isJoyDragging) {
            isJoyDragging = false;
            joyArea.releasePointerCapture(e.pointerId);
            resetJoy();
        }
    });

    // Speed Selector
    document.querySelectorAll('.btn-spd').forEach(btn => {
        btn.addEventListener('click', () => {
            document.querySelectorAll('.btn-spd').forEach(b => b.classList.remove('active'));
            btn.classList.add('active');
            maxSpeed = parseFloat(btn.dataset.speed);
            addLog('info', `속도 제한 변경: ${maxSpeed} m/s`);
        });
    });

    // Keyboard Controls
    const keyState = { w: false, a: false, s: false, d: false };
    function updateKeyCmd() {
        let lin = 0.0;
        let ang = 0.0;
        if (keyState.w) lin += maxSpeed;
        if (keyState.s) lin -= maxSpeed;
        if (keyState.a) ang += maxAngular;
        if (keyState.d) ang -= maxAngular;
        targetLinear = lin;
        targetAngular = ang;
        if (lin === 0.0 && ang === 0.0) {
            sendCmdImmediate(0.0, 0.0);
        }
    }

    window.addEventListener('keydown', (e) => {
        const k = e.key.toLowerCase();
        let handled = false;
        if (k === 'w' || k === 'arrowup') { keyState.w = true; handled = true; }
        if (k === 's' || k === 'arrowdown') { keyState.s = true; handled = true; }
        if (k === 'a' || k === 'arrowleft') { keyState.a = true; handled = true; }
        if (k === 'd' || k === 'arrowright') { keyState.d = true; handled = true; }
        if (handled) {
            e.preventDefault();
            updateKeyCmd();
        }
    });

    window.addEventListener('keyup', (e) => {
        const k = e.key.toLowerCase();
        let handled = false;
        if (k === 'w' || k === 'arrowup') { keyState.w = false; handled = true; }
        if (k === 's' || k === 'arrowdown') { keyState.s = false; handled = true; }
        if (k === 'a' || k === 'arrowleft') { keyState.a = false; handled = true; }
        if (k === 'd' || k === 'arrowright') { keyState.d = false; handled = true; }
        if (handled) {
            e.preventDefault();
            updateKeyCmd();
        }
    });

    // --- 3. API Communication & Logs ---
    function addLog(type, text) {
        const time = new Date().toLocaleTimeString('ko-KR', { hour12: false });
        const item = document.createElement('div');
        item.className = `log-item ${type}`;
        item.textContent = `[${time}] ${text}`;
        logBox.appendChild(item);
        logBox.scrollTop = logBox.scrollHeight;
    }

    async function sendInitialPose(x, y, yaw) {
        statusBadge.className = 'status-badge processing';
        statusText.textContent = '위치 설정 적용 중...';
        aiReplyText.textContent = `📍 로봇 위치 설정: (${x.toFixed(2)}, ${y.toFixed(2)}, ${(yaw * 180 / Math.PI).toFixed(1)}°)`;
        try {
            await fetch('/api/initial_pose', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ x, y, yaw })
            });
            robotPose = { x, y, yaw };
            drawMap();
            statusBadge.className = 'status-badge';
            statusText.textContent = '온라인 연결됨';
            addLog('success', `초기 위치 보정 완료: (${x.toFixed(2)}, ${y.toFixed(2)})`);
        } catch (e) {
            addLog('error', `초기 위치 설정 실패: ${e}`);
        }
    }

    async function sendGoalPose(x, y, yaw) {
        statusBadge.className = 'status-badge moving';
        statusText.textContent = '🚗 자율주행 진행 중...';
        aiReplyText.textContent = `🎯 목표 지점으로 자율주행: (${x.toFixed(2)}, ${y.toFixed(2)})`;
        try {
            await fetch('/api/goal_pose', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ x, y, yaw })
            });
            addLog('info', `자율주행 목표 전송: (${x.toFixed(2)}, ${y.toFixed(2)})`);
        } catch (e) {
            addLog('error', `목표 전송 실패: ${e}`);
        }
    }

    async function sendMoveWaypoint(targetId, targetName) {
        statusBadge.className = 'status-badge moving';
        statusText.textContent = '🚗 자율주행 진행 중...';
        aiReplyText.textContent = `🚀 [${targetName}] 장소로 자율주행 이동을 시작합니다.`;
        try {
            const resp = await fetch('/api/navigate', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ waypoint: targetId })
            });
            addLog('info', `웨이포인트 이동 시작: ${targetName} (${targetId})`);
        } catch (e) {
            addLog('error', `웨이포인트 이동 요청 오류: ${e}`);
        }
    }

    async function stopRobot() {
        statusBadge.className = 'status-badge failed';
        statusText.textContent = '🛑 긴급 정지 완료';
        aiReplyText.textContent = '🛑 로봇을 즉시 비상 정지시켰습니다.';
        await fetch('/api/stop', { method: 'POST' });
        addLog('warn', '🛑 로봇 비상 정지 명령 전송');
    }

    // Quick Command Buttons
    document.getElementById('quickStopBtn').addEventListener('click', stopRobot);
    document.getElementById('quickHomeBtn').addEventListener('click', () => sendMoveWaypoint('home', '홈 스테이션'));
    document.getElementById('quickPointABtn').addEventListener('click', () => sendMoveWaypoint('point_a', '지점 A'));
    document.getElementById('quickPointBBtn').addEventListener('click', () => sendMoveWaypoint('point_b', '지점 B'));
    document.getElementById('quickPointCBtn').addEventListener('click', () => sendMoveWaypoint('point_c', '지점 C'));
    document.getElementById('quickPatrolBtn').addEventListener('click', async () => {
        aiReplyText.textContent = '🚨 [전체 순찰] 지점 A -> B -> C -> 홈 순찰 미션을 시작합니다.';
        await fetch('/api/navigate', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ waypoint: 'patrol' }) });
        addLog('info', '🚨 전체 순찰 미션 발동');
    });

    document.getElementById('quickStraightBtn').addEventListener('click', async () => {
        aiReplyText.textContent = '🏎️ [테스트] 2초 전진 테스트를 실행합니다.';
        await fetch('/cmd', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ linear: 0.20, angular: 0.0 }) });
        setTimeout(() => fetch('/cmd', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ linear: 0.0, angular: 0.0 }) }), 2000);
        addLog('info', '🏎️ 2초 직진 구동 실행');
    });

    document.getElementById('quickRoundTripBtn').addEventListener('click', async () => {
        aiReplyText.textContent = '🔄 [스킬] 2초 직진 + 180도 회전 복귀 테스트 실행';
        await fetch('/api/action', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ action: 'round_trip' }) });
        addLog('info', '🔄 2초 왕복 테스트 동작 전송');
    });

    // Populate Waypoints List UI
    function renderWaypointsList() {
        waypointsList.innerHTML = '';
        waypoints.forEach(wp => {
            const item = document.createElement('div');
            item.className = 'waypoint-item';
            item.innerHTML = `
                <div class="wp-info">
                    <span class="wp-name" style="color: ${wp.color}; font-weight:700;">📍 ${wp.name}</span>
                    <span class="wp-coords">X: ${wp.x.toFixed(2)}m, Y: ${wp.y.toFixed(2)}m, Yaw: ${wp.yaw}°</span>
                </div>
                <button class="wp-btn">이동 ➔</button>
            `;
            item.addEventListener('click', () => sendMoveWaypoint(wp.id, wp.name));
            waypointsList.appendChild(item);
        });
    }
    renderWaypointsList();

    // Status Polling Loop (10Hz)
    setInterval(() => {
        fetch('/api/status')
            .then(r => r.json())
            .then(data => {
                if (data && data.pose) {
                    robotPose = data.pose;
                    lblX.innerText = robotPose.x.toFixed(2);
                    lblY.innerText = robotPose.y.toFixed(2);
                    lblYaw.innerText = (robotPose.yaw * 180 / Math.PI).toFixed(1);
                    drawMap();
                }
            }).catch(() => {});
    }, 100);
});
