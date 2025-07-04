const values = { x: 0, y: 0, z: 0 };
const forceSend = { x: true, y: true, z: true };
const lastSent = { x: 0, y: 0, z: 0 };

const display = document.getElementById('joystick_values');
const statusEl = document.getElementById('ws_indicator');
const batteryEl = document.getElementById('battery_value');
const batteryFill = document.getElementById('battery_level_fill');

function updateStatus(className, text) {
    statusEl.className = className;
    statusEl.textContent = text;
}

function withDeadzoneAndSmoothing(axis, current, threshold = 0.01, smoothing = 0.2) {
    if (forceSend[axis]) {
        lastSent[axis] = current;
        forceSend[axis] = false;
        return current;
    }
    const delta = current - lastSent[axis];
    if (Math.abs(delta) < threshold) return lastSent[axis];
    lastSent[axis] += delta * smoothing;
    return lastSent[axis];
}

function updateDisplay() {
    display.textContent = `X: ${values.x.toFixed(2)}, Y: ${values.y.toFixed(2)}, Z: ${values.z.toFixed(2)}`;
}

// WebSocket setup
let ws = null;
let reconnectDelay = 1000;
let heartbeatInterval = null;
let batteryInterval = null;

function startHeartbeat() {
    if (heartbeatInterval) clearInterval(heartbeatInterval);
    heartbeatInterval = setInterval(() => {
        if (ws && ws.readyState === WebSocket.OPEN) {
            try {
                ws.send(JSON.stringify({ type: "ping" }));
            } catch (e) {
                updateStatus("error", "Ping Failed");
                ws.close();
            }
        }
    }, 3000);
}

function startBatteryPolling() {
    if (batteryInterval) clearInterval(batteryInterval);
    batteryInterval = setInterval(() => {
        if (ws && ws.readyState === WebSocket.OPEN) {
            try {
                ws.send(JSON.stringify({ type: "battery" }));
            } catch (e) {
                console.warn("Battery poll failed", e);
            }
        }
    }, 10000); // Poll every 10 seconds
}

function connectWebSocket() {
    updateStatus("connecting", "Connecting...");
    ws = new WebSocket(`ws://${location.hostname}:80`);

    ws.onopen = () => {
        updateStatus("connected", "Connected");
        startHeartbeat();
        startBatteryPolling();
    };

    ws.onmessage = e => {
        try {
            const msg = JSON.parse(e.data);
            if (msg.type === "pong") {
                // Heartbeat response
            } else if (msg.type === "battery") {
                if (batteryEl) batteryEl.textContent = `${msg.value}%`;
                if (batteryFill) batteryFill.setAttribute("width", (msg.value / 100) * 20);
            }
        } catch (err) {
            console.warn("Non-JSON or unhandled message:", e.data);
        }
    };

    ws.onclose = () => {
        updateStatus("disconnected", "Disconnected");
        clearInterval(heartbeatInterval);
        clearInterval(batteryInterval);
        setTimeout(connectWebSocket, reconnectDelay);
    };

    ws.onerror = () => {
        updateStatus("error", "WebSocket Error");
        ws.close();
    };
}

connectWebSocket();

setInterval(() => {
    const data = { type: "control" };
    for (const axis in values) {
        data[axis] = withDeadzoneAndSmoothing(axis, values[axis]);
    }
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(data));
    }
    updateDisplay();
}, 50);

// Slider setup
document.querySelectorAll('.slider').forEach(slider => {
    const axis = slider.dataset.axis;
    const thumb = slider.querySelector('.slider-thumb');

    const min = parseFloat(slider.dataset.min || "-1");
    const max = parseFloat(slider.dataset.max || "1");
    const step = parseFloat(slider.dataset.step || "0.01");
    const def = parseFloat(slider.dataset.default || "0");
    const reset = slider.dataset.reset?.toLowerCase() === "true";

    values[axis] = def;
    forceSend[axis] = true;
    lastSent[axis] = def;

    const height = slider.clientHeight;
    const valueToPos = v => (1 - (v - min) / (max - min)) * height;
    const posToValue = p => max - (p / height) * (max - min);

    const setValueFromPosition = y => {
        const rect = slider.getBoundingClientRect();
        let offset = Math.max(0, Math.min(y - rect.top, rect.height));
        let value = Math.round(posToValue(offset) / step) * step;
        value = Math.max(min, Math.min(max, value));
        values[axis] = value;
        forceSend[axis] = true;
        thumb.style.top = `${valueToPos(value) - thumb.offsetHeight / 2}px`;
        updateDisplay();
    };

    const setValueDirect = v => {
        values[axis] = v;
        forceSend[axis] = true;
        thumb.style.top = `${valueToPos(v) - thumb.offsetHeight / 2}px`;
        updateDisplay();
    };

    setValueDirect(def);

    let touchId = null;
    slider.addEventListener('touchstart', e => {
        for (const t of e.changedTouches) {
            if (touchId === null) {
                touchId = t.identifier;
                setValueFromPosition(t.clientY);
            }
        }
    });

    slider.addEventListener('touchmove', e => {
        for (const t of e.changedTouches) {
            if (t.identifier === touchId) {
                setValueFromPosition(t.clientY);
                e.preventDefault();
            }
        }
    }, { passive: false });

    slider.addEventListener('touchend', e => {
        for (const t of e.changedTouches) {
            if (t.identifier === touchId) {
                if (reset) setValueDirect(def);
                touchId = null;
            }
        }
    });

    slider.addEventListener('touchcancel', () => {
        if (reset) setValueDirect(def);
        touchId = null;
    });

    let mouseActive = false;
    slider.addEventListener('mousedown', e => {
        mouseActive = true;
        setValueFromPosition(e.clientY);
    });

    window.addEventListener('mousemove', e => {
        if (mouseActive) setValueFromPosition(e.clientY);
    });

    window.addEventListener('mouseup', () => {
        if (mouseActive && reset) setValueDirect(def);
        mouseActive = false;
    });
});
function setupJoystick(id, axisX = true, axisY = false) {
    const container = document.getElementById(id);
    if (!container) return;

    const knob = container.querySelector('.knob');
    let dragging = false;

    function handleMove(clientX, clientY) {
        const rect = container.getBoundingClientRect();
        const cx = rect.left + rect.width / 2;
        const cy = rect.top + rect.height / 2;
        const dx = clientX - cx;
        const dy = clientY - cy;
        const r = rect.width / 2;

        const nx = Math.max(-1, Math.min(1, dx / r));
        const ny = Math.max(-1, Math.min(1, dy / r));

        const tx = axisX ? nx : 0;
        const ty = axisY ? ny : 0;

        // Update logic
        if (id === "left_stick") {
            values.x = tx;
            values.y = -ty;
            forceSend.x = true;
            forceSend.y = true;
        } else if (id === "right_stick") {
            values.z = tx;
            forceSend.z = true;
        }

        // Move knob visually
        if (axisX) knob.style.left = `${(tx + 1) * 50}%`;
        if (axisY) knob.style.top = `${(ty + 1) * 50}%`;

        updateDisplay();
    }

    function endDrag() {
        dragging = false;
        knob.style.left = `50%`;
        if (axisY) knob.style.top = `50%`;

        if (id === "left_stick") {
            values.x = 0;
            values.y = 0;
            forceSend.x = true;
            forceSend.y = true;
        } else if (id === "right_stick") {
            values.z = 0;
            forceSend.z = true;
        }
        updateDisplay();
    }

    function startDrag(e) {
        dragging = true;
        const move = (ev) => {
            if (dragging) {
                const point = ev.touches ? ev.touches[0] : ev;
                handleMove(point.clientX, point.clientY);
            }
        };
        const end = () => {
            endDrag();
            window.removeEventListener("mousemove", move);
            window.removeEventListener("mouseup", end);
            window.removeEventListener("touchmove", move);
            window.removeEventListener("touchend", end);
        };
        window.addEventListener("mousemove", move);
        window.addEventListener("mouseup", end);
        window.addEventListener("touchmove", move, { passive: false });
        window.addEventListener("touchend", end);
    }

    container.addEventListener("mousedown", startDrag);
    container.addEventListener("touchstart", startDrag, { passive: false });
}


setupJoystick("left_stick", true, true);
setupJoystick("right_stick", true, false);
