// HTML Elements for Joystick Controls and Debug Output
const leftJoystick = document.getElementById("leftJoystick");
const leftKnob = document.getElementById("leftKnob");
const rightJoystick = document.getElementById("rightJoystick");
const rightKnob = document.getElementById("rightKnob");
const debugOutput = document.getElementById("debugOutput");

// Centre positions and maximum travel distance for joysticks
const center = { x: 75, y: 75 };
const maxDistance = 60;
let leftJoystickData = { Vx: 0, Vy: 0 };
let rightJoystickData = { omega: 0 };

// Function to add debug messages to the output area
function appendDebugMessage(message) {
    const p = document.createElement("p");
    p.textContent = message;
    debugOutput.appendChild(p);
    debugOutput.scrollTop = debugOutput.scrollHeight; // Auto-scroll to latest message
}

// Initialize WebSocket with reconnection logic
let retryCount = 0;
const maxRetries = 10;

function initWebSocket() {
    const ws = new WebSocket("ws://192.168.1.109/ws");

    ws.onopen = function() {
        appendDebugMessage("WebSocket connection opened successfully.");
        retryCount = 0; // Reset retry count on successful connection
    };

    ws.onclose = function(event) {
        if (retryCount < maxRetries) {
            appendDebugMessage(`WebSocket connection closed. Reconnecting... (${retryCount + 1}/${maxRetries})`);
            retryCount++;
            setTimeout(initWebSocket, 1000); // Retry after 1 second
        } else {
            appendDebugMessage("Maximum reconnection attempts reached.");
        }
    };

    ws.onerror = function(error) {
        console.error("WebSocket error encountered:", error);
        appendDebugMessage("WebSocket error encountered: " + JSON.stringify(error, null, 2));
    };

    ws.onmessage = function(event) {
        appendDebugMessage("Message received from server: " + event.data);
    };

    // Make WebSocket globally accessible
    window.ws = ws;
}

// Start the WebSocket connection
initWebSocket();

// Send joystick data to the server if WebSocket is open
function sendVelocityData() {
    if (window.ws && ws.readyState === WebSocket.OPEN) {
        const data = JSON.stringify({ ...leftJoystickData, ...rightJoystickData });
        appendDebugMessage(`Sending data: Vx=${leftJoystickData.Vx}, Vy=${leftJoystickData.Vy}, omega=${rightJoystickData.omega}`);
        ws.send(data);
    } else {
        appendDebugMessage("WebSocket is not open; cannot send data.");
    }
}

// Update joystick position and velocity data
function updateJoystick(event, knob, joystickData, axis) {
    const rect = knob.parentNode.getBoundingClientRect();
    const x = event.touches[0].clientX - rect.left - center.x;
    const y = event.touches[0].clientY - rect.top - center.y;
    const distance = Math.sqrt(x * x + y * y);

    if (distance < maxDistance) {
        knob.style.transform = `translate(${x}px, ${y}px)`;
        joystickData[axis[0]] = (x / maxDistance);
        joystickData[axis[1]] = (y / maxDistance);
    }
    sendVelocityData();
}

// Reset joystick to centre position and velocity to zero
function resetJoystick(knob, joystickData, axis) {
    knob.style.transform = "translate(-50%, -50%)";
    joystickData[axis[0]] = 0;
    joystickData[axis[1]] = 0;
    sendVelocityData();
}

// Update omega for the right joystick (constrained to x-axis movement)
function updateOmega(event, knob) {
    const rect = knob.parentNode.getBoundingClientRect();
    const x = event.touches[0].clientX - rect.left - center.x; // Horizontal movement relative to center
    const constrainedX = Math.max(-maxDistance, Math.min(x, maxDistance)); // Constrain within maxDistance

    // Update omega based on the horizontal position
    rightJoystickData.omega = (constrainedX / maxDistance) * 0.5;

    // Move the knob horizontally
    knob.style.transform = `translate(${constrainedX}px, -50%)`; // Constrain to x-axis
    sendVelocityData();
}

// Reset the right joystick to the centre position and omega to zero
function resetOmega(knob) {
    knob.style.transform = "translate(-50%, -50%)"; // Centre the knob
    rightJoystickData.omega = 0; // Reset omega
    sendVelocityData();
}

// Event listeners for left joystick (linear movement)
leftJoystick.addEventListener("touchmove", (e) => {
    e.preventDefault(); // Prevent scrolling or other default touch behaviour
    updateJoystick(e, leftKnob, leftJoystickData, ["Vx", "Vy"]);
});
leftJoystick.addEventListener("touchend", () => {
    resetJoystick(leftKnob, leftJoystickData, ["Vx", "Vy"]);
});

// Event listeners for right joystick (rotation)
rightJoystick.addEventListener("touchmove", (e) => {
    e.preventDefault(); // Prevent scrolling or other default touch behaviour
    updateOmega(e, rightKnob);
});
rightJoystick.addEventListener("touchend", () => {
    resetOmega(rightKnob);
});
