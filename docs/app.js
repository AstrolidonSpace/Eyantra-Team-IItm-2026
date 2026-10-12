/* ========================================================
   pico_controller_PID.cpp Interactive Simulation & Architecture
   e-Yantra Khojo-Drone Theme (Team KD_6479)
   ======================================================== */

// --- Global Simulation & Controller State ---
const state = {
    // Spatial coordinates in WhyCode arena frame
    setpoint: [1.5, -1.5, 13.0],
    position: [0.0, 0.0, 5.0],
    prev_position: [0.0, 0.0, 5.0],
    velocity: [0.0, 0.0, 0.0],
    attitude: { roll: 0.0, pitch: 0.0 }, // in radians

    // Timing
    dt: 0.033, // 33ms (~30.3 Hz as per line 94)
    lastTime: performance.now(),

    // PID Gains (Mirroring pid_values.yaml & pico_controller_PID.cpp)
    gains: {
        roll: { kp: 66.0, ki: 0.016, kd: 90.0 },
        pitch: { kp: 66.0, ki: 0.016, kd: 90.0 },
        pos_z: { kp: 2.49, ki: 0.024, kd: 1.80 },
        vel_z: { kp: 15.0, ki: 0.0, kd: 0.0 },
    },

    // Integrators and derivative histories
    error: [0.0, 0.0, 0.0],
    prev_error: [0.0, 0.0, 0.0],
    error_sum: [0.0, 0.0], // Roll, Pitch
    pos_z_error_sum: 0.0,
    vel_z_error_sum: 0.0,
    prev_vel_z_error: 0.0,

    windup_guard: 40.0,
    max_vertical_vel: 3.0,

    // Actuator outputs (1000 - 2000 µs)
    rc: {
        roll: 1500,
        pitch: 1500,
        throttle: 1500,
        yaw: 1500,
        aux4: 2000 // Armed
    },

    // Calculated internals for telemetry display
    desired_vel_z: 0.0,
    actual_vel_z: 0.0,
    vel_z_error: 0.0,

    // Error history for live chart
    errorHistory: {
        roll: [],
        pitch: [],
        throttle: [],
        maxPoints: 80
    },

    rotorAngle: 0.0,
    isDraggingSetpoint: false
};

// --- DOM References ---
const flightCanvas = document.getElementById('flightCanvas');
const fCtx = flightCanvas.getContext('2d');
const errorChart = document.getElementById('errorChart');
const eCtx = errorChart.getContext('2d');

// --- Helper Functions ---
function clamp(val, min, max) {
    return Math.max(min, Math.min(max, val));
}

// --- Controller Logic (Mirroring pico_controller_PID.cpp Lines 232–305) ---
function updateControllerPID() {
    const { setpoint, position, prev_position, dt, gains, windup_guard, max_vertical_vel } = state;

    // 1. Position Errors (WhyCode Units)
    // Note coordinate mapping from line 238-240:
    // error[0] = setpoint[1] - drone_position[1]; // roll_error (Y)
    // error[1] = setpoint[0] - drone_position[0]; // pitch_error (X)
    // error[2] = setpoint[2] - drone_position[2]; // throttle_error (Z)
    state.error[0] = setpoint[1] - position[1];
    state.error[1] = setpoint[0] - position[0];
    state.error[2] = setpoint[2] - position[2];

    // Record for live chart
    state.errorHistory.roll.push(state.error[0]);
    state.errorHistory.pitch.push(state.error[1]);
    state.errorHistory.throttle.push(state.error[2]);
    if (state.errorHistory.roll.length > state.errorHistory.maxPoints) {
        state.errorHistory.roll.shift();
        state.errorHistory.pitch.shift();
        state.errorHistory.throttle.shift();
    }

    // 3. Roll (Y) and Pitch (X) Position PID
    const out = [0.0, 0.0];
    const rollPitchGains = [gains.roll, gains.pitch];

    for (let i = 0; i < 2; i++) {
        state.error_sum[i] += state.error[i];
        state.error_sum[i] = clamp(state.error_sum[i], -windup_guard, windup_guard);

        const derivative = state.error[i] - state.prev_error[i];
        out[i] = (rollPitchGains[i].kp * state.error[i]) + 
                 (rollPitchGains[i].ki * state.error_sum[i]) + 
                 (rollPitchGains[i].kd * derivative);
        state.prev_error[i] = state.error[i];
    }

    // ================= CASCADED THROTTLE (Z) =================
    // A. Measure actual vertical velocity (units/sec)
    state.actual_vel_z = (position[2] - prev_position[2]) / dt;
    state.prev_position[2] = position[2];

    // B. OUTER LOOP: Height Error -> Target Velocity
    state.pos_z_error_sum += state.error[2] * dt;
    state.pos_z_error_sum = clamp(state.pos_z_error_sum, -windup_guard, windup_guard);

    const pos_z_derivative = (state.error[2] - state.prev_error[2]) / dt;
    state.prev_error[2] = state.error[2];

    state.desired_vel_z = (gains.pos_z.kp * state.error[2]) + 
                          (gains.pos_z.ki * state.pos_z_error_sum) + 
                          (gains.pos_z.kd * pos_z_derivative);

    // Clamp desired velocity to safe limits
    state.desired_vel_z = clamp(state.desired_vel_z, -max_vertical_vel, max_vertical_vel);

    // C. INNER LOOP (Fixed baseline): Velocity Error -> Throttle PWM
    state.vel_z_error = state.desired_vel_z - state.actual_vel_z;
    state.vel_z_error_sum += state.vel_z_error * dt;
    state.vel_z_error_sum = clamp(state.vel_z_error_sum, -windup_guard, windup_guard);

    const vel_z_derivative = (state.vel_z_error - state.prev_vel_z_error) / dt;
    state.prev_vel_z_error = state.vel_z_error;

    const out_throttle = (gains.vel_z.kp * state.vel_z_error) + 
                         (gains.vel_z.ki * state.vel_z_error_sum) + 
                         (gains.vel_z.kd * vel_z_derivative);

    // 4. Map to Neutral 1500 Midpoint
    const target_roll     = Math.round(1500.0 + out[0]);
    const target_pitch    = Math.round(1500.0 + out[1]);
    const target_throttle = Math.round(1500.0 - out_throttle);

    // 5. Output Clamping & Arm Flag
    state.rc.roll     = clamp(target_roll, 1000, 2000);
    state.rc.pitch    = clamp(target_pitch, 1000, 2000);
    state.rc.throttle = clamp(target_throttle, 1000, 2000);
    state.rc.yaw      = 1500;
    state.rc.aux4     = 2000;
}

// --- Simplified Drone Flight Physics Engine ---
function updatePhysics() {
    const { rc, dt } = state;

    // Convert RC commands into simulated accelerations
    // Roll command deflects Y, Pitch command deflects X
    const rollForce = (rc.roll - 1500) * 0.045;
    const pitchForce = (rc.pitch - 1500) * 0.045;

    // Throttle command: 1500 is hover equilibrium, >1500 climbs, <1500 descends
    // Recall: WhyCode Z axis increases downwards or upwards in arena
    const thrust = (1500 - rc.throttle) * 0.065;

    // Drag coefficients
    const dragX = -state.velocity[0] * 1.8;
    const dragY = -state.velocity[1] * 1.8;
    const dragZ = -state.velocity[2] * 2.2;

    // Accelerations
    const ax = pitchForce + dragX;
    const ay = rollForce + dragY;
    const az = thrust + dragZ;

    // Numerical integration (Euler)
    state.velocity[0] += ax * dt;
    state.velocity[1] += ay * dt;
    state.velocity[2] += az * dt;

    state.position[0] += state.velocity[0] * dt;
    state.position[1] += state.velocity[1] * dt;
    state.position[2] += state.velocity[2] * dt;

    // Hard bounds on arena floor/ceiling
    state.position[2] = clamp(state.position[2], 0.0, 25.0);

    // Visual tilt based on velocity and forces
    state.attitude.roll = clamp(state.velocity[1] * 0.08, -0.4, 0.4);
    state.attitude.pitch = clamp(-state.velocity[0] * 0.08, -0.4, 0.4);

    state.rotorAngle += 0.45;
}

// --- Coordinate Projection (3D Arena to 2D Canvas) ---
function project(x, y, z) {
    const originX = flightCanvas.width * 0.5;
    const originY = flightCanvas.height * 0.65;
    const scaleX = 52;
    const scaleY = 32;
    const scaleZ = 12;

    // Isometric projection
    const screenX = originX + (x - y) * scaleX * 0.707;
    const screenY = originY + (x + y) * scaleY * 0.408 - (z * scaleZ);
    return { x: screenX, y: screenY };
}

// --- Render Flight Arena ---
function drawArena() {
    fCtx.clearRect(0, 0, flightCanvas.width, flightCanvas.height);

    // Subtle arena background glow
    const bgGrad = fCtx.createRadialGradient(
        flightCanvas.width / 2, flightCanvas.height / 2, 50,
        flightCanvas.width / 2, flightCanvas.height / 2, 350
    );
    bgGrad.addColorStop(0, 'rgba(0, 229, 255, 0.04)');
    bgGrad.addColorStop(1, 'rgba(0, 0, 0, 0)');
    fCtx.fillStyle = bgGrad;
    fCtx.fillRect(0, 0, flightCanvas.width, flightCanvas.height);

    // Grid Floor
    fCtx.strokeStyle = 'rgba(255, 255, 255, 0.06)';
    fCtx.lineWidth = 1;

    const gridSize = 4;
    for (let i = -gridSize; i <= gridSize; i++) {
        const p1 = project(i, -gridSize, 0);
        const p2 = project(i, gridSize, 0);
        fCtx.beginPath();
        fCtx.moveTo(p1.x, p1.y);
        fCtx.lineTo(p2.x, p2.y);
        fCtx.stroke();

        const p3 = project(-gridSize, i, 0);
        const p4 = project(gridSize, i, 0);
        fCtx.beginPath();
        fCtx.moveTo(p3.x, p3.y);
        fCtx.lineTo(p4.x, p4.y);
        fCtx.stroke();
    }

    // Origin Axes
    const o = project(0, 0, 0);
    const axX = project(1.5, 0, 0);
    const axY = project(0, 1.5, 0);
    const axZ = project(0, 0, 4);

    fCtx.lineWidth = 2;
    // X Axis (Pitch, Cyan)
    fCtx.strokeStyle = 'rgba(0, 229, 255, 0.6)';
    fCtx.beginPath(); fCtx.moveTo(o.x, o.y); fCtx.lineTo(axX.x, axX.y); fCtx.stroke();

    // Y Axis (Roll, Purple)
    fCtx.strokeStyle = 'rgba(139, 92, 246, 0.6)';
    fCtx.beginPath(); fCtx.moveTo(o.x, o.y); fCtx.lineTo(axY.x, axY.y); fCtx.stroke();

    // Z Axis (Altitude, Emerald)
    fCtx.strokeStyle = 'rgba(16, 185, 129, 0.6)';
    fCtx.beginPath(); fCtx.moveTo(o.x, o.y); fCtx.lineTo(axZ.x, axZ.y); fCtx.stroke();

    // 1. Draw Target Waypoint
    const spGround = project(state.setpoint[0], state.setpoint[1], 0);
    const sp3D = project(state.setpoint[0], state.setpoint[1], state.setpoint[2]);

    // Ground projection pillar
    fCtx.strokeStyle = 'rgba(0, 229, 255, 0.25)';
    fCtx.setLineDash([4, 4]);
    fCtx.beginPath();
    fCtx.moveTo(spGround.x, spGround.y);
    fCtx.lineTo(sp3D.x, sp3D.y);
    fCtx.stroke();
    fCtx.setLineDash([]);

    // Waypoint ground ring
    fCtx.strokeStyle = 'rgba(0, 229, 255, 0.4)';
    fCtx.beginPath();
    fCtx.ellipse(spGround.x, spGround.y, 16, 8, 0, 0, Math.PI * 2);
    fCtx.stroke();

    // Pulsing diamond target
    const pulse = Math.sin(Date.now() * 0.005) * 3;
    fCtx.fillStyle = 'rgba(0, 229, 255, 0.9)';
    fCtx.shadowColor = '#00e5ff';
    fCtx.shadowBlur = 12;
    fCtx.beginPath();
    fCtx.moveTo(sp3D.x, sp3D.y - 12 - pulse);
    fCtx.lineTo(sp3D.x + 9 + pulse, sp3D.y);
    fCtx.lineTo(sp3D.x, sp3D.y + 12 + pulse);
    fCtx.lineTo(sp3D.x - 9 - pulse, sp3D.y);
    fCtx.closePath();
    fCtx.fill();
    fCtx.shadowBlur = 0;

    // 2. Draw Drone Shadow on Floor
    const drGround = project(state.position[0], state.position[1], 0);
    fCtx.fillStyle = 'rgba(0, 0, 0, 0.5)';
    fCtx.beginPath();
    fCtx.ellipse(drGround.x, drGround.y, 22, 11, 0, 0, Math.PI * 2);
    fCtx.fill();

    // 3. Draw Drone Body & Propellers
    const dr3D = project(state.position[0], state.position[1], state.position[2]);

    // Altitude tether line
    fCtx.strokeStyle = 'rgba(16, 185, 129, 0.35)';
    fCtx.setLineDash([2, 3]);
    fCtx.beginPath();
    fCtx.moveTo(drGround.x, drGround.y);
    fCtx.lineTo(dr3D.x, dr3D.y);
    fCtx.stroke();
    fCtx.setLineDash([]);

    fCtx.save();
    fCtx.translate(dr3D.x, dr3D.y);
    fCtx.rotate(state.attitude.roll);

    // Quadrotor cross arms
    fCtx.strokeStyle = '#94a3b8';
    fCtx.lineWidth = 3.5;
    fCtx.beginPath();
    fCtx.moveTo(-28, -12); fCtx.lineTo(28, 12);
    fCtx.moveTo(-28, 12);  fCtx.lineTo(28, -12);
    fCtx.stroke();

    // Drone central chassis
    fCtx.fillStyle = '#0f172a';
    fCtx.strokeStyle = '#00e5ff';
    fCtx.lineWidth = 1.5;
    fCtx.beginPath();
    fCtx.arc(0, 0, 11, 0, Math.PI * 2);
    fCtx.fill();
    fCtx.stroke();

    // WhyCode Marker on top of drone
    fCtx.fillStyle = '#ffffff';
    fCtx.beginPath();
    fCtx.arc(0, 0, 6, 0, Math.PI * 2);
    fCtx.fill();
    fCtx.fillStyle = '#000000';
    fCtx.beginPath();
    fCtx.arc(0, 0, 2.5, 0, Math.PI * 2);
    fCtx.fill();

    // Spinning rotors
    const rotorOffsets = [
        { x: -28, y: -12 }, { x: 28, y: 12 },
        { x: -28, y: 12 },  { x: 28, y: -12 }
    ];

    rotorOffsets.forEach((pos, idx) => {
        fCtx.save();
        fCtx.translate(pos.x, pos.y);
        fCtx.rotate(state.rotorAngle * (idx % 2 === 0 ? 1 : -1));

        fCtx.strokeStyle = 'rgba(0, 229, 255, 0.65)';
        fCtx.lineWidth = 2;
        fCtx.beginPath();
        fCtx.ellipse(0, 0, 12, 4, 0, 0, Math.PI * 2);
        fCtx.stroke();

        fCtx.restore();
    });

    fCtx.restore();
}

// --- Render Live /pos_error Chart ---
function drawErrorChart() {
    eCtx.clearRect(0, 0, errorChart.width, errorChart.height);

    const w = errorChart.width;
    const h = errorChart.height;
    const midY = h / 2;

    // Grid & Zero Line
    eCtx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
    eCtx.lineWidth = 1;
    eCtx.beginPath();
    eCtx.moveTo(0, midY); eCtx.lineTo(w, midY);
    eCtx.stroke();

    const drawLine = (data, color) => {
        if (data.length < 2) return;
        eCtx.strokeStyle = color;
        eCtx.lineWidth = 1.8;
        eCtx.beginPath();

        const step = w / (state.errorHistory.maxPoints - 1);
        const scale = 8.5; // error scaling factor

        data.forEach((val, i) => {
            const x = i * step;
            const y = clamp(midY - (val * scale), 4, h - 4);
            if (i === 0) eCtx.moveTo(x, y);
            else eCtx.lineTo(x, y);
        });

        eCtx.stroke();
    };

    drawLine(state.errorHistory.roll, '#00e5ff');     // Cyan (Roll)
    drawLine(state.errorHistory.pitch, '#8b5cf6');    // Purple (Pitch)
    drawLine(state.errorHistory.throttle, '#10b981'); // Emerald (Altitude)
}

// --- Telemetry UI Update ---
function updateTelemetryUI() {
    // RC Meters (1000 - 2000 µs mapped to 0% - 100%)
    const rollPct = ((state.rc.roll - 1000) / 1000) * 100;
    const pitchPct = ((state.rc.pitch - 1000) / 1000) * 100;
    const throttlePct = ((state.rc.throttle - 1000) / 1000) * 100;

    document.getElementById('val-rc-roll').textContent = `${state.rc.roll} µs`;
    document.getElementById('bar-rc-roll').style.width = `${rollPct}%`;

    document.getElementById('val-rc-pitch').textContent = `${state.rc.pitch} µs`;
    document.getElementById('bar-rc-pitch').style.width = `${pitchPct}%`;

    document.getElementById('val-rc-throttle').textContent = `${state.rc.throttle} µs`;
    document.getElementById('bar-rc-throttle').style.width = `${throttlePct}%`;

    // Numerical readouts
    document.getElementById('tel-des-vel').textContent = `${state.desired_vel_z.toFixed(2)} m/s`;
    document.getElementById('tel-act-vel').textContent = `${state.actual_vel_z.toFixed(2)} m/s`;
    document.getElementById('tel-vel-err').textContent = `${state.vel_z_error.toFixed(2)} m/s`;

    // Overlay tags on canvas
    document.getElementById('overlay-pos').textContent = 
        `Drone: (${state.position[0].toFixed(2)}, ${state.position[1].toFixed(2)}, ${state.position[2].toFixed(2)})`;
    document.getElementById('overlay-sp').textContent = 
        `Target: (${state.setpoint[0].toFixed(2)}, ${state.setpoint[1].toFixed(2)}, ${state.setpoint[2].toFixed(2)})`;
}

// --- Main Simulation Loop (Runs at 60 FPS requestAnimationFrame) ---
function simLoop(time) {
    // Controller loop runs at 33ms step
    updateControllerPID();
    updatePhysics();
    drawArena();
    drawErrorChart();
    updateTelemetryUI();

    requestAnimationFrame(simLoop);
}

// --- Canvas Interactive Click / Drag Setpoint ---
flightCanvas.addEventListener('mousedown', (e) => {
    state.isDraggingSetpoint = true;
    handleCanvasClick(e);
});

window.addEventListener('mouseup', () => {
    state.isDraggingSetpoint = false;
});

flightCanvas.addEventListener('mousemove', (e) => {
    if (state.isDraggingSetpoint) {
        handleCanvasClick(e);
    }
});

function handleCanvasClick(e) {
    const rect = flightCanvas.getBoundingClientRect();
    const clickX = e.clientX - rect.left;
    const clickY = e.clientY - rect.top;

    // Inverse approx for setting (x, y) target
    const originX = flightCanvas.width * 0.5;
    const originY = flightCanvas.height * 0.65;
    const dx = clickX - originX;
    const dy = (clickY - originY) + (state.setpoint[2] * 12);

    const x = (dx / 36.7 + dy / 13.0) / 2.0;
    const y = (-dx / 36.7 + dy / 13.0) / 2.0;

    state.setpoint[0] = clamp(parseFloat(x.toFixed(1)), -3.5, 3.5);
    state.setpoint[1] = clamp(parseFloat(y.toFixed(1)), -3.5, 3.5);

    // Deselect preset buttons
    document.querySelectorAll('.btn-preset').forEach(b => b.classList.remove('active'));
}

// --- Presets & Disturbances ---
document.getElementById('preset-default').addEventListener('click', function() {
    state.setpoint = [1.5, -1.5, 13.0];
    setActivePreset(this);
});

document.getElementById('preset-hover').addEventListener('click', function() {
    state.setpoint = [0.0, 0.0, 10.0];
    setActivePreset(this);
});

document.getElementById('preset-step').addEventListener('click', function() {
    state.setpoint = [-2.0, 2.0, 16.0];
    setActivePreset(this);
});

document.getElementById('btn-disturb').addEventListener('click', function() {
    // Apply sudden velocity impulse (wind gust)
    state.velocity[0] += (Math.random() - 0.5) * 3.5;
    state.velocity[1] += (Math.random() - 0.5) * 3.5;
    state.velocity[2] += (Math.random() - 0.5) * 4.0;
});

function setActivePreset(btn) {
    document.querySelectorAll('.btn-preset').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
}

// --- Gain Sliders Event Listeners ---
function bindSlider(sliderId, labelId, obj, key, factor = 1) {
    const slider = document.getElementById(sliderId);
    const label = document.getElementById(labelId);
    slider.addEventListener('input', (e) => {
        const val = parseFloat(e.target.value) * factor;
        obj[key] = val;
        label.textContent = val.toFixed(slider.step.includes('.') ? slider.step.split('.')[1].length : 0);
    });
}

bindSlider('slider-kp-roll', 'lbl-kp-roll', state.gains.roll, 'kp');
bindSlider('slider-ki-roll', 'lbl-ki-roll', state.gains.roll, 'ki');
bindSlider('slider-kd-roll', 'lbl-kd-roll', state.gains.roll, 'kd');

bindSlider('slider-kp-pitch', 'lbl-kp-pitch', state.gains.pitch, 'kp');
bindSlider('slider-ki-pitch', 'lbl-ki-pitch', state.gains.pitch, 'ki');
bindSlider('slider-kd-pitch', 'lbl-kd-pitch', state.gains.pitch, 'kd');

bindSlider('slider-kp-posz', 'lbl-kp-posz', state.gains.pos_z, 'kp');
bindSlider('slider-ki-posz', 'lbl-ki-posz', state.gains.pos_z, 'ki');
bindSlider('slider-kd-posz', 'lbl-kd-posz', state.gains.pos_z, 'kd');

document.getElementById('btn-reset-gains').addEventListener('click', () => {
    state.gains.roll = { kp: 66.0, ki: 0.016, kd: 90.0 };
    state.gains.pitch = { kp: 66.0, ki: 0.016, kd: 90.0 };
    state.gains.pos_z = { kp: 2.49, ki: 0.024, kd: 1.80 };

    document.getElementById('slider-kp-roll').value = 66;
    document.getElementById('lbl-kp-roll').textContent = "66.0";
    document.getElementById('slider-ki-roll').value = 0.016;
    document.getElementById('lbl-ki-roll').textContent = "0.016";
    document.getElementById('slider-kd-roll').value = 90;
    document.getElementById('lbl-kd-roll').textContent = "90.0";

    document.getElementById('slider-kp-pitch').value = 66;
    document.getElementById('lbl-kp-pitch').textContent = "66.0";
    document.getElementById('slider-ki-pitch').value = 0.016;
    document.getElementById('lbl-ki-pitch').textContent = "0.016";
    document.getElementById('slider-kd-pitch').value = 90;
    document.getElementById('lbl-kd-pitch').textContent = "90.0";

    document.getElementById('slider-kp-posz').value = 2.49;
    document.getElementById('lbl-kp-posz').textContent = "2.49";
    document.getElementById('slider-ki-posz').value = 0.024;
    document.getElementById('lbl-ki-posz').textContent = "0.024";
    document.getElementById('slider-kd-posz').value = 1.80;
    document.getElementById('lbl-kd-posz').textContent = "1.80";
});

// --- Pipeline Flow Detail Cards Content ---
const pipelineDetails = {
    'pipe-whycode': {
        title: "Stage 01: WhyCode Perception & Pose Extraction",
        badge: "Lines 201–208",
        desc: "Overhead global camera tracks the WhyCode circular marker pattern affixed to the top plate of the Swift Pico quadrotor. Marker coordinates arrive asynchronously via <code>/whycode_node/markers</code> and update the 3D position vector.",
        formula: "drone_position[0] = msg.markers[0].position.position.x;\ndrone_position[1] = msg.markers[0].position.position.y;\ndrone_position[2] = msg.markers[0].position.position.z;",
        notes: [
            "QoS depth is set to 1 to ensure zero queue lag and prevent processing stale coordinates.",
            "If the marker is briefly occluded, the node preserves previous position to maintain stability."
        ]
    },
    'pipe-error': {
        title: "Stage 02: Tracking Error Computation & Telemetry Publishing",
        badge: "Lines 237–246",
        desc: "Computes the signed spatial error between the target setpoint and the actual drone position. Publishes the error vector to <code>/pos_error</code> at 30.3 Hz for live tuning visualizers and evaluation rosbags.",
        formula: "error[0] = setpoint[1] - drone_position[1]; // roll_error (Y-axis)\nerror[1] = setpoint[0] - drone_position[0]; // pitch_error (X-axis)\nerror[2] = setpoint[2] - drone_position[2]; // throttle_error (Z-axis)",
        notes: [
            "Crucial axis mapping: WhyCode X controls Pitch, while WhyCode Y controls Roll.",
            "Published continuously to /pos_error so evaluation tools can measure steady-state error."
        ]
    },
    'pipe-lateral': {
        title: "Stage 03: Lateral Roll & Pitch Independent Position PID",
        badge: "Lines 248–258",
        desc: "Computes attitude corrections for Roll and Pitch using independent parallel PID loops with anti-windup clamping to prevent integrator runaway during setpoint changes.",
        formula: "error_sum[i] = std::clamp(error_sum[i] + error[i], -40.0f, 40.0f);\nfloat derivative = error[i] - prev_error[i];\nout[i] = (Kp[i] * error[i]) + (Ki[i] * error_sum[i]) + (Kd[i] * derivative);\nprev_error[i] = error[i];",
        notes: [
            "windup_guard = 40.0f clamps accumulated integral sum.",
            "Tuned live via /roll_pid and /pitch_pid topics from the controller tuner GUI."
        ]
    },
    'pipe-cascaded': {
        title: "Stage 04: Cascaded Altitude (Z) Dual-Loop Architecture",
        badge: "Lines 260–290",
        desc: "The core innovation in pico_controller_PID.cpp. Instead of directly mapping height error to motor thrust, an Outer Loop computes desired vertical velocity, and an Inner Loop acts as a dynamic brake against overshoot.",
        formula: "// Outer Loop (Position -> Velocity)\ndesired_vel_z = std::clamp((Kp_pos * e_z) + (Ki_pos * &sum;e_z) + (Kd_pos * de_z/dt), -3.0f, 3.0f);\n\n// Inner Loop (Velocity -> Throttle PWM)\nout_throttle = (Kp_vel_z * (desired_vel_z - actual_vel_z)) + ...;",
        notes: [
            "max_vertical_vel = 3.0 m/s guarantees the drone never climbs or descends too violently.",
            "Inner loop uses fixed baseline gain Kp_vel_z = 15.0 to ensure dampening even during user tuning."
        ]
    },
    'pipe-pwm': {
        title: "Stage 05: RC PWM Neutral Centering, Clamping & Arming",
        badge: "Lines 292–304",
        desc: "Converts the raw controller outputs into standard 1000–2000 µs radio control pulse-width signals centered at 1500 µs neutral, and enforces software arming on AUX4.",
        formula: "cmd_msg.rc_roll     = std::clamp(1500 + out[0], 1000, 2000);\ncmd_msg.rc_pitch    = std::clamp(1500 + out[1], 1000, 2000);\ncmd_msg.rc_throttle = std::clamp(1500 - out_throttle, 1000, 2000); // Inverted sign\ncmd_msg.rc_yaw      = 1500;\ncmd_msg.rc_aux4     = 2000; // Drone armed flag",
        notes: [
            "Notice the subtraction (1500 - out_throttle): accounts for coordinate frame sign convention.",
            "rc_aux4 = 2000 maintains active flight arm status throughout node execution."
        ]
    }
};

function renderPipelineDetail(key) {
    const detail = pipelineDetails[key];
    const box = document.getElementById('pipeline-detail-content');

    box.innerHTML = `
        <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 0.75rem;">
            <h3 style="font-size: 1.15rem; font-weight: 700; color: var(--cyan);">${detail.title}</h3>
            <span class="brand-badge">${detail.badge}</span>
        </div>
        <p style="color: var(--text-secondary); margin-bottom: 1rem; font-size: 0.95rem;">${detail.desc}</p>
        <div class="formula-box" style="margin-bottom: 1rem; white-space: pre-wrap;">${detail.formula}</div>
        <ul style="list-style: none; display: flex; flex-direction: column; gap: 0.5rem; font-size: 0.88rem; color: var(--text-secondary);">
            ${detail.notes.map(n => `<li style="position: relative; padding-left: 1.25rem;"><span style="position: absolute; left: 0; color: var(--cyan);">&#9657;</span>${n}</li>`).join('')}
        </ul>
    `;
}

document.querySelectorAll('.pipe-step').forEach(step => {
    step.addEventListener('click', function() {
        document.querySelectorAll('.pipe-step').forEach(s => s.classList.remove('active'));
        this.classList.add('active');
        renderPipelineDetail(this.dataset.target);
    });
});

renderPipelineDetail('pipe-whycode');

// --- Code Inspector Content & Tabs ---
const codeSnippets = {
    'tab-pid': {
        code: `void pid()
{
    auto cmd_msg = swift_msgs::msg::SwiftMsgs();
    auto error_msg = error_msg::msg::Error();

    // 1. Position Errors (WhyCode Units)
    error[0] = setpoint[1] - drone_position[1]; // roll_error (Y)
    error[1] = setpoint[0] - drone_position[0]; // pitch_error (X)
    error[2] = setpoint[2] - drone_position[2]; // throttle_error (Z)

    // 2. Publish /pos_error for GUI and rosbag
    error_msg.roll_error = error[0];
    error_msg.pitch_error = error[1];
    error_msg.throttle_error = error[2];
    pos_error_pub->publish(error_msg);

    // 3. Roll (Y) and Pitch (X) Position PID
    float out[2] = {0.0f, 0.0f};
    for (int i = 0; i < 2; i++)
    {
        error_sum[i] += error[i];
        error_sum[i] = std::clamp(error_sum[i], -windup_guard, windup_guard);

        float derivative = error[i] - prev_error[i];
        out[i] = (Kp[i] * error[i]) + (Ki[i] * error_sum[i]) + (Kd[i] * derivative);
        prev_error[i] = error[i];
    }

    // ================= CASCADED THROTTLE (Z) =================
    // A. Measure actual vertical velocity (units/sec)
    float actual_vel_z = (drone_position[2] - prev_drone_position[2]) / dt;
    prev_drone_position[2] = drone_position[2];

    // B. OUTER LOOP (Controlled via GUI /throttle_pid): Height Error -> Target Velocity
    pos_z_error_sum += error[2] * dt;
    pos_z_error_sum = std::clamp(pos_z_error_sum, -windup_guard, windup_guard);

    float pos_z_derivative = (error[2] - prev_error[2]) / dt;
    prev_error[2] = error[2];

    float desired_vel_z = (Kp_pos_z * error[2]) + 
                          (Ki_pos_z * pos_z_error_sum) + 
                          (Kd_pos_z * pos_z_derivative);

    // Clamp desired velocity to safe limits
    desired_vel_z = std::clamp(desired_vel_z, -max_vertical_vel, max_vertical_vel);

    // C. INNER LOOP (Fixed baseline): Velocity Error -> Throttle PWM
    float vel_z_error = desired_vel_z - actual_vel_z;
    vel_z_error_sum += vel_z_error * dt;
    vel_z_error_sum = std::clamp(vel_z_error_sum, -windup_guard, windup_guard);

    float vel_z_derivative = (vel_z_error - prev_vel_z_error) / dt;
    prev_vel_z_error = vel_z_error;

    float out_throttle = (Kp_vel_z * vel_z_error) + 
                         (Ki_vel_z * vel_z_error_sum) + 
                         (Kd_vel_z * vel_z_derivative);

    // 4. Map to Neutral 1500 Midpoint
    int target_roll     = static_cast<int>(1500.0f + out[0]);
    int target_pitch    = static_cast<int>(1500.0f + out[1]);
    int target_throttle = static_cast<int>(1500.0f - out_throttle);

    // 5. Output Clamping & Arm Flag
    cmd_msg.rc_roll     = std::clamp(target_roll, min_values[0], max_values[0]);
    cmd_msg.rc_pitch    = std::clamp(target_pitch, min_values[1], max_values[1]);
    cmd_msg.rc_throttle = std::clamp(target_throttle, min_values[2], max_values[2]);
    cmd_msg.rc_yaw      = 1500;
    cmd_msg.rc_aux4     = 2000; // Keep drone armed

    command_pub->publish(cmd_msg);
}`,
        notes: [
            "Executed at fixed 33ms intervals by rclcpp timer (sample_time = 33ms).",
            "Anti-windup guard of ±40.0 prevents integrator explosion during ground-start.",
            "Altitude velocity is clamped at ±3.0 m/s to prevent destructive ascents.",
            "All output signals strictly bounded within [1000, 2000] microseconds."
        ]
    },
    'tab-init': {
        code: `Swift_Pico() : Node("pico_controller")
{
    // Target setpoint [x, y, z] from Task 2A
    setpoint[0] = 1.5f;
    setpoint[1] = -1.5f;
    setpoint[2] = 13.0f;

    // Outer Loop tuned by GUI (/throttle_pid)
    Kp_pos_z = 0.0f;  Ki_pos_z = 0.0f;  Kd_pos_z = 0.0f;

    // Inner Loop hardcoded fixed baseline
    Kp_vel_z = 15.0f; Ki_vel_z = 0.0f;  Kd_vel_z = 0.0f;

    sample_time = 33ms;
    dt = std::chrono::duration<float>(sample_time).count();

    // Publishers
    command_pub = this->create_publisher<swift_msgs::msg::SwiftMsgs>("/drone_command", 10);
    pos_error_pub = this->create_publisher<error_msg::msg::Error>("/pos_error", 10);
    heartbeat_pub = this->create_publisher<std_msgs::msg::Header>("/controller_heartbeat", 10);

    // Tuning GUI subscribers
    throttle_pid_sub = this->create_subscription<controller_msg::msg::PIDTune>(
        "/throttle_pid", 1, std::bind(&Swift_Pico::altitude_set_pid, this, _1));

    // Timers & Arming
    heartbeat_timer = this->create_wall_timer(100ms, std::bind(&Swift_Pico::publish_heartbeat, this));
    std::this_thread::sleep_for(500ms);
    arm();
    pid_timer = this->create_wall_timer(sample_time, std::bind(&Swift_Pico::pid, this));
}`,
        notes: [
            "Sets initial setpoint to [1.5, -1.5, 13.0] as specified in Task 2A.",
            "Initializes baseline Kp_vel_z = 15.0 so altitude has inherent damping.",
            "Arming sequence first calls disarm() (all 1000), waits 500ms, then arms with rc_aux4 = 2000.",
            "Heartbeat timer (100ms / 10Hz) provides liveness proof to safety watchers."
        ]
    },
    'tab-subs': {
        code: `void whycode_callback(const whycode_interfaces::msg::MarkerArray & msg)
{
    if (!msg.markers.empty()) {
        drone_position[0] = msg.markers[0].position.position.x;
        drone_position[1] = msg.markers[0].position.position.y;
        drone_position[2] = msg.markers[0].position.position.z;
    }
}

// Connect GUI sliders directly to the OUTER position loop gains
void altitude_set_pid(const controller_msg::msg::PIDTune & alt)
{
    Kp_pos_z = alt.kp;
    Ki_pos_z = alt.ki;
    Kd_pos_z = alt.kd;
}

void pitch_set_pid(const controller_msg::msg::PIDTune & pitch)
{
    Kp[1] = pitch.kp;
    Ki[1] = pitch.ki;
    Kd[1] = pitch.kd;
}

void roll_set_pid(const controller_msg::msg::PIDTune & roll)
{
    Kp[0] = roll.kp;
    Ki[0] = roll.ki;
    Kd[0] = roll.kd;
}`,
        notes: [
            "WhyCode callback checks msg.markers.empty() to safely handle occlusions.",
            "Tuning callbacks instantly update gains on each incoming slider update message without node restart.",
            "GUI throttle slider controls the OUTER position loop, leaving inner dampening intact."
        ]
    }
};

function switchCodeTab(tabKey) {
    const data = codeSnippets[tabKey];
    document.getElementById('code-content').textContent = data.code;
    const list = document.getElementById('code-notes-list');
    list.innerHTML = data.notes.map(n => `<li>${n}</li>`).join('');
}

document.querySelectorAll('.code-tab').forEach(tab => {
    tab.addEventListener('click', function() {
        document.querySelectorAll('.code-tab').forEach(t => t.classList.remove('active'));
        this.classList.add('active');
        switchCodeTab(this.dataset.tab);
    });
});

switchCodeTab('tab-pid');

// --- Start Simulation Loop ---
requestAnimationFrame(simLoop);
