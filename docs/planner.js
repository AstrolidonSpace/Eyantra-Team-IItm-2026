/**
 * Task 2C: Survivor Detection & Optimal Path Planner Visualizer
 * Team: KD_6479
 * 
 * Interactive 11x11 Arena Canvas, TSP vs Greedy Comparison,
 * 24 Permutations Explorer, and Real-Time Flight Simulator.
 */

// --- 1. Grid & Coordinates Database -----------------------------------------
// Maps all 121 intersections (A1 to K11) to WhyCode metric coordinates (X, Y)
const GRID_COORDINATES = {
    "A1": {"x": 6.1131, "y": 6.1131}, "B1": {"x": 4.9049, "y": 6.1306}, "C1": {"x": 3.6785, "y": 6.1265}, "D1": {"x": 2.4153, "y": 6.0250}, "E1": {"x": 1.2113, "y": 6.0221}, "F1": {"x": 0.0068, "y": 6.0941}, "G1": {"x": -1.1982, "y": 6.0255}, "H1": {"x": -2.4025, "y": 6.0271}, "I1": {"x": -3.6743, "y": 6.1415}, "J1": {"x": -4.8826, "y": 6.1194}, "K1": {"x": -6.0960, "y": 6.1096},
    "A2": {"x": 6.1239, "y": 4.8995}, "B2": {"x": 4.8800, "y": 4.8800}, "C2": {"x": 3.6914, "y": 4.9202}, "D2": {"x": 2.4376, "y": 4.8662}, "E2": {"x": 1.2032, "y": 4.7897}, "F2": {"x": 0.0067, "y": 4.7898}, "G2": {"x": -1.1913, "y": 4.7950}, "H2": {"x": -2.4241, "y": 4.8663}, "I2": {"x": -3.6835, "y": 4.9279}, "J2": {"x": -4.8661, "y": 4.8796}, "K2": {"x": -6.1100, "y": 4.8996},
    "A3": {"x": 6.1260, "y": 3.6781}, "B3": {"x": 4.9186, "y": 3.6902}, "C3": {"x": 3.7042, "y": 3.7042}, "D3": {"x": 2.4595, "y": 3.6857}, "E3": {"x": 1.2098, "y": 3.6139}, "F3": {"x": 0.0066, "y": 3.5752}, "G3": {"x": -1.1970, "y": 3.6157}, "H3": {"x": -2.4447, "y": 3.6835}, "I3": {"x": -3.6870, "y": 3.7007}, "J3": {"x": -4.9155, "y": 3.6987}, "K3": {"x": -6.1124, "y": 3.6782},
    "A4": {"x": 6.0201, "y": 2.4134}, "B4": {"x": 4.8650, "y": 2.4370}, "C4": {"x": 3.6843, "y": 2.4588}, "D4": {"x": 2.4454, "y": 2.4454}, "E4": {"x": 1.2220, "y": 2.4388}, "F4": {"x": 0.0067, "y": 2.3998}, "G4": {"x": -1.2010, "y": 2.4220}, "H4": {"x": -2.4323, "y": 2.4459}, "I4": {"x": -3.6699, "y": 2.4580}, "J4": {"x": -4.8559, "y": 2.4392}, "K4": {"x": -6.0101, "y": 2.4148},
    "A5": {"x": 6.0010, "y": 1.2067}, "B5": {"x": 4.7948, "y": 1.2046}, "C5": {"x": 3.6154, "y": 1.2102}, "D5": {"x": 2.4396, "y": 1.2226}, "E5": {"x": 1.1989, "y": 1.1992}, "F5": {"x": 0.0067, "y": 1.2142}, "G5": {"x": -1.1865, "y": 1.2001}, "H5": {"x": -2.4256, "y": 1.2222}, "I5": {"x": -3.5968, "y": 1.2085}, "J5": {"x": -4.8074, "y": 1.2106}, "K5": {"x": -5.9959, "y": 1.2084},
    "A6": {"x": 6.0404, "y": 0.0067}, "B6": {"x": 4.7881, "y": 0.0067}, "C6": {"x": 3.5627, "y": 0.0066}, "D6": {"x": 2.3995, "y": 0.0067}, "E6": {"x": 1.1992, "y": 0.0067}, "F6": {"x": 0.0068, "y": 0.0068}, "G6": {"x": -1.1980, "y": 0.0067}, "H6": {"x": -2.4221, "y": 0.0067}, "I6": {"x": -3.5879, "y": 0.0067}, "J6": {"x": -4.8000, "y": 0.0067}, "K6": {"x": -6.0400, "y": 0.0067},
    "A7": {"x": 6.0020, "y": -1.2080}, "B7": {"x": 4.8010, "y": -1.2100}, "C7": {"x": 3.6200, "y": -1.2090}, "D7": {"x": 2.4365, "y": -1.2074}, "E7": {"x": 1.2010, "y": -1.1980}, "F7": {"x": 0.0067, "y": -1.2150}, "G7": {"x": -1.1890, "y": -1.2010}, "H7": {"x": -2.4221, "y": -1.2071}, "I7": {"x": -3.6010, "y": -1.2090}, "J7": {"x": -4.7958, "y": -1.1943}, "K7": {"x": -6.0020, "y": -1.2080},
    "A8": {"x": 6.0220, "y": -2.4140}, "B8": {"x": 4.8620, "y": -2.4360}, "C8": {"x": 3.6810, "y": -2.4570}, "D8": {"x": 2.4410, "y": -2.4460}, "E8": {"x": 1.2210, "y": -2.4390}, "F8": {"x": 0.0067, "y": -2.4010}, "G8": {"x": -1.2020, "y": -2.4230}, "H8": {"x": -2.4330, "y": -2.4470}, "I8": {"x": -3.6710, "y": -2.4590}, "J8": {"x": -4.8570, "y": -2.4400}, "K8": {"x": -6.0110, "y": -2.4160},
    "A9": {"x": 6.1250, "y": -3.6790}, "B9": {"x": 4.9190, "y": -3.6910}, "C9": {"x": 3.7015, "y": -3.6878}, "D9": {"x": 2.4600, "y": -3.6860}, "E9": {"x": 1.2100, "y": -3.6140}, "F9": {"x": 0.0066, "y": -3.5760}, "G9": {"x": -1.1980, "y": -3.6160}, "H9": {"x": -2.4450, "y": -3.6840}, "I9": {"x": -3.6880, "y": -3.7010}, "J9": {"x": -4.9160, "y": -3.6990}, "K9": {"x": -6.1130, "y": -3.6790},
    "A10": {"x": 6.1240, "y": -4.9000}, "B10": {"x": 4.8810, "y": -4.8810}, "C10": {"x": 3.6920, "y": -4.9210}, "D10": {"x": 2.4380, "y": -4.8670}, "E10": {"x": 1.2040, "y": -4.7900}, "F10": {"x": 0.0067, "y": -4.7900}, "G10": {"x": -1.1920, "y": -4.7960}, "H10": {"x": -2.4250, "y": -4.8670}, "I10": {"x": -3.6840, "y": -4.9280}, "J10": {"x": -4.8670, "y": -4.8800}, "K10": {"x": -6.1110, "y": -4.9000},
    "A11": {"x": 6.1140, "y": -6.1140}, "B11": {"x": 4.9050, "y": -6.1310}, "C11": {"x": 3.6790, "y": -6.1270}, "D11": {"x": 2.4160, "y": -6.0260}, "E11": {"x": 1.2120, "y": -6.0230}, "F11": {"x": 0.0068, "y": -6.0950}, "G11": {"x": -1.1990, "y": -6.0260}, "H11": {"x": -2.4030, "y": -6.0280}, "I11": {"x": -3.6750, "y": -6.1420}, "J11": {"x": -4.8830, "y": -6.1200}, "K11": {"x": -6.0970, "y": -6.1100}
};

const COLS = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K"];
const ROWS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11];

// --- 2. State & Setup -------------------------------------------------------
const state = {
    survivors: ["H7", "H5", "E4", "D7"], // Default active set
    selectedAlgo: "tsp",                 // 'tsp' or 'greedy'
    previewRoute: null,                  // Override order from permutations table
    hoveredGrid: null,
    
    // Animation State
    animating: false,
    paused: false,
    dronePos: { x: 450, y: 450, z: 13 },
    droneAngle: 0,
    droneLeg: 0,
    legProgress: 0.0,
    holdRemaining: 0.0,
    flightClock: 0.0,
    playbackSpeed: 1,
    trail: [],
    
    // Results cache
    allPermutations: [],
    tspOrder: [],
    tspDist: 0,
    greedyOrder: [],
    greedyDist: 0
};

// --- 3. Mathematical & Combinatorial Helpers --------------------------------
function getMetricCoord(gridName) {
    if (GRID_COORDINATES[gridName]) {
        return GRID_COORDINATES[gridName];
    }
    // Fallback formula if ever unmapped
    const colIdx = COLS.indexOf(gridName[0]);
    const rowIdx = parseInt(gridName.slice(1)) - 1;
    return {
        x: (5 - colIdx) * 1.2034,
        y: (5 - rowIdx) * 1.2034
    };
}

function distMetric(name1, name2) {
    const p1 = getMetricCoord(name1);
    const p2 = getMetricCoord(name2);
    return Math.hypot(p1.x - p2.x, p1.y - p2.y);
}

function gridToPixel(gridName) {
    const colIdx = COLS.indexOf(gridName[0]) + 1; // 1 to 11
    const rowIdx = parseInt(gridName.slice(1));    // 1 to 11
    return {
        x: colIdx * 75,
        y: rowIdx * 75
    };
}

function pixelToNearestGrid(px, py) {
    const colIdx = Math.max(1, Math.min(11, Math.round(px / 75)));
    const rowIdx = Math.max(1, Math.min(11, Math.round(py / 75)));
    return `${COLS[colIdx - 1]}${rowIdx}`;
}

// Full Permutation Generator
function permutations(arr) {
    if (arr.length <= 1) return [arr];
    const result = [];
    for (let i = 0; i < arr.length; i++) {
        const current = arr[i];
        const remaining = arr.slice(0, i).concat(arr.slice(i + 1));
        const remPerms = permutations(remaining);
        for (const p of remPerms) {
            result.push([current].concat(p));
        }
    }
    return result;
}

// Compute TSP & Greedy Solutions
function solveRouting() {
    const start = "F6";
    const perms = permutations(state.survivors);
    const list = [];

    // 1. Evaluate all permutations
    for (const p of perms) {
        const leg1 = distMetric(start, p[0]);
        const leg2 = distMetric(p[0], p[1]);
        const leg3 = distMetric(p[1], p[2]);
        const leg4 = distMetric(p[2], p[3]);
        const total = leg1 + leg2 + leg3 + leg4;
        list.push({
            order: p,
            legs: [leg1, leg2, leg3, leg4],
            total: total
        });
    }

    list.sort((a, b) => a.total - b.total);
    state.allPermutations = list;
    state.tspOrder = list[0].order;
    state.tspDist = list[0].total;

    // 2. Compute Greedy Nearest-Neighbor
    let curr = start;
    const unvisited = new Set(state.survivors);
    const gOrder = [];
    const gLegs = [];
    let gTotal = 0;

    while (unvisited.size > 0) {
        let bestNext = null;
        let minD = Infinity;
        for (const cand of unvisited) {
            const d = distMetric(curr, cand);
            if (d < minD) {
                minD = d;
                bestNext = cand;
            }
        }
        gLegs.push(minD);
        gTotal += minD;
        gOrder.push(bestNext);
        unvisited.delete(bestNext);
        curr = bestNext;
    }

    state.greedyOrder = gOrder;
    state.greedyDist = gTotal;
}

function getActiveRoute() {
    if (state.previewRoute) {
        return state.previewRoute;
    }
    return state.selectedAlgo === "tsp" ? state.tspOrder : state.greedyOrder;
}

function getActiveTotalDist() {
    if (state.previewRoute) {
        let d = distMetric("F6", state.previewRoute[0]);
        for (let i = 0; i < state.previewRoute.length - 1; i++) {
            d += distMetric(state.previewRoute[i], state.previewRoute[i+1]);
        }
        return d;
    }
    return state.selectedAlgo === "tsp" ? state.tspDist : state.greedyDist;
}

// --- 4. Canvas Rendering Engine ---------------------------------------------
const canvas = document.getElementById("arenaCanvas");
const ctx = canvas.getContext("2d");

function drawArena() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // 1. Procedural Satellite Disaster Background
    drawBackdrop();

    // 2. Grid Lattice Lines
    drawGrid();

    // 3. ArUco Marker Boundaries (Corner tags)
    drawArucoMarkers();

    // 4. Hover Highlight
    if (state.hoveredGrid) {
        const hp = gridToPixel(state.hoveredGrid);
        ctx.save();
        ctx.strokeStyle = "rgba(0, 229, 255, 0.7)";
        ctx.fillStyle = "rgba(0, 229, 255, 0.15)";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(hp.x, hp.y, 22, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
        ctx.restore();
    }

    // 5. Active Planned Route Lines
    drawRouteVectors();

    // 6. START F6 Takeoff Pad
    drawStartPad();

    // 7. Critical Survivor Markers
    drawSurvivors();

    // 8. Virtual Drone & Particle Trail
    drawDrone();
}

function drawBackdrop() {
    // Rich gradient mimicking satellite water/terrain
    const grad = ctx.createRadialGradient(450, 450, 50, 450, 450, 600);
    grad.addColorStop(0, "#0e1a2b");
    grad.addColorStop(0.5, "#091322");
    grad.addColorStop(1, "#050b14");
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, 900, 900);

    // Subtle road/terrain overlay features
    ctx.save();
    ctx.strokeStyle = "rgba(255, 255, 255, 0.04)";
    ctx.lineWidth = 14;
    ctx.beginPath();
    ctx.moveTo(100, 200);
    ctx.bezierCurveTo(300, 400, 600, 200, 800, 700);
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(450, 450, 160, 0, Math.PI * 2);
    ctx.stroke();
    ctx.restore();
}

function drawGrid() {
    ctx.save();
    ctx.lineWidth = 1;
    ctx.strokeStyle = "rgba(255, 255, 255, 0.18)";

    // 11x11 Grid Lines
    for (let i = 1; i <= 11; i++) {
        const pos = i * 75;
        // Vertical lines
        ctx.beginPath();
        ctx.moveTo(pos, 50);
        ctx.lineTo(pos, 850);
        ctx.stroke();

        // Horizontal lines
        ctx.beginPath();
        ctx.moveTo(50, pos);
        ctx.lineTo(850, pos);
        ctx.stroke();
    }

    // Column & Row Labels
    ctx.font = "600 13px 'JetBrains Mono', monospace";
    ctx.fillStyle = "rgba(255, 255, 255, 0.65)";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";

    for (let c = 0; c < 11; c++) {
        const x = (c + 1) * 75;
        ctx.fillText(COLS[c], x, 28);
        ctx.fillText(COLS[c], x, 872);
    }

    for (let r = 0; r < 11; r++) {
        const y = (r + 1) * 75;
        ctx.fillText(ROWS[r], 28, y);
        ctx.fillText(ROWS[r], 872, y);
    }
    ctx.restore();
}

function drawArucoMarkers() {
    const markers = [
        { id: 80, x: 25, y: 25, label: "ArUco 80 (TL)" },
        { id: 85, x: 875, y: 25, label: "ArUco 85 (TR)" },
        { id: 90, x: 875, y: 875, label: "ArUco 90 (BR)" },
        { id: 95, x: 25, y: 875, label: "ArUco 95 (BL)" }
    ];

    ctx.save();
    for (const m of markers) {
        ctx.fillStyle = "#ffffff";
        ctx.fillRect(m.x - 18, m.y - 18, 36, 36);
        ctx.fillStyle = "#000000";
        ctx.fillRect(m.x - 14, m.y - 14, 28, 28);
        ctx.fillStyle = "#ffffff";
        ctx.fillRect(m.x - 6, m.y - 6, 12, 12);

        // Subdued tag
        ctx.font = "500 10px 'JetBrains Mono', monospace";
        ctx.fillStyle = "rgba(255, 255, 255, 0.4)";
        ctx.textAlign = "center";
        const offset = m.y < 450 ? 32 : -25;
        ctx.fillText(`ID:${m.id}`, m.x, m.y + offset);
    }
    ctx.restore();
}

function drawStartPad() {
    const p = gridToPixel("F6"); // (450, 450)
    ctx.save();

    // Pulse animation
    const time = Date.now() * 0.003;
    const pulseRad = 26 + Math.sin(time) * 4;

    ctx.strokeStyle = "rgba(0, 229, 255, 0.4)";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(p.x, p.y, pulseRad, 0, Math.PI * 2);
    ctx.stroke();

    // Yellow upward triangle
    ctx.fillStyle = "#facc15";
    ctx.beginPath();
    ctx.moveTo(p.x, p.y - 14);
    ctx.lineTo(p.x + 13, p.y + 10);
    ctx.lineTo(p.x - 13, p.y + 10);
    ctx.closePath();
    ctx.fill();

    // Center dot
    ctx.fillStyle = "#000000";
    ctx.beginPath();
    ctx.arc(p.x, p.y + 2, 3, 0, Math.PI * 2);
    ctx.fill();

    // Text Label
    ctx.font = "700 13px 'Outfit', sans-serif";
    ctx.fillStyle = "#facc15";
    ctx.textAlign = "center";
    ctx.shadowColor = "rgba(0,0,0,0.8)";
    ctx.shadowBlur = 4;
    ctx.fillText("START F6", p.x, p.y - 24);

    ctx.restore();
}

function drawSurvivors() {
    const route = getActiveRoute();

    ctx.save();
    for (let i = 0; i < route.length; i++) {
        const s = route[i];
        const p = gridToPixel(s);
        const orderNum = i + 1;

        // Outer glow rings
        ctx.strokeStyle = "#facc15";
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(p.x, p.y, 32, 0, Math.PI * 2);
        ctx.stroke();

        ctx.strokeStyle = "#ef4444";
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.arc(p.x, p.y, 28, 0, Math.PI * 2);
        ctx.stroke();

        // Red triangle marker inside
        ctx.fillStyle = "#ef4444";
        ctx.beginPath();
        ctx.moveTo(p.x, p.y - 12);
        ctx.lineTo(p.x + 12, p.y + 10);
        ctx.lineTo(p.x - 12, p.y + 10);
        ctx.closePath();
        ctx.fill();

        // Step Badge & Name
        ctx.font = "700 14px 'Outfit', sans-serif";
        ctx.fillStyle = "#facc15";
        ctx.textAlign = "left";
        ctx.shadowColor = "rgba(0,0,0,0.9)";
        ctx.shadowBlur = 6;
        ctx.fillText(`${orderNum}: ${s}`, p.x + 16, p.y - 12);
    }
    ctx.restore();
}

function drawRouteVectors() {
    const route = getActiveRoute();
    const pts = [gridToPixel("F6")].concat(route.map(gridToPixel));

    ctx.save();
    ctx.strokeStyle = state.selectedAlgo === "tsp" ? "#f97316" : "#eab308";
    ctx.lineWidth = 3.5;
    ctx.lineCap = "round";
    ctx.lineJoin = "round";
    ctx.shadowColor = state.selectedAlgo === "tsp" ? "rgba(249, 115, 22, 0.5)" : "rgba(234, 179, 8, 0.5)";
    ctx.shadowBlur = 8;

    ctx.beginPath();
    ctx.moveTo(pts[0].x, pts[0].y);
    for (let i = 1; i < pts.length; i++) {
        ctx.lineTo(pts[i].x, pts[i].y);
    }
    ctx.stroke();

    // Marching dash pulses
    const offset = (Date.now() * 0.02) % 20;
    ctx.setLineDash([8, 12]);
    ctx.lineDashOffset = -offset;
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2;
    ctx.shadowBlur = 0;
    ctx.stroke();

    // Draw directional arrows along segments
    for (let i = 0; i < pts.length - 1; i++) {
        const p1 = pts[i];
        const p2 = pts[i + 1];
        const mx = (p1.x + p2.x) / 2;
        const my = (p1.y + p2.y) / 2;
        const angle = Math.atan2(p2.y - p1.y, p2.x - p1.x);

        ctx.save();
        ctx.translate(mx, my);
        ctx.rotate(angle);
        ctx.fillStyle = state.selectedAlgo === "tsp" ? "#f97316" : "#eab308";
        ctx.beginPath();
        ctx.moveTo(8, 0);
        ctx.lineTo(-6, -6);
        ctx.lineTo(-3, 0);
        ctx.lineTo(-6, 6);
        ctx.closePath();
        ctx.fill();
        ctx.restore();
    }

    ctx.restore();
}

function drawDrone() {
    // Particle Trail
    ctx.save();
    for (let i = 0; i < state.trail.length; i++) {
        const tr = state.trail[i];
        const alpha = (i / state.trail.length) * 0.45;
        ctx.fillStyle = `rgba(0, 229, 255, ${alpha})`;
        ctx.beginPath();
        ctx.arc(tr.x, tr.y, 4, 0, Math.PI * 2);
        ctx.fill();
    }

    const { x, y } = state.dronePos;

    // Hold Timer Ring (during 3-second hold)
    if (state.holdRemaining > 0) {
        const holdRatio = state.holdRemaining / 3.0;
        ctx.strokeStyle = "#f59e0b";
        ctx.lineWidth = 4;
        ctx.beginPath();
        ctx.arc(x, y, 38, -Math.PI / 2, -Math.PI / 2 + (Math.PI * 2 * (1 - holdRatio)));
        ctx.stroke();

        ctx.font = "700 13px 'JetBrains Mono', monospace";
        ctx.fillStyle = "#f59e0b";
        ctx.textAlign = "center";
        ctx.fillText(`${state.holdRemaining.toFixed(1)}s HOLD`, x, y - 44);
    }

    // Drone Body
    ctx.translate(x, y);
    ctx.rotate(state.droneAngle);

    // Motor arms
    ctx.strokeStyle = "#00e5ff";
    ctx.lineWidth = 3.5;
    ctx.beginPath();
    ctx.moveTo(-16, -16); ctx.lineTo(16, 16);
    ctx.moveTo(16, -16); ctx.lineTo(-16, 16);
    ctx.stroke();

    // Central hub
    ctx.fillStyle = "#0f172a";
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(0, 0, 9, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    // WhyCode central marker ring
    ctx.fillStyle = "#00e5ff";
    ctx.beginPath();
    ctx.arc(0, 0, 4, 0, Math.PI * 2);
    ctx.fill();

    // Spinning propellers
    const propPhase = Date.now() * 0.05;
    const motors = [
        { x: -16, y: -16, color: "#10b981" },
        { x:  16, y: -16, color: "#10b981" },
        { x:  16, y:  16, color: "#ef4444" },
        { x: -16, y:  16, color: "#ef4444" }
    ];

    for (const m of motors) {
        ctx.save();
        ctx.translate(m.x, m.y);
        ctx.rotate(propPhase);
        ctx.strokeStyle = m.color;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(-8, 0); ctx.lineTo(8, 0);
        ctx.stroke();
        ctx.restore();
    }

    ctx.restore();
}

// --- 5. Flight Animation Loop ------------------------------------------------
let lastTimestamp = 0;

function updateAnimation(timestamp) {
    if (!lastTimestamp) lastTimestamp = timestamp;
    const dt = Math.min((timestamp - lastTimestamp) / 1000, 0.1) * state.playbackSpeed;
    lastTimestamp = timestamp;

    if (state.animating && !state.paused) {
        state.flightClock += dt;

        const route = getActiveRoute();
        const pts = [gridToPixel("F6")].concat(route.map(gridToPixel));

        if (state.droneLeg < pts.length - 1) {
            const pStart = pts[state.droneLeg];
            const pEnd = pts[state.droneLeg + 1];
            const legDistPx = Math.hypot(pEnd.x - pStart.x, pEnd.y - pStart.y);
            const speedPxPerSec = 110; // ~1.5 m/s in metric scale

            // If we are currently holding at the survivor
            if (state.holdRemaining > 0) {
                state.holdRemaining -= dt;
                document.getElementById("hold-display").textContent = `${Math.max(0, state.holdRemaining).toFixed(1)}s`;

                if (state.holdRemaining <= 0) {
                    state.holdRemaining = 0;
                    state.droneLeg++;
                    state.legProgress = 0;
                    document.getElementById("hold-display").textContent = "Cleared ✓";
                }
            } else {
                // Flying between waypoints
                state.legProgress += (speedPxPerSec * dt) / legDistPx;

                if (state.legProgress >= 1.0) {
                    state.legProgress = 1.0;
                    state.dronePos = { x: pEnd.x, y: pEnd.y, z: 13 };
                    // Reached target survivor: initiate 3-second mandatory hold!
                    state.holdRemaining = 3.0;
                } else {
                    state.dronePos.x = pStart.x + (pEnd.x - pStart.x) * state.legProgress;
                    state.dronePos.y = pStart.y + (pEnd.y - pStart.y) * state.legProgress;
                }

                // Heading angle
                state.droneAngle = Math.atan2(pEnd.y - pStart.y, pEnd.x - pStart.x);

                // Add trail
                state.trail.push({ x: state.dronePos.x, y: state.dronePos.y });
                if (state.trail.length > 50) state.trail.shift();

                const targetName = route[state.droneLeg];
                document.getElementById("leg-display").textContent = `Fly &rarr; ${targetName} (${state.droneLeg + 1}/4)`;
            }
        } else {
            // Mission Finished!
            state.animating = false;
            document.getElementById("btn-play").disabled = false;
            document.getElementById("btn-pause").disabled = true;
            document.getElementById("leg-display").textContent = "Mission Complete! ✓";
            document.getElementById("hold-display").textContent = "4/4 Cleared";
        }

        document.getElementById("clock-display").textContent = `${state.flightClock.toFixed(1)}s`;
    }

    drawArena();
    requestAnimationFrame(updateAnimation);
}

// --- 6. UI Synchronization & DOM Handlers ------------------------------------
function updateHUD() {
    const totalDist = getActiveTotalDist();
    document.getElementById("hud-selected-algo").textContent = 
        `Algorithm: ${state.selectedAlgo === "tsp" ? "Optimal TSP" : "Greedy NN"}`;
    document.getElementById("hud-total-dist").textContent = `Distance: ${totalDist.toFixed(2)} m`;

    document.getElementById("tsp-dist-val").textContent = `${state.tspDist.toFixed(2)} m`;
    document.getElementById("greedy-dist-val").textContent = `${state.greedyDist.toFixed(2)} m`;

    // Savings Box
    const delta = state.greedyDist - state.tspDist;
    const savingsTitle = document.getElementById("savings-title");
    const savingsDetail = document.getElementById("savings-detail");
    const savingsBox = document.getElementById("savings-box");

    if (delta > 0.05) {
        savingsTitle.textContent = `TSP Saves ${delta.toFixed(2)} Meters (~${(delta * 4.5).toFixed(1)}s faster)`;
        savingsDetail.textContent = `Greedy choice gets trapped into crossing the arena diagonally at the end. Brute-force evaluates all 24 permutations to guarantee global optimum.`;
        savingsBox.style.borderColor = "rgba(16, 185, 129, 0.4)";
        savingsBox.style.background = "rgba(16, 185, 129, 0.12)";
    } else {
        savingsTitle.textContent = "Algorithms Agree for this Layout";
        savingsDetail.textContent = `Both Greedy NN and TSP find the optimal ${state.tspDist.toFixed(2)}m route for this symmetrical setup.`;
        savingsBox.style.borderColor = "rgba(255, 255, 255, 0.1)";
        savingsBox.style.background = "rgba(255, 255, 255, 0.05)";
    }

    updateWaypointsTable();
    updatePermutationsTable();
}

function updateWaypointsTable() {
    const tbody = document.querySelector("#waypoints-table tbody");
    tbody.innerHTML = "";

    const route = getActiveRoute();
    const rows = [
        { step: "Takeoff", grid: "F6", coord: getMetricCoord("F6"), alt: "13.0m", hold: "0s" }
    ];

    for (let i = 0; i < route.length; i++) {
        const s = route[i];
        rows.push({
            step: `Goal ${i + 1}`,
            grid: s,
            coord: getMetricCoord(s),
            alt: "13.0m",
            hold: "3s"
        });
    }

    for (const r of rows) {
        const tr = document.createElement("tr");
        tr.innerHTML = `
            <td><strong>${r.step}</strong></td>
            <td><span class="badge ${r.grid === 'F6' ? 'badge-cyan' : 'badge-amber'}">${r.grid}</span></td>
            <td><code>[${r.coord.x.toFixed(2)}, ${r.coord.y.toFixed(2)}]</code></td>
            <td>${r.alt}</td>
            <td><span class="text-emerald font-semibold">${r.hold}</span></td>
        `;
        tbody.appendChild(tr);
    }
}

function updatePermutationsTable() {
    const tbody = document.querySelector("#all-perms-table tbody");
    tbody.innerHTML = "";

    const search = document.getElementById("perm-search").value.toUpperCase().trim();
    let count = 0;

    for (let i = 0; i < state.allPermutations.length; i++) {
        const p = state.allPermutations[i];
        const routeStr = `F6 &rarr; ${p.order.join(" &rarr; ")}`;
        const plainStr = `F6 ${p.order.join(" ")}`;

        if (search && !plainStr.includes(search)) {
            continue;
        }

        count++;
        const delta = p.total - state.tspDist;
        const isBest = i === 0;
        const isGreedy = p.order.join(",") === state.greedyOrder.join(",");

        let statusHtml = "";
        if (isBest) {
            statusHtml = '<span class="badge badge-emerald">Optimal (Best)</span>';
        } else if (isGreedy) {
            statusHtml = '<span class="badge badge-amber">Greedy NN Choice</span>';
        } else {
            statusHtml = '<span class="badge badge-muted">Suboptimal</span>';
        }

        const tr = document.createElement("tr");
        tr.className = `perm-row ${isBest ? 'row-best' : ''} ${state.previewRoute && state.previewRoute.join(',') === p.order.join(',') ? 'row-active' : ''}`;
        tr.innerHTML = `
            <td>#${i + 1}</td>
            <td class="font-mono">${routeStr}</td>
            <td>${p.legs[0].toFixed(2)}m</td>
            <td>${p.legs[1].toFixed(2)}m</td>
            <td>${p.legs[2].toFixed(2)}m</td>
            <td>${p.legs[3].toFixed(2)}m</td>
            <td><strong>${p.total.toFixed(2)}m</strong></td>
            <td>${delta === 0 ? '--' : `+${delta.toFixed(2)}m`}</td>
            <td>${statusHtml}</td>
        `;

        // Click row to preview on canvas!
        tr.addEventListener("click", () => {
            state.previewRoute = p.order;
            resetAnimation();
            updateHUD();
            document.querySelectorAll(".perm-row").forEach(r => r.classList.remove("row-active"));
            tr.classList.add("row-active");
        });

        tbody.appendChild(tr);
    }

    document.getElementById("perm-count").textContent = `Showing ${count} of 24 candidate routes`;
}

function resetAnimation() {
    state.animating = false;
    state.paused = false;
    state.droneLeg = 0;
    state.legProgress = 0;
    state.holdRemaining = 0;
    state.flightClock = 0;
    state.trail = [];
    const pStart = gridToPixel("F6");
    state.dronePos = { x: pStart.x, y: pStart.y, z: 13 };
    state.droneAngle = 0;

    document.getElementById("btn-play").disabled = false;
    document.getElementById("btn-pause").disabled = true;
    document.getElementById("clock-display").textContent = "0.0s";
    document.getElementById("leg-display").textContent = "Takeoff (F6)";
    document.getElementById("hold-display").textContent = "--";
}

// --- 7. Event Listeners -----------------------------------------------------
function initEvents() {
    // Canvas Mouse Interaction
    canvas.addEventListener("mousemove", (e) => {
        const rect = canvas.getBoundingClientRect();
        const scaleX = canvas.width / rect.width;
        const scaleY = canvas.height / rect.height;
        const px = (e.clientX - rect.left) * scaleX;
        const py = (e.clientY - rect.top) * scaleY;

        const nearest = pixelToNearestGrid(px, py);
        state.hoveredGrid = nearest;

        const tip = document.getElementById("canvas-tooltip");
        const coord = getMetricCoord(nearest);
        tip.innerHTML = `<strong>${nearest}</strong> &bull; X: ${coord.x.toFixed(2)}m, Y: ${coord.y.toFixed(2)}m<br><span style="color:#94a3b8">Click to toggle survivor</span>`;
        tip.style.left = `${e.clientX - rect.left + 15}px`;
        tip.style.top = `${e.clientY - rect.top + 15}px`;
        tip.classList.remove("hidden");
    });

    canvas.addEventListener("mouseleave", () => {
        state.hoveredGrid = null;
        document.getElementById("canvas-tooltip").classList.add("hidden");
    });

    // Click canvas to add/move survivors
    canvas.addEventListener("click", () => {
        if (!state.hoveredGrid || state.hoveredGrid === "F6") return;

        const idx = state.survivors.indexOf(state.hoveredGrid);
        if (idx !== -1) {
            // Remove if > 4, or do nothing
            if (state.survivors.length > 4) {
                state.survivors.splice(idx, 1);
            }
        } else {
            if (state.survivors.length >= 4) {
                // Replace the oldest survivor
                state.survivors.shift();
            }
            state.survivors.push(state.hoveredGrid);
        }

        state.previewRoute = null;
        solveRouting();
        resetAnimation();
        updateHUD();
    });

    // Preset Buttons
    document.querySelectorAll(".btn-preset").forEach(btn => {
        btn.addEventListener("click", () => {
            document.querySelectorAll(".btn-preset").forEach(b => b.classList.remove("active"));
            btn.classList.add("active");

            const preset = btn.dataset.preset;
            if (preset === "actual") {
                state.survivors = ["H7", "H5", "E4", "D7"];
            } else if (preset === "guide") {
                state.survivors = ["C9", "D5", "H4", "J7"];
            } else if (preset === "perimeter") {
                state.survivors = ["B2", "J2", "J10", "B10"];
            } else if (preset === "random") {
                const candidates = Object.keys(GRID_COORDINATES).filter(k => k !== "F6");
                const shuffled = candidates.sort(() => 0.5 - Math.random());
                state.survivors = shuffled.slice(0, 4);
            }

            state.previewRoute = null;
            solveRouting();
            resetAnimation();
            updateHUD();
        });
    });

    // Algorithm Radio Toggle
    document.querySelectorAll("input[name='planner-algo']").forEach(input => {
        input.addEventListener("change", (e) => {
            state.selectedAlgo = e.target.value;
            state.previewRoute = null;
            document.querySelectorAll(".algo-radio-card").forEach(c => c.classList.remove("active"));
            e.target.closest(".algo-radio-card").classList.add("active");
            resetAnimation();
            updateHUD();
        });
    });

    // Playback Controls
    document.getElementById("btn-play").addEventListener("click", () => {
        state.animating = true;
        state.paused = false;
        document.getElementById("btn-play").disabled = true;
        document.getElementById("btn-pause").disabled = false;
    });

    document.getElementById("btn-pause").addEventListener("click", () => {
        state.paused = true;
        document.getElementById("btn-play").disabled = false;
        document.getElementById("btn-pause").disabled = true;
    });

    document.getElementById("btn-reset").addEventListener("click", () => {
        resetAnimation();
    });

    // Speed Selector
    document.querySelectorAll(".btn-speed").forEach(btn => {
        btn.addEventListener("click", () => {
            document.querySelectorAll(".btn-speed").forEach(b => b.classList.remove("active"));
            btn.classList.add("active");
            state.playbackSpeed = parseFloat(btn.dataset.speed);
        });
    });

    // Permutation search input
    document.getElementById("perm-search").addEventListener("input", () => {
        updatePermutationsTable();
    });
}

// --- 8. Initialization ------------------------------------------------------
function init() {
    solveRouting();
    resetAnimation();
    updateHUD();
    initEvents();
    requestAnimationFrame(updateAnimation);
}

document.addEventListener("DOMContentLoaded", init);
