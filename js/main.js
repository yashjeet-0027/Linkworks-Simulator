// main.js — Application entry point
// Imports from geometry.js, construction.js, canvas.js
// Replaces all fetch() calls with direct function calls / Web Worker usage.

import { dist, projectPointOnLine, circleIntersection, rigidTransform, calcTransmissionAngle } from './geometry.js';
import { computeFullState } from './construction.js';
import {
    drawGrid, drawPoint, drawLine, drawDashedLine, drawCircle,
    drawGroundSymbol, drawLinkage, drawCouplerCurve
} from './canvas.js';

// ==========================================
// --- DOM References ---
// ==========================================
const canvas = document.getElementById('mechCanvas');
const ctx = canvas.getContext('2d');
const warningBox = document.getElementById('warning-box');
let width, height;

// ==========================================
// --- State Variables ---
// ==========================================
let synthesisMode = 4;
let currentStep = 0;
let showConstruction = true;
let points = { C: [], HR: null, A1: null, A3: null, HC: null, A2: null, A4: null, B1: null, P2: null, P4: null };
let lines = { c13: { p1: null, p2: null }, a13: { p1: null, p2: null } };
let radii = { R: 100, r: 100 };
let config = { A1: 0, A2: 0, A3: 0, A4: 0 };
let fullCouplerCurve = [];
let circuitVisited = [false, false, false, false];
let transform = { x: 0, y: 0, k: 1 };
let isPanning = false, startPan = { x: 0, y: 0 }, hasMoved = false;

// Pivot drag state
let isDraggingHR = false;
let isDraggingHC = false;
let hoveredPivot = null; // 'HR' | 'HC' | null — for cursor feedback

/** Return true if screen point (sx, sy) is within hitRadius px of world point p. */
function hitTestPivot(p, sx, sy, hitRadius = 10) {
    if (!p) return false;
    const sp = toScreen(p);
    return Math.hypot(sx - sp.x, sy - sp.y) <= hitRadius;
}
let anim = {
    initialized: false, playing: false, reqId: null,
    angle: 0, speedMult: 0.025, dir: 1, lastB: null,
    path: [], lengths: {},
    A_current: null, B_current: null, C_current: null,
    B_alt: null,    // alternate kinematic branch (second circle intersection)
    C_alt: null     // coupler point on alternate branch
};

// Web Worker reference for autosolve
let autosolveWorker = null;

const stepsInfo = [
    { title: 'SETUP', desc: 'Precision points placed. Run Auto-Solve or click Next to build manually.' },
    { title: 'STEP 1: Mid-normal C₁C₃', desc: 'Perpendicular bisector c₁₃ computed.' },
    { title: 'STEP 2: Choose H_R', desc: 'Click on dashed line c₁₃ to place H_R.' },
    { title: 'STEP 3: Arc from H_R', desc: 'Adjust R in the inspector below.' },
    { title: 'STEP 4: Find A₁ and A₃', desc: 'Adjust r in the inspector below.' },
    { title: 'STEP 5: Mid-normal A₁A₃', desc: 'Bisector a₁₃ computed.' },
    { title: 'STEP 6: Choose H_C', desc: 'Click on line a₁₃ to place H_C.' },
    { title: 'STEP 7: Crank Circle', desc: 'Drawn automatically.' },
    { title: 'STEP 8: Find A₂ and A₄', desc: 'Finding A₂, A₄ on crank circle.' },
    { title: 'STEP 9: Coupler Edges', desc: 'Coupler edges drawn.' },
    { title: 'STEP 10: Inversion on Follower', desc: 'Locate Point 2 & 4 via rigid transformation.' },
    { title: 'STEP 11: Compute Rocker Tip B₁', desc: 'B₁ found as circumcenter. Analyses defects.' }
];

// ==========================================
// --- Coordinate Transforms ---
// ==========================================
function toScreen(p) { return { x: p.x * transform.k + transform.x, y: p.y * transform.k + transform.y }; }
function toWorld(p) { return { x: (p.x - transform.x) / transform.k, y: (p.y - transform.y) / transform.k }; }

// ==========================================
// --- Warning ---
// ==========================================
function setWarning(msg) {
    if (msg) { warningBox.innerHTML = msg; warningBox.style.display = 'block'; }
    else { warningBox.style.display = 'none'; }
}

// ==========================================
// --- Pivot Coordinate Inspector Sync ---
// ==========================================

/** Write current HR/HC coords into the inspector inputs. */
function syncPivotInputs() {
    const hrX = document.getElementById('hr-x');
    const hrY = document.getElementById('hr-y');
    const hcX = document.getElementById('hc-x');
    const hcY = document.getElementById('hc-y');

    if (points.HR) {
        hrX.value = Math.round(points.HR.x);
        hrY.value = Math.round(points.HR.y);
    } else {
        hrX.value = '';
        hrY.value = '';
    }
    if (points.HC) {
        hcX.value = Math.round(points.HC.x);
        hcY.value = Math.round(points.HC.y);
    } else {
        hcX.value = '';
        hcY.value = '';
    }
}

/** Apply HR/HC values typed into the inspector. */
function applyPivotInputs() {
    const hrX = parseFloat(document.getElementById('hr-x').value);
    const hrY = parseFloat(document.getElementById('hr-y').value);
    const hcX = parseFloat(document.getElementById('hc-x').value);
    const hcY = parseFloat(document.getElementById('hc-y').value);

    let changed = false;

    if (!isNaN(hrX) && !isNaN(hrY)) {
        points.HR = { x: hrX, y: hrY };
        changed = true;
        // Advance step if still at "place HR"
        if (currentStep === 2) currentStep = 3;
    }
    if (!isNaN(hcX) && !isNaN(hcY)) {
        points.HC = { x: hcX, y: hcY };
        changed = true;
        if (currentStep === 6) currentStep = 7;
    }

    if (changed) { updateUI(); calculateKinematics(); }
}

// Wire up pivot inputs — apply on Enter or blur
['hr-x', 'hr-y', 'hc-x', 'hc-y'].forEach(id => {
    const el = document.getElementById(id);
    el.addEventListener('keydown', e => { if (e.key === 'Enter') applyPivotInputs(); });
    el.addEventListener('blur', applyPivotInputs);
});

// ==========================================
// --- Core: Replace fetch('/calculate') ---
// ==========================================
function calculateKinematics() {
    const data = computeFullState(
        points.C, points.HR, points.HC,
        radii.R, radii.r, config, false, synthesisMode
    );

    points.A1 = data.A1; points.A2 = data.A2;
    points.A3 = data.A3; points.A4 = data.A4;
    points.P2 = data.P2; points.P4 = data.P4;
    points.B1 = data.B1;
    lines.c13 = data.lines.c13 || { p1: null, p2: null };
    lines.a13 = data.lines.a13 || { p1: null, p2: null };
    fullCouplerCurve = data.fullCouplerCurve;
    circuitVisited = data.circuitVisited;

    syncPivotInputs();
    updateConfigUI();
    draw();
}

// ==========================================
// --- Auto-Solve (Web Worker) ---
// ==========================================

function updateProgressBar(percent, found) {
    document.getElementById('progress-bar').style.width = `${percent}%`;
    document.getElementById('autosolve-status').innerHTML =
        `Searching… ${percent}% — ${found} found`;
}

function displayResults(solutions) {
    const btnAuto = document.getElementById('btn-autosolve');
    const btnCancel = document.getElementById('btn-cancel-search');
    const status = document.getElementById('autosolve-status');

    btnCancel.style.display = 'none';
    btnAuto.disabled = false;
    status.innerHTML = 'Search complete.';
    document.getElementById('progress-bar').style.width = '100%';

    renderResults(solutions);
}

document.getElementById('btn-autosolve').addEventListener('click', () => {
    if (points.C.length < 4) return;

    const progContainer = document.getElementById('progress-container');
    const status = document.getElementById('autosolve-status');
    const resultsPanel = document.getElementById('autosolve-results');
    const btnCancel = document.getElementById('btn-cancel-search');
    const btnAuto = document.getElementById('btn-autosolve');

    if (autosolveWorker) { autosolveWorker.terminate(); autosolveWorker = null; }

    progContainer.style.display = 'block';
    btnCancel.style.display = 'inline-block';
    btnAuto.disabled = true;
    resultsPanel.innerHTML = '';
    status.innerHTML = 'Starting search…';
    document.getElementById('progress-bar').style.width = '0%';

    autosolveWorker = new Worker('./js/autosolve.worker.js?v=' + Date.now());

    autosolveWorker.onmessage = (e) => {
        if (e.data.type === 'progress') updateProgressBar(e.data.percent, e.data.found);
        if (e.data.type === 'done') {
            autosolveWorker.terminate();
            autosolveWorker = null;
            displayResults(e.data.solutions);
        }
    };
    autosolveWorker.onerror = (err) => {
        console.error('Worker error:', err);
        status.innerHTML = 'Error in worker.';
        btnAuto.disabled = false;
        btnCancel.style.display = 'none';
    };
    const driver = document.getElementById('filter-driver').value;  // 'any'|'crank'|'rocker'
    const driven = document.getElementById('filter-driven').value;  // 'any'|'crank'|'rocker'
    const minGamma = parseInt(document.getElementById('filter-min-gamma').value) || 40;
    const maxRatio = parseFloat(document.getElementById('slider-max-ratio').value);
    const enableRatio = document.getElementById('filter-enable-ratio').checked;
    const enableOrder = document.getElementById('filter-enable-order').checked;
    const enableGamma = document.getElementById('filter-enable-gamma').checked;
    autosolveWorker.postMessage({ C: points.C, filters: { driver, driven, minGamma, enableRatio, enableOrder, enableGamma }, maxRatio: maxRatio, synthesisMode: synthesisMode });
});

document.getElementById('btn-cancel-search').addEventListener('click', () => {
    if (autosolveWorker) { autosolveWorker.terminate(); autosolveWorker = null; }
    document.getElementById('autosolve-status').innerHTML = 'Search cancelled.';
    document.getElementById('btn-cancel-search').style.display = 'none';
    document.getElementById('btn-autosolve').disabled = false;
    document.getElementById('progress-bar').style.width = '0%';
});

// ==========================================
// --- Render Results ---
// ==========================================
function renderResults(solutions) {
    const resultsPanel = document.getElementById('autosolve-results');
    resultsPanel.innerHTML = '';

    if (solutions.length === 0) {
        resultsPanel.innerHTML = '<div style="color:#fca5a5; font-size:0.8rem; padding:5px;">No continuous circuits found. Try spreading the precision points further apart.</div>';
        return;
    }

    solutions.forEach((sol, idx) => {
        const card = document.createElement('div');
        card.className = 'sol-card';
        card.id = `sol-card-${idx}`;
        const icon = sol.grashof ? '✅' : '⚠️';
        const grashofText = sol.grashof ? '(Grashof)' : '(Non-Grashof)';
        card.innerHTML = `<strong>${icon} Sol ${idx + 1} — ${sol.type} <span style="font-size:0.7rem; opacity:0.7;">${grashofText}</span></strong>
            <div style="opacity:0.75; font-size:0.75rem;">Cr: ${sol.linkLengths.crank.toFixed(1)} | Co: ${sol.linkLengths.coupler.toFixed(1)}<br>
            Ro: ${sol.linkLengths.rocker.toFixed(1)} | Fr: ${sol.linkLengths.frame.toFixed(1)}<br>
            <span style="color:var(--accent);">γ_min: ${sol.minGamma !== undefined ? sol.minGamma.toFixed(1) : '--'}° | γ_max: ${sol.maxGamma !== undefined ? sol.maxGamma.toFixed(1) : '--'}°</span></div>`;
        card.onclick = () => loadSolution(sol, idx);
        resultsPanel.appendChild(card);
    });
}

function loadSolution(sol, idx) {
    stopAnimationHelper();

    document.querySelectorAll('.sol-card').forEach((c, i) => {
        if (i === idx) c.classList.add('active'); else c.classList.remove('active');
    });

    const savedCPoints = points.C;
    points = { C: savedCPoints, HR: null, A1: null, A3: null, HC: null, A2: null, A4: null, B1: null, P2: null, P4: null };
    lines = { c13: { p1: null, p2: null }, a13: { p1: null, p2: null } };
    fullCouplerCurve = []; circuitVisited = [false, false, false, false];

    points.HR = sol.HR; points.HC = sol.HC;
    radii.R = sol.R; radii.r = sol.r;
    config = sol.config;

    // Sync sliders
    document.getElementById('slider-R').value = sol.R;
    document.getElementById('val-R').innerText = Math.round(sol.R);
    document.getElementById('slider-r').value = sol.r;
    document.getElementById('val-r').innerText = Math.round(sol.r);

    currentStep = 11;
    updateUI(); draw();
    calculateKinematics(); // also calls syncPivotInputs()
    setTimeout(autoFit, 100);
}

// ==========================================
// --- JSON Export ---
// ==========================================
document.getElementById('btn-export').addEventListener('click', () => {
    if (currentStep < 11 || !points.HC || !points.B1 || !points.A1 || !points.HR) return;

    const L1 = dist(points.HC, points.A1);
    const L2 = dist(points.A1, points.B1);
    const L3 = dist(points.B1, points.HR);
    const L4 = dist(points.HC, points.HR);
    const sorted = [L1, L2, L3, L4].sort((a, b) => a - b);
    const [s, , , l] = sorted;
    const p2 = sorted[1], q = sorted[2];
    const grashof = (s + l <= p2 + q);
    let type = 'Non-Grashof';
    if (grashof) {
        if (s === L1) type = 'Crank-Rocker';
        else if (s === L4) type = 'Double-Crank';
        else type = 'Double-Rocker';
    }

    const data = {
        precision_points: points.C,
        fixed_pivots: { HC: points.HC, HR: points.HR },
        link_lengths: { crank: L1, coupler: L2, rocker: L3, frame: L4 },
        grashof, type, driver: 'crank', driven: 'rocker'
    };

    const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url; a.download = 'linkage_solution.json'; a.click();
    URL.revokeObjectURL(url);
});

// ==========================================
// --- Auto-Fit ---
// ==========================================
function autoFit() {
    let activePts = [];
    if (points.C) activePts.push(...points.C);
    if (points.HR) activePts.push(points.HR);
    if (points.HC) activePts.push(points.HC);
    ['A1', 'A2', 'A3', 'A4', 'B1', 'P2', 'P4'].forEach(k => { if (points[k]) activePts.push(points[k]); });
    if (fullCouplerCurve.length > 0) activePts.push(...fullCouplerCurve);

    if (activePts.length === 0) return;

    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    activePts.forEach(p => {
        if (p.x < minX) minX = p.x; if (p.x > maxX) maxX = p.x;
        if (p.y < minY) minY = p.y; if (p.y > maxY) maxY = p.y;
    });

    const pWidth = Math.max(maxX - minX, 10);
    const pHeight = Math.max(maxY - minY, 10);
    const scaleX = (width * 0.8) / pWidth;
    const scaleY = (height * 0.8) / pHeight;
    let newScale = Math.min(scaleX, scaleY);
    newScale = Math.min(Math.max(newScale, 0.1), 10);

    const cx = (minX + maxX) / 2;
    const cy = (minY + maxY) / 2;
    transform.k = newScale;
    transform.x = width / 2 - cx * newScale;
    transform.y = height / 2 - cy * newScale;

    document.getElementById('val-zoom').innerText = Math.round(newScale * 100);
    document.getElementById('slider-zoom').value = Math.max(-100, Math.min(100, 100 * Math.log10(newScale)));

    if (!anim.playing) draw();
}
document.getElementById('btn-autofit').addEventListener('click', autoFit);

// ==========================================
// --- Legend ---
// ==========================================
function updateLegend() {
    const box = document.getElementById('legend-content');
    if (!box) return;
    let html = '';
    if (points.C.length > 0) html += `<div><span style="color:#f87171; font-size:1.1em;">●</span> C — Precision points</div>`;
    if (points.A1 || points.A2) html += `<div><span style="color:#fb923c; font-size:1.1em;">●</span> A — Crank positions</div>`;
    if (points.HC) html += `<div><span style="color:#94a3b8; font-size:1.1em;">●</span> H<sub>C</sub> — Crank pivot</div>`;
    if (points.HR) html += `<div><span style="color:#94a3b8; font-size:1.1em;">●</span> H<sub>R</sub> — Rocker pivot</div>`;
    if (points.B1) html += `<div><span style="color:#94a3b8; font-size:1.1em;">●</span> B₁ — Rocker tip</div>`;
    if (currentStep >= 7 && points.HC && points.A1) html += `<div><span style="color:#4ade80; font-weight:700;">—</span> Crank circle</div>`;
    if (fullCouplerCurve.length > 0) html += `<div><span style="color:#f87171; font-weight:700;">—</span> Coupler curve</div>`;
    box.innerHTML = html;
}
document.getElementById('btn-toggle-legend').addEventListener('click', () => {
    const box = document.getElementById('legend-box');
    box.style.display = box.style.display === 'none' ? 'block' : 'none';
});

// ==========================================
// --- Draw ---
// ==========================================
/** Draw a glowing highlight ring around a pivot point (world coords). */
function drawPivotHighlight(p, color) {
    if (!p) return;
    const sp = toScreen(p);
    ctx.save();
    ctx.beginPath();
    ctx.arc(sp.x, sp.y, 14, 0, Math.PI * 2);
    ctx.strokeStyle = color;
    ctx.lineWidth = 2.5;
    ctx.shadowColor = color;
    ctx.shadowBlur = 12;
    ctx.stroke();
    ctx.restore();
}

function draw() {
    ctx.clearRect(0, 0, width, height);
    drawGrid(ctx, width, height, transform);
    let activeWarning = '';

    // Draw precision points C
    if (currentStep >= 11 && fullCouplerCurve.length > 0) {
        points.C.forEach((p, i) => {
            const color = circuitVisited[i] ? 'rgba(40, 167, 69, 0.9)' : 'rgba(220, 53, 69, 0.9)';
            const label = `C${i + 1}` + (circuitVisited[i] ? ' ✓' : ' ❌');
            drawPoint(ctx, p, label, transform, color, 6);
        });
    } else {
        points.C.forEach((p, i) => drawPoint(ctx, p, `C${i + 1}`, transform, 'rgba(255, 0, 0, 0.9)', 6));
    }

    // Coupler curve (static preview)
    if (fullCouplerCurve.length > 1) drawCouplerCurve(ctx, fullCouplerCurve, transform, false);
    // Animated path trace
    if (anim.path && anim.path.length > 1) drawCouplerCurve(ctx, anim.path, transform, true);

    if (showConstruction) {
        ctx.save();
        if (currentStep >= 11) ctx.globalAlpha = 0.15;

        if (currentStep >= 1 && points.C.length >= 3) {
            drawDashedLine(ctx, points.C[0], points.C[2], transform, '#999');
            if (lines.c13 && lines.c13.p1) drawDashedLine(ctx, lines.c13.p1, lines.c13.p2, transform, '#999');
        }
        if (currentStep >= 2) drawGroundSymbol(ctx, points.HR, 'H_R', transform);
        if (currentStep >= 3) drawCircle(ctx, points.HR, radii.R, transform, '#999', true);
        if (currentStep >= 4) {
            drawCircle(ctx, points.C[0], radii.r, transform, '#ccc', true);
            drawCircle(ctx, points.C[2], radii.r, transform, '#ccc', true);
            if (points.A1) drawPoint(ctx, points.A1, 'A1', transform, 'orange');
            if (points.A3) drawPoint(ctx, points.A3, 'A3', transform, 'orange');
            if (currentStep === 4 && (!points.A1 || !points.A3)) activeWarning = '⚠️ Arcs do not intersect! Adjust R and r.';
        }
        if (currentStep >= 5 && points.A1 && points.A3) {
            if (lines.a13 && lines.a13.p1) drawDashedLine(ctx, lines.a13.p1, lines.a13.p2, transform, '#999');
            drawDashedLine(ctx, points.A1, points.A3, transform, '#999');
        }
        if (currentStep >= 6 && points.HC) drawGroundSymbol(ctx, points.HC, 'H_C', transform);
        if (currentStep >= 7 && points.HC && points.A1) {
            drawCircle(ctx, points.HC, dist(points.HC, points.A1), transform, 'green');
        }
        if (currentStep >= 8 && points.HC && points.A1) {
            drawCircle(ctx, points.C[1], radii.r, transform, '#ddd', true);
            drawCircle(ctx, points.C[3], radii.r, transform, '#ddd', true);
            if (points.A2) drawPoint(ctx, points.A2, 'A2', transform, 'orange');
            if (points.A4) drawPoint(ctx, points.A4, 'A4', transform, 'orange');
            if (currentStep === 8 && (!points.A2 || !points.A4)) activeWarning = '⚠️ Arcs cannot reach Crank Circle.';
        }
        if (currentStep >= 9) {
            ['A1', 'A2', 'A3', 'A4'].forEach((aName, i) => {
                if (points[aName]) drawLine(ctx, points.C[i], points[aName], transform, 'blue');
            });
        }
        if (currentStep >= 10 && points.P2 && points.P4) {
            drawPoint(ctx, points.P2, 'P2', transform, 'magenta');
            drawPoint(ctx, points.P4, 'P4', transform, 'magenta');
            drawDashedLine(ctx, points.HR, points.P2, transform, '#f0f');
            drawDashedLine(ctx, points.HR, points.P4, transform, '#f0f');
            drawDashedLine(ctx, points.P2, points.P4, transform, '#f0f');
        }
        if (currentStep >= 11 && points.HR) {
            if (points.B1) drawCircle(ctx, points.B1, dist(points.B1, points.HR), transform, '#6f42c1', true);
            else if (currentStep === 11) activeWarning = '⚠️ Algorithm failed: Collinear inversion points.';
        }
        ctx.restore();
    }

    // Final linkage overlay
    // ── Ghost / alternate-branch (rendered first, underneath primary) ──
    if (currentStep >= 11 && anim.initialized && anim.B_alt && anim.C_alt) {
        const drawA = anim.A_current; // same crank position for both branches
        drawLinkage(ctx, points.HC, points.HR, drawA, anim.B_alt, anim.C_alt, transform, true);
        // Small ghost dots so the alternate nodes are clearly identifiable
        drawPoint(ctx, anim.B_alt,  '', transform, 'rgba(180,160,220,0.55)', 4);
        drawPoint(ctx, anim.C_alt,  '', transform, 'rgba(200,200,255,0.55)', 4);
    }
    // ── Primary linkage ──
    if (currentStep >= 11 && points.HC && points.B1 && points.A1 && points.HR) {
        const drawA = anim.initialized ? anim.A_current : points.A1;
        const drawB = anim.initialized ? anim.B_current : points.B1;
        const drawC = anim.initialized ? anim.C_current : points.C[0];

        const currentGamma = calcTransmissionAngle(drawA, drawB, points.HR);
        const liveGammaEl = document.getElementById('live-gamma');
        if (liveGammaEl) liveGammaEl.innerHTML = `Live γ: ${currentGamma.toFixed(1)}°`;

        drawLinkage(ctx, points.HC, points.HR, drawA, drawB, drawC, transform, false, currentGamma);
        drawGroundSymbol(ctx, points.HC, 'H_C', transform);
        drawGroundSymbol(ctx, points.HR, 'H_R', transform);
        drawPoint(ctx, drawA, 'A', transform, 'orange', 6);
        drawPoint(ctx, drawB, 'B₁', transform, 'black', 6);
        drawPoint(ctx, drawC, anim.initialized ? 'Tracker' : 'C₁', transform, 'red', 6);
    } else {
        const liveGammaEl = document.getElementById('live-gamma');
        if (liveGammaEl) liveGammaEl.innerHTML = `Live γ: --°`;
    }

    // Pivot drag / hover highlight rings (drawn on top of everything)
    if (isDraggingHR || hoveredPivot === 'HR') drawPivotHighlight(points.HR, '#60a5fa');
    if (isDraggingHC || hoveredPivot === 'HC') drawPivotHighlight(points.HC, '#34d399');

    setWarning(activeWarning);
    updateLegend();
}

// ==========================================
// --- Animation Loop ---
// ==========================================
function animationLoop() {
    if (!anim.playing) return;
    anim.angle += anim.speedMult * anim.dir;
    const A_next = {
        x: points.HC.x + anim.lengths.L1 * Math.cos(anim.angle),
        y: points.HC.y + anim.lengths.L1 * Math.sin(anim.angle)
    };
    const intersects = circleIntersection(A_next, anim.lengths.L2, points.HR, anim.lengths.L3);
    if (intersects.length === 0) {
        anim.dir *= -1;
        anim.angle += anim.speedMult * anim.dir * 2;
    } else {
        // Identify primary (closest to lastB) and alternate branch
        let B_next, B_other;
        if (intersects.length > 1) {
            const d1 = dist(intersects[0], anim.lastB);
            const d2 = dist(intersects[1], anim.lastB);
            if (d2 < d1) {
                B_next  = intersects[1];
                B_other = intersects[0];
            } else {
                B_next  = intersects[0];
                B_other = intersects[1];
            }
            anim.B_alt = B_other;
            anim.C_alt = rigidTransform(points.C[0], points.A1, points.B1, A_next, B_other);
        } else {
            B_next = intersects[0];
            anim.B_alt = null;
            anim.C_alt = null;
        }

        anim.lastB = B_next;
        anim.A_current = A_next;
        anim.B_current = B_next;
        anim.C_current = rigidTransform(points.C[0], points.A1, points.B1, A_next, B_next);
        anim.path.push(anim.C_current);
        if (anim.path.length > 800) anim.path.shift();

        // Keep Switch Branch button enabled only when an alternate branch exists
        const btnBranch = document.getElementById('btn-switch-branch');
        if (btnBranch) {
            const hasAlt = anim.B_alt !== null;
            btnBranch.disabled = !hasAlt;
            btnBranch.style.opacity  = hasAlt ? '1'   : '0.45';
            btnBranch.style.cursor   = hasAlt ? 'pointer' : 'not-allowed';
        }
    }
    draw();
    anim.reqId = requestAnimationFrame(animationLoop);
}

// ==========================================
// --- Resize (canvas fills #canvas-area) ---
// ==========================================
function resize() {
    const area = document.getElementById('canvas-area');
    canvas.width = area.clientWidth;
    canvas.height = area.clientHeight;
    width = canvas.width; height = canvas.height;
    if (transform.x === 0 && transform.y === 0 && width > 0) {
        transform.x = width / 2; transform.y = height / 2;
    }
    draw();
}
window.addEventListener('resize', resize);

// ==========================================
// --- UI Update Helpers ---
// ==========================================
function updateConfigUI() {
    document.getElementById('btn-cfg-a1').innerText = `A₁: ${config.A1 === 0 ? '●1 / 2' : '1 / ●2'}`;
    document.getElementById('btn-cfg-a3').innerText = `A₃: ${config.A3 === 0 ? '●1 / 2' : '1 / ●2'}`;
    document.getElementById('btn-cfg-a2').innerText = `A₂: ${config.A2 === 0 ? '●1 / 2' : '1 / ●2'}`;
    document.getElementById('btn-cfg-a4').innerText = `A₄: ${config.A4 === 0 ? '●1 / 2' : '1 / ●2'}`;
    document.getElementById('btn-cfg-a2').disabled = currentStep < 8;
    document.getElementById('btn-cfg-a4').disabled = currentStep < 8;
    document.getElementById('btn-autosolve').disabled = points.C.length < synthesisMode;
    document.getElementById('btn-export').disabled = (currentStep < 11 || !points.HC || !points.B1);

    const checkEl = document.getElementById('config-circuit-check');
    if (currentStep >= 11 && fullCouplerCurve.length > 0) {
        const reached = circuitVisited.filter(v => v).length;
        checkEl.style.display = 'block';
        if (reached === 4) {
            checkEl.innerHTML = '✅ All 4 points on same circuit';
            checkEl.style.color = '#22c55e';
        } else {
            checkEl.innerHTML = `⚠️ Circuit defect: ${reached}/4 points reached`;
            checkEl.style.color = '#ef4444';
        }
    } else {
        checkEl.style.display = 'none';
    }
}

function setDisplay(id, show, mode = 'flex') {
    const el = document.getElementById(id);
    if (el) el.style.display = show ? mode : 'none';
}

function updateUI() {
    for (let i = 0; i < stepsInfo.length; i++) {
        const el = document.getElementById(`step-${i}`);
        if (el) el.className = `step ${i === currentStep ? 'active' : ''} ${i < currentStep ? 'done' : ''}`;
    }
    const activeEl = document.getElementById(`step-${currentStep}`);
    if (activeEl) activeEl.scrollIntoView({ behavior: 'smooth', block: 'nearest' });

    // Inspector panel visibility
    const showR  = (currentStep === 3 || currentStep === 4 || currentStep >= 11);
    const showR_small = (currentStep >= 4);
    const showCfg = (currentStep >= 4);

    setDisplay('slider-R-group', showR, 'flex');
    setDisplay('div-R',          showR, 'block');
    setDisplay('slider-r-group', showR_small, 'flex');
    setDisplay('div-r',          showR_small, 'block');
    setDisplay('config-panel',   showCfg, 'flex');

    // Sidebar controls visibility
    setDisplay('btn-next',         currentStep < 11,  'block');
    setDisplay('btn-toggle-const', currentStep >= 11, 'block');
    setDisplay('animation-palette', currentStep >= 11, 'flex');

    updateConfigUI();
}

function initUI() {
    const container = document.getElementById('steps-container');
    container.innerHTML = '';
    stepsInfo.forEach((step, i) => {
        const div = document.createElement('div');
        div.className = `step ${i === currentStep ? 'active' : ''} ${i < currentStep ? 'done' : ''}`;
        div.id = `step-${i}`;
        div.innerHTML = `<div class="step-title">${step.title}</div><div class="step-desc">${step.desc}</div>`;
        container.appendChild(div);
    });
    updateUI();
}

function stopAnimationHelper() {
    anim.initialized = false;
    anim.playing = false;
    cancelAnimationFrame(anim.reqId);
    const btnPlay = document.getElementById('btn-play-pause');
    if (btnPlay) {
        btnPlay.innerText = 'Play Simulation';
        btnPlay.style.backgroundColor = '';
        btnPlay.style.color = '';
    }
}

// ==========================================
// --- Splitter Drag Logic ---
// ==========================================
function makeSplitter(splitterId, options) {
    const splitter = document.getElementById(splitterId);
    const {
        getTarget,          // () => HTMLElement to resize
        getSize,            // (el) => current px size
        setSize,            // (el, px) => set the size
        axis,               // 'x' | 'y'
        min = 150,
        max = Infinity,
        onDone              // optional callback after drag
    } = options;

    let dragging = false;
    let startPos = 0;
    let startSize = 0;

    splitter.addEventListener('mousedown', (e) => {
        e.preventDefault();
        dragging = true;
        startPos = axis === 'x' ? e.clientX : e.clientY;
        startSize = getSize(getTarget());
        splitter.classList.add('dragging');
        document.body.style.userSelect = 'none';
        document.body.style.cursor = axis === 'x' ? 'col-resize' : 'row-resize';
    });

    window.addEventListener('mousemove', (e) => {
        if (!dragging) return;
        const delta = axis === 'x' ? e.clientX - startPos : e.clientY - startPos;
        const newSize = Math.min(Math.max(startSize + delta, min), max);
        setSize(getTarget(), newSize);
        if (onDone) onDone();
    });

    window.addEventListener('mouseup', () => {
        if (!dragging) return;
        dragging = false;
        splitter.classList.remove('dragging');
        document.body.style.userSelect = '';
        document.body.style.cursor = '';
        if (onDone) onDone();
        resize(); // re-measure canvas
    });
}

// Left splitter: resizes #sidebar-left horizontally
makeSplitter('splitter-left', {
    axis: 'x',
    min: 200, max: 520,
    getTarget: () => document.getElementById('sidebar-left'),
    getSize: (el) => el.getBoundingClientRect().width,
    setSize: (el, px) => {
        el.style.width = px + 'px';
        el.style.minWidth = px + 'px'; // keep CSS min-width in sync
    },
    onDone: () => resize()
});

// Bottom splitter: resizes #inspector vertically (drag up = taller inspector)
makeSplitter('splitter-bottom', {
    axis: 'y',
    min: 90, max: 280,
    getTarget: () => document.getElementById('inspector'),
    getSize: (el) => el.getBoundingClientRect().height,
    setSize: (el, px) => {
        el.style.height = px + 'px';
        el.style.minHeight = px + 'px';
    },
    onDone: () => resize()
});

// Right splitter: resizes #right-sidebar horizontally
// Note: dragging RIGHT shrinks the sidebar (delta is inverted).
makeSplitter('splitter-right', {
    axis: 'x',
    min: 200, max: 520,
    getTarget: () => document.getElementById('right-sidebar'),
    getSize: (el) => el.getBoundingClientRect().width,
    setSize: (el, px) => {
        el.style.width = px + 'px';
        el.style.minWidth = px + 'px';
    },
    onDone: () => resize()
});

// ==========================================
// --- Event Listeners ---
// ==========================================

// Config toggle buttons
['a1', 'a2', 'a3', 'a4'].forEach((a) => {
    document.getElementById(`btn-cfg-${a}`).addEventListener('click', () => {
        stopAnimationHelper();
        const key = a.toUpperCase();
        config[key] = 1 - config[key];
        calculateKinematics();
    });
});

// Apply precision-point coordinates
document.getElementById('btn-apply-coords').addEventListener('click', () => {
    const newPoints = [];
    for (let i = 1; i <= synthesisMode; i++) {
        const xVal = document.getElementById(`c${i}-x`).value;
        const yVal = document.getElementById(`c${i}-y`).value;
        if (xVal !== '' && yVal !== '') newPoints.push({ x: parseFloat(xVal), y: parseFloat(yVal) });
    }
    points.C = newPoints;
    draw(); updateConfigUI();
    if (newPoints.length === synthesisMode) calculateKinematics();
});

// ── Canvas mousedown: hit-test pivots first, then fall through to pan ──
canvas.addEventListener('mousedown', (e) => {
    const rect = canvas.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;

    // Only allow pivot drag when a pivot actually exists
    if (hitTestPivot(points.HR, sx, sy)) {
        isDraggingHR = true;
        hasMoved = false;
        stopAnimationHelper(); // stop kinematic animation while tweaking
        canvas.style.cursor = 'grabbing';
        return; // don't start pan
    }
    if (hitTestPivot(points.HC, sx, sy)) {
        isDraggingHC = true;
        hasMoved = false;
        stopAnimationHelper();
        canvas.style.cursor = 'grabbing';
        return;
    }

    // No pivot hit — normal pan
    isPanning = true;
    hasMoved = false;
    startPan = { x: e.clientX - transform.x, y: e.clientY - transform.y };
});

// ── Window mousemove: route to pivot-drag OR pan, and update hover cursor ──
window.addEventListener('mousemove', (e) => {
    const rect = canvas.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;

    // ── Pivot drag in progress ──
    if (isDraggingHR || isDraggingHC) {
        hasMoved = true;
        const worldPt = toWorld({ x: sx, y: sy });

        if (isDraggingHR) {
            points.HR = worldPt;
        } else {
            points.HC = worldPt;
        }

        // Live-sync inspector inputs while dragging
        syncPivotInputs();
        calculateKinematics(); // recalculates and redraws
        return;
    }

    // ── Normal pan ──
    if (isPanning) {
        const newX = e.clientX - startPan.x;
        const newY = e.clientY - startPan.y;
        if (Math.abs(newX - transform.x) > 3 || Math.abs(newY - transform.y) > 3) hasMoved = true;
        transform.x = newX; transform.y = newY;
        if (!anim.playing) draw();
        return;
    }

    // ── Hover cursor feedback (not dragging, not panning) ──
    const prevHovered = hoveredPivot;
    if (hitTestPivot(points.HR, sx, sy)) {
        hoveredPivot = 'HR';
        canvas.style.cursor = 'grab';
    } else if (hitTestPivot(points.HC, sx, sy)) {
        hoveredPivot = 'HC';
        canvas.style.cursor = 'grab';
    } else {
        hoveredPivot = null;
        canvas.style.cursor = 'crosshair';
    }
    // Redraw only if hover state changed (avoids constant redraws)
    if (hoveredPivot !== prevHovered && !anim.playing) draw();
});

// ── Window mouseup: end drag or pan ──
window.addEventListener('mouseup', () => {
    if (isDraggingHR || isDraggingHC) {
        isDraggingHR = false;
        isDraggingHC = false;
        canvas.style.cursor = 'crosshair';
        // One final full recalc + UI update after drag ends
        calculateKinematics();
        updateUI();
        return;
    }
    isPanning = false;
});

// Wheel zoom
canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    const delta = -e.deltaY * 0.001;
    const newScale = Math.min(Math.max(0.1, transform.k * Math.exp(delta)), 10);
    const rect = canvas.getBoundingClientRect();
    const mouseX = e.clientX - rect.left;
    const mouseY = e.clientY - rect.top;
    transform.x = mouseX - (mouseX - transform.x) * (newScale / transform.k);
    transform.y = mouseY - (mouseY - transform.y) * (newScale / transform.k);
    transform.k = newScale;
    document.getElementById('val-zoom').innerText = Math.round(newScale * 100);
    document.getElementById('slider-zoom').value = Math.max(-100, Math.min(100, 100 * Math.log10(newScale)));
    if (!anim.playing) draw();
}, { passive: false });

// Zoom slider
document.getElementById('slider-zoom').addEventListener('input', (e) => {
    const sliderVal = parseInt(e.target.value);
    const newScale = Math.pow(10, sliderVal / 100);
    document.getElementById('val-zoom').innerText = Math.round(newScale * 100);
    const centerX = width / 2, centerY = height / 2;
    transform.x = centerX - (centerX - transform.x) * (newScale / transform.k);
    transform.y = centerY - (centerY - transform.y) * (newScale / transform.k);
    transform.k = newScale;
    if (!anim.playing) draw();
});

// Canvas click (place points)
canvas.addEventListener('click', (e) => {
    if (hasMoved) return;
    const rect = canvas.getBoundingClientRect();
    const pScreen = { x: e.clientX - rect.left, y: e.clientY - rect.top };
    const p = toWorld(pScreen);
    switch (currentStep) {
        case 0:
            if (points.C.length < synthesisMode) {
                points.C.push(p);
                document.getElementById(`c${points.C.length}-x`).value = Math.round(p.x);
                document.getElementById(`c${points.C.length}-y`).value = Math.round(p.y);
            }
            break;
        case 2:
            if (lines.c13 && lines.c13.p1) {
                points.HR = projectPointOnLine(p, lines.c13.p1, lines.c13.p2);
                syncPivotInputs();
            }
            break;
        case 6:
            if (lines.a13 && lines.a13.p1) {
                points.HC = projectPointOnLine(p, lines.a13.p1, lines.a13.p2);
                syncPivotInputs();
            }
            break;
    }
    updateUI(); draw(); calculateKinematics();
});

// Radius sliders (now in inspector)
document.getElementById('slider-R').addEventListener('input', (e) => {
    radii.R = parseInt(e.target.value);
    document.getElementById('val-R').innerText = radii.R;
    calculateKinematics();
});
document.getElementById('slider-r').addEventListener('input', (e) => {
    radii.r = parseInt(e.target.value);
    document.getElementById('val-r').innerText = radii.r;
    calculateKinematics();
});

// Speed slider
document.getElementById('slider-speed').addEventListener('input', (e) => {
    const val = parseInt(e.target.value);
    document.getElementById('val-speed').innerText = val;
    anim.speedMult = val * 0.0005;
});

// Max Link Ratio slider
document.getElementById('slider-max-ratio').addEventListener('input', (e) => {
    const val = parseFloat(e.target.value);
    document.getElementById('val-max-ratio').innerText = val.toFixed(1);
});

// Toggle construction visibility
document.getElementById('btn-toggle-const').addEventListener('click', (e) => {
    showConstruction = !showConstruction;
    e.target.innerText = showConstruction ? 'Hide Construction' : 'Show Construction';
    if (!anim.playing) draw();
});

// Play/Pause
document.getElementById('btn-play-pause').addEventListener('click', (e) => {
    if (!anim.initialized) {
        anim.initialized = true; anim.playing = true;
        anim.angle = Math.atan2(points.A1.y - points.HC.y, points.A1.x - points.HC.x);
        anim.lastB = { x: points.B1.x, y: points.B1.y };
        anim.path = []; anim.dir = 1;
        anim.lengths = {
            L1: dist(points.HC, points.A1),
            L2: dist(points.A1, points.B1),
            L3: dist(points.HR, points.B1)
        };
        animationLoop();
    } else {
        anim.playing = !anim.playing;
        if (anim.playing) animationLoop();
        else cancelAnimationFrame(anim.reqId);
    }
    e.target.innerText = anim.playing ? 'Pause' : 'Play Simulation';
    e.target.style.backgroundColor = anim.playing ? '#ca8a04' : '';
    e.target.style.color = anim.playing ? '#fff' : '';
    updateUI();
});

// Stop animation
document.getElementById('btn-stop-anim').addEventListener('click', () => {
    stopAnimationHelper();
    // Clear branch state on reset
    anim.B_alt = null; anim.C_alt = null;
    const btnBranch = document.getElementById('btn-switch-branch');
    if (btnBranch) { btnBranch.disabled = true; btnBranch.style.opacity = '0.45'; btnBranch.style.cursor = 'not-allowed'; }
    draw(); updateUI();
});

// Switch kinematic branch (swap primary ↔ alternate intersection)
document.getElementById('btn-switch-branch').addEventListener('click', () => {
    if (!anim.initialized || !anim.B_alt) return;

    // Swap B and C between the two branches
    [anim.B_current, anim.B_alt]   = [anim.B_alt,   anim.B_current];
    [anim.C_current, anim.C_alt]   = [anim.C_alt,   anim.C_current];

    // Reset tracking so the animation follows the new branch continuously
    anim.lastB = anim.B_current;
    anim.path  = [];   // clear the tracer path so it doesn't show a jump

    draw(); // immediate visual update (works even when paused)
});

// Next step
document.getElementById('btn-next').addEventListener('click', () => {
    if (currentStep === 0 && points.C.length < synthesisMode) { alert(`Place ${synthesisMode} precision points first.`); return; }
    if (currentStep === 11) return;
    if (currentStep === 2 && !points.HR) { alert('Click dashed line to place H_R, or enter coordinates in the inspector.'); return; }
    if (currentStep === 4 && (!points.A1 || !points.A3)) { alert('Adjust R and r so arcs intersect.'); return; }
    if (currentStep === 6 && !points.HC) { alert('Click dashed line to place H_C, or enter coordinates in the inspector.'); return; }
    if (currentStep === 8 && (!points.A2 || !points.A4)) { alert('Arcs do not intersect crank circle.'); return; }
    if (currentStep < 11) { currentStep++; updateUI(); draw(); calculateKinematics(); }
});

// New problem
document.getElementById('btn-new-problem').addEventListener('click', () => {
    stopAnimationHelper();
    currentStep = 0; showConstruction = true;
    document.getElementById('btn-toggle-const').innerText = 'Hide Construction';
    points = { C: [], HR: null, A1: null, A3: null, HC: null, A2: null, A4: null, B1: null, P2: null, P4: null };
    lines = { c13: { p1: null, p2: null }, a13: { p1: null, p2: null } };
    fullCouplerCurve = []; circuitVisited = [false, false, false, false, false];
    config = { A1: 0, A2: 0, A3: 0, A4: 0 };
    for (let i = 1; i <= 5; i++) {
        const idX = document.getElementById(`c${i}-x`);
        if (idX) idX.value = '';
        const idY = document.getElementById(`c${i}-y`);
        if (idY) idY.value = '';
    }
    ['hr-x', 'hr-y', 'hc-x', 'hc-y'].forEach(id => document.getElementById(id).value = '');
    syncPivotInputs();
    document.getElementById('autosolve-status').style.display = 'none';
    document.getElementById('autosolve-results').innerHTML = '';
    document.getElementById('progress-container').style.display = 'none';
    if (autosolveWorker) { autosolveWorker.terminate(); autosolveWorker = null; }
    transform = { x: width / 2, y: height / 2, k: 1 };
    document.getElementById('slider-zoom').value = 0;
    document.getElementById('val-zoom').innerText = 100;
    updateUI(); draw();
});

// Mode Toggle
document.getElementById('mode-toggle').addEventListener('change', (e) => {
    synthesisMode = parseInt(e.target.value);
    document.getElementById('c5-row').style.display = synthesisMode === 5 ? 'flex' : 'none';
    document.getElementById('btn-new-problem').click(); // Reset when switching modes
    document.getElementById('setup-hint').innerText = `Click canvas to place points or enter X/Y below.`;
});

// Reset construction (keep C points)
document.getElementById('btn-reset').addEventListener('click', () => {
    stopAnimationHelper();
    currentStep = 0; showConstruction = true;
    document.getElementById('btn-toggle-const').innerText = 'Hide Construction';
    const savedC = points.C;
    points = { C: savedC, HR: null, A1: null, A3: null, HC: null, A2: null, A4: null, B1: null, P2: null, P4: null };
    lines = { c13: { p1: null, p2: null }, a13: { p1: null, p2: null } };
    fullCouplerCurve = []; circuitVisited = [false, false, false, false];
    config = { A1: 0, A2: 0, A3: 0, A4: 0 };
    document.querySelectorAll('.sol-card').forEach(c => c.classList.remove('active'));
    syncPivotInputs();
    transform = { x: width / 2, y: height / 2, k: 1 };
    document.getElementById('slider-zoom').value = 0;
    document.getElementById('val-zoom').innerText = 100;
    updateUI(); draw();
});

// Step back
document.getElementById('btn-back').addEventListener('click', () => {
    stopAnimationHelper();
    if (currentStep > 0) currentStep--;
    if (currentStep < 11) { points.B1 = null; fullCouplerCurve = []; circuitVisited = [false, false, false, false]; }
    if (currentStep < 10) { points.P2 = null; points.P4 = null; }
    if (currentStep < 8)  { points.A2 = null; points.A4 = null; }
    if (currentStep < 6)  { points.HC = null; }
    if (currentStep < 5)  { lines.a13 = { p1: null, p2: null }; }
    if (currentStep < 4)  { points.A1 = null; points.A3 = null; }
    if (currentStep < 2)  { points.HR = null; }
    showConstruction = true;
    document.getElementById('btn-toggle-const').innerText = 'Hide Construction';
    syncPivotInputs();
    updateUI(); draw(); calculateKinematics();
});

// ==========================================
// --- Init ---
// ==========================================
initUI();
setTimeout(resize, 100);
