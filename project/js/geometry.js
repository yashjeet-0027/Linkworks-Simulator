// geometry.js — Ported from app.py math helpers + kinematic engine
// All functions exported as ES6 named exports.

export function dist(p1, p2) {
    return Math.hypot(p2.x - p1.x, p2.y - p1.y);
}

export function midpoint(p1, p2) {
    return { x: (p1.x + p2.x) / 2, y: (p1.y + p2.y) / 2 };
}

export function perpBisector(p1, p2, length = 5000) {
    const mid = midpoint(p1, p2);
    const dx = p2.x - p1.x;
    const dy = p2.y - p1.y;
    const len = Math.hypot(dx, dy);
    if (len === 0) return null;
    return {
        p1: { x: mid.x + (-dy / len) * length, y: mid.y + (dx / len) * length },
        p2: { x: mid.x - (-dy / len) * length, y: mid.y - (dx / len) * length }
    };
}

export function projectPointOnLine(p, lS, lE) {
    const l2 = Math.pow(Math.hypot(lE.x - lS.x, lE.y - lS.y), 2);
    if (l2 === 0) return lS;
    const t = ((p.x - lS.x) * (lE.x - lS.x) + (p.y - lS.y) * (lE.y - lS.y)) / l2;
    return { x: lS.x + t * (lE.x - lS.x), y: lS.y + t * (lE.y - lS.y) };
}

export function lineIntersection(l1, l2) {
    const x1 = l1.p1.x, y1 = l1.p1.y;
    const x2 = l1.p2.x, y2 = l1.p2.y;
    const x3 = l2.p1.x, y3 = l2.p1.y;
    const x4 = l2.p2.x, y4 = l2.p2.y;
    
    const den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (Math.abs(den) < 1e-10) return null;

    const numP1 = x1 * y2 - y1 * x2;
    const numP2 = x3 * y4 - y3 * x4;

    const px = (numP1 * (x3 - x4) - (x1 - x2) * numP2) / den;
    const py = (numP1 * (y3 - y4) - (y1 - y2) * numP2) / den;

    return { x: px, y: py };
}

export function circleIntersection(c1, r1, c2, r2) {
    const d = dist(c1, c2);
    if (d > r1 + r2 || d < Math.abs(r1 - r2) || d === 0) return [];
    const a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
    const h = Math.sqrt(Math.max(0, r1 * r1 - a * a));
    const p2 = {
        x: c1.x + a * (c2.x - c1.x) / d,
        y: c1.y + a * (c2.y - c1.y) / d
    };
    return [
        { x: p2.x + h * (c2.y - c1.y) / d, y: p2.y - h * (c2.x - c1.x) / d },
        { x: p2.x - h * (c2.y - c1.y) / d, y: p2.y + h * (c2.x - c1.x) / d }
    ];
}

export function rigidTransform(p, frameP1, frameP2, targetP1, targetP2) {
    const angleFrame = Math.atan2(frameP2.y - frameP1.y, frameP2.x - frameP1.x);
    const angleTarget = Math.atan2(targetP2.y - targetP1.y, targetP2.x - targetP1.x);
    const dA = angleTarget - angleFrame;
    const vx = p.x - frameP1.x;
    const vy = p.y - frameP1.y;
    return {
        x: targetP1.x + vx * Math.cos(dA) - vy * Math.sin(dA),
        y: targetP1.y + vx * Math.sin(dA) + vy * Math.cos(dA)
    };
}

export function circumcenter(p1, p2, p3) {
    const d = 2 * (p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y));
    if (Math.abs(d) < 1e-5) return null;
    const ux = ((p1.x ** 2 + p1.y ** 2) * (p2.y - p3.y) +
                (p2.x ** 2 + p2.y ** 2) * (p3.y - p1.y) +
                (p3.x ** 2 + p3.y ** 2) * (p1.y - p2.y)) / d;
    const uy = ((p1.x ** 2 + p1.y ** 2) * (p3.x - p2.x) +
                (p2.x ** 2 + p2.y ** 2) * (p1.x - p3.x) +
                (p3.x ** 2 + p3.y ** 2) * (p2.x - p1.x)) / d;
    return { x: ux, y: uy };
}

// ==========================================
// --- Kinematic Engine ---
// ==========================================

/**
 * Transmission angle γ — angle between the coupler (BA) and the rocker (B→HR) at joint B.
 * Returns degrees in [0, 180]. Values near 0° or 180° indicate poor force transmission.
 */
export function calcTransmissionAngle(A, B, HR) {
    // Vector from B to A (coupler direction)
    const bax = A.x - B.x, bay = A.y - B.y;
    // Vector from B to HR (rocker direction)
    const bhrx = HR.x - B.x, bhry = HR.y - B.y;
    const dot   = bax * bhrx + bay * bhry;
    const magBA  = Math.hypot(bax, bay);
    const magBHR = Math.hypot(bhrx, bhry);
    if (magBA === 0 || magBHR === 0) return 90; // degenerate — return neutral
    const cosG = Math.max(-1, Math.min(1, dot / (magBA * magBHR)));
    return (Math.acos(cosG) * 180) / Math.PI; // degrees
}

export function computeFullPath(C_points, HC, A1, B1, HR, fastSweep = false) {
    const L1 = dist(HC, A1);
    const L2 = dist(A1, B1);
    const L3 = dist(HR, B1);
    const startAngle = Math.atan2(A1.y - HC.y, A1.x - HC.x);
    const lastB = { x: B1.x, y: B1.y };

    const step = fastSweep ? 0.05 : 0.01;
    const maxSteps = Math.floor((Math.PI * 2) / step + 10);

    let minGamma = 180, maxGamma = 0;

    // Forward sweep
    let theta = startAngle;
    const forwardPath = [];
    let currLastB = { ...lastB };
    for (let i = 0; i < maxSteps; i++) {
        const A_next = { x: HC.x + L1 * Math.cos(theta), y: HC.y + L1 * Math.sin(theta) };
        const intersects = circleIntersection(A_next, L2, HR, L3);
        if (!intersects.length) break;
        let B_next = intersects[0];
        if (intersects.length > 1 && dist(intersects[1], currLastB) < dist(intersects[0], currLastB)) {
            B_next = intersects[1];
        }
        const gamma = calcTransmissionAngle(A_next, B_next, HR);
        if (gamma < minGamma) minGamma = gamma;
        if (gamma > maxGamma) maxGamma = gamma;
        currLastB = B_next;
        forwardPath.push(rigidTransform(C_points[0], A1, B1, A_next, B_next));
        theta += step;
    }

    // Backward sweep
    theta = startAngle - step;
    const backwardPath = [];
    currLastB = { ...lastB };
    for (let i = 0; i < maxSteps; i++) {
        const A_next = { x: HC.x + L1 * Math.cos(theta), y: HC.y + L1 * Math.sin(theta) };
        const intersects = circleIntersection(A_next, L2, HR, L3);
        if (!intersects.length) break;
        let B_next = intersects[0];
        if (intersects.length > 1 && dist(intersects[1], currLastB) < dist(intersects[0], currLastB)) {
            B_next = intersects[1];
        }
        const gamma = calcTransmissionAngle(A_next, B_next, HR);
        if (gamma < minGamma) minGamma = gamma;
        if (gamma > maxGamma) maxGamma = gamma;
        currLastB = B_next;
        backwardPath.push(rigidTransform(C_points[0], A1, B1, A_next, B_next));
        theta -= step;
    }

    backwardPath.reverse();
    const finalPath = [...backwardPath, ...forwardPath];

    const circuitVisited = [false, false, false, false];
    const hitIndices = [-1, -1, -1, -1];
    for (let idx = 0; idx < C_points.length; idx++) {
        let minD = Infinity;
        let bestK = -1;
        for (let k = 0; k < finalPath.length; k++) {
            const p = finalPath[k];
            const d = dist(C_points[idx], p);
            if (d < minD) { minD = d; bestK = k; }
        }
        if (minD < 15) {
            circuitVisited[idx] = true;
            hitIndices[idx] = bestK;
        }
    }

    return { path: finalPath, circuitVisited, hitIndices, minGamma, maxGamma };
}

export function evaluateGrashof(HC, A1, B1, HR) {
    const L1 = dist(HC, A1);   // input (crank) link
    const L2 = dist(A1, B1);   // coupler
    const L3 = dist(B1, HR);   // output (rocker) link
    const L4 = dist(HC, HR);   // frame

    const sorted = [L1, L2, L3, L4].sort((a, b) => a - b);
    const [s, p, q, l] = sorted;

    const isGrashof = (s + l <= p + q);

    let mechType   = 'Non-Grashof';
    let inputType  = 'Rocker';   // L1 type
    let outputType = 'Rocker';   // L3 type

    if (isGrashof) {
        if      (s === L1) { mechType = 'Crank-Rocker';  inputType = 'Crank';  outputType = 'Rocker'; }
        else if (s === L4) { mechType = 'Double-Crank';  inputType = 'Crank';  outputType = 'Crank';  }
        else if (s === L3) { mechType = 'Rocker-Crank';  inputType = 'Rocker'; outputType = 'Crank';  }
        else               { mechType = 'Double-Rocker'; inputType = 'Rocker'; outputType = 'Rocker'; }
    }

    const ratio = s > 0 ? l / s : Infinity;
    return {
        isGrashof, mechType,
        inputType, outputType,
        linkLengths: { crank: L1, coupler: L2, rocker: L3, frame: L4 },
        ratio
    };
}
