// autosolve.worker.js — Web Worker for brute-force parameter search
// Uses importScripts() (no ES modules) for broad browser compatibility.
// All geometry/construction logic is inlined directly.

// ==========================================
// --- Inlined Geometry Functions ---
// ==========================================

function dist(p1, p2) {
    return Math.hypot(p2.x - p1.x, p2.y - p1.y);
}

function midpoint(p1, p2) {
    return { x: (p1.x + p2.x) / 2, y: (p1.y + p2.y) / 2 };
}

function perpBisector(p1, p2, length) {
    length = length || 5000;
    var mid = midpoint(p1, p2);
    var dx = p2.x - p1.x;
    var dy = p2.y - p1.y;
    var len = Math.hypot(dx, dy);
    if (len === 0) return null;
    return {
        p1: { x: mid.x + (-dy / len) * length, y: mid.y + (dx / len) * length },
        p2: { x: mid.x - (-dy / len) * length, y: mid.y - (dx / len) * length }
    };
}

function lineIntersection(l1, l2) {
    var x1 = l1.p1.x, y1 = l1.p1.y;
    var x2 = l1.p2.x, y2 = l1.p2.y;
    var x3 = l2.p1.x, y3 = l2.p1.y;
    var x4 = l2.p2.x, y4 = l2.p2.y;
    
    var den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (Math.abs(den) < 1e-10) return null;

    var numP1 = x1 * y2 - y1 * x2;
    var numP2 = x3 * y4 - y3 * x4;

    var px = (numP1 * (x3 - x4) - (x1 - x2) * numP2) / den;
    var py = (numP1 * (y3 - y4) - (y1 - y2) * numP2) / den;

    return { x: px, y: py };
}

function circleIntersection(c1, r1, c2, r2) {
    var d = dist(c1, c2);
    if (d > r1 + r2 || d < Math.abs(r1 - r2) || d === 0) return [];
    var a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
    var h = Math.sqrt(Math.max(0, r1 * r1 - a * a));
    var p2 = {
        x: c1.x + a * (c2.x - c1.x) / d,
        y: c1.y + a * (c2.y - c1.y) / d
    };
    return [
        { x: p2.x + h * (c2.y - c1.y) / d, y: p2.y - h * (c2.x - c1.x) / d },
        { x: p2.x - h * (c2.y - c1.y) / d, y: p2.y + h * (c2.x - c1.x) / d }
    ];
}

function rigidTransform(p, frameP1, frameP2, targetP1, targetP2) {
    var aF = Math.atan2(frameP2.y - frameP1.y, frameP2.x - frameP1.x);
    var aT = Math.atan2(targetP2.y - targetP1.y, targetP2.x - targetP1.x);
    var dA = aT - aF;
    var vx = p.x - frameP1.x;
    var vy = p.y - frameP1.y;
    return {
        x: targetP1.x + vx * Math.cos(dA) - vy * Math.sin(dA),
        y: targetP1.y + vx * Math.sin(dA) + vy * Math.cos(dA)
    };
}

function circumcenter(p1, p2, p3) {
    var d = 2 * (p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y));
    if (Math.abs(d) < 1e-5) return null;
    var ux = ((p1.x * p1.x + p1.y * p1.y) * (p2.y - p3.y) +
              (p2.x * p2.x + p2.y * p2.y) * (p3.y - p1.y) +
              (p3.x * p3.x + p3.y * p3.y) * (p1.y - p2.y)) / d;
    var uy = ((p1.x * p1.x + p1.y * p1.y) * (p3.x - p2.x) +
              (p2.x * p2.x + p2.y * p2.y) * (p1.x - p3.x) +
              (p3.x * p3.x + p3.y * p3.y) * (p2.x - p1.x)) / d;
    return { x: ux, y: uy };
}

/**
 * Transmission angle γ — angle between coupler (BA) and rocker (B→HR) at joint B.
 * Returns degrees in [0, 180].
 */
function calcTransmissionAngle(A, B, HR) {
    var bax = A.x - B.x, bay = A.y - B.y;
    var bhrx = HR.x - B.x, bhry = HR.y - B.y;
    var dot    = bax * bhrx + bay * bhry;
    var magBA  = Math.hypot(bax, bay);
    var magBHR = Math.hypot(bhrx, bhry);
    if (magBA === 0 || magBHR === 0) return 90;
    var cosG = Math.max(-1, Math.min(1, dot / (magBA * magBHR)));
    return (Math.acos(cosG) * 180) / Math.PI;
}

function computeFullPath(C_points, HC, A1, B1, HR, fastSweep) {
    var L1 = dist(HC, A1);
    var L2 = dist(A1, B1);
    var L3 = dist(HR, B1);
    var startAngle = Math.atan2(A1.y - HC.y, A1.x - HC.x);
    var lastB = { x: B1.x, y: B1.y };

    var step = fastSweep ? 0.05 : 0.01;
    var maxSteps = Math.floor((Math.PI * 2) / step + 10);

    var minGamma = 180, maxGamma = 0;

    var theta = startAngle;
    var forwardPath = [];
    var currLastB = { x: lastB.x, y: lastB.y };
    for (var i = 0; i < maxSteps; i++) {
        var A_next = { x: HC.x + L1 * Math.cos(theta), y: HC.y + L1 * Math.sin(theta) };
        var intersects = circleIntersection(A_next, L2, HR, L3);
        if (!intersects.length) break;
        var B_next = intersects[0];
        if (intersects.length > 1 && dist(intersects[1], currLastB) < dist(intersects[0], currLastB)) {
            B_next = intersects[1];
        }
        var gamma = calcTransmissionAngle(A_next, B_next, HR);
        if (gamma < minGamma) minGamma = gamma;
        if (gamma > maxGamma) maxGamma = gamma;
        currLastB = B_next;
        forwardPath.push(rigidTransform(C_points[0], A1, B1, A_next, B_next));
        theta += step;
    }

    theta = startAngle - step;
    var backwardPath = [];
    currLastB = { x: lastB.x, y: lastB.y };
    for (var j = 0; j < maxSteps; j++) {
        var A_next2 = { x: HC.x + L1 * Math.cos(theta), y: HC.y + L1 * Math.sin(theta) };
        var intersects2 = circleIntersection(A_next2, L2, HR, L3);
        if (!intersects2.length) break;
        var B_next2 = intersects2[0];
        if (intersects2.length > 1 && dist(intersects2[1], currLastB) < dist(intersects2[0], currLastB)) {
            B_next2 = intersects2[1];
        }
        var gamma2 = calcTransmissionAngle(A_next2, B_next2, HR);
        if (gamma2 < minGamma) minGamma = gamma2;
        if (gamma2 > maxGamma) maxGamma = gamma2;
        currLastB = B_next2;
        backwardPath.push(rigidTransform(C_points[0], A1, B1, A_next2, B_next2));
        theta -= step;
    }

    backwardPath.reverse();
    var finalPath = backwardPath.concat(forwardPath);

    var circuitVisited = Array(C_points.length).fill(false);
    var hitIndices = Array(C_points.length).fill(-1);
    for (var idx = 0; idx < C_points.length; idx++) {
        var minD = Infinity;
        var bestK = -1;
        for (var k = 0; k < finalPath.length; k++) {
            var dd = dist(C_points[idx], finalPath[k]);
            if (dd < minD) { minD = dd; bestK = k; }
        }
        if (minD < 15) {
            circuitVisited[idx] = true;
            hitIndices[idx] = bestK;
        }
    }

    return { path: finalPath, circuitVisited: circuitVisited, hitIndices: hitIndices, minGamma: minGamma, maxGamma: maxGamma };
}

// ==========================================
// --- Inlined Construction Logic ---
// ==========================================

function runSynthesisCore(C, HR, HC, R, r, config, fastSweep) {
    var res = {
        A1: null, A2: null, A3: null, A4: null,
        P2: null, P4: null, B1: null,
        fullCouplerCurve: [],
        circuitVisited: Array(C.length).fill(false),
        hitIndices: Array(C.length).fill(-1),
        lines: { c13: null, a13: null }
    };

    if (!C || C.length < 3) return res;
    res.lines.c13 = perpBisector(C[0], C[2]);
    if (!HR) return res;

    var int1 = circleIntersection(C[0], r, HR, R);
    var int3 = circleIntersection(C[2], r, HR, R);
    if (int1.length) res.A1 = int1[config.A1 % int1.length];
    if (int3.length) res.A3 = int3[config.A3 % int3.length];
    if (res.A1 && res.A3) {
        res.lines.a13 = perpBisector(res.A1, res.A3);
    }
    if (!HC || !res.A1 || C.length < 4) return res;

    var crankR = dist(HC, res.A1);
    var int2 = circleIntersection(C[1], r, HC, crankR);
    var int4 = circleIntersection(C[3], r, HC, crankR);
    if (int2.length) res.A2 = int2[config.A2 % int2.length];
    if (int4.length) res.A4 = int4[config.A4 % int4.length];

    if (res.A1 && res.A2 && res.A4) {
        res.P2 = rigidTransform(HR, C[1], res.A2, C[0], res.A1);
        res.P4 = rigidTransform(HR, C[3], res.A4, C[0], res.A1);
    }

    if (res.P2 && res.P4) {
        res.B1 = circumcenter(res.P2, HR, res.P4);
        if (res.B1) {
            var cpResult = computeFullPath(C, HC, res.A1, res.B1, HR, fastSweep);
            res.fullCouplerCurve = cpResult.path;
            res.circuitVisited  = cpResult.circuitVisited;
            res.hitIndices      = cpResult.hitIndices;
            res.minGamma        = cpResult.minGamma;
            res.maxGamma        = cpResult.maxGamma;
        }
    }

    return res;
}

function evaluateGrashof(HC, A1, B1, HR) {
    var L1 = dist(HC, A1);   // input (crank) link
    var L2 = dist(A1, B1);   // coupler
    var L3 = dist(B1, HR);   // output (rocker) link
    var L4 = dist(HC, HR);   // frame
    var sorted = [L1, L2, L3, L4].sort(function(a, b) { return a - b; });
    var s = sorted[0], p = sorted[1], q = sorted[2], l = sorted[3];
    var isGrashof = (s + l <= p + q);
    var mechType   = 'Non-Grashof';
    var inputType  = 'Rocker';   // L1 role
    var outputType = 'Rocker';   // L3 role
    if (isGrashof) {
        if      (s === L1) { mechType = 'Crank-Rocker';  inputType = 'Crank';  outputType = 'Rocker'; }
        else if (s === L4) { mechType = 'Double-Crank';  inputType = 'Crank';  outputType = 'Crank';  }
        else if (s === L3) { mechType = 'Rocker-Crank';  inputType = 'Rocker'; outputType = 'Crank';  }
        else               { mechType = 'Double-Rocker'; inputType = 'Rocker'; outputType = 'Rocker'; }
    }
    var ratio = s > 0 ? l / s : Infinity;
    return {
        isGrashof: isGrashof, mechType: mechType,
        inputType: inputType, outputType: outputType,
        linkLengths: { crank: L1, coupler: L2, rocker: L3, frame: L4 },
        ratio: ratio
    };
}

// ==========================================
// --- Brute-Force Search ---
// ==========================================

function runSearch(C, filters, maxRatio, mode) {
    // Normalise filter strings: 'any' | 'crank' | 'rocker' (lower-case from dropdown values)
    var filterDriver = filters && filters.driver ? filters.driver.toLowerCase() : 'any';
    var filterDriven = filters && filters.driven ? filters.driven.toLowerCase() : 'any';
    var filterMinGamma = filters && filters.minGamma !== undefined ? filters.minGamma : 40;
    var filterMaxRatio = maxRatio !== undefined ? maxRatio : 20;
    var filterEnableRatio = filters && filters.enableRatio !== undefined ? filters.enableRatio : true;
    var filterEnableOrder = filters && filters.enableOrder !== undefined ? filters.enableOrder : true;
    var filterEnableGamma = filters && filters.enableGamma !== undefined ? filters.enableGamma : true;
    var synthesisMode = mode || 4;
    console.log('[Worker] filters received:', filterDriver, '/', filterDriven, '| minGamma:', filterMinGamma, '| maxRatio:', filterMaxRatio, '| enableGamma:', filterEnableGamma, '| mode:', synthesisMode);
    var c13 = perpBisector(C[0], C[2]);
    var dist_c13 = dist(C[0], C[2]);
    var mid_c13 = midpoint(C[0], C[2]);

    if (!c13 || dist_c13 === 0) {
        postMessage({ type: 'done', solutions: [] });
        return;
    }

    var dx = c13.p2.x - c13.p1.x;
    var dy = c13.p2.y - c13.p1.y;
    var length = Math.hypot(dx, dy);
    var ux = dx / length, uy = dy / length;

    var n_HR = 8, n_R = 9, n_r = 9;

    var HR_points = [];
    if (synthesisMode === 5 && C.length === 5) {
        var c23 = perpBisector(C[1], C[2]);
        var c15 = perpBisector(C[0], C[4]);
        var customHR = null;
        if (c23 && c15) customHR = lineIntersection(c23, c15);

        if (customHR) {
            HR_points.push(customHR);
        } else {
            postMessage({ type: 'done', solutions: [] });
            return;
        }
    } else {
        // 8 evenly spaced HR positions along c13
        for (var hi = 0; hi < n_HR; hi++) {
            var t = -3 * dist_c13 + (6 * dist_c13 * hi / (n_HR - 1));
            HR_points.push({ x: mid_c13.x + t * ux, y: mid_c13.y + t * uy });
        }
    }

    // Logarithmic spacing for R and r
    var minVal = 0.5 * dist_c13;
    var maxVal = 3.0 * dist_c13;
    var R_vals = [], r_vals = [];
    for (var ri = 0; ri < n_R; ri++) {
        R_vals.push(minVal * Math.pow(maxVal / minVal, ri / (n_R - 1)));
    }
    for (var rri = 0; rri < n_r; rri++) {
        r_vals.push(minVal * Math.pow(maxVal / minVal, rri / (n_r - 1)));
    }

    var totalIter = HR_points.length * n_R * n_r * 16; // 4 configs × 4 pivots
    var iterCount = 0;
    var validSolutions = [];
    var cancelled = false;

    for (var i = 0; i < HR_points.length && !cancelled; i++) {
        var hr = HR_points[i];
        var progressPct = Math.round((i / HR_points.length) * 100);

        for (var Ri = 0; Ri < R_vals.length && !cancelled; Ri++) {
            for (var rri2 = 0; rri2 < r_vals.length && !cancelled; rri2++) {
                var R = R_vals[Ri];
                var r = r_vals[rri2];

                var int1 = circleIntersection(C[0], r, hr, R);
                var int3 = circleIntersection(C[2], r, hr, R);
                if (!int1.length || !int3.length) { iterCount += 4; continue; }

                for (var c1 = 0; c1 < 2; c1++) {
                    for (var c3 = 0; c3 < 2; c3++) {
                        var A1 = int1[c1 % int1.length];
                        var A3 = int3[c3 % int3.length];
                        var a13 = perpBisector(A1, A3);
                        if (!a13) { iterCount += 4; continue; }

                        var midA = midpoint(A1, A3);
                        var dxa = a13.p2.x - a13.p1.x;
                        var dya = a13.p2.y - a13.p1.y;
                        var len_a = Math.hypot(dxa, dya);
                        if (len_a === 0) { iterCount += 4; continue; }
                        var hc = { x: midA.x + (dxa / len_a) * R, y: midA.y + (dya / len_a) * R };

                        for (var c2 = 0; c2 < 2; c2++) {
                            for (var c4 = 0; c4 < 2; c4++) {
                                var cfg = { A1: c1, A2: c2, A3: c3, A4: c4 };
                                var res = runSynthesisCore(C, hr, hc, R, r, cfg, true);
                                iterCount++;

                                if (res.circuitVisited.every(function(v) { return v; })) {
                                    var gInfo = evaluateGrashof(hc, res.A1, res.B1, hr);

                                    // ── Apply Driver / Driven filters ──
                                    if (filterDriver !== 'any' && gInfo.inputType.toLowerCase() !== filterDriver) { iterCount++; continue; }
                                    if (filterDriven !== 'any' && gInfo.outputType.toLowerCase() !== filterDriven) { iterCount++; continue; }

                                    // ── Apply Maximum Link Ratio Filter ──
                                    if (filterEnableRatio && gInfo.ratio > filterMaxRatio) { iterCount++; continue; }

                                    // ── Apply Order Defect Filter ──
                                    if (filterEnableOrder) {
                                        var h = res.hitIndices;
                                        var isIncreasing = true;
                                        var isDecreasing = true;
                                        for (var ki = 0; ki < h.length - 1; ki++) {
                                            if (h[ki] >= h[ki + 1]) isIncreasing = false;
                                            if (h[ki] <= h[ki + 1]) isDecreasing = false;
                                        }
                                        if (!isIncreasing && !isDecreasing) { iterCount++; continue; }
                                    }

                                    // ── Apply Transmission Angle Filter ──
                                    if (filterEnableGamma) {
                                        var worst_gamma = Math.min(res.minGamma, 180 - res.maxGamma);
                                        if (worst_gamma < filterMinGamma) { iterCount++; continue; }
                                    }

                                    validSolutions.push({
                                        HR: hr, HC: hc, R: R, r: r, config: cfg,
                                        linkLengths: gInfo.linkLengths,
                                        grashof: gInfo.isGrashof,
                                        type: gInfo.mechType,
                                        inputType:  gInfo.inputType,
                                        outputType: gInfo.outputType,
                                        minGamma: res.minGamma,
                                        maxGamma: res.maxGamma,
                                        ratio: gInfo.ratio
                                    });
                                }

                                if (iterCount % 100 === 0) {
                                    postMessage({ type: 'progress', percent: progressPct, found: validSolutions.length });
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // Sort: Grashof first, then lowest link ratio (most balanced)
    validSolutions.sort(function(a, b) {
        if (a.grashof !== b.grashof) return a.grashof ? -1 : 1;
        return a.ratio - b.ratio;
    });

    // Return top 5 results
    var topSolutions = validSolutions.slice(0, 5);

    // If no Grashof found, best non-Grashof is already first after sort
    postMessage({ type: 'done', solutions: topSolutions });
}

// ==========================================
// --- Message Handler ---
// ==========================================

self.onmessage = function(e) {
    var data = e.data;
    if (data && data.C) {
        runSearch(data.C, data.filters, data.maxRatio, data.synthesisMode);
    }
};
