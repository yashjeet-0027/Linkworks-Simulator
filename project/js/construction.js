// construction.js — Port of the Flask /calculate endpoint logic
// Imports from geometry.js and exports computeFullState().

import {
    dist, perpBisector, circleIntersection, rigidTransform,
    circumcenter, computeFullPath, lineIntersection
} from './geometry.js';

/**
 * Computes the full linkage state given all synthesis parameters.
 * Direct port of run_synthesis_core() from app.py.
 *
 * @param {Array}   C       - Array of precision points [{x,y}, ...]
 * @param {Object}  HR      - Fixed rocker pivot {x, y}
 * @param {Object}  HC      - Fixed crank pivot {x, y}
 * @param {number}  R       - Radius R
 * @param {number}  r       - Radius r
 * @param {Object}  config  - { A1: 0|1, A2: 0|1, A3: 0|1, A4: 0|1 }
 * @param {boolean} fastSweep - Use coarser step for coupler curve (default false)
 * @returns {Object} Same response structure as the Flask /calculate endpoint
 */
export function computeFullState(C, HR, HC, R, r, config, fastSweep = false, mode = 4) {
    const res = {
        A1: null, A2: null, A3: null, A4: null,
        P2: null, P4: null, B1: null,
        fullCouplerCurve: [],
        circuitVisited: [false, false, false, false, false],
        hitIndices: [-1, -1, -1, -1, -1],
        minGamma: 180, maxGamma: 0,
        lines: { c13: null, a13: null }
    };

    if (!C || C.length < 3) return res;

    res.lines.c13 = perpBisector(C[0], C[2]);

    if (mode === 5 && C.length === 5) {
        const c23 = perpBisector(C[1], C[2]);
        const c15 = perpBisector(C[0], C[4]);
        if (c23 && c15) {
            const customHR = lineIntersection(c23, c15);
            if (customHR) HR = customHR;
        }
    }

    if (!HR) return res;

    const int1 = circleIntersection(C[0], r, HR, R);
    const int3 = circleIntersection(C[2], r, HR, R);

    if (int1.length) res.A1 = int1[config.A1 % int1.length];
    if (int3.length) res.A3 = int3[config.A3 % int3.length];

    if (res.A1 && res.A3) {
        res.lines.a13 = perpBisector(res.A1, res.A3);
    }

    if (!HC || !res.A1 || C.length < 4) return res;

    const crankR = dist(HC, res.A1);
    const int2 = circleIntersection(C[1], r, HC, crankR);
    const int4 = circleIntersection(C[3], r, HC, crankR);

    if (int2.length) res.A2 = int2[config.A2 % int2.length];
    if (int4.length) res.A4 = int4[config.A4 % int4.length];

    if (res.A1 && res.A2 && res.A4) {
        res.P2 = rigidTransform(HR, C[1], res.A2, C[0], res.A1);
        res.P4 = rigidTransform(HR, C[3], res.A4, C[0], res.A1);
    }

    if (res.P2 && res.P4) {
        res.B1 = circumcenter(res.P2, HR, res.P4);
        if (res.B1) {
            const { path, circuitVisited, hitIndices, minGamma, maxGamma } = computeFullPath(C, HC, res.A1, res.B1, HR, fastSweep);
            res.fullCouplerCurve = path;
            res.circuitVisited   = circuitVisited;
            res.hitIndices       = hitIndices;
            res.minGamma         = minGamma;
            res.maxGamma         = maxGamma;
        }
    }

    return res;
}
