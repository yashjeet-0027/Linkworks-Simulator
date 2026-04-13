// canvas.js — Pure, stateless canvas drawing functions
// All functions take ctx as first argument. No side effects on global state.

// ==========================================
// --- Grid & Background ---
// ==========================================

export function drawGrid(ctx, width, height, transform) {
    const { x: tx, y: ty, k } = transform;
    const toScreen = (p) => ({ x: p.x * k + tx, y: p.y * k + ty });
    const toWorld = (p) => ({ x: (p.x - tx) / k, y: (p.y - ty) / k });

    ctx.strokeStyle = '#e9ecef';
    ctx.lineWidth = 1;
    ctx.setLineDash([]);
    const gridSize = 50;

    const tl = toWorld({ x: 0, y: 0 });
    const br = toWorld({ x: width, y: height });
    const startX = Math.floor(tl.x / gridSize) * gridSize;
    const startY = Math.floor(tl.y / gridSize) * gridSize;

    for (let x = startX; x <= br.x; x += gridSize) {
        const sp1 = toScreen({ x, y: tl.y }), sp2 = toScreen({ x, y: br.y });
        ctx.beginPath(); ctx.moveTo(sp1.x, sp1.y); ctx.lineTo(sp2.x, sp2.y); ctx.stroke();
    }
    for (let y = startY; y <= br.y; y += gridSize) {
        const sp1 = toScreen({ x: tl.x, y }), sp2 = toScreen({ x: br.x, y });
        ctx.beginPath(); ctx.moveTo(sp1.x, sp1.y); ctx.lineTo(sp2.x, sp2.y); ctx.stroke();
    }

    const origin = toScreen({ x: 0, y: 0 });
    ctx.lineWidth = 2;
    ctx.strokeStyle = 'rgba(100, 100, 100, 0.6)';
    ctx.setLineDash([8, 8]);
    ctx.beginPath(); ctx.moveTo(0, origin.y); ctx.lineTo(width, origin.y); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(origin.x, 0); ctx.lineTo(origin.x, height); ctx.stroke();
    ctx.fillStyle = '#666';
    ctx.font = 'bold 14px Arial';
    ctx.fillText('X', width - 20, origin.y - 10);
    ctx.fillText('Y', origin.x + 10, 20);
    ctx.fillText('(0,0)', origin.x + 5, origin.y - 5);
    ctx.setLineDash([]);
}

// ==========================================
// --- Primitives ---
// ==========================================

export function drawPoint(ctx, p, lbl, transform, color = '#000', size = 5) {
    if (!p) return;
    const sp = { x: p.x * transform.k + transform.x, y: p.y * transform.k + transform.y };
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(sp.x, sp.y, size, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = '#333';
    ctx.font = 'bold 14px Arial';
    ctx.fillText(lbl, sp.x + 8, sp.y - 8);
}

export function drawLabel(ctx, text, p, transform, color = '#333') {
    if (!p) return;
    const sp = { x: p.x * transform.k + transform.x, y: p.y * transform.k + transform.y };
    ctx.fillStyle = color;
    ctx.font = 'bold 14px Arial';
    ctx.fillText(text, sp.x + 8, sp.y - 8);
}

export function drawLine(ctx, p1, p2, transform, color = '#999', dashed = false, thickness = 1.5) {
    if (!p1 || !p2) return;
    const sp1 = { x: p1.x * transform.k + transform.x, y: p1.y * transform.k + transform.y };
    const sp2 = { x: p2.x * transform.k + transform.x, y: p2.y * transform.k + transform.y };
    ctx.strokeStyle = color;
    ctx.lineWidth = thickness;
    ctx.setLineDash(dashed ? [5, 5] : []);
    ctx.beginPath();
    ctx.moveTo(sp1.x, sp1.y);
    ctx.lineTo(sp2.x, sp2.y);
    ctx.stroke();
    ctx.setLineDash([]);
}

export function drawDashedLine(ctx, p1, p2, transform, color = '#999', thickness = 1.5) {
    drawLine(ctx, p1, p2, transform, color, true, thickness);
}

export function drawArc(ctx, center, radius, startAngle, endAngle, transform, color = '#999', dashed = false) {
    if (!center) return;
    const sc = { x: center.x * transform.k + transform.x, y: center.y * transform.k + transform.y };
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    ctx.setLineDash(dashed ? [5, 5] : []);
    ctx.beginPath();
    ctx.arc(sc.x, sc.y, radius * transform.k, startAngle, endAngle);
    ctx.stroke();
    ctx.setLineDash([]);
}

export function drawCircle(ctx, center, radius, transform, color = '#999', dashed = false) {
    if (!center) return;
    const sc = { x: center.x * transform.k + transform.x, y: center.y * transform.k + transform.y };
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    ctx.setLineDash(dashed ? [5, 5] : []);
    ctx.beginPath();
    ctx.arc(sc.x, sc.y, radius * transform.k, 0, Math.PI * 2);
    ctx.stroke();
    ctx.setLineDash([]);
}

export function drawGroundSymbol(ctx, p, lbl, transform) {
    if (!p) return;
    const sp = { x: p.x * transform.k + transform.x, y: p.y * transform.k + transform.y };
    ctx.fillStyle = 'black';
    ctx.strokeStyle = 'black';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([]);
    ctx.beginPath();
    ctx.moveTo(sp.x, sp.y);
    ctx.lineTo(sp.x - 12, sp.y + 18);
    ctx.lineTo(sp.x + 12, sp.y + 18);
    ctx.closePath();
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(sp.x - 18, sp.y + 18);
    ctx.lineTo(sp.x + 18, sp.y + 18);
    ctx.stroke();
    for (let i = -12; i <= 12; i += 6) {
        ctx.beginPath();
        ctx.moveTo(sp.x + i, sp.y + 18);
        ctx.lineTo(sp.x + i - 6, sp.y + 26);
        ctx.stroke();
    }
    ctx.fillStyle = '#333';
    ctx.font = 'bold 16px Arial';
    ctx.fillText(lbl, sp.x + 18, sp.y + 5);
    ctx.fillStyle = 'white';
    ctx.beginPath();
    ctx.arc(sp.x, sp.y, 4, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = 'black';
    ctx.stroke();
}

// ==========================================
// --- Linkage Drawing ---
// ==========================================

/**
 * Draw the four-bar linkage triangle.
 * @param {boolean} isGhost - If true, render as a faint ghost (alternate branch).
 */
export function drawLinkage(ctx, HC, HR, A, B, C_pt, transform, isGhost = false, currentGamma = 90) {
    if (!HC || !HR || !A || !B || !C_pt) return;
    const toScreen = (p) => ({ x: p.x * transform.k + transform.x, y: p.y * transform.k + transform.y });

    if (isGhost) {
        // Ghost / alternate-branch: muted semi-transparent grays
        drawLine(ctx, HC, HR, transform, 'rgba(200,200,220,0.28)', true,  1.5); // frame (dashed)
        drawLine(ctx, HC, A,  transform, 'rgba(110,200,130,0.35)', false, 2.5); // crank (faint green)
        drawLine(ctx, HR, B,  transform, 'rgba(160,130,210,0.35)', false, 2.5); // rocker (faint purple)

        // Coupler triangle — very faint fill, dashed outline
        const spA = toScreen(A), spB = toScreen(B), spC = toScreen(C_pt);
        ctx.fillStyle   = 'rgba(255, 255, 255, 0.07)';
        ctx.strokeStyle = 'rgba(130, 170, 255, 0.28)';
        ctx.lineWidth   = 1.5;
        ctx.setLineDash([5, 5]);
        ctx.beginPath();
        ctx.moveTo(spA.x, spA.y);
        ctx.lineTo(spB.x, spB.y);
        ctx.lineTo(spC.x, spC.y);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
        ctx.setLineDash([]);
    } else {
        // Standard (primary branch) rendering
        const isBadGamma = currentGamma < 45 || currentGamma > 135;
        const mainColor = isBadGamma ? '#ef4444' : '#198754';
        const rockerColor = isBadGamma ? '#ef4444' : '#6f42c1';
        const couplerFill = isBadGamma ? 'rgba(239, 68, 68, 0.3)' : 'rgba(13, 110, 253, 0.3)';
        const couplerStroke = isBadGamma ? '#ef4444' : '#0d6efd';

        ctx.save();
        if (isBadGamma) {
            ctx.shadowBlur = 12;
            ctx.shadowColor = '#ef4444';
        }

        drawLine(ctx, HC, HR, transform, '#555',    true,  2); // frame link (dashed)
        drawLine(ctx, HC, A,  transform, mainColor, false, 4); // crank
        drawLine(ctx, HR, B,  transform, rockerColor, false, 4); // rocker

        // Coupler triangle
        const spA = toScreen(A), spB = toScreen(B), spC = toScreen(C_pt);
        ctx.fillStyle   = couplerFill;
        ctx.strokeStyle = couplerStroke;
        ctx.lineWidth   = 2;
        ctx.setLineDash([]);
        ctx.beginPath();
        ctx.moveTo(spA.x, spA.y);
        ctx.lineTo(spB.x, spB.y);
        ctx.lineTo(spC.x, spC.y);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
        ctx.restore();
    }
}

// ==========================================
// --- Coupler Curve ---
// ==========================================

export function drawCouplerCurve(ctx, curve, transform, animated = false) {
    if (!curve || curve.length < 2) return;
    const toScreen = (p) => ({ x: p.x * transform.k + transform.x, y: p.y * transform.k + transform.y });

    ctx.strokeStyle = animated ? 'rgba(220, 53, 69, 0.8)' : 'rgba(220, 53, 69, 0.3)';
    ctx.lineWidth = animated ? 3 : 2.5;
    ctx.setLineDash(animated ? [] : [6, 6]);
    ctx.beginPath();
    const sp0 = toScreen(curve[0]);
    ctx.moveTo(sp0.x, sp0.y);
    for (let i = 1; i < curve.length; i++) {
        const sp = toScreen(curve[i]);
        ctx.lineTo(sp.x, sp.y);
    }
    ctx.stroke();
    ctx.setLineDash([]);
}

// ==========================================
// --- Arrow (utility) ---
// ==========================================

export function drawArrow(ctx, from, to, transform, color = '#333', thickness = 2) {
    if (!from || !to) return;
    const toScreen = (p) => ({ x: p.x * transform.k + transform.x, y: p.y * transform.k + transform.y });
    const sf = toScreen(from);
    const st = toScreen(to);
    const angle = Math.atan2(st.y - sf.y, st.x - sf.x);
    const headLen = 12;

    ctx.strokeStyle = color;
    ctx.lineWidth = thickness;
    ctx.setLineDash([]);
    ctx.beginPath();
    ctx.moveTo(sf.x, sf.y);
    ctx.lineTo(st.x, st.y);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(st.x, st.y);
    ctx.lineTo(st.x - headLen * Math.cos(angle - Math.PI / 6), st.y - headLen * Math.sin(angle - Math.PI / 6));
    ctx.lineTo(st.x - headLen * Math.cos(angle + Math.PI / 6), st.y - headLen * Math.sin(angle + Math.PI / 6));
    ctx.closePath();
    ctx.fillStyle = color;
    ctx.fill();
}
