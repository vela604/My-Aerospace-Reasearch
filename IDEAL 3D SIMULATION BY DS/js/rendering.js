import { sim, rotateVector, quatToEuler } from './physics.js';

let toggles = { traj: true, grid: true, impact: true, forces: true, vel: true, acc: true, drag: false };
export function setToggles(newToggles) { toggles = { ...toggles, ...newToggles }; }
export function getToggles() { return toggles; }

function mapPoint(world, mapFunc, xRange, yRange, w, h) {
    let [wx, wy] = mapFunc(world);
    let cx = ((wx - xRange[0]) / (xRange[1] - xRange[0])) * w;
    let cy = h - ((wy - yRange[0]) / (yRange[1] - yRange[0])) * h;
    return [cx, cy];
}

export function drawProjection(canvas, mapFunc, xRange, yRange, label, viewType) {
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const w = canvas.width,
        h = canvas.height;
    if (w === 0 || h === 0) return;
    ctx.fillStyle = '#010510';
    ctx.fillRect(0, 0, w, h);
    
    // Grid
    if (toggles.grid) {
        ctx.strokeStyle = '#1a3a4a';
        ctx.lineWidth = 0.5;
        for (let i = 0; i <= 5; i++) {
            let x = xRange[0] + (i / 5) * (xRange[1] - xRange[0]);
            let cx = ((x - xRange[0]) / (xRange[1] - xRange[0])) * w;
            ctx.beginPath();
            ctx.moveTo(cx, 0);
            ctx.lineTo(cx, h);
            ctx.stroke();
            let y = yRange[0] + (i / 5) * (yRange[1] - yRange[0]);
            let cy = h - ((y - yRange[0]) / (yRange[1] - yRange[0])) * h;
            ctx.beginPath();
            ctx.moveTo(0, cy);
            ctx.lineTo(w, cy);
            ctx.stroke();
        }
    }
    
    // Trajectory
    if (toggles.traj && sim.trail.length > 1) {
        ctx.beginPath();
        ctx.strokeStyle = '#00d4ffaa';
        ctx.lineWidth = 1.5;
        for (let i = 0; i < sim.trail.length; i++) {
            let [wx, wy] = mapFunc(sim.trail[i]);
            let cx = ((wx - xRange[0]) / (xRange[1] - xRange[0])) * w;
            let cy = h - ((wy - yRange[0]) / (yRange[1] - yRange[0])) * h;
            if (i === 0) ctx.moveTo(cx, cy);
            else ctx.lineTo(cx, cy);
        }
        ctx.stroke();
    }
    
    // Impact line and point
    if (toggles.impact && sim.y > 0) {
        let ay = sim.ay;
        let b = sim.vy;
        let c = sim.y;
        let disc = b * b - 2 * ay * c;
        let tImpact = null;
        if (disc >= 0) {
            let sqrtDisc = Math.sqrt(disc);
            let t1 = (-b + sqrtDisc) / ay;
            let t2 = (-b - sqrtDisc) / ay;
            if (ay < 0) {
                if (t1 > 0 && t2 > 0) tImpact = Math.min(t1, t2);
                else if (t1 > 0) tImpact = t1;
                else if (t2 > 0) tImpact = t2;
            } else {
                tImpact = (t1 > 0) ? t1 : (t2 > 0 ? t2 : null);
            }
        }
        if (tImpact && tImpact > 0) {
            let impX = sim.x + sim.vx * tImpact;
            let impZ = sim.z + sim.vz * tImpact;
            let p0 = mapFunc([sim.x, sim.y, sim.z]);
            let p1 = mapFunc([impX, 0, impZ]);
            let [x0, y0] = [((p0[0] - xRange[0]) / (xRange[1] - xRange[0])) * w, h - ((p0[1] - yRange[0]) / (yRange[1] - yRange[0])) * h];
            let [x1, y1] = [((p1[0] - xRange[0]) / (xRange[1] - xRange[0])) * w, h - ((0 - yRange[0]) / (yRange[1] - yRange[0])) * h];
            ctx.beginPath();
            ctx.moveTo(x0, y0);
            ctx.lineTo(x1, y1);
            ctx.strokeStyle = '#ff2244';
            ctx.lineWidth = 1.5;
            ctx.setLineDash([6, 6]);
            ctx.stroke();
            ctx.setLineDash([]);
            ctx.fillStyle = '#ff2244';
            ctx.beginPath();
            ctx.arc(x1, y1, 5, 0, 2 * Math.PI);
            ctx.fill();
            ctx.fillStyle = '#ff2244';
            ctx.font = '8px monospace';
            ctx.fillText('IMPACT', x1 + 6, y1 - 4);
        }
    }
    
    // Detailed rocket drawing
    const map = (px, py) => {
        let cx = ((px - xRange[0]) / (xRange[1] - xRange[0])) * w;
        let cy = h - ((py - yRange[0]) / (yRange[1] - yRange[0])) * h;
        return [cx, cy];
    };
    drawDetailedRocket(ctx, map, mapFunc, viewType);
    
    // Velocity vector
    if (toggles.vel) {
        let p = mapFunc([sim.x, sim.y, sim.z]);
        let [cx, cy] = map(p[0], p[1]);
        let vmag = Math.hypot(sim.vx, sim.vy, sim.vz);
        if (vmag > 0.5) {
            let vx_proj, vy_proj;
            if (viewType === 'XY') { vx_proj = sim.vx;
                vy_proj = sim.vy; }
            else if (viewType === 'ZY') { vx_proj = sim.vz;
                vy_proj = sim.vy; }
            else { vx_proj = sim.vx;
                vy_proj = sim.vz; }
            let angle = Math.atan2(-vy_proj, vx_proj);
            let len = Math.min(80, vmag * 3);
            let ex = cx + Math.cos(angle) * len;
            let ey = cy + Math.sin(angle) * len;
            ctx.beginPath();
            ctx.moveTo(cx, cy);
            ctx.lineTo(ex, ey);
            ctx.strokeStyle = '#ffcc00';
            ctx.lineWidth = 2;
            ctx.stroke();
            ctx.beginPath();
            ctx.moveTo(ex, ey);
            ctx.lineTo(ex - 6 * Math.cos(angle - 0.4), ey - 6 * Math.sin(angle - 0.4));
            ctx.lineTo(ex - 6 * Math.cos(angle + 0.4), ey - 6 * Math.sin(angle + 0.4));
            ctx.fillStyle = '#ffcc00';
            ctx.fill();
        }
    }
    
    // Acceleration vector
    if (toggles.acc) {
        let p = mapFunc([sim.x, sim.y, sim.z]);
        let [cx, cy] = map(p[0], p[1]);
        let amag = Math.hypot(sim.ax, sim.ay, sim.az);
        if (amag > 0.5) {
            let ax_proj, ay_proj;
            if (viewType === 'XY') { ax_proj = sim.ax;
                ay_proj = sim.ay; }
            else if (viewType === 'ZY') { ax_proj = sim.az;
                ay_proj = sim.ay; }
            else { ax_proj = sim.ax;
                ay_proj = sim.az; }
            let angle = Math.atan2(-ay_proj, ax_proj);
            let len = Math.min(60, amag * 8);
            let ex = cx + Math.cos(angle) * len;
            let ey = cy + Math.sin(angle) * len;
            ctx.beginPath();
            ctx.moveTo(cx, cy);
            ctx.lineTo(ex, ey);
            ctx.strokeStyle = '#00ff88';
            ctx.lineWidth = 1.5;
            ctx.setLineDash([4, 3]);
            ctx.stroke();
            ctx.setLineDash([]);
        }
    }
    
    ctx.fillStyle = '#00d4ff';
    ctx.font = '9px Orbitron';
    ctx.fillText(label, 10, 20);
}

function drawDetailedRocket(ctx, map, worldToProj, viewType) {
    let bottom = rotateVector(sim.q, [0, -sim.comOffset, 0]);
    let top = rotateVector(sim.q, [0, sim.rocketLen - sim.comOffset, 0]);
    let pBottom = worldToProj([sim.x + bottom[0], sim.y + bottom[1], sim.z + bottom[2]]);
    let pTop = worldToProj([sim.x + top[0], sim.y + top[1], sim.z + top[2]]);
    let [x1, y1] = map(pBottom[0], pBottom[1]);
    let [x2, y2] = map(pTop[0], pTop[1]);
    let dx = x2 - x1,
        dy = y2 - y1;
    let angle = Math.atan2(dy, dx);
    let length = Math.hypot(dx, dy);
    let radPx = Math.max(4, length * (sim.radius / sim.rocketLen));
    
    ctx.save();
    ctx.translate(x1, y1);
    ctx.rotate(angle);
    
    ctx.fillStyle = '#2a5a7a';
    ctx.strokeStyle = '#00d4ff';
    ctx.lineWidth = 1;
    ctx.fillRect(0, -radPx / 2, length, radPx);
    ctx.strokeRect(0, -radPx / 2, length, radPx);
    
    ctx.fillStyle = '#5abadd';
    ctx.beginPath();
    ctx.moveTo(length, -radPx / 2);
    ctx.lineTo(length + radPx * 0.7, 0);
    ctx.lineTo(length, radPx / 2);
    ctx.fill();
    ctx.stroke();
    
    let fin = radPx * 0.6;
    ctx.fillStyle = '#2a5a7a';
    ctx.beginPath();
    ctx.moveTo(0, -radPx / 2);
    ctx.lineTo(-fin, -radPx / 2 - fin * 0.4);
    ctx.lineTo(0, -radPx / 2 + fin * 0.3);
    ctx.fill();
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(0, radPx / 2);
    ctx.lineTo(-fin, radPx / 2 + fin * 0.4);
    ctx.lineTo(0, radPx / 2 - fin * 0.3);
    ctx.fill();
    ctx.stroke();
    
    ctx.fillStyle = '#00d4ff';
    ctx.beginPath();
    ctx.arc(length * 0.7, -radPx * 0.2, radPx * 0.2, 0, 2 * Math.PI);
    ctx.fill();
    ctx.beginPath();
    ctx.arc(length * 0.7, radPx * 0.2, radPx * 0.2, 0, 2 * Math.PI);
    ctx.fill();
    
    let fuelFrac = sim.fuelMass / (sim.fuelMass + sim.dryMass);
    ctx.fillStyle = 'rgba(255,200,0,0.2)';
    ctx.fillRect(length * 0.2, -radPx * 0.3, length * 0.2, radPx * 0.6);
    ctx.fillStyle = `rgba(255, ${100 + fuelFrac * 155}, 0, 0.7)`;
    ctx.fillRect(length * 0.2, -radPx * 0.3 + radPx * 0.6 * (1 - fuelFrac), length * 0.2, radPx * 0.6 * fuelFrac);
    
    let gimRad = sim.pitchGimbal * 0.1 * Math.PI / 180;
    ctx.save();
    ctx.translate(0, 0);
    ctx.rotate(gimRad);
    ctx.fillStyle = sim.thr_main > 0 ? '#ff6b00' : '#cc4400';
    ctx.fillRect(-radPx * 0.4, 0, radPx * 0.8, radPx * 0.5);
// Main thruster flame – clear direction, large size
if (sim.thr_main > 0) {
    // Flame length in pixels (absolute, independent of rocket scale)
    let flameLen = 100 + (sim.thr_main / 400000) * 30;
    // Wide base
    let baseW = Math.max(10, radPx * 0.8);
    // Draw flame as a triangle pointing "down" (positive Y) in the rotated frame
    ctx.fillStyle = '#ff6600';
    ctx.beginPath();
    ctx.moveTo(-baseW / 2, radPx * 0.5); // bottom left of nozzle
    ctx.lineTo(0, radPx * 0.5 + flameLen); // tip
    ctx.lineTo(baseW / 2, radPx * 0.5); // bottom right
    ctx.fill();
    // Inner brighter tip
    ctx.fillStyle = '#ffcc00';
    ctx.beginPath();
    ctx.moveTo(-baseW / 4, radPx * 0.5);
    ctx.lineTo(0, radPx * 0.5 + flameLen * 0.8);
    ctx.lineTo(baseW / 4, radPx * 0.5);
    ctx.fill();
}
    ctx.restore();
    
    if (Math.abs(sim.rcsX) + Math.abs(sim.rcsZ) > 0) {
        ctx.fillStyle = '#00ff88';
        ctx.beginPath();
        ctx.arc(length * 0.3, -radPx / 2 - 3, 3, 0, 2 * Math.PI);
        ctx.fill();
        ctx.beginPath();
        ctx.arc(length * 0.3, radPx / 2 + 3, 3, 0, 2 * Math.PI);
        ctx.fill();
    }
    
    ctx.strokeStyle = '#cc44ff';
    ctx.beginPath();
    let comX = (sim.comOffset / sim.rocketLen) * length;
    ctx.moveTo(comX - 4, -radPx / 2 - 4);
    ctx.lineTo(comX + 4, -radPx / 2 - 4);
    ctx.moveTo(comX, -radPx / 2 - 8);
    ctx.lineTo(comX, -radPx / 2);
    ctx.stroke();
    
    ctx.restore();
}

export function drawAttitudeIndicator() {
    let canvas = document.getElementById('attitudeCanvas');
    if (!canvas) return;
    let ctx = canvas.getContext('2d');
    canvas.width = 200;
    canvas.height = 150;
    ctx.fillStyle = '#02070f';
    ctx.fillRect(0, 0, 200, 150);
    let e = quatToEuler(sim.q);
    let pitch = e.pitch,
        roll = e.roll;
    ctx.save();
    ctx.translate(100, 75);
    ctx.strokeStyle = '#00d4ff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(0, 0, 40, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(0, -45);
    ctx.lineTo(0, 45);
    ctx.moveTo(-45, 0);
    ctx.lineTo(45, 0);
    ctx.stroke();
    ctx.save();
    ctx.rotate(-roll);
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(0, -35);
    ctx.lineTo(8, -25);
    ctx.lineTo(-8, -25);
    ctx.fillStyle = '#ffaa44';
    ctx.fill();
    ctx.restore();
    ctx.fillStyle = '#6ff';
    ctx.font = 'bold 12px monospace';
    ctx.fillText(`Pitch: ${(pitch * 180 / Math.PI).toFixed(0)}°`, -60, -50);
    ctx.fillStyle = '#0f6';
    ctx.fillText(`Roll: ${(roll * 180 / Math.PI).toFixed(0)}°`, -60, -30);
    ctx.restore();
}

export function drawAllCanvases() {
    drawProjection(document.getElementById('canvasXY'), (p) => [p[0], p[1]], [-150, 150], [0, 300], 'X-Y (Range vs Alt)', 'XY');
    drawProjection(document.getElementById('canvasZY'), (p) => [p[2], p[1]], [-150, 150], [0, 300], 'Z-Y (Lateral vs Alt)', 'ZY');
    drawProjection(document.getElementById('canvasXZ'), (p) => [p[0], p[2]], [-150, 150], [-150, 150], 'X-Z (Top view)', 'XZ');
    drawAttitudeIndicator();
}

export function resizeAllCanvases() {
    ['canvasXY', 'canvasZY', 'canvasXZ'].forEach(id => {
        let c = document.getElementById(id);
        if (c) {
            let rect = c.parentElement.getBoundingClientRect();
            c.width = rect.width;
            c.height = rect.height;
        }
    });
    let att = document.getElementById('attitudeCanvas');
    if (att) { att.width = 200;
        att.height = 150; }
    drawAllCanvases();
}