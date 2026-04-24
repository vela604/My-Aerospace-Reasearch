// Constants
export const Ve_main = 3200;
export const Ve_rcs = 2800;
export const Cd = 0.35;
export const rho_air = 1.225;

// Global simulation state
export let sim = {
    t: 0,
    x: 0,
    y: 0,
    z: 0,
    vx: 0,
    vy: 0,
    vz: 0,
    q: [1, 0, 0, 0],
    wx: 0,
    wy: 0,
    wz: 0,
    dryMass: 5000,
    fuelMass: 3000,
    totalMass: 8000,
    rocketLen: 15,
    radius: 1.5,
    gravity: 9.81,
    Ixx: 0,
    Iyy: 0,
    Izz: 0,
    comOffset: 0,
    thr_main: 0,
    pitchGimbal: 0,
    yawGimbal: 0,
    rcsX: 0,
    rcsZ: 0,
    rollTorque: 0,
    Fx_tot: 0,
    Fy_tot: 0,
    Fz_tot: 0,
    tau_x: 0,
    tau_y: 0,
    tau_z: 0,
    ax: 0,
    ay: 0,
    az: 0,
    trail: []
};

// Mass properties (inertia with parallel axis)
export function updateMassProperties() {
    let mFuel = Math.max(0, sim.fuelMass);
    sim.totalMass = sim.dryMass + mFuel;
    if (sim.totalMass <= 0) return;
    const len = sim.rocketLen;
    const r = sim.radius;
    const fuelFrac = mFuel / sim.totalMass;
    const fuelCom = 0.35 * len;
    const dryCom = 0.55 * len;
    sim.comOffset = fuelFrac * fuelCom + (1 - fuelFrac) * dryCom;
    const I_roll_cm = 0.5 * sim.totalMass * r * r;
    const I_trans_cm = (1 / 12) * sim.totalMass * (3 * r * r + len * len);
    const d = sim.comOffset - len / 2;
    sim.Ixx = I_roll_cm + sim.totalMass * d * d;
    sim.Iyy = I_trans_cm + sim.totalMass * d * d;
    sim.Izz = I_trans_cm + sim.totalMass * d * d;
}

// Rotate vector by quaternion
export function rotateVector(q, v) {
    const [w, x, y, z] = q;
    const [vx, vy, vz] = v;
    return [
        (w * w + x * x - y * y - z * z) * vx + 2 * (x * y - w * z) * vy + 2 * (x * z + w * y) * vz,
        2 * (x * y + w * z) * vx + (w * w - x * x + y * y - z * z) * vy + 2 * (y * z - w * x) * vz,
        2 * (x * z - w * y) * vx + 2 * (y * z + w * x) * vy + (w * w - x * x - y * y + z * z) * vz
    ];
}

// Quaternion multiplication
export function quatMult(q1, q2) {
    return [
        q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
        q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2],
        q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1],
        q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]
    ];
}

// Quaternion to Euler (radians)
export function quatToEuler(q) {
    const pitch = Math.asin(2 * (q[0] * q[2] - q[3] * q[1]));
    const yaw = Math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
    const roll = Math.atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));
    return { pitch, yaw, roll };
}

// Forces, torques, accelerations (with gimbal scaling and alpha clamp)
export function computeForcesAndTorques(dragEnabled) {
    const m = sim.totalMass;
    const g = sim.gravity;
    let thrust_mag = sim.thr_main;
    if (sim.fuelMass <= 0) thrust_mag = 0;
    
    const gimbal_scale = 0.1;
    let pitch_eff = sim.pitchGimbal * gimbal_scale * Math.PI / 180;
    let yaw_eff = sim.yawGimbal * gimbal_scale * Math.PI / 180;
    
    let thrust_dir = [0, 1, 0];
    let tx = thrust_dir[0];
    let ty = thrust_dir[1] * Math.cos(pitch_eff) - thrust_dir[2] * Math.sin(pitch_eff);
    let tz = thrust_dir[1] * Math.sin(pitch_eff) + thrust_dir[2] * Math.cos(pitch_eff);
    let fx = tx * Math.cos(yaw_eff) - ty * Math.sin(yaw_eff);
    let fy = tx * Math.sin(yaw_eff) + ty * Math.cos(yaw_eff);
    let fz = tz;
    let thrust_body = [thrust_mag * fx, thrust_mag * fy, thrust_mag * fz];
    let thrust_world = rotateVector(sim.q, thrust_body);
    
    let rcs_body = [sim.rcsX, 0, sim.rcsZ];
    let rcs_world = rotateVector(sim.q, rcs_body);
    let gravity_world = [0, -m * g, 0];
    let drag_world = [0, 0, 0];
    if (dragEnabled) {
        let vmag = Math.hypot(sim.vx, sim.vy, sim.vz);
        if (vmag > 0.01) {
            let area = 2 * sim.radius * sim.rocketLen;
            let Fd = 0.5 * rho_air * Cd * area * vmag * vmag;
            drag_world = [-Fd * sim.vx / vmag, -Fd * sim.vy / vmag, -Fd * sim.vz / vmag];
        }
    }
    let total_force = [
        thrust_world[0] + rcs_world[0] + gravity_world[0] + drag_world[0],
        thrust_world[1] + rcs_world[1] + gravity_world[1] + drag_world[1],
        thrust_world[2] + rcs_world[2] + gravity_world[2] + drag_world[2]
    ];
    
    // Torques (body frame)
    let r_eng = [0, -sim.comOffset, 0];
    let torque_main = [
        r_eng[1] * thrust_body[2] - r_eng[2] * thrust_body[1],
        r_eng[2] * thrust_body[0] - r_eng[0] * thrust_body[2],
        r_eng[0] * thrust_body[1] - r_eng[1] * thrust_body[0]
    ];
    let posX = [0, sim.rocketLen * 0.6, 0];
    let posZ = [sim.radius * 1.2, 0, 0];
    let Fx_body = [sim.rcsX, 0, 0];
    let Fz_body = [0, 0, sim.rcsZ];
    let torqueX = [
        posX[1] * Fx_body[2] - posX[2] * Fx_body[1],
        posX[2] * Fx_body[0] - posX[0] * Fx_body[2],
        posX[0] * Fx_body[1] - posX[1] * Fx_body[0]
    ];
    let torqueZ = [
        posZ[1] * Fz_body[2] - posZ[2] * Fz_body[1],
        posZ[2] * Fz_body[0] - posZ[0] * Fz_body[2],
        posZ[0] * Fz_body[1] - posZ[1] * Fz_body[0]
    ];
    let torque_rcs = [torqueX[0] + torqueZ[0], torqueX[1] + torqueZ[1], torqueX[2] + torqueZ[2]];
    let torque_roll = [sim.rollTorque, 0, 0];
    let total_torque = [
        torque_main[0] + torque_rcs[0] + torque_roll[0],
        torque_main[1] + torque_rcs[1] + torque_roll[1],
        torque_main[2] + torque_rcs[2] + torque_roll[2]
    ];
    
    let alpha = [
        total_torque[0] / sim.Ixx,
        total_torque[1] / sim.Iyy,
        total_torque[2] / sim.Izz
    ];
    const maxAlpha = 2.0;
    alpha[0] = Math.min(maxAlpha, Math.max(-maxAlpha, alpha[0]));
    alpha[1] = Math.min(maxAlpha, Math.max(-maxAlpha, alpha[1]));
    alpha[2] = Math.min(maxAlpha, Math.max(-maxAlpha, alpha[2]));
    
    sim.Fx_tot = total_force[0];
    sim.Fy_tot = total_force[1];
    sim.Fz_tot = total_force[2];
    sim.tau_x = total_torque[0];
    sim.tau_y = total_torque[1];
    sim.tau_z = total_torque[2];
    sim.ax = total_force[0] / m;
    sim.ay = total_force[1] / m;
    sim.az = total_force[2] / m;
    
    return { acc: [sim.ax, sim.ay, sim.az], alpha };
}

// Single physics step (Euler with sub‑steps inside main loop)
export function physicsStep(dt, dragEnabled) {
    if (sim.fuelMass > 0) {
        let mdot = sim.thr_main / Ve_main + (Math.abs(sim.rcsX) + Math.abs(sim.rcsZ) + Math.abs(sim.rollTorque) / sim.radius) / Ve_rcs;
        let dm = Math.min(sim.fuelMass, mdot * dt);
        sim.fuelMass -= dm;
        updateMassProperties();
    }
    let { acc, alpha } = computeForcesAndTorques(dragEnabled);
    sim.vx += acc[0] * dt;
    sim.vy += acc[1] * dt;
    sim.vz += acc[2] * dt;
    sim.x += sim.vx * dt;
    sim.y += sim.vy * dt;
    sim.z += sim.vz * dt;
    sim.wx += alpha[0] * dt;
    sim.wy += alpha[1] * dt;
    sim.wz += alpha[2] * dt;
    let dq = [0, sim.wx * dt, sim.wy * dt, sim.wz * dt];
    let newq = [sim.q[0] + dq[0], sim.q[1] + dq[1], sim.q[2] + dq[2], sim.q[3] + dq[3]];
    let norm = Math.hypot(newq[0], newq[1], newq[2], newq[3]);
    if (norm > 1e-8) sim.q = [newq[0] / norm, newq[1] / norm, newq[2] / norm, newq[3] / norm];
    if (sim.y < 0 && sim.vy < 0) {
        sim.y = 0;
        sim.vy *= -0.2;
    }
}