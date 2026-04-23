// ==================== VECTOR3 CLASS ====================
class Vector3 {
    constructor(x = 0, y = 0, z = 0) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
    
    add(v) {
        return new Vector3(this.x + v.x, this.y + v.y, this.z + v.z);
    }
    
    subtract(v) {
        return new Vector3(this.x - v.x, this.y - v.y, this.z - v.z);
    }
    
    scale(s) {
        return new Vector3(this.x * s, this.y * s, this.z * s);
    }
    
    dot(v) {
        return this.x * v.x + this.y * v.y + this.z * v.z;
    }
    
    cross(v) {
        return new Vector3(
            this.y * v.z - this.z * v.y,
            this.z * v.x - this.x * v.z,
            this.x * v.y - this.y * v.x
        );
    }
    
    magnitude() {
        return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
    }
    
    normalize() {
        const mag = this.magnitude();
        if (mag === 0) return new Vector3(0, 0, 1);
        return new Vector3(this.x / mag, this.y / mag, this.z / mag);
    }
    
    clone() {
        return new Vector3(this.x, this.y, this.z);
    }
    
    toArray() {
        return [this.x, this.y, this.z];
    }
}

// ==================== ATMOSPHERIC MODEL ====================
class AtmosphericModel {
    constructor() {
        this.sea_level_density = 1.225; // kg/m^3
        this.scale_height = 8500; // meters
        this.enabled = false;
        this.Cd = 0.25; // coefficient of drag for rocket
    }
    
    getDensity(altitude) {
        if (!this.enabled) return 0;
        if (altitude < 0) altitude = 0;
        return this.sea_level_density * Math.exp(-altitude / this.scale_height);
    }
    
    getDragForce(velocity, cross_section_area, altitude) {
        if (!this.enabled) return new Vector3(0, 0, 0);
        
        const speed = velocity.magnitude();
        if (speed < 0.01) return new Vector3(0, 0, 0);
        
        const density = this.getDensity(altitude);
        const drag_magnitude = 0.5 * density * speed * speed * this.Cd * cross_section_area;
        const drag_direction = velocity.normalize().scale(-1);
        
        return drag_direction.scale(drag_magnitude);
    }
}

// ==================== THRUSTER CLASS ====================
class Thruster {
    constructor(max_thrust = 500000, exhaust_velocity = 3800) {
        this.max_thrust = max_thrust;
        this.exhaust_velocity = exhaust_velocity; // m/s (constant)
        this.throttle = 0; // 0 to 1
        this.gimbal_angle = 0; // degrees (-30 to +30)
        this.fuel_flow_rate = 0; // kg/s
    }
    
    setThrottle(throttle) {
        this.throttle = Math.max(0, Math.min(1, throttle));
        this.updateFuelFlow();
    }
    
    setGimbalAngle(angle) {
        this.gimbal_angle = Math.max(-30, Math.min(30, angle));
    }
    
    updateFuelFlow() {
        // F = ṁ * Ve, so ṁ = F / Ve
        const thrust_magnitude = this.throttle * this.max_thrust;
        this.fuel_flow_rate = thrust_magnitude / this.exhaust_velocity;
    }
    
    getThrust() {
        return this.throttle * this.max_thrust;
    }
    
    getThrustVector(pitch, roll, yaw) {
        // Gimbal angle applied as pitch offset
        const gimbal_rad = this.gimbal_angle * Math.PI / 180;
        const pitch_effective = pitch + gimbal_rad;
        
        const thrust_mag = this.getThrust();
        
        // Thrust vector in body frame (pointing forward)
        const thrust_x = thrust_mag * Math.sin(gimbal_rad);
        const thrust_z = thrust_mag * Math.cos(gimbal_rad);
        
        return new Vector3(thrust_x, 0, thrust_z);
    }
}

// ==================== RCS CLASS ====================
class RCS {
    constructor() {
        this.rcs_force = 5000; // N per RCS thruster
        this.arm_length = 1.5; // m from CoM to RCS nozzle
        
        // 6 RCS thrusters
        this.pitch_up = 0;    // -1, 0, 1
        this.pitch_down = 0;
        this.roll_left = 0;
        this.roll_right = 0;
        this.yaw_left = 0;
        this.yaw_right = 0;
    }
    
    setRCS(axis, direction) {
        // axis: 'pitch', 'roll', 'yaw'
        // direction: -1 (left/down), 0 (off), 1 (right/up)
        
        if (axis === 'pitch') {
            this.pitch_up = Math.max(0, direction);
            this.pitch_down = Math.max(0, -direction);
        } else if (axis === 'roll') {
            this.roll_right = Math.max(0, direction);
            this.roll_left = Math.max(0, -direction);
        } else if (axis === 'yaw') {
            this.yaw_right = Math.max(0, direction);
            this.yaw_left = Math.max(0, -direction);
        }
    }
    
    getTorque() {
        const torque_x = (this.pitch_up - this.pitch_down) * this.rcs_force * this.arm_length;
        const torque_y = (this.roll_right - this.roll_left) * this.rcs_force * this.arm_length;
        const torque_z = (this.yaw_right - this.yaw_left) * this.rcs_force * this.arm_length;
        
        return new Vector3(torque_x, torque_y, torque_z);
    }
    
    getTotalForce() {
        const rcs_forces = this.pitch_up + this.pitch_down + this.roll_left + 
                          this.roll_right + this.yaw_left + this.yaw_right;
        return rcs_forces * this.rcs_force;
    }
}

// ==================== ROCKET CLASS ====================
class Rocket {
    constructor() {
        // Physical properties
        this.dry_mass = 1000; // kg
        this.fuel_mass = 500; // kg
        this.current_mass = this.dry_mass + this.fuel_mass;
        this.cross_section_area = Math.PI * 0.2 * 0.2; // m^2 (radius 0.2m)
        this.length = 3; // meters
        
        // State vectors
        this.position = new Vector3(0, 0, 0); // meters (z=0 is ground)
        this.velocity = new Vector3(0, 0, 0); // m/s
        this.acceleration = new Vector3(0, 0, 0); // m/s^2
        
        // Angles in radians (pitch, roll, yaw = Euler angles)
        this.pitch = 0; // rotation around Y
        this.roll = 0;  // rotation around X
        this.yaw = 0;   // rotation around Z
        
        // Angular velocity in rad/s
        this.angular_velocity = new Vector3(0, 0, 0);
        this.angular_acceleration = new Vector3(0, 0, 0);
        
        // Center of Mass offset from geometric center (changes with fuel burn)
        this.com_offset = 0; // meters along body axis
        
        // Inertia tensor (approximate as cylinder)
        this.updateInertia();
        
        // Thrusters
        this.thruster = new Thruster(500000, 3800); // 500kN max thrust
        this.rcs = new RCS();
        
        // Atmosphere
        this.atmosphere = new AtmosphericModel();
        
        // Tracking
        this.trajectory = [];
        this.max_altitude = 0;
        this.time = 0;
        this.crashed = false;
    }
    
    updateInertia() {
        // Approximate as cylinder: Ixx = (1/4)*m*r^2 + (1/12)*m*L^2
        // Iyy = Ixx (symmetric)
        // Izz = (1/2)*m*r^2
        
        const r = 0.2;
        const L = this.length;
        
        const Ixx = (1/4) * this.current_mass * r * r + (1/12) * this.current_mass * L * L;
        const Iyy = Ixx;
        const Izz = (1/2) * this.current_mass * r * r;
        
        // Create inertia matrix
        this.inertia = {
            xx: Ixx,
            yy: Iyy,
            zz: Izz,
            xy: 0,
            xz: 0,
            yz: 0
        };
    }
    
    consumeFuel(mass_rate, dt) {
        const consumed = mass_rate * dt;
        this.fuel_mass = Math.max(0, this.fuel_mass - consumed);
        
        // Update total mass
        this.current_mass = this.dry_mass + this.fuel_mass;
        
        // Update CoM - shifts forward as fuel burns
        const fuel_ratio = this.fuel_mass / 500; // normalized
        this.com_offset = 0.3 * fuel_ratio; // offset in meters
        
        // Update inertia
        this.updateInertia();
    }
    
    recordTrajectory() {
        if (this.trajectory.length > 10000) {
            this.trajectory.shift(); // Keep last 10000 points
        }
        this.trajectory.push({
            x: this.position.x,
            y: this.position.y,
            z: this.position.z,
            time: this.time
        });
    }
    
    checkCrash() {
        if (this.position.z <= 0 && this.time > 0.1) {
            this.crashed = true;
            return true;
        }
        return false;
    }
    
    predictImpact() {
        if (this.velocity.z >= 0) return null; // Still going up
        
        // Simple ballistic prediction (ignoring drag for speed)
        const t_impact = -this.velocity.z / 9.81;
        const impact_distance = this.velocity.x * t_impact + 0.5 * this.acceleration.x * t_impact * t_impact;
        const impact_x = this.position.x + impact_distance;
        
        return {
            x: impact_x,
            y: this.position.y,
            z: 0,
            time: this.time + t_impact
        };
    }
}

// ==================== SIMULATION CLASS ====================
class Simulation {
    constructor() {
        this.rocket = new Rocket();
        this.running = false;
        this.dt = 0.01; // 10ms timestep (100Hz)
        this.last_time = performance.now();
        this.accumulated_time = 0;
        
        // Input state
        this.input = {
            thrust_delta: 0,
            gimbal_delta: 0,
            rcs_pitch: 0,
            rcs_roll: 0,
            rcs_yaw: 0
        };
        
        // Canvas references
        this.canvases = {
            XY: document.getElementById('canvasXY'),
            ZY: document.getElementById('canvasZY'),
            XZ: document.getElementById('canvasXZ')
        };
        
        this.scale = 0.1; // pixels per meter
        this.setupKeyboardControls();
        this.requestAnimationFrameID = null;
    }
    
    setupKeyboardControls() {
        document.addEventListener('keydown', (e) => {
            switch(e.key) {
                case 'ArrowUp':
                    this.rocket.thruster.setThrottle(this.rocket.thruster.throttle + 0.05);
                    break;
                case 'ArrowDown':
                    this.rocket.thruster.setThrottle(this.rocket.thruster.throttle - 0.05);
                    break;
                case 'a': case 'A':
                    this.rocket.thruster.setGimbalAngle(this.rocket.thruster.gimbal_angle - 2);
                    break;
                case 'd': case 'D':
                    this.rocket.thruster.setGimbalAngle(this.rocket.thruster.gimbal_angle + 2);
                    break;
                case 'w': case 'W':
                    this.rocket.rcs.setRCS('pitch', 1);
                    break;
                case 's': case 'S':
                    this.rocket.rcs.setRCS('pitch', -1);
                    break;
                case 'q': case 'Q':
                    this.rocket.rcs.setRCS('roll', -1);
                    break;
                case 'e': case 'E':
                    this.rocket.rcs.setRCS('roll', 1);
                    break;
                case 'z': case 'Z':
                    this.rocket.rcs.setRCS('yaw', -1);
                    break;
                case 'x': case 'X':
                    this.rocket.rcs.setRCS('yaw', 1);
                    break;
                case ' ':
                    e.preventDefault();
                    this.rocket.atmosphere.enabled = !this.rocket.atmosphere.enabled;
                    break;
            }
        });
        
        document.addEventListener('keyup', (e) => {
            if (['w', 's', 'q', 'e', 'z', 'x', 'W', 'S', 'Q', 'E', 'Z', 'X'].includes(e.key)) {
                this.rocket.rcs.setRCS('pitch', 0);
                this.rocket.rcs.setRCS('roll', 0);
                this.rocket.rcs.setRCS('yaw', 0);
            }
        });
    }
    
    computeForces() {
        const forces = {
            gravity: new Vector3(0, 0, -this.rocket.current_mass * 9.81),
            thrust: this.rocket.thruster.getThrustVector(this.rocket.pitch, this.rocket.roll, this.rocket.yaw),
            drag: this.rocket.atmosphere.getDragForce(this.rocket.velocity, this.rocket.cross_section_area, this.rocket.position.z)
        };
        
        return forces;
    }
    
    computeTorques() {
        const forces = this.computeForces();
        
        // Torque from gimbal offset (thrust not aligned with CoM)
        const gimbal_offset = 0.5; // meters along body axis
        const torque_gimbal = new Vector3(
            this.rocket.thruster.gimbal_angle * Math.PI / 180 * gimbal_offset * this.rocket.thruster.getThrust(),
            0,
            0
        );
        
        // RCS torques
        const torque_rcs = this.rocket.rcs.getTorque();
        
        return {
            gimbal: torque_gimbal,
            rcs: torque_rcs,
            total: torque_gimbal.add(torque_rcs)
        };
    }
    
    stateDerivative(state) {
        // state = { position, velocity, angles, angular_velocity }
        
        const forces = this.computeForces();
        const torques = this.computeTorques();
        
        // Linear acceleration: a = F / m
        const total_force = forces.gravity.add(forces.thrust).add(forces.drag);
        const acceleration = total_force.scale(1 / this.rocket.current_mass);
        
        // Angular acceleration: α = I^-1 * τ
        const torque_total = torques.total;
        const angular_accel_x = torque_total.x / this.rocket.inertia.xx;
        const angular_accel_y = torque_total.y / this.rocket.inertia.yy;
        const angular_accel_z = torque_total.z / this.rocket.inertia.zz;
        const angular_acceleration = new Vector3(angular_accel_x, angular_accel_y, angular_accel_z);
        
        return {
            position_derivative: state.velocity,
            velocity_derivative: acceleration,
            angle_derivative: state.angular_velocity,
            angular_velocity_derivative: angular_acceleration,
            forces: forces,
            torques: torques
        };
    }
    
    rk4Step(dt) {
        // RK4 integration
        const state0 = {
            position: this.rocket.position.clone(),
            velocity: this.rocket.velocity.clone(),
            angles: new Vector3(this.rocket.pitch, this.rocket.roll, this.rocket.yaw),
            angular_velocity: this.rocket.angular_velocity.clone()
        };
        
        const k1 = this.stateDerivative(state0);
        
        const state1 = {
            position: state0.position.add(k1.position_derivative.scale(dt/2)),
            velocity: state0.velocity.add(k1.velocity_derivative.scale(dt/2)),
            angles: state0.angles.add(k1.angle_derivative.scale(dt/2)),
            angular_velocity: state0.angular_velocity.add(k1.angular_velocity_derivative.scale(dt/2))
        };
        
        const k2 = this.stateDerivative(state1);
        
        const state2 = {
            position: state0.position.add(k2.position_derivative.scale(dt/2)),
            velocity: state0.velocity.add(k2.velocity_derivative.scale(dt/2)),
            angles: state0.angles.add(k2.angle_derivative.scale(dt/2)),
            angular_velocity: state0.angular_velocity.add(k2.angular_velocity_derivative.scale(dt/2))
        };
        
        const k3 = this.stateDerivative(state2);
        
        const state3 = {
            position: state0.position.add(k3.position_derivative.scale(dt)),
            velocity: state0.velocity.add(k3.velocity_derivative.scale(dt)),
            angles: state0.angles.add(k3.angle_derivative.scale(dt)),
            angular_velocity: state0.angular_velocity.add(k3.angular_velocity_derivative.scale(dt))
        };
        
        const k4 = this.stateDerivative(state3);
        
        // Update state
        this.rocket.position = state0.position.add(
            k1.position_derivative.add(k2.position_derivative.scale(2)).add(k3.position_derivative.scale(2)).add(k4.position_derivative).scale(dt/6)
        );
        
        this.rocket.velocity = state0.velocity.add(
            k1.velocity_derivative.add(k2.velocity_derivative.scale(2)).add(k3.velocity_derivative.scale(2)).add(k4.velocity_derivative).scale(dt/6)
        );
        
        const angles = state0.angles.add(
            k1.angle_derivative.add(k2.angle_derivative.scale(2)).add(k3.angle_derivative.scale(2)).add(k4.angle_derivative).scale(dt/6)
        );
        this.rocket.pitch = angles.x;
        this.rocket.roll = angles.y;
        this.rocket.yaw = angles.z;
        
        this.rocket.angular_velocity = state0.angular_velocity.add(
            k1.angular_velocity_derivative.add(k2.angular_velocity_derivative.scale(2)).add(k3.angular_velocity_derivative.scale(2)).add(k4.angular_velocity_derivative).scale(dt/6)
        );
        
        // Update acceleration for display
        this.rocket.acceleration = k1.velocity_derivative;
        
        // Consume fuel
        this.rocket.consumeFuel(this.rocket.thruster.fuel_flow_rate, dt);
        
        // Track trajectory
        this.rocket.recordTrajectory();
        
        // Update max altitude
        if (this.rocket.position.z > this.rocket.max_altitude) {
            this.rocket.max_altitude = this.rocket.position.z;
        }
        
        // Update time
        this.rocket.time += dt;
    }
    
    render() {
        this.renderCanvas(this.canvases.XY, 'X', 'Y');
        this.renderCanvas(this.canvases.ZY, 'Z', 'Y');
        this.renderCanvas(this.canvases.XZ, 'X', 'Z');
        this.updateDataDisplay();
    }
    
    renderCanvas(canvas, axis1, axis2) {
        const ctx = canvas.getContext('2d');
        const width = canvas.width;
        const height = canvas.height;
        
        // Clear
        ctx.fillStyle = '#000';
        ctx.fillRect(0, 0, width, height);
        
        // Grid
        ctx.strokeStyle = '#111';
        ctx.lineWidth = 1;
        for (let i = -500; i <= 500; i += 100) {
            const x1 = width/2 + i * this.scale;
            const y1 = height/2 - 500 * this.scale;
            const x2 = width/2 + i * this.scale;
            const y2 = height/2 + 500 * this.scale;
            ctx.beginPath();
            ctx.moveTo(x1, y1);
            ctx.lineTo(x2, y2);
            ctx.stroke();
        }
        
        // Trajectory trail
        ctx.strokeStyle = '#00ff00';
        ctx.lineWidth = 1;
        ctx.beginPath();
        for (let i = 0; i < this.rocket.trajectory.length; i++) {
            const point = this.rocket.trajectory[i];
            const px = width/2 + this.getCoord(point, axis1) * this.scale;
            const py = height/2 - this.getCoord(point, axis2) * this.scale;
            
            if (i === 0) ctx.moveTo(px, py);
            else ctx.lineTo(px, py);
        }
        ctx.stroke();
        
        // Rocket body
        const rx = width/2 + this.getCoord(this.rocket.position, axis1) * this.scale;
        const ry = height/2 - this.getCoord(this.rocket.position, axis2) * this.scale;
        
        // Draw rocket as a circle
        ctx.fillStyle = '#ff0088';
        ctx.beginPath();
        ctx.arc(rx, ry, 8, 0, Math.PI * 2);
        ctx.fill();
        
        // Velocity vector
        ctx.strokeStyle = '#0088ff';
        ctx.lineWidth = 2;
        const vx = this.getCoord(this.rocket.velocity, axis1) * 0.1;
        const vy = this.getCoord(this.rocket.velocity, axis2) * 0.1;
        ctx.beginPath();
        ctx.moveTo(rx, ry);
        ctx.lineTo(rx + vx, ry - vy);
        ctx.stroke();
        
        // Thrust vector
        const forces = this.computeForces();
        ctx.strokeStyle = '#ff0000';
        ctx.lineWidth = 2;
        const fx = this.getCoord(forces.thrust, axis1) * 0.00001;
        const fy = this.getCoord(forces.thrust, axis2) * 0.00001;
        ctx.beginPath();
        ctx.moveTo(rx, ry);
        ctx.lineTo(rx + fx, ry - fy);
        ctx.stroke();
        
        // CoM marker
        ctx.fillStyle = '#ffff00';
        ctx.fillRect(rx - 2, ry - 2, 4, 4);
    }
    
    getCoord(vec, axis) {
        if (axis === 'X') return vec.x;
        if (axis === 'Y') return vec.y;
        if (axis === 'Z') return vec.z;
        return 0;
    }
    
    updateDataDisplay() {
        document.getElementById('posX').innerText = this.rocket.position.x.toFixed(2);
        document.getElementById('posY').innerText = this.rocket.position.y.toFixed(2);
        document.getElementById('posZ').innerText = this.rocket.position.z.toFixed(2);
        document.getElementById('altitude').innerText = this.rocket.position.z.toFixed(2);
        
        document.getElementById('velX').innerText = this.rocket.velocity.x.toFixed(2);
        document.getElementById('velY').innerText = this.rocket.velocity.y.toFixed(2);
        document.getElementById('velZ').innerText = this.rocket.velocity.z.toFixed(2);
        
        const speed = this.rocket.velocity.magnitude();
        document.getElementById('speed').innerText = speed.toFixed(2);
        
        document.getElementById('acclX').innerText = this.rocket.acceleration.x.toFixed(2);
        document.getElementById('acclY').innerText = this.rocket.acceleration.y.toFixed(2);
        document.getElementById('acclZ').innerText = this.rocket.acceleration.z.toFixed(2);
        
        const total_accel = this.rocket.acceleration.magnitude();
        document.getElementById('totalAccel').innerText = total_accel.toFixed(2);
        
        document.getElementById('pitch').innerText = (this.rocket.pitch * 180 / Math.PI).toFixed(2);
        document.getElementById('roll').innerText = (this.rocket.roll * 180 / Math.PI).toFixed(2);
        document.getElementById('yaw').innerText = (this.rocket.yaw * 180 / Math.PI).toFixed(2);
        
        document.getElementById('angVelX').innerText = (this.rocket.angular_velocity.x * 180 / Math.PI).toFixed(2);
        document.getElementById('angVelY').innerText = (this.rocket.angular_velocity.y * 180 / Math.PI).toFixed(2);
        document.getElementById('angVelZ').innerText = (this.rocket.angular_velocity.z * 180 / Math.PI).toFixed(2);
        
        document.getElementById('currentMass').innerText = this.rocket.current_mass.toFixed(2);
        document.getElementById('fuel').innerText = this.rocket.fuel_mass.toFixed(2);
        document.getElementById('fuelPercent').innerText = (this.rocket.fuel_mass / 500 * 100).toFixed(2);
        
        document.getElementById('mainThrust').innerText = this.rocket.thruster.getThrust().toFixed(0);
        document.getElementById('throttle').innerText = (this.rocket.thruster.throttle * 100).toFixed(1);
        document.getElementById('fuelFlow').innerText = this.rocket.thruster.fuel_flow_rate.toFixed(3);
        document.getElementById('gimbalDisplay').innerText = this.rocket.thruster.gimbal_angle.toFixed(2);
        
        const g_force = total_accel / 9.81;
        document.getElementById('gForce').innerText = g_force.toFixed(2);
        
        const twr = this.rocket.thruster.getThrust() / (this.rocket.current_mass * 9.81);
        document.getElementById('TWRatio').innerText = twr.toFixed(3);
        
        document.getElementById('timeElapsed').innerText = this.rocket.time.toFixed(2);
        document.getElementById('maxAltitude').innerText = this.rocket.max_altitude.toFixed(2);
        
        const impact = this.rocket.predictImpact();
        if (impact) {
            document.getElementById('impactPred').innerText = impact.x.toFixed(0);
        }
        
        let status = 'Flying';
        if (this.rocket.crashed) status = 'CRASHED';
        else if (this.rocket.fuel_mass <= 0) status = 'Out of Fuel';
        else if (speed > 3000) status = 'HYPERSONIC';
        else if (speed > 1000) status = 'Supersonic';
        
        document.getElementById('status').innerText = status;
    }
    
    play() {
        this.running = true;
        this.loop();
    }
    
    pause() {
        this.running = false;
    }
    
    reset() {
        this.running = false;
        this.rocket = new Rocket();
        this.render();
    }
    
    toggleDrag() {
        this.rocket.atmosphere.enabled = !this.rocket.atmosphere.enabled;
    }
    
    loop = () => {
        if (this.running && !this.rocket.crashed && this.rocket.position.z >= 0) {
            this.rk4Step(this.dt);
            this.render();
            
            if (this.rocket.checkCrash()) {
                this.rocket.crashed = true;
            }
        }
        
        if (this.running) {
            this.requestAnimationFrameID = requestAnimationFrame(this.loop);
        }
    }
}

// ==================== INITIALIZATION ====================
let simulation = null;

window.addEventListener('load', () => {
    simulation = new Simulation();
    simulation.render();
});

// Export for use in HTML
window.simulation = simulation;
