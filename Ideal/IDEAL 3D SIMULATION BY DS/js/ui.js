import { sim, updateMassProperties } from './physics.js';
import { getToggles, setToggles, drawAllCanvases, resizeAllCanvases } from './rendering.js';

export function updateUIinputs() {
    document.getElementById('thrustVal').innerText = sim.thr_main;
    document.getElementById('gimPitchVal').innerText = sim.pitchGimbal;
    document.getElementById('gimYawVal').innerText = sim.yawGimbal;
    document.getElementById('rcsXVal').innerText = sim.rcsX;
    document.getElementById('rcsZVal').innerText = sim.rcsZ;
    document.getElementById('rollTorqueVal').innerText = sim.rollTorque;
}

export function updateDataDisplay() {
    let e = { pitch: 0, yaw: 0, roll: 0 }; // will be set from main
    // Avoid circular dependency – we'll call quatToEuler from main and pass values
    // For now, simple placeholders. The main.js will override.
}

export function bindUIEvents(startCallback, stopCallback, resetCallback) {
    document.getElementById('btnStart').onclick = startCallback;
    document.getElementById('btnStop').onclick = stopCallback;
    document.getElementById('btnReset').onclick = resetCallback;
    
    document.getElementById('mainThrust').oninput = () => { sim.thr_main = parseFloat(document.getElementById('mainThrust').value);
        updateUIinputs(); };
    document.getElementById('gimbalPitch').oninput = () => { sim.pitchGimbal = parseFloat(document.getElementById('gimbalPitch').value);
        updateUIinputs(); };
    document.getElementById('gimbalYaw').oninput = () => { sim.yawGimbal = parseFloat(document.getElementById('gimbalYaw').value);
        updateUIinputs(); };
    document.getElementById('rcsX').oninput = () => { sim.rcsX = parseFloat(document.getElementById('rcsX').value);
        updateUIinputs(); };
    document.getElementById('rcsZ').oninput = () => { sim.rcsZ = parseFloat(document.getElementById('rcsZ').value);
        updateUIinputs(); };
    document.getElementById('rollTorque').oninput = () => { sim.rollTorque = parseFloat(document.getElementById('rollTorque').value);
        updateUIinputs(); };
    document.getElementById('dryMass').oninput = resetCallback;
    document.getElementById('fuelMass').oninput = resetCallback;
    document.getElementById('rocketLen').oninput = resetCallback;
    document.getElementById('rocketRad').oninput = resetCallback;
    document.getElementById('gravity').oninput = resetCallback;
    
    const toggles = getToggles();
    const toggleMap = {
        toggTraj: 'traj',
        toggGrid: 'grid',
        toggImpact: 'impact',
        toggDrag: 'drag',
        toggForces: 'forces',
        toggVel: 'vel',
        toggAcc: 'acc'
    };
    for (let [id, key] of Object.entries(toggleMap)) {
        let el = document.getElementById(id);
        if (el) {
            el.onclick = () => {
                toggles[key] = !toggles[key];
                el.classList.toggle('on', toggles[key]);
                if (key === 'drag') {
                    // drag state is read directly from toggles in physics (passed as param)
                }
                drawAllCanvases();
            };
        }
    }
}