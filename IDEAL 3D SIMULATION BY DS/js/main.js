import { sim, updateMassProperties, physicsStep, quatToEuler } from './physics.js';
import { drawAllCanvases, resizeAllCanvases, getToggles } from './rendering.js';
import { updateUIinputs, updateDataDisplay, bindUIEvents } from './ui.js';

let runFlag = false;
let lastTimestamp = null;

function simulateLoop(now) {
  if (!runFlag) return;
  if (!lastTimestamp) { lastTimestamp = now;
    requestAnimationFrame(simulateLoop); return; }
  let dt = Math.min(0.033, (now - lastTimestamp) / 1000);
  if (dt <= 0) { lastTimestamp = now;
    requestAnimationFrame(simulateLoop); return; }
  lastTimestamp = now;
  for (let i = 0; i < 2; i++) physicsStep(dt / 2, getToggles().drag);
  sim.t += dt;
  updateDataDisplayReal();
  drawAllCanvases();
  requestAnimationFrame(simulateLoop);
}

function updateDataDisplayReal() {
  const e = quatToEuler(sim.q);
  document.getElementById('posX').innerText = sim.x.toFixed(2);
  document.getElementById('posY').innerText = sim.y.toFixed(2);
  document.getElementById('posZ').innerText = sim.z.toFixed(2);
  document.getElementById('velX').innerText = sim.vx.toFixed(2);
  document.getElementById('velY').innerText = sim.vy.toFixed(2);
  document.getElementById('velZ').innerText = sim.vz.toFixed(2);
  document.getElementById('speed').innerText = Math.hypot(sim.vx, sim.vy, sim.vz).toFixed(2);
  document.getElementById('pitchAng').innerText = (e.pitch * 180 / Math.PI).toFixed(1);
  document.getElementById('yawAng').innerText = (e.yaw * 180 / Math.PI).toFixed(1);
  document.getElementById('rollAng').innerText = (e.roll * 180 / Math.PI).toFixed(1);
  document.getElementById('omegaX').innerText = sim.wx.toFixed(3);
  document.getElementById('omegaY').innerText = sim.wy.toFixed(3);
  document.getElementById('omegaZ').innerText = sim.wz.toFixed(3);
  document.getElementById('fuelLeft').innerText = sim.fuelMass.toFixed(1) + ' kg';
  document.getElementById('totalMass').innerText = sim.totalMass.toFixed(1);
  let mdot = sim.thr_main / 3200 + (Math.abs(sim.rcsX) + Math.abs(sim.rcsZ) + Math.abs(sim.rollTorque) / sim.radius) / 2800;
  document.getElementById('mdotTotal').innerText = mdot.toFixed(3);
  document.getElementById('comPos').innerText = sim.comOffset.toFixed(3);
  document.getElementById('fThrust').innerText = Math.hypot(sim.Fx_tot, sim.Fy_tot, sim.Fz_tot).toFixed(0);
  document.getElementById('fGrav').innerText = (sim.totalMass * sim.gravity).toFixed(0);
  document.getElementById('tauPitch').innerText = sim.tau_y.toFixed(1);
  document.getElementById('tauYaw').innerText = sim.tau_z.toFixed(1);
  document.getElementById('tauRoll').innerText = sim.tau_x.toFixed(1);
  document.getElementById('accX').innerText = sim.ax.toFixed(2);
  document.getElementById('accY').innerText = sim.ay.toFixed(2);
  document.getElementById('accZ').innerText = sim.az.toFixed(2);
  let fuelPercent = (sim.fuelMass / (sim.fuelMass + sim.dryMass)) * 100;
  document.getElementById('fuelFillBar').style.width = Math.max(0, fuelPercent) + '%';
  let mins = Math.floor(sim.t / 60),
    secs = Math.floor(sim.t % 60);
  document.getElementById('simClock').innerHTML = `T+${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}.${Math.floor(sim.t * 10) % 10}`;
  document.getElementById('statusDot').className = runFlag ? 'dot active' : 'dot stopped';
  document.getElementById('statusText').innerText = runFlag ? 'RUNNING' : 'STOPPED';
}

function startSim() { if (!runFlag) { runFlag = true;
    lastTimestamp = null;
    requestAnimationFrame(simulateLoop); } }

function stopSim() { runFlag = false; }

function resetSim() {
  stopSim();
  sim.t = 0;
  sim.x = 0;
  sim.y = 0;
  sim.z = 0;
  sim.vx = 0;
  sim.vy = 0;
  sim.vz = 0;
  sim.wx = 0;
  sim.wy = 0;
  sim.wz = 0;
  sim.q = [1, 0, 0, 0];
  sim.dryMass = parseFloat(document.getElementById('dryMass').value);
  sim.fuelMass = parseFloat(document.getElementById('fuelMass').value);
  sim.rocketLen = parseFloat(document.getElementById('rocketLen').value);
  sim.radius = parseFloat(document.getElementById('rocketRad').value);
  sim.gravity = parseFloat(document.getElementById('gravity').value);
  sim.thr_main = 0;
  sim.pitchGimbal = 0;
  sim.yawGimbal = 0;
  sim.rcsX = 0;
  sim.rcsZ = 0;
  sim.rollTorque = 0;
  updateMassProperties();
  sim.trail = [
    [0, 0, 0]
  ];
  document.getElementById('mainThrust').value = 0;
  document.getElementById('gimbalPitch').value = 0;
  document.getElementById('gimbalYaw').value = 0;
  document.getElementById('rcsX').value = 0;
  document.getElementById('rcsZ').value = 0;
  document.getElementById('rollTorque').value = 0;
  updateUIinputs();
  updateDataDisplayReal();
  drawAllCanvases();
}

window.addEventListener('load', () => {
  bindUIEvents(startSim, stopSim, resetSim);
  resetSim();
  resizeAllCanvases();
  window.addEventListener('resize', resizeAllCanvases);
});