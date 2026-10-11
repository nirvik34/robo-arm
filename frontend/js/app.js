/**
 * 6-DOF Robotic Arm Web Controller & Real Physics Digital Twin
 * Three.js WebGL 3D Simulation // 50Hz Exact Angle Telemetry // WebSocket
 */

class RoboticArmApp {
  constructor() {
    this.ws = null;
    this.state = {
      cur: [90.0, 90.0, 90.0, 90.0, 90.0, 20.0],
      tgt: [90.0, 90.0, 90.0, 90.0, 90.0, 20.0],
      active: [false, false, false, false, false, false],
      attached: false,
      speed: 70.0,
      dist: -1.0,
      routine: 0,
      connected: false,
      simulated: false,
      physics: null
    };

    this.isDraggingSlider = [false, false, false, false, false, false];
    this.channelMaxAngles = [180.0, 180.0, 180.0, 180.0, 180.0, 180.0];
    this.sliderThrottleTimers = [null, null, null, null, null, null];
    this.sliderLastSentMs = [0, 0, 0, 0, 0, 0];
    this.sliderReleaseGraceMs = [0, 0, 0, 0, 0, 0];
    this.sweepIntervals = {};
    this.videoStream = null;
    this.cameraActive = false;
    this.scanInterval = null;

    // Three.js 3D System
    this.scene = null;
    this.camera = null;
    this.renderer = null;
    this.controls = null;
    this.armJoints = [];
    this.clawJaws = { left: null, right: null };
    this.payloadMesh = null;

    // 3D Motion Trail & Target Gizmo
    this.targetMarker = null;
    this.trailLine = null;
    this.trailPoints = [];
    this.maxTrailPoints = 250;
    this.ikDebounceTimer = null;
    this.lastSolvedIk = null;

    this.init();
  }

  init() {
    this.setupThreeJs();
    this.setupCartesianPlanner();
    this.setupWebSocket();
    this.setupUIListeners();
    this.loadPorts();
    this.loadInventory();
  }

  /* ========================================================================
     THREE.JS 3D ROBOTIC ARM DIGITAL TWIN
     ======================================================================== */
  setupThreeJs() {
    const container = document.getElementById('viewport3dContainer');
    const canvas = document.getElementById('canvas3d');
    if (!container || !canvas || typeof THREE === 'undefined') return;

    const width = container.clientWidth || 600;
    const height = container.clientHeight || 420;

    // 1. Scene & Camera
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x070b16);
    this.scene.fog = new THREE.FogExp2(0x070b16, 0.0015);

    this.camera = new THREE.PerspectiveCamera(45, width / height, 1, 3000);
    this.camera.position.set(380, 280, 380);

    // 2. Renderer
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
    this.renderer.setSize(width, height);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    // 3. OrbitControls
    if (typeof THREE.OrbitControls !== 'undefined') {
      this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
      this.controls.enableDamping = true;
      this.controls.dampingFactor = 0.08;
      this.controls.maxPolarAngle = Math.PI / 2 - 0.02; // Keep above floor
      this.controls.minDistance = 150;
      this.controls.maxDistance = 1200;
      this.controls.target.set(0, 140, 0);
    }

    // 4. Lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.65);
    this.scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0x00f3ff, 1.2);
    dirLight.position.set(200, 450, 250);
    dirLight.castShadow = true;
    dirLight.shadow.mapSize.width = 1024;
    dirLight.shadow.mapSize.height = 1024;
    this.scene.add(dirLight);

    const warmLight = new THREE.DirectionalLight(0xffaa55, 0.6);
    warmLight.position.set(-250, 300, -200);
    this.scene.add(warmLight);

    // 5. Floor Grid
    const grid = new THREE.GridHelper(800, 40, 0x00f3ff, 0x1e2c47);
    grid.position.y = 0;
    this.scene.add(grid);

    // 6. Build Robotic Arm Mesh Hierarchy
    this.buildRobotArmModel();

    // 7. 3D Target Marker Gizmo & Motion Trail Ribbon
    this.setup3dTargetGizmo();
    this.setup3dMotionTrail();

    // 7. Animation Loop
    const animate = () => {
      requestAnimationFrame(animate);
      if (this.controls) this.controls.update();
      this.update3dJointAngles();
      this.renderer.render(this.scene, this.camera);
    };
    animate();

    // Resize Handler
    window.addEventListener('resize', () => {
      const w = container.clientWidth;
      const h = container.clientHeight;
      if (this.camera && this.renderer) {
        this.camera.aspect = w / h;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(w, h);
      }
    });

    // Camera Reset Button
    document.getElementById('resetCamBtn')?.addEventListener('click', () => {
      this.camera.position.set(380, 280, 380);
      if (this.controls) {
        this.controls.target.set(0, 140, 0);
        this.controls.update();
      }
    });

    // Toggle Grid
    let gridVisible = true;
    document.getElementById('toggleAxesBtn')?.addEventListener('click', () => {
      gridVisible = !gridVisible;
      grid.visible = gridVisible;
    });
  }

  buildRobotArmModel() {
    const metalMat = new THREE.MeshStandardMaterial({
      color: 0x1e293b,
      metalness: 0.85,
      roughness: 0.25
    });

    const jointMat = new THREE.MeshStandardMaterial({
      color: 0x00f3ff,
      metalness: 0.9,
      roughness: 0.2,
      emissive: 0x003344
    });

    const linkMat = new THREE.MeshStandardMaterial({
      color: 0x334155,
      metalness: 0.6,
      roughness: 0.4
    });

    const clawMat = new THREE.MeshStandardMaterial({
      color: 0x0284c7,
      metalness: 0.8,
      roughness: 0.3
    });

    // Base Pedestal (Stationary)
    const basePedestalGeo = new THREE.CylinderGeometry(55, 65, 30, 32);
    const basePedestal = new THREE.Mesh(basePedestalGeo, metalMat);
    basePedestal.position.y = 15;
    basePedestal.castShadow = true;
    basePedestal.receiveShadow = true;
    this.scene.add(basePedestal);

    // JOINT 0: Base / Waist Turntable (Rotates around Y)
    const baseTurntable = new THREE.Group();
    baseTurntable.position.y = 30;
    this.scene.add(baseTurntable);

    const turntableMesh = new THREE.Mesh(new THREE.CylinderGeometry(48, 48, 20, 32), jointMat);
    turntableMesh.position.y = 10;
    baseTurntable.add(turntableMesh);

    // Shoulder Brackets
    const bracketGeo = new THREE.BoxGeometry(12, 40, 36);
    const leftBracket = new THREE.Mesh(bracketGeo, metalMat);
    leftBracket.position.set(-22, 35, 0);
    baseTurntable.add(leftBracket);
    const rightBracket = new THREE.Mesh(bracketGeo, metalMat);
    rightBracket.position.set(22, 35, 0);
    baseTurntable.add(rightBracket);

    // JOINT 1: Shoulder (Pitches around X)
    const shoulderJoint = new THREE.Group();
    shoulderJoint.position.set(0, 45, 0);
    baseTurntable.add(shoulderJoint);

    const shoulderPin = new THREE.Mesh(new THREE.CylinderGeometry(14, 14, 46, 24), jointMat);
    shoulderPin.rotation.z = Math.PI / 2;
    shoulderJoint.add(shoulderPin);

    // Link 1: Upper Arm Link (125mm)
    const link1 = new THREE.Group();
    shoulderJoint.add(link1);

    const armMesh = new THREE.Mesh(new THREE.BoxGeometry(22, 125, 26), linkMat);
    armMesh.position.y = 62.5;
    armMesh.castShadow = true;
    link1.add(armMesh);

    // JOINT 2: Elbow (Pitches around X)
    const elbowJoint = new THREE.Group();
    elbowJoint.position.set(0, 125, 0);
    link1.add(elbowJoint);

    const elbowPin = new THREE.Mesh(new THREE.CylinderGeometry(13, 13, 38, 24), jointMat);
    elbowPin.rotation.z = Math.PI / 2;
    elbowJoint.add(elbowPin);

    // Link 2: Forearm Link (125mm)
    const link2 = new THREE.Group();
    elbowJoint.add(link2);

    const forearmMesh = new THREE.Mesh(new THREE.BoxGeometry(18, 125, 22), linkMat);
    forearmMesh.position.y = 62.5;
    forearmMesh.castShadow = true;
    link2.add(forearmMesh);

    // JOINT 3: Wrist Pitch (Pitches around X)
    const wristPitchJoint = new THREE.Group();
    wristPitchJoint.position.set(0, 125, 0);
    link2.add(wristPitchJoint);

    const wristPin = new THREE.Mesh(new THREE.CylinderGeometry(11, 11, 30, 24), jointMat);
    wristPin.rotation.z = Math.PI / 2;
    wristPitchJoint.add(wristPin);

    // JOINT 4: Wrist Roll (Rotates around Y)
    const wristRollJoint = new THREE.Group();
    wristRollJoint.position.set(0, 25, 0);
    wristPitchJoint.add(wristRollJoint);

    const rollCylinder = new THREE.Mesh(new THREE.CylinderGeometry(15, 15, 20, 24), metalMat);
    rollCylinder.position.y = 10;
    wristRollJoint.add(rollCylinder);

    // Gripper Base & Claw Mechanism
    const gripperMount = new THREE.Group();
    gripperMount.position.y = 20;
    wristRollJoint.add(gripperMount);

    const gripperBlock = new THREE.Mesh(new THREE.BoxGeometry(45, 12, 22), metalMat);
    gripperBlock.position.y = 6;
    gripperMount.add(gripperBlock);

    // JOINT 5: Gripper Jaws (Claw)
    const leftJaw = new THREE.Mesh(new THREE.BoxGeometry(6, 42, 10), clawMat);
    leftJaw.position.set(-16, 26, 0);
    leftJaw.castShadow = true;
    gripperMount.add(leftJaw);

    const rightJaw = new THREE.Mesh(new THREE.BoxGeometry(6, 42, 10), clawMat);
    rightJaw.position.set(16, 26, 0);
    rightJaw.castShadow = true;
    gripperMount.add(rightJaw);

    this.clawJaws = { left: leftJaw, right: rightJaw };

    // Payload Box attached to gripper
    const payloadGeo = new THREE.BoxGeometry(24, 24, 24);
    const payloadMat = new THREE.MeshStandardMaterial({
      color: 0xf59e0b,
      metalness: 0.2,
      roughness: 0.6,
      emissive: 0x442200
    });
    this.payloadMesh = new THREE.Mesh(payloadGeo, payloadMat);
    this.payloadMesh.position.set(0, 32, 0);
    this.payloadMesh.castShadow = true;
    gripperMount.add(this.payloadMesh);

    // Store references
    this.armJoints = [
      baseTurntable,    // CH 0 (Yaw around Y)
      shoulderJoint,    // CH 1 (Pitch around X)
      elbowJoint,       // CH 2 (Pitch around X)
      wristPitchJoint,  // CH 3 (Pitch around X)
      wristRollJoint,   // CH 4 (Roll around Y)
      gripperMount      // CH 5 (Gripper Assembly)
    ];
  }

  update3dJointAngles() {
    if (!this.armJoints || this.armJoints.length < 5) return;

    const cur = this.state.cur;

    // CH 0: Base / Waist Yaw
    // 90 is centered. In Three.js: (90 - cur[0]) in radians
    const ang0 = THREE.MathUtils.degToRad(cur[0] - 90);
    this.armJoints[0].rotation.y = -ang0;

    // CH 1: Shoulder Pitch
    // 90 is upright vertical (0 rad).
    const ang1 = THREE.MathUtils.degToRad(cur[1] - 90);
    this.armJoints[1].rotation.x = ang1;

    // CH 2: Elbow Pitch
    // 90 is straight in-line with upper arm.
    const ang2 = THREE.MathUtils.degToRad(cur[2] - 90);
    this.armJoints[2].rotation.x = ang2;

    // CH 3: Wrist Pitch
    const ang3 = THREE.MathUtils.degToRad(cur[3] - 90);
    this.armJoints[3].rotation.x = ang3;

    // CH 4: Wrist Roll
    const ang4 = THREE.MathUtils.degToRad(cur[4] - 90);
    this.armJoints[4].rotation.y = ang4;

    // CH 5: Gripper Claw Jaws
    // cur[5]: 20° is open (spread apart), 80° is tightly clamped
    const clawVal = cur[5];
    const jawSpread = THREE.MathUtils.lerp(18, 6, (clawVal - 20) / 60);
    if (this.clawJaws.left && this.clawJaws.right) {
      this.clawJaws.left.position.x = -jawSpread;
      this.clawJaws.right.position.x = jawSpread;
    }

    // Toggle payload mesh visibility if payload mass > 0
    if (this.payloadMesh) {
      const payloadMass = this.state.physics?.gravity?.payload_mass_g || 40;
      this.payloadMesh.visible = (payloadMass > 5 && clawVal > 35);
    }

    // Dynamic 3D Motion Trail Ribbon
    if (this.trailLine && this.armJoints && this.armJoints[5]) {
      const tcpPos = new THREE.Vector3();
      this.armJoints[5].getWorldPosition(tcpPos);
      tcpPos.y += 32; // Offset to claw TCP

      const len = this.trailPoints.length;
      if (len === 0 || this.trailPoints[len - 1].distanceTo(tcpPos) > 1.8) {
        this.trailPoints.push(tcpPos.clone());
        if (this.trailPoints.length > this.maxTrailPoints) {
          this.trailPoints.shift();
        }
        this.trailLine.geometry.setFromPoints(this.trailPoints);
      }
    }
  }

  /* ========================================================================
     WEBSOCKET & TELEMETRY STREAM
     ======================================================================== */
  setupWebSocket() {
    const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${location.host}/ws`;

    this.updateStatusBadge('connecting', 'Connecting...');

    this.ws = new WebSocket(wsUrl);

    this.ws.onopen = () => {
      this.updateStatusBadge('connected', 'Hardware Online');
      this.appendLog('SYS', 'WebSocket connected to Robotic Arm Server');
    };

    this.ws.onclose = () => {
      this.updateStatusBadge('connecting', 'Reconnecting...');
      setTimeout(() => this.setupWebSocket(), 2000);
    };

    this.ws.onerror = (err) => {
      this.updateStatusBadge('error', 'Connection Error');
    };

    this.ws.onmessage = (event) => {
      try {
        const msg = jsonParse(event.data);
        if (!msg) return;

        if (msg.type === 'telemetry') {
          this.handleTelemetry(msg.data);
        } else if (msg.type === 'log') {
          this.appendLog(msg.direction, msg.text);
        }
      } catch (e) {
        console.error('WS Parse Error:', e);
      }
    };
  }

  handleTelemetry(data) {
    if (!data) return;
    this.state = Object.assign(this.state, data);

    // Update connection status
    if (data.simulated) {
      this.updateStatusBadge('simulated', 'Virtual Physics Sim');
    } else if (data.connected) {
      this.updateStatusBadge('connected', 'Hardware Online');
    } else {
      this.updateStatusBadge('connecting', 'Detecting Board...');
    }

    // PCA9685 Hardware Status Banner
    const hwBanner = document.getElementById('hardwareAlertBanner');
    const bTitle = document.getElementById('bannerTitle');
    const bText = document.getElementById('bannerText');
    if (hwBanner) {
      if (data.pca === true) {
        hwBanner.style.display = 'flex';
        hwBanner.className = 'hardware-banner banner-success';
        if (bTitle) bTitle.textContent = '🟢 PCA9685 Motor Driver Active (I2C 0x40) - Servos Powered & Operational';
        if (bText) bText.textContent = 'Communication with 16-channel PWM driver is healthy. Channels 0 to 5 are active.';
      } else if (data.pca === false && data.connected && !data.simulated) {
        hwBanner.style.display = 'flex';
        hwBanner.className = 'hardware-banner banner-warning';
        if (bTitle) bTitle.textContent = '⚠️ PCA9685 Motor Driver Not Detected on I2C (Address 0x40)';
        if (bText) bText.innerHTML = 'Motors cannot move without PCA9685 power. <strong>Hardware Checklist:</strong><br>1) Connect PCA9685 <strong>VCC</strong> (logic) to Arduino <strong>5V</strong> and <strong>GND</strong> to <strong>GND</strong>.<br>2) Connect <strong>SDA</strong> to <strong>A4</strong> and <strong>SCL</strong> to <strong>A5</strong>.<br>3) Connect <strong>External 5V–6V Power (Battery/Adapter)</strong> to the green <strong>V+ screw terminal</strong> and switch it ON. <em>Servos cannot move on USB power alone!</em>';
      } else {
        hwBanner.style.display = 'none';
      }
    }

    // Ultrasonic Sonar
    if (data.dist !== undefined) {
      const distText = data.dist > 0 ? `${data.dist.toFixed(1)} cm` : '-- cm';
      document.getElementById('sonarDistanceText').textContent = `Sonar: ${distText}`;
      const radarVal = document.getElementById('radarDistanceValue');
      if (radarVal) radarVal.textContent = data.dist > 0 ? data.dist.toFixed(1) : '--';

      const radarBadge = document.getElementById('radarTargetStatus');
      if (radarBadge) {
        if (data.dist > 0 && data.dist <= 30) {
          radarBadge.textContent = 'PAYLOAD DETECTED (< 30cm)';
          radarBadge.className = 'inv-badge badge-ready';
        } else if (data.dist > 30) {
          radarBadge.textContent = 'CLEAR / NO PAYLOAD';
          radarBadge.className = 'inv-badge';
          radarBadge.style.background = 'rgba(255,255,255,0.05)';
        } else {
          radarBadge.textContent = 'CHECKING PAYLOAD...';
          radarBadge.className = 'inv-badge';
        }
      }
    }

    // Auto-stop any running sweeps if arm power is detached
    if (data.attached === false) {
      this.stopAllSweeps();
    }

    // Update 6 Motors
    if (data.cur && data.tgt) {
      for (let i = 0; i < 6; i++) {
        const curAngle = data.cur[i] !== undefined ? data.cur[i] : 90.0;
        const tgtAngle = data.tgt[i] !== undefined ? data.tgt[i] : 90.0;

        // Exact decimal readout
        const valEl = document.getElementById(`val-${i}`);
        if (valEl) valEl.textContent = curAngle.toFixed(2);

        const tgtEl = document.getElementById(`tgt-${i}`);
        if (tgtEl) tgtEl.textContent = tgtAngle.toFixed(2);

        // Pulse width (~544 to ~2400 us)
        const maxA = (this.channelMaxAngles && this.channelMaxAngles[i]) || 180.0;
        const pulseUs = Math.round(544 + (curAngle / maxA) * (2400 - 544));
        const pulseEl = document.getElementById(`pulse-${i}`);
        if (pulseEl) pulseEl.textContent = `~${pulseUs} µs`;

        // Radial SVG gauge
        this.updateGauge(i, curAngle);

        // Slider (only sync if user is not currently dragging it and release grace window has passed)
        const nowMs = performance.now();
        if (!this.isDraggingSlider[i] && nowMs >= this.sliderReleaseGraceMs[i]) {
          const slider = document.getElementById(`slider-${i}`);
          if (slider && document.activeElement !== slider) {
            slider.value = tgtAngle;
          }
        }

        // Power Status Pill
        const pill = document.getElementById(`pill-status-${i}`);
        const pwrBtn = document.getElementById(`toggle-pwr-${i}`);
        const isActive = data.active ? data.active[i] : false;

        // If this channel was relaxed, ensure any sweep on it is halted
        if (!isActive) {
          this.stopSweep(i);
        }

        if (pill) {
          pill.textContent = isActive ? 'ENGAGED' : 'RELAXED';
          pill.className = `ch-status-pill ${isActive ? 'pill-active' : 'pill-relaxed'}`;
        }
        if (pwrBtn) {
          pwrBtn.textContent = isActive ? 'Relax' : 'Power ON';
        }
      }
    }

    // Physics Engine Telemetry
    if (data.physics) {
      this.handlePhysicsTelemetry(data.physics);
    }

    // Routine Stepper
    if (data.routine !== undefined) {
      this.updateRoutineStepper(data.routine);
    }
  }

  handlePhysicsTelemetry(physics) {
    if (!physics) return;

    // Forward Kinematics (End-Effector XYZ)
    if (physics.fk && physics.fk.end_effector) {
      const [x, y, z] = physics.fk.end_effector;
      document.getElementById('fkX').textContent = x.toFixed(2);
      document.getElementById('fkY').textContent = y.toFixed(2);
      document.getElementById('fkZ').textContent = z.toFixed(2);
      document.getElementById('fkReach').textContent = (physics.fk.reach_mm || 0).toFixed(2);
      document.getElementById('fkDist').textContent = (physics.fk.total_dist_mm || 0).toFixed(2);
    }

    // Gravitational Torques
    if (physics.gravity) {
      const g = physics.gravity;
      const torques = g.torques_nm || [0, 0, 0, 0, 0, 0];
      const loads = g.load_pct || [0, 0, 0, 0, 0, 0];
      const droops = g.droop_deg || [0, 0, 0, 0, 0, 0];

      // Shoulder (CH 1)
      const tSh = Math.abs(torques[1]);
      document.getElementById('torqueShoulderVal').textContent = `${tSh.toFixed(2)} N·m (${loads[1]}%)`;
      document.getElementById('torqueBarShoulder').style.width = `${loads[1]}%`;
      document.getElementById('droopShoulderText').textContent = `${droops[1].toFixed(2)}°`;
      document.getElementById('offsetShoulderText').textContent = `+${droops[1].toFixed(2)}°`;

      // Elbow (CH 2)
      const tEl = Math.abs(torques[2]);
      document.getElementById('torqueElbowVal').textContent = `${tEl.toFixed(2)} N·m (${loads[2]}%)`;
      document.getElementById('torqueBarElbow').style.width = `${loads[2]}%`;
      document.getElementById('droopElbowText').textContent = `${droops[2].toFixed(2)}°`;
      document.getElementById('offsetElbowText').textContent = `+${droops[2].toFixed(2)}°`;

      // Wrist (CH 3)
      const tWr = Math.abs(torques[3]);
      document.getElementById('torqueWristVal').textContent = `${tWr.toFixed(2)} N·m (${loads[3]}%)`;
      document.getElementById('torqueBarWrist').style.width = `${loads[3]}%`;
      document.getElementById('droopWristText').textContent = `${droops[3].toFixed(2)}°`;
      document.getElementById('offsetWristText').textContent = `+${droops[3].toFixed(2)}°`;

      // Payload Mass
      const massG = g.payload_mass_g || 40;
      document.getElementById('payloadMassDisplay').textContent = `${massG.toFixed(1)} g (${(massG / 1000).toFixed(3)} kg)`;
      const massSlider = document.getElementById('payloadMassSlider');
      if (massSlider && document.activeElement !== massSlider) {
        massSlider.value = massG;
      }
    }
  }

  updateGauge(channel, angle) {
    const arc = document.getElementById(`arc-${channel}`);
    const needle = document.getElementById(`needle-${channel}`);
    if (!arc || !needle) return;

    // Arc perimeter is ~126 (semi-circle of radius 40)
    const maxA = (this.channelMaxAngles && this.channelMaxAngles[channel]) || 180.0;
    const norm = Math.max(0, Math.min(maxA, angle)) / maxA;
    const offset = 126 * (1.0 - norm);
    arc.style.strokeDashoffset = offset;

    // Needle rotates -90deg at 0° to +90deg at max°
    const needleDeg = -90 + norm * 180;
    needle.style.transform = `rotate(${needleDeg}deg)`;
  }

  updateRoutineStepper(routineStep) {
    const stepperText = document.getElementById('routineStatusText');
    const items = document.querySelectorAll('.step-item');

    items.forEach((item, idx) => {
      item.classList.toggle('active', (idx + 1) === routineStep);
    });

    if (routineStep === 0) {
      if (stepperText) stepperText.textContent = 'Idle / Ready';
    } else {
      if (stepperText) stepperText.textContent = `Executing Step ${routineStep} of 8...`;
    }
  }

  updateStatusBadge(type, text) {
    const badge = document.getElementById('statusBadge');
    const statusText = document.getElementById('statusText');
    if (!badge || !statusText) return;

    badge.className = `status-badge badge-${type}`;
    statusText.textContent = text;
  }

  /* ========================================================================
     CARTESIAN WORKSPACE CONTROLLER & JOINT-SPACE MOTION PLANNER (RxInfer)
     ======================================================================== */
  setup3dTargetGizmo() {
    this.targetMarker = new THREE.Group();

    // Outer luminous wireframe sphere
    const sphereMat = new THREE.MeshStandardMaterial({
      color: 0x00f3ff,
      emissive: 0x0088aa,
      emissiveIntensity: 0.6,
      wireframe: true,
      transparent: true,
      opacity: 0.75
    });
    const sphere = new THREE.Mesh(new THREE.SphereGeometry(10, 16, 16), sphereMat);
    this.targetMarker.add(sphere);

    // Inner bright target core
    const coreMat = new THREE.MeshBasicMaterial({ color: 0xffffff });
    const core = new THREE.Mesh(new THREE.SphereGeometry(3.5, 12, 12), coreMat);
    this.targetMarker.add(core);

    // Coordinate locator ring
    const ringGeo = new THREE.RingGeometry(13, 15, 32);
    const ringMat = new THREE.MeshBasicMaterial({ color: 0x00f3ff, side: THREE.DoubleSide, transparent: true, opacity: 0.6 });
    const ring = new THREE.Mesh(ringGeo, ringMat);
    ring.rotation.x = Math.PI / 2;
    this.targetMarker.add(ring);

    // Initial position at Home: (0, 420, 0) in Three.js
    this.targetMarker.position.set(0, 420, 0);
    this.scene.add(this.targetMarker);
  }

  setup3dMotionTrail() {
    this.trailPoints = [];
    const geometry = new THREE.BufferGeometry();
    const material = new THREE.LineBasicMaterial({
      color: 0x00f3ff,
      linewidth: 3,
      transparent: true,
      opacity: 0.85
    });
    this.trailLine = new THREE.Line(geometry, material);
    this.scene.add(this.trailLine);
  }

  clearMotionTrail() {
    this.trailPoints = [];
    if (this.trailLine) {
      this.trailLine.geometry.setFromPoints([]);
    }
    this.showToast('✨ 3D Motion Trail Cleared');
  }

  setupCartesianPlanner() {
    const inputX = document.getElementById('targetCoordX');
    const inputY = document.getElementById('targetCoordY');
    const inputZ = document.getElementById('targetCoordZ');
    const durSlider = document.getElementById('trajDurationSlider');
    const durVal = document.getElementById('trajDurationValue');

    if (durSlider && durVal) {
      durSlider.addEventListener('input', (e) => {
        durVal.textContent = `${parseFloat(e.target.value).toFixed(1)} s`;
      });
    }

    const onCoordInput = () => {
      this.onTargetCoordsChanged();
    };

    inputX?.addEventListener('input', onCoordInput);
    inputY?.addEventListener('input', onCoordInput);
    inputZ?.addEventListener('input', onCoordInput);

    document.getElementById('moveToXyzBtn')?.addEventListener('click', () => {
      this.executeCartesianMove();
    });

    document.getElementById('safeHomeBtn')?.addEventListener('click', () => {
      this.executeSafeHoming();
    });

    document.getElementById('clearTrailBtn')?.addEventListener('click', () => {
      this.clearMotionTrail();
    });

    setTimeout(() => {
      this.onTargetCoordsChanged();
    }, 300);
  }

  adjustTargetCoord(axis, delta) {
    const input = document.getElementById(`targetCoord${axis.toUpperCase()}`);
    if (!input) return;
    const current = parseFloat(input.value) || 0;
    input.value = (current + delta).toFixed(2);
    this.onTargetCoordsChanged();
  }

  setTargetPreset(x, y, z) {
    const inputX = document.getElementById('targetCoordX');
    const inputY = document.getElementById('targetCoordY');
    const inputZ = document.getElementById('targetCoordZ');
    if (inputX) inputX.value = parseFloat(x).toFixed(2);
    if (inputY) inputY.value = parseFloat(y).toFixed(2);
    if (inputZ) inputZ.value = parseFloat(z).toFixed(2);
    this.onTargetCoordsChanged();
    this.showToast(`🎯 Target Set to (${x}, ${y}, ${z}) mm`);
  }

  onTargetCoordsChanged() {
    const inputX = document.getElementById('targetCoordX');
    const inputY = document.getElementById('targetCoordY');
    const inputZ = document.getElementById('targetCoordZ');
    if (!inputX || !inputY || !inputZ) return;

    const x = parseFloat(inputX.value) || 0;
    const y = parseFloat(inputY.value) || 0;
    const z = parseFloat(inputZ.value) || 0;

    // Convert physics (X, Y, Z) to Three.js coordinates: (-Y, Z, X)
    if (this.targetMarker) {
      this.targetMarker.position.set(-y, z, x);
    }

    if (this.ikDebounceTimer) clearTimeout(this.ikDebounceTimer);
    this.ikDebounceTimer = setTimeout(() => {
      this.solveIkLive(x, y, z);
    }, 120);
  }

  async solveIkLive(x, y, z) {
    try {
      const res = await fetch('/api/ik/solve', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ x, y, z })
      });
      if (!res.ok) return;
      const data = await res.json();
      this.lastSolvedIk = data;

      const badge = document.getElementById('ikStatusBadge');
      const errText = document.getElementById('ikPosErrorText');
      const reachText = document.getElementById('ikReachStatusText');
      const coordsText = document.getElementById('ikAchievedCoordsText');

      if (badge) {
        if (data.reachable) {
          badge.className = 'inv-badge badge-ready';
          badge.textContent = `REACHABLE // ±${data.position_error_mm.toFixed(2)} mm`;
        } else {
          badge.className = 'inv-badge badge-picked';
          badge.textContent = `OUT OF BOUNDS // CLAMPED (${data.position_error_mm.toFixed(1)} mm)`;
        }
      }

      if (errText) {
        errText.textContent = `${data.position_error_mm.toFixed(2)} mm (${data.position_error_mm < 1.0 ? 'Sub-millimeter Exact' : 'Boundary Scaled'})`;
        errText.style.color = data.position_error_mm < 1.0 ? 'var(--emerald)' : 'var(--amber)';
      }

      if (reachText) {
        reachText.textContent = data.reachable ? 'Within Physical Workspace' : 'Target Beyond Reach (Scaled)';
        reachText.style.color = data.reachable ? 'var(--cyan)' : 'var(--amber)';
      }

      if (coordsText && data.achieved_coords) {
        const [ax, ay, az] = data.achieved_coords;
        coordsText.textContent = `X: ${ax.toFixed(2)}, Y: ${ay.toFixed(2)}, Z: ${az.toFixed(2)} mm`;
      }

      if (data.solved_angles) {
        for (let i = 0; i < 6; i++) {
          const el = document.getElementById(`ikSolCh${i}`);
          if (el) el.textContent = `${data.solved_angles[i].toFixed(2)}°`;
        }
      }
    } catch (e) {
      console.error('IK solve error:', e);
    }
  }

  async executeCartesianMove() {
    const inputX = document.getElementById('targetCoordX');
    const inputY = document.getElementById('targetCoordY');
    const inputZ = document.getElementById('targetCoordZ');
    const durSlider = document.getElementById('trajDurationSlider');
    if (!inputX || !inputY || !inputZ) return;

    this.stopAllSweeps();

    const x = parseFloat(inputX.value) || 0;
    const y = parseFloat(inputY.value) || 0;
    const z = parseFloat(inputZ.value) || 0;
    const duration = parseFloat(durSlider?.value || 1.5);

    const btn = document.getElementById('moveToXyzBtn');
    const progWrap = document.getElementById('trajProgressWrap');
    const progBar = document.getElementById('trajProgressBar');
    const progPct = document.getElementById('trajProgressPct');
    const phaseLabel = document.getElementById('trajPhaseLabel');

    if (btn) btn.disabled = true;
    if (progWrap) progWrap.style.display = 'block';
    if (phaseLabel) phaseLabel.textContent = `Generating Quintic Minimum-Jerk Trajectory (${duration}s)...`;

    try {
      const res = await fetch('/api/ik/move', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ x, y, z, duration, num_steps: 25 })
      });
      const data = await res.json();
      if (!res.ok) {
        this.showToast(`❌ Move Failed: ${data.detail || 'Error'}`);
        if (progWrap) progWrap.style.display = 'none';
        if (btn) btn.disabled = false;
        return;
      }

      this.showToast(`🚀 Moving to (${x}, ${y}, ${z}) mm via Joint-Space S-Curve`);

      const startTime = performance.now();
      const totalMs = duration * 1000;
      const animInterval = setInterval(() => {
        const elapsed = performance.now() - startTime;
        const pct = Math.min(100, Math.round((elapsed / totalMs) * 100));
        if (progBar) progBar.style.width = `${pct}%`;
        if (progPct) progPct.textContent = `${pct}%`;

        if (pct >= 100) {
          clearInterval(animInterval);
          if (phaseLabel) phaseLabel.textContent = '✅ Trajectory Execution Complete!';
          setTimeout(() => {
            if (progWrap) progWrap.style.display = 'none';
            if (btn) btn.disabled = false;
          }, 600);
        }
      }, 50);

    } catch (e) {
      this.showToast(`❌ Error: ${e.message}`);
      if (progWrap) progWrap.style.display = 'none';
      if (btn) btn.disabled = false;
    }
  }

  async executeSafeHoming() {
    this.stopAllSweeps();

    const btn = document.getElementById('safeHomeBtn');
    const progWrap = document.getElementById('trajProgressWrap');
    const progBar = document.getElementById('trajProgressBar');
    const progPct = document.getElementById('trajProgressPct');
    const phaseLabel = document.getElementById('trajPhaseLabel');

    if (btn) btn.disabled = true;
    if (progWrap) progWrap.style.display = 'block';
    if (phaseLabel) phaseLabel.textContent = '🏠 Planning Collision-Free Lift-First Homing Path...';

    try {
      const res = await fetch('/api/pose/home_safe', { method: 'POST' });
      const data = await res.json();

      this.showToast('🏠 Safe Anti-Collision Homing Initiated!');
      this.setTargetPreset(0, 0, 420);

      const totalMs = 2600;
      const startTime = performance.now();
      const animInterval = setInterval(() => {
        const elapsed = performance.now() - startTime;
        const pct = Math.min(100, Math.round((elapsed / totalMs) * 100));
        if (progBar) progBar.style.width = `${pct}%`;
        if (progPct) progPct.textContent = `${pct}%`;

        if (elapsed < 1200) {
          if (phaseLabel) phaseLabel.textContent = 'Phase 1: Vertical Clearance Lift (Protecting Table)...';
        } else {
          if (phaseLabel) phaseLabel.textContent = 'Phase 2: Slew Base & Lock into Safe Home (0, 0, 420mm)...';
        }

        if (pct >= 100) {
          clearInterval(animInterval);
          if (phaseLabel) phaseLabel.textContent = '✅ Arm Locked at Home Standby: (0, 0, 420) mm';
          setTimeout(() => {
            if (progWrap) progWrap.style.display = 'none';
            if (btn) btn.disabled = false;
          }, 800);
        }
      }, 50);

    } catch (e) {
      this.showToast(`❌ Error: ${e.message}`);
      if (progWrap) progWrap.style.display = 'none';
      if (btn) btn.disabled = false;
    }
  }

  /* ========================================================================
     UI LISTENERS & CONTROL INTERACTIONS
     ======================================================================== */
  setupUIListeners() {
    // 1. Motor Sliders (Channels 0 to 5)
    for (let ch = 0; ch < 6; ch++) {
      const slider = document.getElementById(`slider-${ch}`);
      if (!slider) continue;

      const sendThrottled = (val, immediate = false) => {
        const now = performance.now();
        if (immediate || (now - this.sliderLastSentMs[ch] >= 35)) {
          this.sliderLastSentMs[ch] = now;
          if (this.sliderThrottleTimers[ch]) {
            clearTimeout(this.sliderThrottleTimers[ch]);
            this.sliderThrottleTimers[ch] = null;
          }
          this.setChannelAngle(ch, val);
        } else {
          if (!this.sliderThrottleTimers[ch]) {
            this.sliderThrottleTimers[ch] = setTimeout(() => {
              this.sliderThrottleTimers[ch] = null;
              this.sliderLastSentMs[ch] = performance.now();
              const latestVal = parseFloat(slider.value);
              this.setChannelAngle(ch, latestVal);
            }, 35);
          }
        }
      };

      slider.addEventListener('pointerdown', () => {
        this.isDraggingSlider[ch] = true;
      });

      slider.addEventListener('input', (e) => {
        const val = parseFloat(e.target.value);
        sendThrottled(val, false);
      });

      // 'change' event fires on release: always lock exact final degree immediately!
      slider.addEventListener('change', (e) => {
        const val = parseFloat(e.target.value);
        if (this.sliderThrottleTimers[ch]) {
          clearTimeout(this.sliderThrottleTimers[ch]);
          this.sliderThrottleTimers[ch] = null;
        }
        sendThrottled(val, true);
      });

      const onPointerRelease = () => {
        if (this.isDraggingSlider[ch]) {
          this.isDraggingSlider[ch] = false;
          this.sliderReleaseGraceMs[ch] = performance.now() + 250;
          if (this.sliderThrottleTimers[ch]) {
            clearTimeout(this.sliderThrottleTimers[ch]);
            this.sliderThrottleTimers[ch] = null;
          }
          sendThrottled(parseFloat(slider.value), true);
        }
      };

      window.addEventListener('pointerup', onPointerRelease);
      window.addEventListener('touchend', onPointerRelease);
      slider.addEventListener('mouseup', onPointerRelease);

      // Power Toggle Button
      const pwrBtn = document.getElementById(`toggle-pwr-${ch}`);
      if (pwrBtn) {
        pwrBtn.addEventListener('click', () => {
          const isActive = this.state.active ? this.state.active[ch] : false;
          if (isActive) {
            this.stopSweep(ch);
            this.sendPost(`/api/relax/${ch}`);
            this.showToast(`🛑 CH ${ch} Relaxed (PWM off)`);
          } else {
            this.sendPost('/api/set_servo', { channel: ch, angle: this.state.cur[ch] });
            this.showToast(`⚡ CH ${ch} Powered & Engaged`);
          }
        });
      }
    }

    // 2. Hardcoded Action Buttons
    document.getElementById('hardcodedPickDropBtn')?.addEventListener('click', () => {
      this.stopAllSweeps();
      this.runRoutine(false);
    });

    document.getElementById('hardcodedReverseBtn')?.addEventListener('click', () => {
      this.stopAllSweeps();
      this.runRoutine(true);
    });

    document.getElementById('haltRoutineBtn')?.addEventListener('click', () => {
      this.stopAllSweeps();
      this.sendPost('/api/routine/halt');
      this.showToast('⏹ Routine & Sweeps Halted Immediately');
    });

    document.getElementById('poseStandbyBtn')?.addEventListener('click', () => {
      this.stopAllSweeps();
      this.executeSafeHoming();
    });

    document.getElementById('posePickBtn')?.addEventListener('click', () => {
      this.stopAllSweeps();
      this.sendPost('/api/pose/PICK');
      this.showToast('📦 Moving to Pick Pose');
    });

    document.getElementById('poseLiftBtn')?.addEventListener('click', () => {
      this.stopAllSweeps();
      this.sendPost('/api/pose/LIFT');
      this.showToast('🏗️ Lifting with Gravity Compensation');
    });

    document.getElementById('poseDropBtn')?.addEventListener('click', () => {
      this.stopAllSweeps();
      this.sendPost('/api/pose/DROP');
      this.showToast('📥 Moving to Drop Pose');
    });

    document.getElementById('toggleClawBtn')?.addEventListener('click', () => {
      this.stopSweep(5);
      this.sendPost('/api/pose/CLAW');
    });

    // 3. Attach / Relax All
    document.getElementById('attachAllBtn')?.addEventListener('click', () => {
      this.sendPost('/api/attach');
      this.showToast('⚡ All Servos Engaged & Powered');
    });

    document.getElementById('relaxAllBtn')?.addEventListener('click', () => {
      this.stopAllSweeps();
      this.sendPost('/api/detach');
      this.showToast('🛑 All Servos Relaxed & Sweeps Stopped');
    });

    // 4. Master Speed Slider
    const speedSlider = document.getElementById('masterSpeedSlider');
    if (speedSlider) {
      speedSlider.addEventListener('input', (e) => {
        const val = parseFloat(e.target.value);
        document.getElementById('masterSpeedValue').textContent = `${val}°/s`;
        this.sendPost('/api/speed', { speed: val });
      });
    }

    // 5. Physics Suite: Payload Mass & Gravity Comp
    const massSlider = document.getElementById('payloadMassSlider');
    if (massSlider) {
      massSlider.addEventListener('input', (e) => {
        const g = parseFloat(e.target.value);
        this.sendPost('/api/physics/config', { payload_mass_kg: g / 1000.0 });
      });
    }

    const compToggle = document.getElementById('gravityCompToggle');
    if (compToggle) {
      compToggle.addEventListener('change', (e) => {
        const enabled = e.target.checked;
        const txt = document.getElementById('gravityCompStatusText');
        if (txt) {
          txt.textContent = enabled ? 'COMPENSATION ON' : 'COMPENSATION OFF';
          txt.style.color = enabled ? 'var(--emerald)' : 'var(--red)';
        }
        this.sendPost('/api/physics/config', { compensation_enabled: enabled });
      });
    }

    // 6. Terminal Console
    document.getElementById('sendRawCmdBtn')?.addEventListener('click', () => {
      const input = document.getElementById('rawCmdInput');
      if (input && input.value.trim()) {
        this.sendRaw(input.value.trim());
        input.value = '';
      }
    });

    document.getElementById('rawCmdInput')?.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') {
        const input = document.getElementById('rawCmdInput');
        if (input && input.value.trim()) {
          this.sendRaw(input.value.trim());
          input.value = '';
        }
      }
    });

    document.getElementById('clearLogBtn')?.addEventListener('click', () => {
      const term = document.getElementById('serialTerminal');
      if (term) term.innerHTML = '';
    });

    // 7. Ports & Sim Mode
    document.getElementById('refreshPortsBtn')?.addEventListener('click', () => {
      this.loadPorts();
    });

    document.getElementById('connectPortBtn')?.addEventListener('click', () => {
      const port = document.getElementById('portSelect')?.value;
      if (port) {
        this.sendPost('/api/connect', { port, baudrate: 115200 });
      }
    });

    document.getElementById('disconnectPortBtn')?.addEventListener('click', () => {
      this.sendPost('/api/disconnect');
    });

    document.getElementById('simModeBtn')?.addEventListener('click', () => {
      const isSim = this.state.simulated;
      this.sendPost(`/api/simulation?enable=${!isSim}`);
    });

    document.getElementById('scanI2cBtn')?.addEventListener('click', () => {
      this.sendRaw('I2CSCAN');
      this.showToast('🔍 Scanning I2C Bus for PCA9685...');
    });

    // 8. Flash Firmware Modal
    document.getElementById('openFlashModalBtn')?.addEventListener('click', () => {
      const modal = document.getElementById('flashModal');
      if (modal) modal.style.display = 'flex';
    });

    document.getElementById('closeFlashModalBtn')?.addEventListener('click', () => {
      const modal = document.getElementById('flashModal');
      if (modal) modal.style.display = 'none';
    });

    document.getElementById('startFlashBtn')?.addEventListener('click', () => {
      this.flashFirmware();
    });

    // 9. QR Scanner Camera & Presets
    document.getElementById('toggleCameraBtn')?.addEventListener('click', () => {
      this.toggleCamera();
    });

    document.getElementById('openQrCodesModalBtn')?.addEventListener('click', () => {
      this.openQrCodesModal();
    });

    document.getElementById('closeQrCodesModalBtn')?.addEventListener('click', () => {
      const modal = document.getElementById('qrCodesModal');
      if (modal) modal.style.display = 'none';
    });

    document.getElementById('submitManualQrBtn')?.addEventListener('click', () => {
      const input = document.getElementById('manualQrInput');
      if (input && input.value.trim()) {
        this.simulateScan(input.value.trim());
        input.value = '';
      }
    });
  }

  /* ========================================================================
     SERVO & ROUTINE EXECUTION HELPERS
     ======================================================================== */
  setChannelAngle(channel, angle, isSweep = false) {
    if (!isSweep) {
      this.stopSweep(channel);
    }
    const maxA = (this.channelMaxAngles && this.channelMaxAngles[channel]) || 180.0;
    const ang = Math.max(0.0, Math.min(maxA, parseFloat(angle)));

    // Fast local UI update (zero perceived lag)
    const valEl = document.getElementById(`val-${channel}`);
    if (valEl) valEl.textContent = ang.toFixed(2);
    const tgtEl = document.getElementById(`tgt-${channel}`);
    if (tgtEl) tgtEl.textContent = ang.toFixed(2);
    this.updateGauge(channel, ang);
    if (this.state && this.state.cur) {
      this.state.cur[channel] = ang;
      this.state.tgt[channel] = ang;
    }

    // Use lightning-fast WebSocket if connected, fallback to REST
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify({ action: 'set_servo', channel, angle: ang }));
    } else {
      this.sendPost('/api/set_servo', { channel, angle: ang });
    }
  }

  stepChannel(channel, delta) {
    this.stopSweep(channel);
    const slider = document.getElementById(`slider-${channel}`);
    const currentVal = slider ? parseFloat(slider.value) : (this.state.cur[channel] || 90.0);
    const maxA = (this.channelMaxAngles && this.channelMaxAngles[channel]) || 180.0;
    const nextVal = Math.max(0.0, Math.min(maxA, Math.round((currentVal + delta) * 10) / 10));
    if (slider) slider.value = nextVal;
    this.setChannelAngle(channel, nextVal);
  }

  holdChannel(channel) {
    this.stopSweep(channel);
    const cur = this.state.cur[channel] !== undefined ? this.state.cur[channel] : 90.0;
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify({ action: 'hold', channel }));
    } else {
      this.sendPost(`/api/hold/${channel}`);
    }
    this.showToast(`🛑 CH ${channel} Locked & Holding at ${cur.toFixed(2)}°`);
  }

  setChannelRange(channel, maxAngle) {
    const maxA = parseFloat(maxAngle) || 180.0;
    this.channelMaxAngles[channel] = maxA;
    const slider = document.getElementById(`slider-${channel}`);
    if (slider) {
      slider.max = maxA;
    }
    const cur = this.state.cur[channel] !== undefined ? this.state.cur[channel] : 90.0;
    this.updateGauge(channel, cur);
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify({ action: 'range', channel, maxAngle: maxA }));
    } else {
      this.sendPost('/api/range', { channel, maxAngle: maxA });
    }
    this.showToast(`📐 CH ${channel} Range set to 0° – ${maxA}°`);
  }

  isolateChannel(channel) {
    // Crucial: Stop ALL sweeps across all channels immediately
    this.stopAllSweeps();
    this.sendPost(`/api/isolate/${channel}`);
    this.showToast(`⚡ Isolated CH ${channel}: Only this motor energized (All sweeps halted)`);
  }

  stopSweep(channel) {
    if (this.sweepIntervals && this.sweepIntervals[channel]) {
      clearInterval(this.sweepIntervals[channel]);
      delete this.sweepIntervals[channel];
      const btn = document.getElementById(`sweep-btn-${channel}`);
      if (btn) {
        btn.textContent = '▶ Sweep';
        btn.className = 'btn btn-xs btn-primary';
      }
      return true;
    }
    return false;
  }

  stopAllSweeps() {
    let count = 0;
    for (let i = 0; i < 6; i++) {
      if (this.stopSweep(i)) {
        count++;
      }
    }
    return count;
  }

  testSweepChannel(channel) {
    // If currently sweeping this channel, clicking button immediately toggles it OFF!
    if (this.sweepIntervals[channel]) {
      this.stopSweep(channel);
      this.showToast(`⏹ Sweep Stopped on CH ${channel}`);
      return;
    }

    // Stop any other active sweeps first for mechanical safety
    this.stopAllSweeps();

    // Define safe, realistic sweep limits for each joint
    const limits = [
      { min: 45.0, max: 135.0, name: "Base Yaw" },           // CH 0: Base Yaw
      { min: 55.0, max: 95.0,  name: "Shoulder Pitch" },     // CH 1: Shoulder (safe elevation)
      { min: 60.0, max: 120.0, name: "Elbow Pitch" },        // CH 2: Elbow (safe flexion)
      { min: 70.0, max: 115.0, name: "Wrist Pitch" },        // CH 3: Wrist Pitch
      { min: 45.0, max: 135.0, name: "Wrist Roll" },         // CH 4: Wrist Roll
      { min: 20.0, max: 80.0,  name: "Gripper Claw" }        // CH 5: Gripper (20=Open, 80=Grip)
    ];

    const lim = limits[channel] || { min: 45.0, max: 135.0, name: `CH ${channel}` };
    let goingToMax = true;
    let target = lim.min;

    // Active button UI highlight
    const btn = document.getElementById(`sweep-btn-${channel}`);
    if (btn) {
      btn.textContent = '⏹ Stop';
      btn.className = 'btn btn-xs btn-danger';
    }

    // Immediately power and move to first sweep target
    this.setChannelAngle(channel, target, true);
    this.showToast(`▶ Sweeping CH ${channel} (${lim.min}° ↔ ${lim.max}°) - Click Stop, Isolate or Relax to halt`);

    this.sweepIntervals[channel] = setInterval(() => {
      // Check if channel was deactivated
      if (this.state.attached === false || (this.state.active && !this.state.active[channel])) {
        this.stopSweep(channel);
        return;
      }
      target = goingToMax ? lim.max : lim.min;
      goingToMax = !goingToMax;
      this.setChannelAngle(channel, target, true);
    }, 1800);
  }

  runRoutine(reverse) {
    this.stopAllSweeps();
    this.sendPost('/api/routine/pick_drop', { reverse: !!reverse });
    const name = reverse ? 'Reverse Transfer (Drop -> Pick)' : 'Hardcoded Pick & Drop';
    this.showToast(`▶ Routine Started: ${name}`);
  }

  /* ========================================================================
     QR SCANNER & INVENTORY
     ======================================================================== */
  simulateScan(qrCode) {
    this.appendLog('SYS', `Scanned QR Code: ${qrCode}`);
    this.sendPost('/api/scan_qr', { qr_data: qrCode }).then(res => {
      if (res && res.item) {
        this.showToast(`📦 Item Recognized: ${res.item.name}`);
        this.loadInventory();
      }
    });
  }

  async toggleCamera() {
    const video = document.getElementById('scannerVideo');
    const canvas = document.getElementById('scannerCanvas');
    const laser = document.getElementById('scannerLaser');
    const placeholder = document.getElementById('scannerPlaceholder');
    const btn = document.getElementById('toggleCameraBtn');

    if (this.cameraActive) {
      if (this.videoStream) {
        this.videoStream.getTracks().forEach(t => t.stop());
        this.videoStream = null;
      }
      clearInterval(this.scanInterval);
      video.style.display = 'none';
      laser.style.display = 'none';
      placeholder.style.display = 'block';
      btn.textContent = '📷 Start Camera';
      this.cameraActive = false;
      return;
    }

    try {
      this.videoStream = await navigator.mediaDevices.getUserMedia({ video: { facingMode: 'environment' } });
      video.srcObject = this.videoStream;
      video.setAttribute('playsinline', true);
      await video.play();

      video.style.display = 'block';
      laser.style.display = 'block';
      placeholder.style.display = 'none';
      btn.textContent = '⏹ Stop Camera';
      this.cameraActive = true;

      const ctx = canvas.getContext('2d');
      this.scanInterval = setInterval(() => {
        if (video.readyState === video.HAVE_ENOUGH_DATA && typeof jsQR !== 'undefined') {
          canvas.height = video.videoHeight;
          canvas.width = video.videoWidth;
          ctx.drawImage(video, 0, 0, canvas.width, canvas.height);
          const imgData = ctx.getImageData(0, 0, canvas.width, canvas.height);
          const code = jsQR(imgData.data, imgData.width, imgData.height);
          if (code && code.data) {
            this.simulateScan(code.data);
          }
        }
      }, 250);
    } catch (err) {
      this.showToast(`Camera Error: ${err.message}`);
    }
  }

  openQrCodesModal() {
    const modal = document.getElementById('qrCodesModal');
    const container = document.getElementById('qrCodesContainer');
    if (!modal || !container || typeof QRCode === 'undefined') return;

    container.innerHTML = '';
    const codes = [
      { id: 'INV-BOX-01', name: 'Box #1 (Electronics)' },
      { id: 'INV-BOX-02', name: 'Box #2 (Tools)' },
      { id: 'INV-BOX-03', name: 'Box #3 (Sensors)' },
      { id: 'RETURN-BIN-01', name: 'Return Bin 1 (Reverse)' }
    ];

    codes.forEach(c => {
      const box = document.createElement('div');
      box.style.background = '#090e1a';
      box.style.border = '1px solid #1e2c47';
      box.style.borderRadius = '8px';
      box.style.padding = '12px';
      box.style.textAlign = 'center';

      const title = document.createElement('div');
      title.style.fontSize = '12px';
      title.style.fontWeight = '700';
      title.style.marginBottom = '8px';
      title.textContent = c.name;
      box.appendChild(title);

      const qrDiv = document.createElement('div');
      qrDiv.style.display = 'inline-block';
      qrDiv.style.background = '#fff';
      qrDiv.style.padding = '8px';
      qrDiv.style.borderRadius = '4px';
      box.appendChild(qrDiv);

      new QRCode(qrDiv, {
        text: c.id,
        width: 120,
        height: 120,
        colorDark: '#000000',
        colorLight: '#ffffff',
        correctLevel: QRCode.CorrectLevel.M
      });

      const lbl = document.createElement('div');
      lbl.style.fontSize = '10.5px';
      lbl.style.fontFamily = 'var(--font-mono)';
      lbl.style.color = 'var(--cyan)';
      lbl.style.marginTop = '6px';
      lbl.textContent = c.id;
      box.appendChild(lbl);

      container.appendChild(box);
    });

    modal.style.display = 'flex';
  }

  loadInventory() {
    fetch('/api/inventory')
      .then(res => res.json())
      .then(data => {
        const tbody = document.getElementById('inventoryTableBody');
        if (!tbody || !data.items) return;
        tbody.innerHTML = '';

        data.items.forEach(it => {
          const tr = document.createElement('tr');
          tr.innerHTML = `
            <td><strong style="color: var(--cyan);">${it.id}</strong></td>
            <td>${it.name}</td>
            <td>${it.pick_base}° ➔ ${it.drop_base}°</td>
            <td><span class="inv-badge badge-ready">${it.status}</span></td>
            <td>
              <button class="btn btn-xs btn-outline" onclick="window.app.simulateScan('${it.id}')">
                ▶ Run
              </button>
            </td>
          `;
          tbody.appendChild(tr);
        });
      })
      .catch(console.error);
  }

  /* ========================================================================
     SERIAL & FIRMWARE HELPERS
     ======================================================================== */
  loadPorts() {
    fetch('/api/ports')
      .then(res => res.json())
      .then(data => {
        const select = document.getElementById('portSelect');
        if (!select || !data.ports) return;
        select.innerHTML = '';
        data.ports.forEach(p => {
          const opt = document.createElement('option');
          opt.value = p.port;
          opt.textContent = `${p.port} (${p.description})`;
          if (p.port === data.current) opt.selected = true;
          select.appendChild(opt);
        });
      })
      .catch(console.error);
  }

  flashFirmware() {
    const logBox = document.getElementById('flashLogOutput');
    const pBar = document.getElementById('flashProgress');
    const btn = document.getElementById('startFlashBtn');

    if (pBar) pBar.style.display = 'block';
    if (btn) btn.disabled = true;
    if (logBox) logBox.textContent = 'Initiating compile and flash via arduino-cli...\n';

    fetch('/api/flash', { method: 'POST' })
      .then(res => res.json())
      .then(res => {
        if (pBar) pBar.style.display = 'none';
        if (btn) btn.disabled = false;
        if (res.success) {
          if (logBox) logBox.textContent += '✅ Firmware flashed successfully! Reconnected to Arduino Uno.\n';
          this.showToast('⚡ Firmware Flashed Successfully!');
        } else {
          if (logBox) logBox.textContent += `❌ Flash error (${res.stage}):\n${res.error}\n`;
        }
      })
      .catch(err => {
        if (pBar) pBar.style.display = 'none';
        if (btn) btn.disabled = false;
        if (logBox) logBox.textContent += `❌ Communication error: ${err.message}\n`;
      });
  }

  sendRaw(cmd) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify({ action: 'raw', cmd }));
    } else {
      fetch('/api/move_all', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ raw_cmd: cmd })
      }).catch(console.error);
    }
  }

  sendPost(url, data = {}) {
    return fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(data)
    })
    .then(r => r.json())
    .catch(console.error);
  }

  appendLog(direction, text) {
    const terminal = document.getElementById('serialTerminal');
    if (!terminal) return;

    const line = document.createElement('div');
    const dirClass = direction.toLowerCase();
    line.className = `log-line log-${dirClass}`;
    line.textContent = `[${direction}] ${text}`;

    terminal.appendChild(line);
    terminal.scrollTop = terminal.scrollHeight;

    // Limit log lines to 200
    while (terminal.childNodes.length > 200) {
      terminal.removeChild(terminal.firstChild);
    }
  }

  showToast(message) {
    const container = document.getElementById('toastContainer');
    if (!container) return;

    const toast = document.createElement('div');
    toast.className = 'toast';
    toast.textContent = message;

    container.appendChild(toast);
    setTimeout(() => {
      toast.style.opacity = '0';
      toast.style.transition = 'opacity 0.3s';
      setTimeout(() => toast.remove(), 300);
    }, 2800);
  }
}

function jsonParse(str) {
  try { return JSON.parse(str); } catch (e) { return null; }
}

// Global bootstrap
window.addEventListener('DOMContentLoaded', () => {
  window.app = new RoboticArmApp();
});
