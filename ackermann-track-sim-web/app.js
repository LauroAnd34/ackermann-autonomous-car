const canvas = document.getElementById("simCanvas");
const ctx = canvas.getContext("2d");
const cameraCanvas = document.getElementById("cameraCanvas");
const cameraCtx = cameraCanvas.getContext("2d");
const processedCanvas = document.getElementById("processedCanvas");
const processedCtx = processedCanvas.getContext("2d");

const ids = [
  "trackInput", "trackName", "runBtn", "pauseBtn", "resetBtn", "showCenterline",
  "showRays", "showPath", "showSigns", "signEditMode", "signType", "randomizeSignsBtn",
  "clearSignsBtn", "signList", "signReadout", "environmentPreset", "envLight", "envContrast",
  "envNoise", "envShadow", "envLightOut", "envContrastOut", "envNoiseOut", "envShadowOut",
  "trackMode", "wheelbase", "maxSteer", "speed", "kp", "ki", "kd",
  "pwmMin", "pwmCenter", "pwmMax", "wheelbaseOut", "maxSteerOut", "speedOut",
  "kpOut", "kiOut", "kdOut", "steerMetric", "pwmMetric", "speedMetric",
  "controllerMode", "directionMode", "codeEditor", "applyCodeBtn", "restoreCodeBtn", "codeStatus", "visionReadout",
  "errorMetric", "headingMetric", "fpsMetric", "clock", "hz", "log"
];
const ui = Object.fromEntries(ids.map((id) => [id, document.getElementById(id)]));

const trackCanvas = document.createElement("canvas");
trackCanvas.width = canvas.width;
trackCanvas.height = canvas.height;
const trackCtx = trackCanvas.getContext("2d");

let trackPixels = null;
let customController = null;
let lastCameraImage = null;
let whiteThreshold = 185;
const START_POSES = {
  outer_forward: { x: 92, y: 900, heading: -Math.PI / 2 },
  outer_reverse: { x: 827, y: 300, heading: Math.PI / 2 },
  city_forward: { x: 456, y: 1040, heading: -Math.PI / 2 },
  city_reverse: { x: 414, y: 270, heading: Math.PI / 2 }
};
const SIGN_TYPES = {
  proceed_left: { label: "Vire à esquerda", short: "L", color: "#168fd3", decision: "converter para a rua à esquerda" },
  proceed_right: { label: "Vire à direita", short: "R", color: "#168fd3", decision: "converter para a rua à direita" },
  proceed_forward: { label: "Siga em frente", short: "F", color: "#168fd3", decision: "seguir em frente no cruzamento" },
  stop: { label: "Pare", short: "STOP", color: "#ee584f", decision: "parar; destino encontrado" },
  no_entry: { label: "Não entre", short: "NO", color: "#ee584f", decision: "não entrar nessa rua" },
  dead_end: { label: "Rua sem saída", short: "T", color: "#168fd3", decision: "não escolher essa rua" },
  tunnel: { label: "Túnel", short: "TU", color: "#6f7b86", decision: "trecho especial de túnel" },
  bridge: { label: "Ponte", short: "BR", color: "#6f7b86", decision: "trecho especial de ponte" }
};

const DEFAULT_CONTROLLER_CODE = `function control(input, api) {
  // input.rightLineOffsetPx: linha contínua direita em relação ao carro.
  // Valor positivo = linha está à direita. O alvo nominal é 42 px.
  // Quando a tracejada e a contínua direita aparecem, o alvo fica dentro da faixa direita,
  // com margem contra a linha tracejada/contramão.
  const desiredRightLine = input.laneCenterTargetPx ?? 42;
  const cityMode = input.trackMode === "city";
  let steer = 0;

  if (input.maneuver?.active && input.maneuver.phase !== "enter_intersection") {
    steer = input.maneuver.steerBias;
  } else if (input.crossBarrierAhead?.distance < 34 && !input.sign) {
    steer = 0;
  } else if (input.laneSafetyFault) {
    steer = 0.28;
  } else if (input.rightLineOffsetPx !== null) {
    steer = (input.rightLineOffsetPx - desiredRightLine) / (cityMode ? 96 : 76) + input.rightLineSlope * (cityMode ? 0.62 : 1.05);
    if (cityMode && input.confidence > 0.42 && Number.isFinite(input.headingDeg)) {
      const targetRoadHeading = Math.round(input.headingDeg / 90) * 90;
      const roadHeadingError = ((((targetRoadHeading - input.headingDeg) + 180) % 360) + 360) % 360 - 180;
      steer += roadHeadingError / 42;
      if (roadHeadingError < -6) steer = Math.min(steer, roadHeadingError / 52);
      if (roadHeadingError > 6) steer = Math.max(steer, roadHeadingError / 52);
    }
  } else {
    steer = input.lastSteer * 0.97 || input.searchSteer;
  }

  if (input.leftLineOffsetPx !== null && input.leftLineOffsetPx > -50 && (input.rightLineOffsetPx === null || input.rightLineOffsetPx > 44)) {
    steer += 0.34;
  }
  if (input.rightLineOffsetPx !== null && input.centerDashedOffsetPx === null && input.rightLineOffsetPx > 78) {
    steer += 0.16;
  }
  if (input.rightLineOffsetPx !== null && input.centerDashedOffsetPx === null && input.rightLineOffsetPx > 98) {
    steer += 0.26;
  }
  if (input.rightLineOffsetPx !== null && input.rightLineOffsetPx < 36) {
    steer -= 0.22;
  }
  if (input.wrongLaneFault) {
    steer += 0.14;
  }

  const hasLine = input.rightLineOffsetPx !== null;
  if (!hasLine) {
    steer = input.lastSteer * 0.97 || input.searchSteer;
  }
  const stopForSign = input.maneuver?.stop;
  const waitForSign = input.crossBarrierAhead && input.crossBarrierAhead.distance < 34 && !input.sign && !input.maneuver?.active;
  const stopForLaneSafety = input.laneSafetyFault?.severe && !input.wrongLaneFault && !input.maneuver?.active;

  return {
    steer: api.clamp(steer, -1, 1),
    speed: stopForSign || waitForSign || stopForLaneSafety ? 0 : input.laneSafetyFault ? 0.18 : input.maneuver?.active ? 0.22 : !hasLine ? 0.08 : Math.abs(steer) > 0.38 ? 0.26 : input.confidence < 0.62 ? 0.20 : cityMode ? 0.34 : 0.50
  };
}`;

const state = {
  running: false,
  time: 0,
  lastTs: 0,
  fps: 0,
  car: { x: 92, y: 900, heading: -Math.PI / 2, steer: 0 },
  path: [],
  detections: [],
  logLines: [],
  lastError: 0,
  lastSteerCommand: 0,
  currentSpeed: 0,
  confidence: 0,
  lastInput: null,
  signGenerators: [],
  activeSign: null,
  nextSignId: 1,
  handledSigns: new Set(),
  maneuver: null,
  headingHold: null,
  stopUntil: 0
};

function log(message) {
  const t = state.time.toFixed(2).padStart(6, "0");
  state.logLines.unshift(`[${t}s] ${message}`);
  state.logLines = state.logLines.slice(0, 12);
  ui.log.textContent = state.logLines.join("\n");
}

function params() {
  return {
    wheelbase: Number(ui.wheelbase.value),
    maxSteer: Number(ui.maxSteer.value) * Math.PI / 180,
    speed: Number(ui.speed.value),
    pwmMin: Number(ui.pwmMin.value),
    pwmCenter: Number(ui.pwmCenter.value),
    pwmMax: Number(ui.pwmMax.value)
  };
}

function envParams() {
  return {
    light: Number(ui.envLight.value),
    contrast: Number(ui.envContrast.value),
    noise: Number(ui.envNoise.value),
    shadow: Number(ui.envShadow.value)
  };
}

function worldScale() {
  return 100;
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function normalizeDeg(rad) {
  return ((((rad * 180 / Math.PI) + 180) % 360) + 360) % 360 - 180;
}

function normalizeRad(rad) {
  return Math.atan2(Math.sin(rad), Math.cos(rad));
}

function refreshTrackPixels() {
  trackPixels = trackCtx.getImageData(0, 0, trackCanvas.width, trackCanvas.height);
}

function pixelAt(x, y) {
  const px = Math.round(x);
  const py = Math.round(y);
  if (!trackPixels || px < 0 || py < 0 || px >= trackCanvas.width || py >= trackCanvas.height) {
    return [0, 0, 0, 255];
  }
  const i = (py * trackCanvas.width + px) * 4;
  return [trackPixels.data[i], trackPixels.data[i + 1], trackPixels.data[i + 2], 255];
}

function seededNoise(x, y, t = 0) {
  const n = Math.sin(x * 12.9898 + y * 78.233 + Math.floor(t * 12) * 37.719) * 43758.5453;
  return (n - Math.floor(n)) * 2 - 1;
}

function applyEnvironmentToRgb(r, g, b, wx, wy, px = 0, py = 0) {
  const env = envParams();
  const shadowWave = 0.5 + 0.5 * Math.sin((wx + wy * 0.7) * 0.018 + state.time * 0.7);
  const shadow = 1 - env.shadow * shadowWave;
  const vignette = 1 - env.shadow * 0.25 * Math.hypot((px || 160) - 160, (py || 90) - 90) / 185;
  const gain = env.light * shadow * vignette;
  const noise = env.noise * seededNoise(wx + px, wy + py, state.time);
  const adjust = (value) => clamp(((value - 128) * env.contrast + 128) * gain + noise, 0, 255);
  return [adjust(r), adjust(g), adjust(b)];
}

function sampleWhite(x, y) {
  const [r, g, b] = pixelAt(x, y);
  const [er, eg, eb] = applyEnvironmentToRgb(r, g, b, x, y);
  return er > whiteThreshold && eg > whiteThreshold && eb > Math.max(150, whiteThreshold - 15);
}

function loadOfficialTrack() {
  const img = new Image();
  img.onload = () => {
    trackCtx.fillStyle = "#211d1e";
    trackCtx.fillRect(0, 0, trackCanvas.width, trackCanvas.height);
    trackCtx.drawImage(img, 0, 0, trackCanvas.width, trackCanvas.height);
    refreshTrackPixels();
    ui.trackName.textContent = "pista oficial FIRA";
    resetCar();
    log("Pista oficial carregada");
  };
  img.onerror = () => {
    drawDefaultTrack();
    refreshTrackPixels();
    resetCar();
    log("Pista oficial não encontrada; usando pista gerada");
  };
  img.src = "./assets/official_track.png";
}

function drawDefaultTrack() {
  const w = trackCanvas.width;
  const h = trackCanvas.height;
  trackCtx.fillStyle = "#202421";
  trackCtx.fillRect(0, 0, w, h);
  trackCtx.strokeStyle = "#f5f5ef";
  trackCtx.lineCap = "round";
  trackCtx.lineJoin = "round";
  trackCtx.lineWidth = 6;
  trackCtx.beginPath();
  trackCtx.moveTo(130, 900);
  trackCtx.bezierCurveTo(80, 590, 100, 260, 220, 150);
  trackCtx.bezierCurveTo(390, 0, 750, 60, 815, 230);
  trackCtx.bezierCurveTo(910, 480, 850, 930, 660, 1040);
  trackCtx.bezierCurveTo(450, 1165, 185, 1070, 130, 900);
  trackCtx.stroke();
}

function resetCar() {
  const mode = ui.trackMode?.value || "city";
  const direction = ui.directionMode?.value || "forward";
  const pose = START_POSES[`${mode}_${direction}`] || START_POSES.city_forward;
  state.time = 0;
  state.car = { x: pose.x, y: pose.y, heading: pose.heading, steer: 0 };
  state.path = [];
  state.detections = [];
  state.lastError = 0;
  state.lastSteerCommand = 0;
  state.currentSpeed = 0;
  state.confidence = 0;
  state.lastInput = null;
  state.activeSign = null;
  state.handledSigns = new Set();
  state.maneuver = null;
  state.headingHold = null;
  state.stopUntil = 0;
  log(`Reset feito (${mode === "city" ? "cidade/placas" : "pista externa"} - ${direction === "reverse" ? "voltando" : "indo"})`);
}

function localFrame() {
  const c = state.car;
  return frameFromHeading(c.heading);
}

function frameFromHeading(heading) {
  return {
    forward: { x: Math.cos(heading), y: Math.sin(heading) },
    right: { x: -Math.sin(heading), y: Math.cos(heading) }
  };
}

function scanWhiteGroups(base, right, halfWidth) {
  const groups = [];
  let current = null;
  for (let offset = -halfWidth; offset <= halfWidth; offset += 2) {
    const x = base.x + right.x * offset;
    const y = base.y + right.y * offset;
    if (sampleWhite(x, y)) {
      if (!current) current = { start: offset, end: offset, count: 0 };
      current.end = offset;
      current.count += 1;
    } else if (current) {
      groups.push(toGroup(current));
      current = null;
    }
  }
  if (current) groups.push(toGroup(current));
  return groups;
}

function toGroup(raw) {
  return {
    start: raw.start,
    end: raw.end,
    center: (raw.start + raw.end) / 2,
    width: raw.end - raw.start + 2,
    count: raw.count
  };
}

function perceiveRightLine() {
  const c = state.car;
  const { forward, right } = localFrame();
  const scanDistances = [18, 30, 44, 62, 84, 110, 140, 174];
  const expectedRight = clamp(state.lastInput?.rightLineOffsetPx ?? 42, 24, 112);
  const usableRight = [];
  const usableLeft = [];
  const usableDashed = [];
  const detections = [];

  for (const dist of scanDistances) {
    const base = { x: c.x + forward.x * dist, y: c.y + forward.y * dist };
    const groups = scanWhiteGroups(base, right, 165)
      .filter((g) => g.width >= 2 && g.width <= 38);

    if (groups.length >= 7) {
      detections.push({ base, leftHit: null, rightHit: null, centerOffset: null, skipped: true });
      continue;
    }

    const rightLine = groups
      .filter((g) => g.center > 14 && g.center < 145)
      .sort((a, b) => {
        const scoreA = Math.abs(a.center - expectedRight) + Math.max(0, a.width - 16) * 2;
        const scoreB = Math.abs(b.center - expectedRight) + Math.max(0, b.width - 16) * 2;
        return scoreA - scoreB;
      })[0] || null;
    const leftLine = groups
      .filter((g) => g.center < -30)
      .sort((a, b) => Math.abs(a.center + 52) - Math.abs(b.center + 52))[0] || null;
    const dashed = groups
      .filter((g) => Math.abs(g.center) <= 42)
      .sort((a, b) => Math.abs(a.center) - Math.abs(b.center))[0] || null;

    if (rightLine) usableRight.push({ offset: rightLine.center, dist, weight: clamp(2.2 - dist / 125, 0.45, 2.0) });
    if (leftLine) usableLeft.push({ offset: leftLine.center, dist, weight: clamp(1.6 - dist / 175, 0.45, 1.45) });
    if (dashed) usableDashed.push({ offset: dashed.center, dist, weight: 1 });

    detections.push({
      base,
      leftHit: leftLine ? hitAt(base, right, leftLine.center) : null,
      rightHit: rightLine ? hitAt(base, right, rightLine.center) : null,
      centerOffset: rightLine ? rightLine.center - 64 : dashed?.center ?? null
    });
  }

  const rightLineOffsetPx = stabilizeOffset(weightedOffset(usableRight), state.lastInput?.rightLineOffsetPx, usableRight.length);
  const leftLineOffsetPx = weightedOffset(usableLeft);
  const centerDashedOffsetPx = stabilizeOffset(weightedOffset(usableDashed), state.lastInput?.centerDashedOffsetPx, usableDashed.length);
  const rightLineSlope = lineSlope(usableRight);
  const laneCenterTargetPx = laneTarget(rightLineOffsetPx, centerDashedOffsetPx);
  const observedSign = getObservedSign();
  const actionableSign = getVisibleSign();
  updateTrafficState(actionableSign);
  const confidence = clamp(usableRight.length * 0.18, 0, 1);
  const crossBarrierAhead = detectCrossBarrierAhead(forward, right);
  const laneSafetyFault = detectLaneSafetyFault(rightLineOffsetPx, centerDashedOffsetPx, laneCenterTargetPx, confidence);
  const wrongLaneFault = detectWrongLaneFault(centerDashedOffsetPx, confidence);
  const input = {
    rightLineOffsetPx,
    leftLineOffsetPx,
    centerDashedOffsetPx,
    rightLineSlope,
    laneCenterTargetPx,
    rightLineSamples: usableRight.length,
    leftLineSamples: usableLeft.length,
    dashedSamples: usableDashed.length,
    confidence,
    sign: observedSign,
    actionableSign,
    crossBarrierAhead,
    laneSafetyFault,
    wrongLaneFault,
    maneuver: currentManeuverInput(),
    searchSteer: ui.directionMode.value === "reverse" ? -0.18 : 0.18,
    lastSteer: state.lastSteerCommand,
    speedSetting: params().speed,
    time: state.time,
    headingDeg: normalizeDeg(state.car.heading),
    trackMode: ui.trackMode.value
  };

  state.detections = detections;
  state.confidence = confidence;
  state.activeSign = observedSign;
  state.lastInput = input;
  state.lastError = rightLineOffsetPx === null ? 0 : (rightLineOffsetPx - (laneCenterTargetPx ?? 42)) / worldScale();
  return input;
}

function hitAt(base, right, offset) {
  return { x: base.x + right.x * offset, y: base.y + right.y * offset, offset };
}

function detectCrossBarrierAhead(forward, right) {
  if (ui.trackMode.value !== "city" || state.maneuver || state.stopUntil > state.time) return null;
  for (const dist of [30, 44, 60, 78]) {
    const base = { x: state.car.x + forward.x * dist, y: state.car.y + forward.y * dist };
    const groups = scanWhiteGroups(base, right, 92);
    const barrier = groups.find((g) => g.width >= 48 && Math.abs(g.center) < 34);
    if (barrier) {
      return {
        distance: dist,
        center: Number(barrier.center.toFixed(1)),
        width: Number(barrier.width.toFixed(1))
      };
    }
  }
  return null;
}

function detectLaneSafetyFault(rightOffset, dashedOffset, targetOffset, confidence) {
  if (ui.trackMode.value !== "city" || state.maneuver || state.stopUntil > state.time) return null;
  if (state.handledSigns.size === 0) return null;
  if (rightOffset === null || confidence < 0.54) return null;
  const target = targetOffset ?? 42;
  const error = rightOffset - target;
  const dashGap = dashedOffset === null ? null : Math.abs(dashedOffset);
  if (error > 52 || (error > 42 && dashGap !== null && dashGap < 18)) {
    return {
      error: Number(error.toFixed(1)),
      right: Number(rightOffset.toFixed(1)),
      target: Number(target.toFixed(1)),
      dash: dashedOffset === null ? null : Number(dashedOffset.toFixed(1))
    };
  }
  return null;
}

function detectWrongLaneFault(dashedOffset, confidence) {
  if (ui.trackMode.value !== "city" || state.maneuver || state.stopUntil > state.time) return null;
  if (state.handledSigns.size === 0 || dashedOffset === null || confidence < 0.24) return null;
  if (dashedOffset > 6) {
    return {
      dash: Number(dashedOffset.toFixed(1)),
      expected: "tracejada à esquerda da câmera"
    };
  }
  return null;
}

function weightedOffset(items) {
  if (!items.length) return null;
  const total = items.reduce((sum, item) => sum + item.weight, 0);
  return items.reduce((sum, item) => sum + item.offset * item.weight, 0) / total;
}

function stabilizeOffset(raw, previous, sampleCount) {
  if (raw === null || raw === undefined) return null;
  if (previous === null || previous === undefined || !Number.isFinite(previous)) return raw;
  const maxStep = sampleCount >= 5 ? 28 : sampleCount >= 3 ? 16 : 10;
  const limited = previous + clamp(raw - previous, -maxStep, maxStep);
  const alpha = sampleCount >= 5 ? 0.72 : sampleCount >= 3 ? 0.52 : 0.32;
  return previous * (1 - alpha) + limited * alpha;
}

function lineSlope(items) {
  if (items.length < 2) return 0;
  const sorted = [...items].sort((a, b) => a.dist - b.dist);
  const near = sorted.slice(0, 3);
  const far = sorted.slice(-3);
  const nearOffset = near.reduce((sum, item) => sum + item.offset, 0) / near.length;
  const farOffset = far.reduce((sum, item) => sum + item.offset, 0) / far.length;
  const distSpan = Math.max(1, far[far.length - 1].dist - near[0].dist);
  return clamp((farOffset - nearOffset) / distSpan, -1, 1);
}

function laneTarget(rightOffset, dashedOffset) {
  if (rightOffset === null || dashedOffset === null) return null;
  const laneWidth = rightOffset - dashedOffset;
  if (laneWidth < 34 || laneWidth > 120) return null;
  return clamp(laneWidth * 0.42, 30, 50);
}

function updateTrafficState(sign) {
  if (state.stopUntil > state.time) return;
  if (state.stopUntil && state.time >= state.stopUntil) {
    state.stopUntil = 0;
    state.maneuver = null;
  }
  if (!sign || state.handledSigns.has(sign.id)) {
    if (state.maneuver && state.time > state.maneuver.until) state.maneuver = null;
    return;
  }
  if (!isActionableSign(sign)) return;

  if (sign.type === "stop") {
    state.stopUntil = state.time + 1.15;
    state.handledSigns.add(sign.id);
    state.maneuver = null;
    log(`STOP atendido no gerador ${sign.id}`);
    return;
  }

  if (sign.type === "proceed_left" || sign.type === "proceed_right" || sign.type === "proceed_forward") {
    const turnDir = sign.type === "proceed_left" ? -1 : sign.type === "proceed_right" ? 1 : 0;
    const duration = sign.type === "proceed_forward" ? 0.8 : 9.8;
    const turnDelay = turnDir ? 0.35 : 0;
    const approach = Number.isFinite(sign.approachHeading) ? sign.approachHeading : state.car.heading;
    const { forward, right } = frameFromHeading(approach);
    const side = turnDir > 0 ? right : { x: -right.x, y: -right.y };
    const exitPoint = turnDir
      ? {
          x: sign.x + forward.x * 82 + side.x * 118,
          y: sign.y + forward.y * 82 + side.y * 118
        }
      : null;
    state.maneuver = {
      type: sign.type,
      signId: sign.id,
      turnDir,
      targetHeading: turnDir ? normalizeRad(approach + turnDir * Math.PI / 2) : state.car.heading,
      exitPoint,
      startedAt: state.time,
      turnDelayUntil: state.time + turnDelay,
      until: state.time + duration + turnDelay
    };
    state.handledSigns.add(sign.id);
    log(`${SIGN_TYPES[sign.type].label} iniciado no gerador ${sign.id}`);
  }
}

function currentManeuverInput() {
  if (state.stopUntil > state.time) {
    return { stop: true, active: false, remaining: Number((state.stopUntil - state.time).toFixed(2)) };
  }
  if (state.headingHold && state.time <= state.headingHold.until) {
    const error = normalizeRad(state.headingHold.heading - state.car.heading);
    const lockFrame = frameFromHeading(state.headingHold.heading);
    const currentLateral = state.car.x * lockFrame.right.x + state.car.y * lockFrame.right.y;
    const lateralError = Number.isFinite(state.headingHold.desiredLateral)
      ? state.headingHold.desiredLateral - currentLateral
      : 0;
    return {
      active: true,
      type: "lane_settle",
      steerBias: clamp(error * 0.95 + lateralError / 70, -0.58, 0.58),
      lateralError: Number(lateralError.toFixed(1)),
      remaining: Number((state.headingHold.until - state.time).toFixed(2)),
      targetHeadingDeg: Number(normalizeDeg(state.headingHold.heading).toFixed(1))
    };
  }
  if (state.headingHold && state.time > state.headingHold.until) state.headingHold = null;
  if (!state.maneuver || state.time > state.maneuver.until) return null;
  if (state.maneuver.turnDir && state.maneuver.turnDelayUntil && state.time < state.maneuver.turnDelayUntil) {
    return {
      active: true,
      type: state.maneuver.type,
      phase: "enter_intersection",
      steerBias: 0,
      progress: 0,
      targetHeadingDeg: Number(normalizeDeg(state.maneuver.targetHeading).toFixed(1)),
      envelope: 0.35,
      remaining: Number((state.maneuver.turnDelayUntil - state.time).toFixed(2))
    };
  }
  const turnStart = state.maneuver.turnDelayUntil ?? state.maneuver.startedAt ?? state.time - 0.01;
  const progress = 1 - ((state.maneuver.until - state.time) / (state.maneuver.until - turnStart));
  const elapsed = state.time - turnStart;
  if (state.maneuver.turnDir && elapsed > 0.85) {
    const remainingHeading = normalizeRad(state.maneuver.targetHeading - state.car.heading);
    const exitDist = state.maneuver.exitPoint
      ? Math.hypot(state.maneuver.exitPoint.x - state.car.x, state.maneuver.exitPoint.y - state.car.y)
      : 0;
    if (Math.abs(remainingHeading) < 0.13 && exitDist < 34) {
      const lockFrame = frameFromHeading(state.maneuver.targetHeading);
      const desiredLateral = state.maneuver.exitPoint
        ? state.maneuver.exitPoint.x * lockFrame.right.x + state.maneuver.exitPoint.y * lockFrame.right.y
        : state.car.x * lockFrame.right.x + state.car.y * lockFrame.right.y;
      state.headingHold = {
        heading: state.maneuver.targetHeading,
        desiredLateral,
        until: state.time + 24
      };
      state.maneuver = null;
      return currentManeuverInput();
    }
  }
  const envelope = Math.sin(clamp(progress, 0, 1) * Math.PI);
  let steerBias = 0;
  if (state.maneuver.turnDir) {
    const exit = state.maneuver.exitPoint;
    const desiredHeading = exit
      ? Math.atan2(exit.y - state.car.y, exit.x - state.car.x)
      : state.maneuver.targetHeading;
    const headingError = normalizeRad(desiredHeading - state.car.heading);
    const finalHeadingError = normalizeRad(state.maneuver.targetHeading - state.car.heading);
    const dist = exit ? Math.hypot(exit.x - state.car.x, exit.y - state.car.y) : 999;
    const blend = dist < 58 ? clamp(1 - dist / 58, 0, 1) : 0;
    const command = normalizeRad(headingError * (1 - blend) + finalHeadingError * blend) / (Math.PI / 2);
    steerBias = clamp(command * 1.35, -0.88, 0.88);
    if (Math.abs(steerBias) < 0.24) steerBias = 0.24 * Math.sign(steerBias || state.maneuver.turnDir);
  }
  return {
    active: true,
    type: state.maneuver.type,
    steerBias: state.maneuver.turnDir ? steerBias : 0,
    progress: Number(clamp(progress, 0, 1).toFixed(2)),
    targetHeadingDeg: Number(normalizeDeg(state.maneuver.targetHeading).toFixed(1)),
    envelope: Number(clamp(envelope, 0.35, 1).toFixed(2)),
    remaining: Number((state.maneuver.until - state.time).toFixed(2))
  };
}

function builtInRightLineController(input, api) {
  const desiredRightLine = input.laneCenterTargetPx ?? 42;
  const cityMode = input.trackMode === "city";
  let steer = 0;
  if (input.maneuver?.active && input.maneuver.phase !== "enter_intersection") {
    steer = input.maneuver.steerBias;
  } else if (input.crossBarrierAhead?.distance < 34 && !input.sign) {
    steer = 0;
  } else if (input.laneSafetyFault) {
    steer = 0.28;
  } else if (input.rightLineOffsetPx !== null) {
    steer = (input.rightLineOffsetPx - desiredRightLine) / (cityMode ? 96 : 76) + input.rightLineSlope * (cityMode ? 0.62 : 1.05);
    if (cityMode && input.confidence > 0.42 && Number.isFinite(input.headingDeg)) {
      const targetRoadHeading = Math.round(input.headingDeg / 90) * 90;
      const roadHeadingError = ((((targetRoadHeading - input.headingDeg) + 180) % 360) + 360) % 360 - 180;
      steer += roadHeadingError / 42;
      if (roadHeadingError < -6) steer = Math.min(steer, roadHeadingError / 52);
      if (roadHeadingError > 6) steer = Math.max(steer, roadHeadingError / 52);
    }
  } else {
    steer = input.lastSteer * 0.97 || input.searchSteer;
  }
  if (input.leftLineOffsetPx !== null && input.leftLineOffsetPx > -50 && (input.rightLineOffsetPx === null || input.rightLineOffsetPx > 44)) steer += 0.34;
  if (input.rightLineOffsetPx !== null && input.centerDashedOffsetPx === null && input.rightLineOffsetPx > 78) steer += 0.16;
  if (input.rightLineOffsetPx !== null && input.centerDashedOffsetPx === null && input.rightLineOffsetPx > 98) steer += 0.26;
  if (input.rightLineOffsetPx !== null && input.rightLineOffsetPx < 36) steer -= 0.22;
  if (input.wrongLaneFault) steer += 0.14;
  const hasLine = input.rightLineOffsetPx !== null;
  if (!hasLine) steer = input.lastSteer * 0.97 || input.searchSteer;
  const stopForSign = input.maneuver?.stop;
  const waitForSign = input.crossBarrierAhead && input.crossBarrierAhead.distance < 34 && !input.sign && !input.maneuver?.active;
  const stopForLaneSafety = input.laneSafetyFault?.severe && !input.wrongLaneFault && !input.maneuver?.active;
  return {
    steer: api.clamp(steer, -1, 1),
    speed: stopForSign || waitForSign || stopForLaneSafety ? 0 : input.laneSafetyFault ? 0.18 : input.maneuver?.active ? 0.22 : !hasLine ? 0.08 : Math.abs(steer) > 0.38 ? 0.26 : input.confidence < 0.62 ? 0.20 : cityMode ? 0.34 : 0.50
  };
}

function applyController(input) {
  const api = { clamp, Math };
  let output;
  try {
    output = ui.controllerMode.value === "customCode" && customController
      ? customController(input, api)
      : builtInRightLineController(input, api);
  } catch (error) {
    ui.codeStatus.textContent = `Erro no controlador: ${error.message}`;
    output = builtInRightLineController(input, api);
  }
  const steerCommand = clamp(Number(output?.steer ?? 0), -1, 1);
  const speedCommand = clamp(Number(output?.speed ?? params().speed), 0, params().speed);
  state.lastSteerCommand = steerCommand;
  return { steerCommand, speedCommand };
}

function update(dt) {
  const p = params();
  const input = perceiveRightLine();
  const control = applyController(input);
  const c = state.car;
  const pxPerMeter = worldScale();
  const v = control.speedCommand * pxPerMeter;
  const targetSteer = control.steerCommand * p.maxSteer;
  state.currentSpeed = control.speedCommand;

  c.steer += (targetSteer - c.steer) * Math.min(1, dt * 8);
  c.heading += (v / (p.wheelbase * pxPerMeter)) * Math.tan(c.steer) * dt;
  c.x += Math.cos(c.heading) * v * dt;
  c.y += Math.sin(c.heading) * v * dt;
  state.path.push({ x: c.x, y: c.y });
  state.path = state.path.slice(-1000);
  state.time += dt;
}

function drawCar() {
  const c = state.car;
  ctx.save();
  ctx.translate(c.x, c.y);
  ctx.rotate(c.heading);
  ctx.fillStyle = "#111315";
  ctx.strokeStyle = "#d7dee2";
  ctx.lineWidth = 2;
  ctx.fillRect(-24, -13, 48, 26);
  ctx.strokeRect(-24, -13, 48, 26);
  ctx.fillStyle = "#f2c230";
  ctx.fillRect(5, -10, 14, 20);
  ctx.strokeStyle = "#24d0c4";
  ctx.lineWidth = 4;
  ctx.beginPath();
  ctx.moveTo(-15, -16);
  ctx.lineTo(-15, -25);
  ctx.moveTo(-15, 16);
  ctx.lineTo(-15, 25);
  ctx.stroke();
  for (const y of [-16, 16]) {
    ctx.save();
    ctx.translate(17, y);
    ctx.rotate(c.steer);
    ctx.beginPath();
    ctx.moveTo(0, -9);
    ctx.lineTo(0, 9);
    ctx.stroke();
    ctx.restore();
  }
  ctx.restore();
}

function servoPwm(steer, p) {
  if (steer >= 0) return p.pwmCenter + (steer / p.maxSteer) * (p.pwmMax - p.pwmCenter);
  return p.pwmCenter + (steer / p.maxSteer) * (p.pwmCenter - p.pwmMin);
}

function renderCamera() {
  const img = cameraCtx.createImageData(cameraCanvas.width, cameraCanvas.height);
  const c = state.car;
  const { forward, right } = localFrame();
  for (let py = 0; py < cameraCanvas.height; py += 1) {
    const v = py / (cameraCanvas.height - 1);
    const forwardDist = 38 + (1 - v) * (1 - v) * 250;
    const halfWidth = 42 + (1 - v) * 138;
    for (let px = 0; px < cameraCanvas.width; px += 1) {
      const u = (px / (cameraCanvas.width - 1) - 0.5) * 2;
      const lateral = u * halfWidth;
      const wx = c.x + forward.x * forwardDist + right.x * lateral;
      const wy = c.y + forward.y * forwardDist + right.y * lateral;
      const [r, g, b] = pixelAt(wx, wy);
      const [er, eg, eb] = applyEnvironmentToRgb(r, g, b, wx, wy, px, py);
      const idx = (py * cameraCanvas.width + px) * 4;
      img.data[idx] = er;
      img.data[idx + 1] = eg;
      img.data[idx + 2] = eb;
      img.data[idx + 3] = 255;
    }
  }
  cameraCtx.putImageData(img, 0, 0);
  lastCameraImage = img;
  cameraCtx.strokeStyle = "rgba(242, 194, 48, 0.9)";
  cameraCtx.lineWidth = 1;
  cameraCtx.beginPath();
  cameraCtx.moveTo(cameraCanvas.width / 2, cameraCanvas.height - 1);
  cameraCtx.lineTo(cameraCanvas.width / 2, 0);
  cameraCtx.stroke();
  renderCameraCorridor();
  renderCameraSigns();
}

function renderCameraCorridor() {
  const prediction = predictTrajectory();
  const alpha = prediction.maneuver?.active ? 0.95 : 0.45;
  drawCameraCorridorLine(prediction.points, -prediction.halfLane, `rgba(36,208,196,${alpha})`);
  drawCameraCorridorLine(prediction.points, prediction.halfLane, `rgba(242,194,48,${alpha})`);
}

function drawCameraCorridorLine(points, lateralOffset, color) {
  const projected = points
    .map((pt) => {
      const rightAtPoint = { x: -Math.sin(pt.heading), y: Math.cos(pt.heading) };
      return projectWorldToCamera(pt.x + rightAtPoint.x * lateralOffset, pt.y + rightAtPoint.y * lateralOffset);
    })
    .filter(Boolean);
  if (projected.length < 2) return;
  cameraCtx.save();
  cameraCtx.strokeStyle = color;
  cameraCtx.lineWidth = 2;
  cameraCtx.lineCap = "round";
  cameraCtx.lineJoin = "round";
  cameraCtx.beginPath();
  projected.forEach((pt, index) => {
    if (index === 0) cameraCtx.moveTo(pt.x, pt.y);
    else cameraCtx.lineTo(pt.x, pt.y);
  });
  cameraCtx.stroke();
  cameraCtx.restore();
}

function projectWorldToCamera(x, y) {
  const c = state.car;
  const { forward, right } = localFrame();
  const dx = x - c.x;
  const dy = y - c.y;
  const forwardDist = dx * forward.x + dy * forward.y;
  const lateral = dx * right.x + dy * right.y;
  return projectToCamera(forwardDist, lateral);
}

function renderCameraSigns() {
  const visible = getVisibleSigns().slice(0, 3);
  for (const sign of visible) {
    const projected = projectToCamera(sign.forward, sign.lateral);
    if (!projected) continue;
    drawSignSymbol(cameraCtx, sign.type, projected.x, projected.y, projected.size, 0);
  }
}

function projectToCamera(forwardDist, lateral) {
  if (forwardDist < 22 || forwardDist > 245) return null;
  const v = 1 - Math.sqrt(clamp((forwardDist - 38) / 250, 0, 1));
  const halfWidth = 42 + (1 - v) * 138;
  if (Math.abs(lateral) > halfWidth) return null;
  return {
    x: cameraCanvas.width / 2 + (lateral / halfWidth) * (cameraCanvas.width / 2),
    y: v * (cameraCanvas.height - 1),
    size: clamp(30 - forwardDist / 12, 10, 24)
  };
}

function renderProcessing() {
  if (!lastCameraImage) return;
  const out = processedCtx.createImageData(processedCanvas.width, processedCanvas.height);
  const data = lastCameraImage.data;
  for (let i = 0; i < data.length; i += 4) {
    const white = data[i] > whiteThreshold && data[i + 1] > whiteThreshold && data[i + 2] > Math.max(150, whiteThreshold - 15);
    out.data[i] = white ? 255 : 0;
    out.data[i + 1] = white ? 255 : 0;
    out.data[i + 2] = white ? 255 : 0;
    out.data[i + 3] = 255;
  }
  processedCtx.putImageData(out, 0, 0);

  processedCtx.strokeStyle = "rgba(242,194,48,0.85)";
  processedCtx.lineWidth = 1;
  for (const y of [132, 118, 102, 86, 70, 54, 38]) {
    processedCtx.beginPath();
    processedCtx.moveTo(0, y);
    processedCtx.lineTo(processedCanvas.width, y);
    processedCtx.stroke();
  }

  processedCtx.strokeStyle = "rgba(36,208,196,0.9)";
  processedCtx.beginPath();
  processedCtx.moveTo(processedCanvas.width / 2, 0);
  processedCtx.lineTo(processedCanvas.width / 2, processedCanvas.height);
  processedCtx.stroke();

  processedCtx.strokeStyle = "rgba(36,208,196,0.45)";
  processedCtx.setLineDash([5, 5]);
  processedCtx.beginPath();
  const targetOffset = state.lastInput?.laneCenterTargetPx ?? 42;
  processedCtx.moveTo(processedCanvas.width / 2 + targetOffset, 0);
  processedCtx.lineTo(processedCanvas.width / 2 + targetOffset, processedCanvas.height);
  processedCtx.stroke();
  processedCtx.setLineDash([]);

  processedCtx.fillStyle = "#24d0c4";
  const input = state.lastInput;
  if (input) {
    drawProcessingPoint(input.rightLineOffsetPx, 116, "#24d0c4");
    drawProcessingPoint(input.leftLineOffsetPx, 142, "#ee584f");
    drawProcessingPoint(input.centerDashedOffsetPx, 90, "#f2c230");
    ui.visionReadout.textContent = [
      `right: ${fmt(input.rightLineOffsetPx)} px (${input.rightLineSamples} amostras)`,
      `left : ${fmt(input.leftLineOffsetPx)} px (${input.leftLineSamples} amostras)`,
      `dash : ${fmt(input.centerDashedOffsetPx)} px (${input.dashedSamples} amostras)`,
      `target faixa dir.: ${fmt(input.laneCenterTargetPx ?? 42)} px`,
      `bloqueio: ${input.crossBarrierAhead ? `${input.crossBarrierAhead.distance}px largura ${input.crossBarrierAhead.width}` : "none"}`,
      `contramao: ${input.wrongLaneFault ? `tracejada à direita ${input.wrongLaneFault.dash}px` : "ok"}`,
      `segurança: ${input.laneSafetyFault ? `fora da faixa erro ${input.laneSafetyFault.error}` : "ok"}`,
      `slope: ${input.rightLineSlope.toFixed(2)}  conf: ${input.confidence.toFixed(2)}`,
      `thresh: ${whiteThreshold}`
    ].join("\n");
  } else {
    ui.visionReadout.textContent = `aguardando frame\nthresh: ${whiteThreshold}`;
  }
}

function drawProcessingPoint(offset, y, color) {
  if (offset === null || offset === undefined) return;
  const x = processedCanvas.width / 2 + offset;
  processedCtx.fillStyle = color;
  processedCtx.beginPath();
  processedCtx.arc(clamp(x, 0, processedCanvas.width), y, 5, 0, Math.PI * 2);
  processedCtx.fill();
}

function fmt(value) {
  return value === null || value === undefined ? "none" : value.toFixed(1);
}

function render() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.drawImage(trackCanvas, 0, 0, canvas.width, canvas.height);

  if (ui.showPath.checked && state.path.length > 1) {
    ctx.strokeStyle = "#42d357";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(state.path[0].x, state.path[0].y);
    for (const pt of state.path) ctx.lineTo(pt.x, pt.y);
    ctx.stroke();
  }

  if (ui.showSigns.checked) renderSignsOnTrack();
  renderPlannedCorridor();

  if (ui.showRays.checked) {
    for (const d of state.detections) {
      ctx.strokeStyle = d.skipped ? "rgba(238,88,79,0.75)" : "rgba(242,194,48,0.85)";
      ctx.lineWidth = 1;
      ctx.beginPath();
      if (d.leftHit) { ctx.moveTo(d.base.x, d.base.y); ctx.lineTo(d.leftHit.x, d.leftHit.y); }
      if (d.rightHit) { ctx.moveTo(d.base.x, d.base.y); ctx.lineTo(d.rightHit.x, d.rightHit.y); }
      ctx.stroke();
    }
  }

  if (ui.showCenterline.checked) {
    ctx.fillStyle = "#24d0c4";
    for (const d of state.detections.filter((v) => v.centerOffset !== null)) {
      ctx.beginPath();
      ctx.arc(d.base.x, d.base.y, 3, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  drawCar();
  renderCamera();
  renderProcessing();
  updateTelemetry();
}

function updateTelemetry() {
  const p = params();
  const pwm = servoPwm(state.car.steer, p);
  const steerText = `${(state.car.steer * 180 / Math.PI).toFixed(1)}°`;
  const speedText = `${state.currentSpeed.toFixed(2)} m/s`;
  const errorText = `${state.lastError.toFixed(3)} m`;
  const headingText = `${normalizeDeg(state.car.heading).toFixed(1)}°`;
  ui.steerMetric.textContent = steerText;
  ui.pwmMetric.textContent = `${pwm.toFixed(0)} µs`;
  ui.speedMetric.textContent = speedText;
  ui.errorMetric.textContent = errorText;
  ui.headingMetric.textContent = headingText;
  document.getElementById("steerMetricMirror").textContent = steerText;
  document.getElementById("speedMetricMirror").textContent = speedText;
  document.getElementById("errorMetricMirror").textContent = errorText;
  document.getElementById("headingMetricMirror").textContent = headingText;
  ui.fpsMetric.textContent = `${state.fps.toFixed(0)}`;
  ui.clock.textContent = `${state.time.toFixed(3)}s`;
  ui.hz.textContent = `${state.fps.toFixed(1)} Hz`;
  updateSignReadout();
  document.body.dataset.simState = JSON.stringify({
    time: Number(state.time.toFixed(3)),
    x: Number(state.car.x.toFixed(2)),
    y: Number(state.car.y.toFixed(2)),
    headingDeg: Number(normalizeDeg(state.car.heading).toFixed(2)),
    steerDeg: Number((state.car.steer * 180 / Math.PI).toFixed(2)),
    speedMps: Number(state.currentSpeed.toFixed(2)),
    rightLineOffsetPx: roundOrNull(state.lastInput?.rightLineOffsetPx),
    leftLineOffsetPx: roundOrNull(state.lastInput?.leftLineOffsetPx),
    centerDashedOffsetPx: roundOrNull(state.lastInput?.centerDashedOffsetPx),
    confidence: Number(state.confidence.toFixed(2)),
    sign: state.activeSign ? { type: state.activeSign.type, forward: roundOrNull(state.activeSign.forward), lateral: roundOrNull(state.activeSign.lateral) } : null,
    crossBarrierAhead: state.lastInput?.crossBarrierAhead || null,
    laneSafetyFault: state.lastInput?.laneSafetyFault || null,
    wrongLaneFault: state.lastInput?.wrongLaneFault || null,
    maneuver: currentManeuverInput(),
    plannedCorridor: plannedCorridorSummary(),
    controllerMode: ui.controllerMode.value,
    trackMode: ui.trackMode.value,
    directionMode: ui.directionMode.value
  });
}

function predictTrajectory() {
  const p = params();
  const maneuver = currentManeuverInput();
  const steerCommand = maneuver?.active ? maneuver.steerBias : state.lastSteerCommand;
  const speed = maneuver?.active ? 0.22 : Math.max(state.currentSpeed, 0.32);
  const pxPerMeter = worldScale();
  const dt = 0.12;
  const horizon = maneuver?.active ? 2.9 : 1.8;
  const pose = { ...state.car };
  const points = [{ x: pose.x, y: pose.y, heading: pose.heading }];

  for (let t = 0; t < horizon; t += dt) {
    const steerRad = clamp(steerCommand, -1, 1) * p.maxSteer;
    const v = speed * pxPerMeter;
    pose.heading += (v / (p.wheelbase * pxPerMeter)) * Math.tan(steerRad) * dt;
    pose.x += Math.cos(pose.heading) * v * dt;
    pose.y += Math.sin(pose.heading) * v * dt;
    points.push({ x: pose.x, y: pose.y, heading: pose.heading });
  }
  return { points, maneuver, halfLane: 28 };
}

function plannedCorridorSummary() {
  const prediction = predictTrajectory();
  const end = prediction.points[prediction.points.length - 1];
  return {
    active: Boolean(prediction.maneuver?.active),
    points: prediction.points.length,
    endX: Number(end.x.toFixed(1)),
    endY: Number(end.y.toFixed(1))
  };
}

function renderPlannedCorridor() {
  const prediction = predictTrajectory();
  if (prediction.points.length < 2) return;
  const alpha = prediction.maneuver?.active ? 0.9 : 0.42;
  drawCorridorLine(ctx, prediction.points, -prediction.halfLane, `rgba(36,208,196,${alpha})`, 3);
  drawCorridorLine(ctx, prediction.points, prediction.halfLane, `rgba(242,194,48,${alpha})`, 3);
}

function drawCorridorLine(targetCtx, points, lateralOffset, color, width) {
  targetCtx.save();
  targetCtx.strokeStyle = color;
  targetCtx.lineWidth = width;
  targetCtx.lineCap = "round";
  targetCtx.lineJoin = "round";
  targetCtx.beginPath();
  points.forEach((pt, index) => {
    const right = { x: -Math.sin(pt.heading), y: Math.cos(pt.heading) };
    const x = pt.x + right.x * lateralOffset;
    const y = pt.y + right.y * lateralOffset;
    if (index === 0) targetCtx.moveTo(x, y);
    else targetCtx.lineTo(x, y);
  });
  targetCtx.stroke();
  targetCtx.restore();
}

function roundOrNull(value) {
  return value === null || value === undefined ? null : Number(value.toFixed(2));
}

function frame(ts) {
  if (!state.lastTs) state.lastTs = ts;
  const rawDt = Math.min(0.04, (ts - state.lastTs) / 1000);
  state.lastTs = ts;
  state.fps = state.fps * 0.9 + (1 / Math.max(rawDt, 0.001)) * 0.1;
  if (state.running) update(rawDt);
  render();
  requestAnimationFrame(frame);
}

function compileCustomCode() {
  const source = ui.codeEditor.value;
  if (looksLikePythonRobotCode(source)) {
    applyPythonRobotProfile(source);
    return;
  }
  try {
    customController = new Function("input", "api", `${source}\nreturn control(input, api);`);
    customController({
      confidence: 0,
      lastSteer: 0,
      rightLineOffsetPx: null,
      leftLineOffsetPx: null,
      centerDashedOffsetPx: null,
      rightLineSlope: 0,
      laneCenterTargetPx: null,
      sign: null,
      maneuver: null,
      searchSteer: ui.directionMode.value === "reverse" ? -0.18 : 0.18,
      trackMode: ui.trackMode.value
    }, { clamp, Math });
    ui.codeStatus.textContent = "Código aplicado.";
    ui.controllerMode.value = "customCode";
  } catch (error) {
    customController = null;
    ui.codeStatus.textContent = `Erro: ${error.message}`;
  }
}

function selectedAllowedSigns() {
  const boxes = [...document.querySelectorAll(".allowedSign")].filter((box) => box.checked);
  const values = boxes.map((box) => box.value);
  return values.length ? values : ["proceed_forward"];
}

function chooseSign(allowed) {
  const forced = ui.signType.value;
  if (forced !== "random" && allowed.includes(forced)) return forced;
  return allowed[Math.floor(Math.random() * allowed.length)] || "proceed_forward";
}

function addSignGenerator(x, y) {
  const allowed = selectedAllowedSigns();
  const generator = {
    id: state.nextSignId++,
    x,
    y,
    allowed,
    approachHeading: state.car.heading,
    type: chooseSign(allowed)
  };
  state.signGenerators.push(generator);
  saveSignGenerators();
  updateSignList();
  log(`Gerador ${generator.id} criado: ${SIGN_TYPES[generator.type].label}`);
}

function randomizeSigns() {
  for (const generator of state.signGenerators) {
    generator.type = chooseSign(generator.allowed);
  }
  saveSignGenerators();
  updateSignList();
  log("Placas sorteadas");
}

function clearSigns() {
  state.signGenerators = [];
  state.activeSign = null;
  saveSignGenerators();
  updateSignList();
  log("Geradores de placa limpos");
}

function getVisibleSigns() {
  const c = state.car;
  const { forward, right } = localFrame();
  return state.signGenerators
    .map((generator) => {
      const dx = generator.x - c.x;
      const dy = generator.y - c.y;
      const f = dx * forward.x + dy * forward.y;
      const l = dx * right.x + dy * right.y;
      const approachError = Number.isFinite(generator.approachHeading)
        ? Math.abs(normalizeRad(c.heading - generator.approachHeading))
        : 0;
      return { ...generator, forward: f, lateral: l, approachError, distance: Math.hypot(dx, dy) };
    })
    .filter((sign) => sign.forward > -18 && sign.forward < 165 && sign.lateral > 10 && sign.lateral < 118)
    .sort((a, b) => a.forward - b.forward);
}

function getVisibleSign() {
  return getVisibleSigns().find((sign) => isActionableSign(sign) && !state.handledSigns.has(sign.id)) || null;
}

function getObservedSign() {
  return getVisibleSigns().find((sign) => !state.handledSigns.has(sign.id)) || null;
}

function isActionableSign(sign) {
  const minForward = sign.type === "stop" ? 8 : 2;
  const maxForward = sign.type === "stop" ? 28 : 18;
  if (sign.type !== "stop") {
    return sign.forward > -18 && sign.forward < 0 && sign.lateral > 8 && sign.lateral < 102 && sign.approachError < 0.85;
  }
  return sign.forward > minForward && sign.forward < maxForward && sign.lateral > 8 && sign.lateral < 102 && sign.approachError < 0.85;
}

function updateSignList() {
  if (!ui.signList) return;
  ui.signList.textContent = state.signGenerators.length
    ? state.signGenerators.map((g) => {
      const allowed = g.allowed.map((type) => SIGN_TYPES[type]?.short || type).join(",");
      const dir = Number.isFinite(g.approachHeading) ? `${normalizeDeg(g.approachHeading).toFixed(0)}°` : "qualquer";
      return `#${g.id} ${SIGN_TYPES[g.type].label}  x=${g.x.toFixed(0)} y=${g.y.toFixed(0)}  sentido=${dir}  permitidas=[${allowed}]`;
    }).join("\n")
    : "sem geradores";
}

function saveSignGenerators() {
  try {
    localStorage.setItem("ackermann.signGenerators", JSON.stringify(state.signGenerators));
    localStorage.setItem("ackermann.nextSignId", String(state.nextSignId));
  } catch (error) {
    log(`Nao foi possivel salvar geradores: ${error.message}`);
  }
}

function loadSignGenerators() {
  try {
    const saved = JSON.parse(localStorage.getItem("ackermann.signGenerators") || "[]");
    if (Array.isArray(saved)) {
      state.signGenerators = saved
        .filter((g) => Number.isFinite(g.x) && Number.isFinite(g.y) && SIGN_TYPES[g.type])
        .map((g) => ({
          id: Number(g.id) || state.nextSignId++,
          x: Number(g.x),
          y: Number(g.y),
          allowed: Array.isArray(g.allowed) && g.allowed.length ? g.allowed.filter((type) => SIGN_TYPES[type]) : ["proceed_forward"],
          approachHeading: Number.isFinite(g.approachHeading) ? Number(g.approachHeading) : null,
          type: SIGN_TYPES[g.type] ? g.type : "proceed_forward"
        }));
    }
    const nextId = Number(localStorage.getItem("ackermann.nextSignId"));
    state.nextSignId = Math.max(nextId || 1, ...state.signGenerators.map((g) => g.id + 1), 1);
  } catch (error) {
    state.signGenerators = [];
    log(`Nao foi possivel carregar geradores: ${error.message}`);
  }
}

function applyScenarioFromUrl() {
  const params = new URLSearchParams(window.location.search);
  const scenario = params.get("scenario");
  if (!scenario) return;
  state.signGenerators = [];
  state.nextSignId = 1;
  if (scenario === "left-sign") {
    state.signGenerators.push({ id: state.nextSignId++, x: 20, y: 780, type: "stop", allowed: ["stop"], approachHeading: -Math.PI / 2 });
  } else if (scenario === "far-stop") {
    state.signGenerators.push({ id: state.nextSignId++, x: 115, y: 610, type: "stop", allowed: ["stop"], approachHeading: -Math.PI / 2 });
  } else if (scenario === "right-stop") {
    state.signGenerators.push({ id: state.nextSignId++, x: 125, y: 820, type: "stop", allowed: ["stop"], approachHeading: -Math.PI / 2 });
  } else if (scenario === "right-left-turn") {
    state.signGenerators.push({ id: state.nextSignId++, x: 125, y: 820, type: "proceed_left", allowed: ["proceed_left"], approachHeading: -Math.PI / 2 });
  } else if (scenario === "city-left-sign") {
    ui.trackMode.value = "city";
    state.signGenerators.push({ id: state.nextSignId++, x: 398, y: 965, type: "stop", allowed: ["stop"], approachHeading: -Math.PI / 2 });
  } else if (scenario === "city-far-stop") {
    ui.trackMode.value = "city";
    state.signGenerators.push({ id: state.nextSignId++, x: 505, y: 770, type: "stop", allowed: ["stop"], approachHeading: -Math.PI / 2 });
  } else if (scenario === "city-right-stop") {
    ui.trackMode.value = "city";
    state.signGenerators.push({ id: state.nextSignId++, x: 505, y: 965, type: "stop", allowed: ["stop"], approachHeading: -Math.PI / 2 });
  } else if (scenario === "city-left-turn") {
    ui.trackMode.value = "city";
    state.signGenerators.push({ id: state.nextSignId++, x: 505, y: 870, type: "proceed_left", allowed: ["proceed_left"], approachHeading: -Math.PI / 2 });
  } else if (scenario === "city-right-turn") {
    ui.trackMode.value = "city";
    state.signGenerators.push({ id: state.nextSignId++, x: 505, y: 870, type: "proceed_right", allowed: ["proceed_right"], approachHeading: -Math.PI / 2 });
  } else if (scenario === "city-reverse-wrong-sign") {
    ui.trackMode.value = "city";
    ui.directionMode.value = "reverse";
    state.signGenerators.push({ id: state.nextSignId++, x: 360, y: 320, type: "proceed_left", allowed: ["proceed_left"], approachHeading: -Math.PI / 2 });
  } else if (scenario === "city-reverse-right-stop") {
    ui.trackMode.value = "city";
    ui.directionMode.value = "reverse";
    state.signGenerators.push({ id: state.nextSignId++, x: 360, y: 320, type: "stop", allowed: ["stop"], approachHeading: Math.PI / 2 });
  }
  updateSignList();
  log(`Cenario carregado: ${scenario}`);
}

function updateSignReadout() {
  const sign = state.activeSign;
  if (!ui.signReadout) return;
  const maneuver = currentManeuverInput();
  if (maneuver?.stop) {
    ui.signReadout.textContent = [
      "STOP em execução",
      `tempo restante: ${maneuver.remaining}s`,
      "placa já marcada como atendida"
    ].join("\n");
    return;
  }
  if (maneuver?.active) {
    if (maneuver.type === "lane_settle") {
      ui.signReadout.textContent = [
        "estabilizando saida da conversao",
        `rumo alvo: ${maneuver.targetHeadingDeg}°`,
        `volante: ${maneuver.steerBias.toFixed(2)}`,
        `tempo restante: ${maneuver.remaining}s`
      ].join("\n");
      return;
    }
    ui.signReadout.textContent = [
      `${SIGN_TYPES[maneuver.type]?.label || maneuver.type}`,
      "manobra em execução",
      `volante: ${maneuver.steerBias.toFixed(2)}`,
      `tempo restante: ${maneuver.remaining}s`
    ].join("\n");
    return;
  }
  if (!sign) {
    if (state.lastInput?.wrongLaneFault) {
      ui.signReadout.textContent = [
        "corrigindo faixa direita",
        "tracejada apareceu à direita da câmera",
        `dash: ${state.lastInput.wrongLaneFault.dash}px`,
        "voltando para a faixa correta"
      ].join("\n");
      return;
    }
    if (state.lastInput?.laneSafetyFault) {
      ui.signReadout.textContent = [
      "corrigindo faixa direita",
      "perto demais da tracejada/contramão",
      `erro: ${state.lastInput.laneSafetyFault.error}px`,
      "recuperando para a direita"
      ].join("\n");
      return;
    }
    if (state.lastInput?.crossBarrierAhead) {
      ui.signReadout.textContent = [
        "aguardando placa",
        "linha continua/bloqueio detectado",
        `distancia: ${state.lastInput.crossBarrierAhead.distance}px`,
        "sem decisao valida: carro parado"
      ].join("\n");
      return;
    }
    ui.signReadout.textContent = "nenhuma placa no campo de visão";
    return;
  }
  const def = SIGN_TYPES[sign.type];
  ui.signReadout.textContent = [
    `${def.label}`,
    `decisão: ${def.decision}`,
    `distância frontal: ${sign.forward.toFixed(1)} px`,
    `lateral: ${sign.lateral.toFixed(1)} px`,
    `gerador #${sign.id}`
  ].join("\n");
}

function renderSignsOnTrack() {
  for (const generator of state.signGenerators) {
    ctx.save();
    ctx.globalAlpha = 0.95;
    ctx.strokeStyle = "#f2c230";
    ctx.lineWidth = 1;
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.arc(generator.x, generator.y, 22, 0, Math.PI * 2);
    ctx.stroke();
    ctx.setLineDash([]);
    drawSignSymbol(ctx, generator.type, generator.x, generator.y, 28, 0);
    ctx.fillStyle = "#f2f4f5";
    ctx.font = "700 11px ui-sans-serif, system-ui";
    ctx.fillText(`#${generator.id}`, generator.x + 18, generator.y - 18);
    ctx.restore();
  }
}

function drawSignSymbol(targetCtx, type, x, y, size, rotation) {
  const def = SIGN_TYPES[type] || SIGN_TYPES.proceed_forward;
  targetCtx.save();
  targetCtx.translate(x, y);
  targetCtx.rotate(rotation);
  targetCtx.lineWidth = Math.max(2, size * 0.09);
  targetCtx.textAlign = "center";
  targetCtx.textBaseline = "middle";
  if (type === "stop") {
    polygon(targetCtx, 8, size * 0.58);
    targetCtx.fillStyle = "#ee584f";
    targetCtx.fill();
    targetCtx.strokeStyle = "#ffffff";
    targetCtx.stroke();
    targetCtx.fillStyle = "#ffffff";
    targetCtx.font = `800 ${Math.max(7, size * 0.24)}px ui-sans-serif, system-ui`;
    targetCtx.fillText("STOP", 0, 0);
  } else if (type === "no_entry") {
    targetCtx.fillStyle = "#ee584f";
    targetCtx.beginPath();
    targetCtx.arc(0, 0, size * 0.5, 0, Math.PI * 2);
    targetCtx.fill();
    targetCtx.strokeStyle = "#ffffff";
    targetCtx.stroke();
    targetCtx.strokeStyle = "#ffffff";
    targetCtx.lineWidth = Math.max(3, size * 0.16);
    targetCtx.beginPath();
    targetCtx.moveTo(-size * 0.32, 0);
    targetCtx.lineTo(size * 0.32, 0);
    targetCtx.stroke();
  } else {
    targetCtx.fillStyle = def.color;
    targetCtx.fillRect(-size * 0.52, -size * 0.38, size * 1.04, size * 0.76);
    targetCtx.strokeStyle = "#dce5e8";
    targetCtx.strokeRect(-size * 0.52, -size * 0.38, size * 1.04, size * 0.76);
    targetCtx.fillStyle = "#ffffff";
    targetCtx.font = `900 ${Math.max(10, size * 0.44)}px ui-sans-serif, system-ui`;
    targetCtx.fillText(def.short, 0, 1);
  }
  targetCtx.restore();
}

function polygon(targetCtx, sides, radius) {
  targetCtx.beginPath();
  for (let i = 0; i < sides; i += 1) {
    const a = -Math.PI / 2 + i * Math.PI * 2 / sides;
    const x = Math.cos(a) * radius;
    const y = Math.sin(a) * radius;
    if (i === 0) targetCtx.moveTo(x, y);
    else targetCtx.lineTo(x, y);
  }
  targetCtx.closePath();
}

function looksLikePythonRobotCode(source) {
  return /\bimport\s+(cv2|RPi|numpy|sys|time)\b/.test(source)
    || /\bclass\s+AutonomousCar\b/.test(source)
    || /\bGPIO\./.test(source)
    || /\bdef\s+detect_lanes\b/.test(source);
}

function applyPythonRobotProfile(source) {
  const getNumber = (name, fallback) => {
    const escaped = name.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
    const match = source.match(new RegExp(`^\\s*${escaped}\\s*=\\s*(-?\\d+(?:\\.\\d+)?)`, "m"));
    return match ? Number(match[1]) : fallback;
  };

  const velBase = getNumber("VEL_BASE", 65);
  const servoMin = getNumber("SERVO_MIN_REAL", -0.5);
  const servoMax = getNumber("SERVO_MAX_REAL", 1.0);
  const servoCenter = getNumber("SERVO_CENTRO", 0.25);
  whiteThreshold = getNumber("LINHA_BRANCA_THRESH", getNumber("white_threshold", 185));

  const kp = getNumber("self.KP", getNumber("KP", null));
  const ki = getNumber("self.KI", getNumber("KI", null));
  const kd = getNumber("self.KD", getNumber("KD", null));
  if (kp !== null) ui.kp.value = clamp(kp, Number(ui.kp.min), Number(ui.kp.max));
  if (ki !== null) ui.ki.value = clamp(ki, Number(ui.ki.min), Number(ui.ki.max));
  if (kd !== null) ui.kd.value = clamp(kd, Number(ui.kd.min), Number(ui.kd.max));

  ui.speed.value = clamp((velBase / 100) * 1.2, Number(ui.speed.min), Number(ui.speed.max)).toFixed(2);
  ui.pwmMin.value = Math.round(1500 + servoMin * 500);
  ui.pwmCenter.value = Math.round(1500 + servoCenter * 500);
  ui.pwmMax.value = Math.round(1500 + servoMax * 500);
  ui.controllerMode.value = "rightLine";
  customController = null;
  syncRangeOutputs();
  resetCar();
  ui.codeStatus.textContent = [
    "Python/RPi importado como perfil de simulação.",
    `threshold=${whiteThreshold}, VEL_BASE=${velBase}, servo=[${servoMin}, ${servoCenter}, ${servoMax}]`
  ].join(" ");
  log("Perfil Python/RPi importado");
}

function bindRanges() {
  const pairs = [
    ["wheelbase", "wheelbaseOut", (v) => `${Number(v).toFixed(2)} m`],
    ["maxSteer", "maxSteerOut", (v) => `${v}°`],
    ["speed", "speedOut", (v) => `${Number(v).toFixed(2)} m/s`],
    ["kp", "kpOut", (v) => Number(v).toFixed(2)],
    ["ki", "kiOut", (v) => Number(v).toFixed(2)],
    ["kd", "kdOut", (v) => Number(v).toFixed(2)],
    ["envLight", "envLightOut", (v) => Number(v).toFixed(2)],
    ["envContrast", "envContrastOut", (v) => Number(v).toFixed(2)],
    ["envNoise", "envNoiseOut", (v) => Number(v).toFixed(0)],
    ["envShadow", "envShadowOut", (v) => Number(v).toFixed(2)]
  ];
  for (const [input, output, fmt] of pairs) {
    const sync = () => { ui[output].textContent = fmt(ui[input].value); };
    ui[input].addEventListener("input", sync);
  }
  syncRangeOutputs();
}

function syncRangeOutputs() {
  const pairs = [
    ["wheelbase", "wheelbaseOut", (v) => `${Number(v).toFixed(2)} m`],
    ["maxSteer", "maxSteerOut", (v) => `${v}°`],
    ["speed", "speedOut", (v) => `${Number(v).toFixed(2)} m/s`],
    ["kp", "kpOut", (v) => Number(v).toFixed(2)],
    ["ki", "kiOut", (v) => Number(v).toFixed(2)],
    ["kd", "kdOut", (v) => Number(v).toFixed(2)],
    ["envLight", "envLightOut", (v) => Number(v).toFixed(2)],
    ["envContrast", "envContrastOut", (v) => Number(v).toFixed(2)],
    ["envNoise", "envNoiseOut", (v) => Number(v).toFixed(0)],
    ["envShadow", "envShadowOut", (v) => Number(v).toFixed(2)]
  ];
  for (const [input, output, fmt] of pairs) {
    ui[output].textContent = fmt(ui[input].value);
  }
}

function applyEnvironmentPreset() {
  const presets = {
    clean: { light: 1, contrast: 1, noise: 0, shadow: 0 },
    sun: { light: 1.28, contrast: 1.28, noise: 8, shadow: 0.22 },
    indoor: { light: 0.86, contrast: 1.12, noise: 12, shadow: 0.12 },
    dark: { light: 0.48, contrast: 0.82, noise: 24, shadow: 0.35 },
    noisy: { light: 0.88, contrast: 1.05, noise: 52, shadow: 0.18 }
  };
  const preset = presets[ui.environmentPreset.value] || presets.clean;
  ui.envLight.value = preset.light;
  ui.envContrast.value = preset.contrast;
  ui.envNoise.value = preset.noise;
  ui.envShadow.value = preset.shadow;
  syncRangeOutputs();
  log(`Ambiente: ${ui.environmentPreset.options[ui.environmentPreset.selectedIndex].text}`);
}

ui.runBtn.addEventListener("click", () => { state.running = true; log("Simulação rodando"); });
ui.pauseBtn.addEventListener("click", () => { state.running = false; log("Simulação pausada"); });
ui.resetBtn.addEventListener("click", resetCar);
ui.trackMode.addEventListener("change", resetCar);
ui.directionMode.addEventListener("change", resetCar);
ui.environmentPreset.addEventListener("change", applyEnvironmentPreset);
ui.randomizeSignsBtn.addEventListener("click", randomizeSigns);
ui.clearSignsBtn.addEventListener("click", clearSigns);
ui.applyCodeBtn.addEventListener("click", compileCustomCode);
ui.restoreCodeBtn.addEventListener("click", () => {
  ui.codeEditor.value = DEFAULT_CONTROLLER_CODE;
  customController = null;
  ui.controllerMode.value = "rightLine";
  resetCar();
  ui.codeStatus.textContent = "Controlador padrão restaurado.";
});

document.querySelectorAll(".navItem").forEach((button) => {
  button.addEventListener("click", () => {
    const target = button.dataset.panelTarget;
    document.querySelectorAll(".navItem").forEach((item) => item.classList.toggle("active", item === button));
    document.querySelectorAll(".controlPanel").forEach((panel) => {
      panel.classList.toggle("active", panel.dataset.panel === target);
    });
  });
});
ui.trackInput.addEventListener("change", (ev) => {
  const file = ev.target.files?.[0];
  if (!file) return;
  const img = new Image();
  img.onload = () => {
    trackCtx.fillStyle = "#202421";
    trackCtx.fillRect(0, 0, trackCanvas.width, trackCanvas.height);
    const scale = Math.min(trackCanvas.width / img.width, trackCanvas.height / img.height);
    const iw = img.width * scale;
    const ih = img.height * scale;
    trackCtx.drawImage(img, (trackCanvas.width - iw) / 2, (trackCanvas.height - ih) / 2, iw, ih);
    refreshTrackPixels();
    ui.trackName.textContent = file.name;
    resetCar();
    log(`Imagem carregada: ${file.name}`);
  };
  img.src = URL.createObjectURL(file);
});

canvas.addEventListener("click", (ev) => {
  const rect = canvas.getBoundingClientRect();
  const x = (ev.clientX - rect.left) * (canvas.width / rect.width);
  const y = (ev.clientY - rect.top) * (canvas.height / rect.height);
  if (ui.signEditMode.checked) {
    addSignGenerator(x, y);
    return;
  }
  state.car.x = x;
  state.car.y = y;
  state.path = [];
  log(`Carro reposicionado: ${state.car.x.toFixed(0)}, ${state.car.y.toFixed(0)}`);
});

window.addEventListener("keydown", (ev) => {
  if (ev.key.toLowerCase() === "a") state.car.heading -= 0.08;
  if (ev.key.toLowerCase() === "d") state.car.heading += 0.08;
});

window.__simDebug = {
  getState: () => JSON.parse(document.body.dataset.simState || "{}"),
  getInput: () => JSON.parse(JSON.stringify(state.lastInput)),
  getCar: () => JSON.parse(JSON.stringify(state.car)),
  snapshot: () => JSON.parse(JSON.stringify({
    time: state.time,
    running: state.running,
    car: state.car,
    speed: state.currentSpeed,
    activeSign: state.activeSign,
    maneuver: currentManeuverInput(),
    headingHold: state.headingHold,
    lastInput: state.lastInput,
    handledSigns: [...state.handledSigns],
    signs: state.signGenerators
  })),
  run: () => { state.running = true; },
  pause: () => { state.running = false; },
  reset: () => resetCar(),
  clearSigns: () => clearSigns(),
  addSign: (x, y, type = "stop", allowed = [type], approachHeading = state.car.heading) => {
    state.signGenerators.push({ id: state.nextSignId++, x, y, type, allowed, approachHeading });
    saveSignGenerators();
    updateSignList();
  },
  setEnvironment: (preset) => {
    ui.environmentPreset.value = preset;
    applyEnvironmentPreset();
  }
};

ui.codeEditor.value = DEFAULT_CONTROLLER_CODE;
ui.controllerMode.value = "rightLine";
loadSignGenerators();
applyScenarioFromUrl();
loadOfficialTrack();
bindRanges();
updateSignList();
requestAnimationFrame(frame);
