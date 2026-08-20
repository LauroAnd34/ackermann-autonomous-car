const telaSimulacao = document.getElementById("simCanvas");
const ctxSimulacao = telaSimulacao.getContext("2d");
const telaCamera = document.getElementById("cameraCanvas");
const ctxCamera = telaCamera.getContext("2d");
const telaProcessamento = document.getElementById("processedCanvas");
const ctxProcessamento = telaProcessamento.getContext("2d");

// Padrao usado no simulador:
// - Modelo: o objeto `estado` guarda carro, placas, telemetria e leituras.
// - Visao: funcoes `renderizar*` desenham pista, camera, processamento e paineis.
// - Controle/Strategy: `controladorLinhaDireitaEmbutido` ou o codigo colado no editor
//   recebem a mesma entrada de sensores e devolvem direcao/velocidade.
// Os nomes do contrato do editor (`input.rightLineOffsetPx`, `api.clamp`, etc.) ficam em
// ingles para manter compatibilidade com os codigos ja testados no simulador.

const idsInterface = [
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
const interfaceUsuario = Object.fromEntries(idsInterface.map((id) => [id, document.getElementById(id)]));

const telaPista = document.createElement("canvas");
telaPista.width = telaSimulacao.width;
telaPista.height = telaSimulacao.height;
const ctxPista = telaPista.getContext("2d");

let pixelsPista = null;
let controladorPersonalizado = null;
let ultimaImagemCamera = null;
let limiarBranco = 185;
const POSES_INICIAIS = {
  outer_forward: { x: 92, y: 900, heading: -Math.PI / 2 },
  outer_reverse: { x: 827, y: 300, heading: Math.PI / 2 },
  city_forward: { x: 456, y: 1040, heading: -Math.PI / 2 },
  city_reverse: { x: 414, y: 270, heading: Math.PI / 2 }
};
const TIPOS_PLACA = {
  proceed_left: { label: "Vire à esquerda", short: "L", color: "#168fd3", decision: "converter para a rua à esquerda" },
  proceed_right: { label: "Vire à direita", short: "R", color: "#168fd3", decision: "converter para a rua à direita" },
  proceed_forward: { label: "Siga em frente", short: "F", color: "#168fd3", decision: "seguir em frente no cruzamento" },
  stop: { label: "Pare", short: "STOP", color: "#ee584f", decision: "parar; destino encontrado" },
  no_entry: { label: "Não entre", short: "NO", color: "#ee584f", decision: "não entrar nessa rua" },
  dead_end: { label: "Rua sem saída", short: "T", color: "#168fd3", decision: "não escolher essa rua" },
  tunnel: { label: "Túnel", short: "TU", color: "#6f7b86", decision: "trecho especial de túnel" },
  bridge: { label: "Ponte", short: "BR", color: "#6f7b86", decision: "trecho especial de ponte" }
};

const CODIGO_CONTROLADOR_PADRAO = `function control(input, api) {
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

const estado = {
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

function registrarLog(message) {
  const t = estado.time.toFixed(2).padStart(6, "0");
  estado.logLines.unshift(`[${t}s] ${message}`);
  estado.logLines = estado.logLines.slice(0, 12);
  interfaceUsuario.log.textContent = estado.logLines.join("\n");
}

function parametros() {
  return {
    wheelbase: Number(interfaceUsuario.wheelbase.value),
    maxSteer: Number(interfaceUsuario.maxSteer.value) * Math.PI / 180,
    speed: Number(interfaceUsuario.speed.value),
    pwmMin: Number(interfaceUsuario.pwmMin.value),
    pwmCenter: Number(interfaceUsuario.pwmCenter.value),
    pwmMax: Number(interfaceUsuario.pwmMax.value)
  };
}

function parametrosAmbiente() {
  return {
    light: Number(interfaceUsuario.envLight.value),
    contrast: Number(interfaceUsuario.envContrast.value),
    noise: Number(interfaceUsuario.envNoise.value),
    shadow: Number(interfaceUsuario.envShadow.value)
  };
}

function escalaMundo() {
  return 100;
}

function limitar(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function normalizarGraus(rad) {
  return ((((rad * 180 / Math.PI) + 180) % 360) + 360) % 360 - 180;
}

function normalizarRad(rad) {
  return Math.atan2(Math.sin(rad), Math.cos(rad));
}

function atualizarPixelsPista() {
  pixelsPista = ctxPista.getImageData(0, 0, telaPista.width, telaPista.height);
}

function pixelEm(x, y) {
  const px = Math.round(x);
  const py = Math.round(y);
  if (!pixelsPista || px < 0 || py < 0 || px >= telaPista.width || py >= telaPista.height) {
    return [0, 0, 0, 255];
  }
  const i = (py * telaPista.width + px) * 4;
  return [pixelsPista.data[i], pixelsPista.data[i + 1], pixelsPista.data[i + 2], 255];
}

function ruidoDeterministico(x, y, t = 0) {
  const n = Math.sin(x * 12.9898 + y * 78.233 + Math.floor(t * 12) * 37.719) * 43758.5453;
  return (n - Math.floor(n)) * 2 - 1;
}

function aplicarAmbienteRgb(r, g, b, wx, wy, px = 0, py = 0) {
  const env = parametrosAmbiente();
  const shadowWave = 0.5 + 0.5 * Math.sin((wx + wy * 0.7) * 0.018 + estado.time * 0.7);
  const shadow = 1 - env.shadow * shadowWave;
  const vignette = 1 - env.shadow * 0.25 * Math.hypot((px || 160) - 160, (py || 90) - 90) / 185;
  const gain = env.light * shadow * vignette;
  const noise = env.noise * ruidoDeterministico(wx + px, wy + py, estado.time);
  const adjust = (value) => limitar(((value - 128) * env.contrast + 128) * gain + noise, 0, 255);
  return [adjust(r), adjust(g), adjust(b)];
}

function amostraBranca(x, y) {
  const [r, g, b] = pixelEm(x, y);
  const [er, eg, eb] = aplicarAmbienteRgb(r, g, b, x, y);
  return er > limiarBranco && eg > limiarBranco && eb > Math.max(150, limiarBranco - 15);
}

function carregarPistaOficial() {
  const img = new Image();
  img.onload = () => {
    ctxPista.fillStyle = "#211d1e";
    ctxPista.fillRect(0, 0, telaPista.width, telaPista.height);
    ctxPista.drawImage(img, 0, 0, telaPista.width, telaPista.height);
    atualizarPixelsPista();
    interfaceUsuario.trackName.textContent = "pista oficial FIRA";
    resetarCarro();
    registrarLog("Pista oficial carregada");
  };
  img.onerror = () => {
    desenharPistaPadrao();
    atualizarPixelsPista();
    resetarCarro();
    registrarLog("Pista oficial não encontrada; usando pista gerada");
  };
  img.src = "./assets/official_track.png";
}

function desenharPistaPadrao() {
  const w = telaPista.width;
  const h = telaPista.height;
  ctxPista.fillStyle = "#202421";
  ctxPista.fillRect(0, 0, w, h);
  ctxPista.strokeStyle = "#f5f5ef";
  ctxPista.lineCap = "round";
  ctxPista.lineJoin = "round";
  ctxPista.lineWidth = 6;
  ctxPista.beginPath();
  ctxPista.moveTo(130, 900);
  ctxPista.bezierCurveTo(80, 590, 100, 260, 220, 150);
  ctxPista.bezierCurveTo(390, 0, 750, 60, 815, 230);
  ctxPista.bezierCurveTo(910, 480, 850, 930, 660, 1040);
  ctxPista.bezierCurveTo(450, 1165, 185, 1070, 130, 900);
  ctxPista.stroke();
}

function resetarCarro() {
  const mode = interfaceUsuario.trackMode?.value || "city";
  const direction = interfaceUsuario.directionMode?.value || "forward";
  const pose = POSES_INICIAIS[`${mode}_${direction}`] || POSES_INICIAIS.city_forward;
  estado.time = 0;
  estado.car = { x: pose.x, y: pose.y, heading: pose.heading, steer: 0 };
  estado.path = [];
  estado.detections = [];
  estado.lastError = 0;
  estado.lastSteerCommand = 0;
  estado.currentSpeed = 0;
  estado.confidence = 0;
  estado.lastInput = null;
  estado.activeSign = null;
  estado.handledSigns = new Set();
  estado.maneuver = null;
  estado.headingHold = null;
  estado.stopUntil = 0;
  registrarLog(`Reset feito (${mode === "city" ? "cidade/placas" : "pista externa"} - ${direction === "reverse" ? "voltando" : "indo"})`);
}

function referencialLocal() {
  const c = estado.car;
  return referencialPorRumo(c.heading);
}

function referencialPorRumo(heading) {
  return {
    forward: { x: Math.cos(heading), y: Math.sin(heading) },
    right: { x: -Math.sin(heading), y: Math.cos(heading) }
  };
}

function varrerGruposBrancos(base, right, halfWidth) {
  const groups = [];
  let current = null;
  for (let offset = -halfWidth; offset <= halfWidth; offset += 2) {
    const x = base.x + right.x * offset;
    const y = base.y + right.y * offset;
    if (amostraBranca(x, y)) {
      if (!current) current = { start: offset, end: offset, count: 0 };
      current.end = offset;
      current.count += 1;
    } else if (current) {
      groups.push(paraGrupo(current));
      current = null;
    }
  }
  if (current) groups.push(paraGrupo(current));
  return groups;
}

function paraGrupo(raw) {
  return {
    start: raw.start,
    end: raw.end,
    center: (raw.start + raw.end) / 2,
    width: raw.end - raw.start + 2,
    count: raw.count
  };
}

function perceberLinhaDireita() {
  const c = estado.car;
  const { forward, right } = referencialLocal();
  const scanDistances = [18, 30, 44, 62, 84, 110, 140, 174];
  const expectedRight = limitar(estado.lastInput?.rightLineOffsetPx ?? 42, 24, 112);
  const usableRight = [];
  const usableLeft = [];
  const usableDashed = [];
  const detections = [];

  for (const dist of scanDistances) {
    const base = { x: c.x + forward.x * dist, y: c.y + forward.y * dist };
    const groups = varrerGruposBrancos(base, right, 165)
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

    if (rightLine) usableRight.push({ offset: rightLine.center, dist, weight: limitar(2.2 - dist / 125, 0.45, 2.0) });
    if (leftLine) usableLeft.push({ offset: leftLine.center, dist, weight: limitar(1.6 - dist / 175, 0.45, 1.45) });
    if (dashed) usableDashed.push({ offset: dashed.center, dist, weight: 1 });

    detections.push({
      base,
      leftHit: leftLine ? pontoAmostra(base, right, leftLine.center) : null,
      rightHit: rightLine ? pontoAmostra(base, right, rightLine.center) : null,
      centerOffset: rightLine ? rightLine.center - 64 : dashed?.center ?? null
    });
  }

  const rightLineOffsetPx = estabilizarDeslocamento(deslocamentoPonderado(usableRight), estado.lastInput?.rightLineOffsetPx, usableRight.length);
  const leftLineOffsetPx = deslocamentoPonderado(usableLeft);
  const centerDashedOffsetPx = estabilizarDeslocamento(deslocamentoPonderado(usableDashed), estado.lastInput?.centerDashedOffsetPx, usableDashed.length);
  const rightLineSlope = inclinacaoLinha(usableRight);
  const laneCenterTargetPx = alvoFaixa(rightLineOffsetPx, centerDashedOffsetPx);
  const observedSign = obterPlacaObservada();
  const actionableSign = obterPlacaVisivel();
  atualizarEstadoTransito(actionableSign);
  const confidence = limitar(usableRight.length * 0.18, 0, 1);
  const crossBarrierAhead = detectarBloqueioFrontal(forward, right);
  const laneSafetyFault = detectarFalhaSegurancaFaixa(rightLineOffsetPx, centerDashedOffsetPx, laneCenterTargetPx, confidence);
  const wrongLaneFault = detectarContramao(centerDashedOffsetPx, confidence);
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
    maneuver: entradaManobraAtual(),
    searchSteer: interfaceUsuario.directionMode.value === "reverse" ? -0.18 : 0.18,
    lastSteer: estado.lastSteerCommand,
    speedSetting: parametros().speed,
    time: estado.time,
    headingDeg: normalizarGraus(estado.car.heading),
    trackMode: interfaceUsuario.trackMode.value
  };

  estado.detections = detections;
  estado.confidence = confidence;
  estado.activeSign = observedSign;
  estado.lastInput = input;
  estado.lastError = rightLineOffsetPx === null ? 0 : (rightLineOffsetPx - (laneCenterTargetPx ?? 42)) / escalaMundo();
  return input;
}

function pontoAmostra(base, right, offset) {
  return { x: base.x + right.x * offset, y: base.y + right.y * offset, offset };
}

function detectarBloqueioFrontal(forward, right) {
  if (interfaceUsuario.trackMode.value !== "city" || estado.maneuver || estado.stopUntil > estado.time) return null;
  for (const dist of [30, 44, 60, 78]) {
    const base = { x: estado.car.x + forward.x * dist, y: estado.car.y + forward.y * dist };
    const groups = varrerGruposBrancos(base, right, 92);
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

function detectarFalhaSegurancaFaixa(rightOffset, dashedOffset, targetOffset, confidence) {
  if (interfaceUsuario.trackMode.value !== "city" || estado.maneuver || estado.stopUntil > estado.time) return null;
  if (estado.handledSigns.size === 0) return null;
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

function detectarContramao(dashedOffset, confidence) {
  if (interfaceUsuario.trackMode.value !== "city" || estado.maneuver || estado.stopUntil > estado.time) return null;
  if (estado.handledSigns.size === 0 || dashedOffset === null || confidence < 0.24) return null;
  if (dashedOffset > 6) {
    return {
      dash: Number(dashedOffset.toFixed(1)),
      expected: "tracejada à esquerda da câmera"
    };
  }
  return null;
}

function deslocamentoPonderado(items) {
  if (!items.length) return null;
  const total = items.reduce((sum, item) => sum + item.weight, 0);
  return items.reduce((sum, item) => sum + item.offset * item.weight, 0) / total;
}

function estabilizarDeslocamento(raw, previous, sampleCount) {
  if (raw === null || raw === undefined) return null;
  if (previous === null || previous === undefined || !Number.isFinite(previous)) return raw;
  const maxStep = sampleCount >= 5 ? 28 : sampleCount >= 3 ? 16 : 10;
  const limited = previous + limitar(raw - previous, -maxStep, maxStep);
  const alpha = sampleCount >= 5 ? 0.72 : sampleCount >= 3 ? 0.52 : 0.32;
  return previous * (1 - alpha) + limited * alpha;
}

function inclinacaoLinha(items) {
  if (items.length < 2) return 0;
  const sorted = [...items].sort((a, b) => a.dist - b.dist);
  const near = sorted.slice(0, 3);
  const far = sorted.slice(-3);
  const nearOffset = near.reduce((sum, item) => sum + item.offset, 0) / near.length;
  const farOffset = far.reduce((sum, item) => sum + item.offset, 0) / far.length;
  const distSpan = Math.max(1, far[far.length - 1].dist - near[0].dist);
  return limitar((farOffset - nearOffset) / distSpan, -1, 1);
}

function alvoFaixa(rightOffset, dashedOffset) {
  if (rightOffset === null || dashedOffset === null) return null;
  const laneWidth = rightOffset - dashedOffset;
  if (laneWidth < 34 || laneWidth > 120) return null;
  return limitar(laneWidth * 0.42, 30, 50);
}

function atualizarEstadoTransito(sign) {
  if (estado.stopUntil > estado.time) return;
  if (estado.stopUntil && estado.time >= estado.stopUntil) {
    estado.stopUntil = 0;
    estado.maneuver = null;
  }
  if (!sign || estado.handledSigns.has(sign.id)) {
    if (estado.maneuver && estado.time > estado.maneuver.until) estado.maneuver = null;
    return;
  }
  if (!placaAcionavel(sign)) return;

  if (sign.type === "stop") {
    estado.stopUntil = estado.time + 1.15;
    estado.handledSigns.add(sign.id);
    estado.maneuver = null;
    registrarLog(`STOP atendido no gerador ${sign.id}`);
    return;
  }

  if (sign.type === "proceed_left" || sign.type === "proceed_right" || sign.type === "proceed_forward") {
    const turnDir = sign.type === "proceed_left" ? -1 : sign.type === "proceed_right" ? 1 : 0;
    const duration = sign.type === "proceed_forward" ? 0.8 : 9.8;
    const turnDelay = turnDir ? 0.35 : 0;
    const approach = Number.isFinite(sign.approachHeading) ? sign.approachHeading : estado.car.heading;
    const { forward, right } = referencialPorRumo(approach);
    const side = turnDir > 0 ? right : { x: -right.x, y: -right.y };
    const exitPoint = turnDir
      ? {
          x: sign.x + forward.x * 82 + side.x * 118,
          y: sign.y + forward.y * 82 + side.y * 118
        }
      : null;
    estado.maneuver = {
      type: sign.type,
      signId: sign.id,
      turnDir,
      targetHeading: turnDir ? normalizarRad(approach + turnDir * Math.PI / 2) : estado.car.heading,
      exitPoint,
      startedAt: estado.time,
      turnDelayUntil: estado.time + turnDelay,
      until: estado.time + duration + turnDelay
    };
    estado.handledSigns.add(sign.id);
    registrarLog(`${TIPOS_PLACA[sign.type].label} iniciado no gerador ${sign.id}`);
  }
}

function entradaManobraAtual() {
  if (estado.stopUntil > estado.time) {
    return { stop: true, active: false, remaining: Number((estado.stopUntil - estado.time).toFixed(2)) };
  }
  if (estado.headingHold && estado.time <= estado.headingHold.until) {
    const error = normalizarRad(estado.headingHold.heading - estado.car.heading);
    const lockFrame = referencialPorRumo(estado.headingHold.heading);
    const currentLateral = estado.car.x * lockFrame.right.x + estado.car.y * lockFrame.right.y;
    const lateralError = Number.isFinite(estado.headingHold.desiredLateral)
      ? estado.headingHold.desiredLateral - currentLateral
      : 0;
    return {
      active: true,
      type: "lane_settle",
      steerBias: limitar(error * 0.95 + lateralError / 70, -0.58, 0.58),
      lateralError: Number(lateralError.toFixed(1)),
      remaining: Number((estado.headingHold.until - estado.time).toFixed(2)),
      targetHeadingDeg: Number(normalizarGraus(estado.headingHold.heading).toFixed(1))
    };
  }
  if (estado.headingHold && estado.time > estado.headingHold.until) estado.headingHold = null;
  if (!estado.maneuver || estado.time > estado.maneuver.until) return null;
  if (estado.maneuver.turnDir && estado.maneuver.turnDelayUntil && estado.time < estado.maneuver.turnDelayUntil) {
    return {
      active: true,
      type: estado.maneuver.type,
      phase: "enter_intersection",
      steerBias: 0,
      progress: 0,
      targetHeadingDeg: Number(normalizarGraus(estado.maneuver.targetHeading).toFixed(1)),
      envelope: 0.35,
      remaining: Number((estado.maneuver.turnDelayUntil - estado.time).toFixed(2))
    };
  }
  const turnStart = estado.maneuver.turnDelayUntil ?? estado.maneuver.startedAt ?? estado.time - 0.01;
  const progress = 1 - ((estado.maneuver.until - estado.time) / (estado.maneuver.until - turnStart));
  const elapsed = estado.time - turnStart;
  if (estado.maneuver.turnDir && elapsed > 0.85) {
    const remainingHeading = normalizarRad(estado.maneuver.targetHeading - estado.car.heading);
    const exitDist = estado.maneuver.exitPoint
      ? Math.hypot(estado.maneuver.exitPoint.x - estado.car.x, estado.maneuver.exitPoint.y - estado.car.y)
      : 0;
    if (Math.abs(remainingHeading) < 0.13 && exitDist < 34) {
      const lockFrame = referencialPorRumo(estado.maneuver.targetHeading);
      const desiredLateral = estado.maneuver.exitPoint
        ? estado.maneuver.exitPoint.x * lockFrame.right.x + estado.maneuver.exitPoint.y * lockFrame.right.y
        : estado.car.x * lockFrame.right.x + estado.car.y * lockFrame.right.y;
      estado.headingHold = {
        heading: estado.maneuver.targetHeading,
        desiredLateral,
        until: estado.time + 24
      };
      estado.maneuver = null;
      return entradaManobraAtual();
    }
  }
  const envelope = Math.sin(limitar(progress, 0, 1) * Math.PI);
  let steerBias = 0;
  if (estado.maneuver.turnDir) {
    const exit = estado.maneuver.exitPoint;
    const desiredHeading = exit
      ? Math.atan2(exit.y - estado.car.y, exit.x - estado.car.x)
      : estado.maneuver.targetHeading;
    const headingError = normalizarRad(desiredHeading - estado.car.heading);
    const finalHeadingError = normalizarRad(estado.maneuver.targetHeading - estado.car.heading);
    const dist = exit ? Math.hypot(exit.x - estado.car.x, exit.y - estado.car.y) : 999;
    const blend = dist < 58 ? limitar(1 - dist / 58, 0, 1) : 0;
    const command = normalizarRad(headingError * (1 - blend) + finalHeadingError * blend) / (Math.PI / 2);
    steerBias = limitar(command * 1.35, -0.88, 0.88);
    if (Math.abs(steerBias) < 0.24) steerBias = 0.24 * Math.sign(steerBias || estado.maneuver.turnDir);
  }
  return {
    active: true,
    type: estado.maneuver.type,
    steerBias: estado.maneuver.turnDir ? steerBias : 0,
    progress: Number(limitar(progress, 0, 1).toFixed(2)),
    targetHeadingDeg: Number(normalizarGraus(estado.maneuver.targetHeading).toFixed(1)),
    envelope: Number(limitar(envelope, 0.35, 1).toFixed(2)),
    remaining: Number((estado.maneuver.until - estado.time).toFixed(2))
  };
}

function controladorLinhaDireitaEmbutido(input, api) {
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

function aplicarControlador(input) {
  const api = { clamp: limitar, limitar, Math };
  let output;
  try {
    output = interfaceUsuario.controllerMode.value === "customCode" && controladorPersonalizado
      ? controladorPersonalizado(input, api)
      : controladorLinhaDireitaEmbutido(input, api);
  } catch (error) {
    interfaceUsuario.codeStatus.textContent = `Erro no controlador: ${error.message}`;
    output = controladorLinhaDireitaEmbutido(input, api);
  }
  const steerCommand = limitar(Number(output?.steer ?? 0), -1, 1);
  const speedCommand = limitar(Number(output?.speed ?? parametros().speed), 0, parametros().speed);
  estado.lastSteerCommand = steerCommand;
  return { steerCommand, speedCommand };
}

function atualizar(dt) {
  const p = parametros();
  const input = perceberLinhaDireita();
  const control = aplicarControlador(input);
  const c = estado.car;
  const pxPerMeter = escalaMundo();
  const v = control.speedCommand * pxPerMeter;
  const targetSteer = control.steerCommand * p.maxSteer;
  estado.currentSpeed = control.speedCommand;

  c.steer += (targetSteer - c.steer) * Math.min(1, dt * 8);
  c.heading += (v / (p.wheelbase * pxPerMeter)) * Math.tan(c.steer) * dt;
  c.x += Math.cos(c.heading) * v * dt;
  c.y += Math.sin(c.heading) * v * dt;
  estado.path.push({ x: c.x, y: c.y });
  estado.path = estado.path.slice(-1000);
  estado.time += dt;
}

function desenharCarro() {
  const c = estado.car;
  ctxSimulacao.save();
  ctxSimulacao.translate(c.x, c.y);
  ctxSimulacao.rotate(c.heading);
  ctxSimulacao.fillStyle = "#111315";
  ctxSimulacao.strokeStyle = "#d7dee2";
  ctxSimulacao.lineWidth = 2;
  ctxSimulacao.fillRect(-24, -13, 48, 26);
  ctxSimulacao.strokeRect(-24, -13, 48, 26);
  ctxSimulacao.fillStyle = "#f2c230";
  ctxSimulacao.fillRect(5, -10, 14, 20);
  ctxSimulacao.strokeStyle = "#24d0c4";
  ctxSimulacao.lineWidth = 4;
  ctxSimulacao.beginPath();
  ctxSimulacao.moveTo(-15, -16);
  ctxSimulacao.lineTo(-15, -25);
  ctxSimulacao.moveTo(-15, 16);
  ctxSimulacao.lineTo(-15, 25);
  ctxSimulacao.stroke();
  for (const y of [-16, 16]) {
    ctxSimulacao.save();
    ctxSimulacao.translate(17, y);
    ctxSimulacao.rotate(c.steer);
    ctxSimulacao.beginPath();
    ctxSimulacao.moveTo(0, -9);
    ctxSimulacao.lineTo(0, 9);
    ctxSimulacao.stroke();
    ctxSimulacao.restore();
  }
  ctxSimulacao.restore();
}

function pwmServo(steer, p) {
  if (steer >= 0) return p.pwmCenter + (steer / p.maxSteer) * (p.pwmMax - p.pwmCenter);
  return p.pwmCenter + (steer / p.maxSteer) * (p.pwmCenter - p.pwmMin);
}

function renderizarCamera() {
  const img = ctxCamera.createImageData(telaCamera.width, telaCamera.height);
  const c = estado.car;
  const { forward, right } = referencialLocal();
  for (let py = 0; py < telaCamera.height; py += 1) {
    const v = py / (telaCamera.height - 1);
    const forwardDist = 38 + (1 - v) * (1 - v) * 250;
    const halfWidth = 42 + (1 - v) * 138;
    for (let px = 0; px < telaCamera.width; px += 1) {
      const u = (px / (telaCamera.width - 1) - 0.5) * 2;
      const lateral = u * halfWidth;
      const wx = c.x + forward.x * forwardDist + right.x * lateral;
      const wy = c.y + forward.y * forwardDist + right.y * lateral;
      const [r, g, b] = pixelEm(wx, wy);
      const [er, eg, eb] = aplicarAmbienteRgb(r, g, b, wx, wy, px, py);
      const idx = (py * telaCamera.width + px) * 4;
      img.data[idx] = er;
      img.data[idx + 1] = eg;
      img.data[idx + 2] = eb;
      img.data[idx + 3] = 255;
    }
  }
  ctxCamera.putImageData(img, 0, 0);
  ultimaImagemCamera = img;
  ctxCamera.strokeStyle = "rgba(242, 194, 48, 0.9)";
  ctxCamera.lineWidth = 1;
  ctxCamera.beginPath();
  ctxCamera.moveTo(telaCamera.width / 2, telaCamera.height - 1);
  ctxCamera.lineTo(telaCamera.width / 2, 0);
  ctxCamera.stroke();
  renderizarCorredorCamera();
  renderizarPlacasCamera();
}

function renderizarCorredorCamera() {
  const prediction = preverTrajetoria();
  const alpha = prediction.maneuver?.active ? 0.95 : 0.45;
  desenharLinhaCorredorCamera(prediction.points, -prediction.halfLane, `rgba(36,208,196,${alpha})`);
  desenharLinhaCorredorCamera(prediction.points, prediction.halfLane, `rgba(242,194,48,${alpha})`);
}

function desenharLinhaCorredorCamera(points, lateralOffset, color) {
  const projected = points
    .map((pt) => {
      const rightAtPoint = { x: -Math.sin(pt.heading), y: Math.cos(pt.heading) };
      return projetarMundoNaCamera(pt.x + rightAtPoint.x * lateralOffset, pt.y + rightAtPoint.y * lateralOffset);
    })
    .filter(Boolean);
  if (projected.length < 2) return;
  ctxCamera.save();
  ctxCamera.strokeStyle = color;
  ctxCamera.lineWidth = 2;
  ctxCamera.lineCap = "round";
  ctxCamera.lineJoin = "round";
  ctxCamera.beginPath();
  projected.forEach((pt, index) => {
    if (index === 0) ctxCamera.moveTo(pt.x, pt.y);
    else ctxCamera.lineTo(pt.x, pt.y);
  });
  ctxCamera.stroke();
  ctxCamera.restore();
}

function projetarMundoNaCamera(x, y) {
  const c = estado.car;
  const { forward, right } = referencialLocal();
  const dx = x - c.x;
  const dy = y - c.y;
  const forwardDist = dx * forward.x + dy * forward.y;
  const lateral = dx * right.x + dy * right.y;
  return projetarNaCamera(forwardDist, lateral);
}

function renderizarPlacasCamera() {
  const visible = obterPlacasVisiveis().slice(0, 3);
  for (const sign of visible) {
    const projected = projetarNaCamera(sign.forward, sign.lateral);
    if (!projected) continue;
    desenharSimboloPlaca(ctxCamera, sign.type, projected.x, projected.y, projected.size, 0);
  }
}

function projetarNaCamera(forwardDist, lateral) {
  if (forwardDist < 22 || forwardDist > 245) return null;
  const v = 1 - Math.sqrt(limitar((forwardDist - 38) / 250, 0, 1));
  const halfWidth = 42 + (1 - v) * 138;
  if (Math.abs(lateral) > halfWidth) return null;
  return {
    x: telaCamera.width / 2 + (lateral / halfWidth) * (telaCamera.width / 2),
    y: v * (telaCamera.height - 1),
    size: limitar(30 - forwardDist / 12, 10, 24)
  };
}

function renderizarProcessamento() {
  if (!ultimaImagemCamera) return;
  const out = ctxProcessamento.createImageData(telaProcessamento.width, telaProcessamento.height);
  const data = ultimaImagemCamera.data;
  for (let i = 0; i < data.length; i += 4) {
    const white = data[i] > limiarBranco && data[i + 1] > limiarBranco && data[i + 2] > Math.max(150, limiarBranco - 15);
    out.data[i] = white ? 255 : 0;
    out.data[i + 1] = white ? 255 : 0;
    out.data[i + 2] = white ? 255 : 0;
    out.data[i + 3] = 255;
  }
  ctxProcessamento.putImageData(out, 0, 0);

  ctxProcessamento.strokeStyle = "rgba(242,194,48,0.85)";
  ctxProcessamento.lineWidth = 1;
  for (const y of [132, 118, 102, 86, 70, 54, 38]) {
    ctxProcessamento.beginPath();
    ctxProcessamento.moveTo(0, y);
    ctxProcessamento.lineTo(telaProcessamento.width, y);
    ctxProcessamento.stroke();
  }

  ctxProcessamento.strokeStyle = "rgba(36,208,196,0.9)";
  ctxProcessamento.beginPath();
  ctxProcessamento.moveTo(telaProcessamento.width / 2, 0);
  ctxProcessamento.lineTo(telaProcessamento.width / 2, telaProcessamento.height);
  ctxProcessamento.stroke();

  ctxProcessamento.strokeStyle = "rgba(36,208,196,0.45)";
  ctxProcessamento.setLineDash([5, 5]);
  ctxProcessamento.beginPath();
  const targetOffset = estado.lastInput?.laneCenterTargetPx ?? 42;
  ctxProcessamento.moveTo(telaProcessamento.width / 2 + targetOffset, 0);
  ctxProcessamento.lineTo(telaProcessamento.width / 2 + targetOffset, telaProcessamento.height);
  ctxProcessamento.stroke();
  ctxProcessamento.setLineDash([]);

  ctxProcessamento.fillStyle = "#24d0c4";
  const input = estado.lastInput;
  if (input) {
    desenharPontoProcessamento(input.rightLineOffsetPx, 116, "#24d0c4");
    desenharPontoProcessamento(input.leftLineOffsetPx, 142, "#ee584f");
    desenharPontoProcessamento(input.centerDashedOffsetPx, 90, "#f2c230");
    interfaceUsuario.visionReadout.textContent = [
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
    interfaceUsuario.visionReadout.textContent = `aguardando frame\nthresh: ${whiteThreshold}`;
  }
}

function desenharPontoProcessamento(offset, y, color) {
  if (offset === null || offset === undefined) return;
  const x = telaProcessamento.width / 2 + offset;
  ctxProcessamento.fillStyle = color;
  ctxProcessamento.beginPath();
  ctxProcessamento.arc(limitar(x, 0, telaProcessamento.width), y, 5, 0, Math.PI * 2);
  ctxProcessamento.fill();
}

function formatarValor(value) {
  return value === null || value === undefined ? "none" : value.toFixed(1);
}

function renderizar() {
  ctxSimulacao.clearRect(0, 0, telaSimulacao.width, telaSimulacao.height);
  ctxSimulacao.drawImage(telaPista, 0, 0, telaSimulacao.width, telaSimulacao.height);

  if (interfaceUsuario.showPath.checked && estado.path.length > 1) {
    ctxSimulacao.strokeStyle = "#42d357";
    ctxSimulacao.lineWidth = 2;
    ctxSimulacao.beginPath();
    ctxSimulacao.moveTo(estado.path[0].x, estado.path[0].y);
    for (const pt of estado.path) ctxSimulacao.lineTo(pt.x, pt.y);
    ctxSimulacao.stroke();
  }

  if (interfaceUsuario.showSigns.checked) renderizarPlacasNaPista();
  renderizarCorredorPlanejado();

  if (interfaceUsuario.showRays.checked) {
    for (const d of estado.detections) {
      ctxSimulacao.strokeStyle = d.skipped ? "rgba(238,88,79,0.75)" : "rgba(242,194,48,0.85)";
      ctxSimulacao.lineWidth = 1;
      ctxSimulacao.beginPath();
      if (d.leftHit) { ctxSimulacao.moveTo(d.base.x, d.base.y); ctxSimulacao.lineTo(d.leftHit.x, d.leftHit.y); }
      if (d.rightHit) { ctxSimulacao.moveTo(d.base.x, d.base.y); ctxSimulacao.lineTo(d.rightHit.x, d.rightHit.y); }
      ctxSimulacao.stroke();
    }
  }

  if (interfaceUsuario.showCenterline.checked) {
    ctxSimulacao.fillStyle = "#24d0c4";
    for (const d of estado.detections.filter((v) => v.centerOffset !== null)) {
      ctxSimulacao.beginPath();
      ctxSimulacao.arc(d.base.x, d.base.y, 3, 0, Math.PI * 2);
      ctxSimulacao.fill();
    }
  }

  desenharCarro();
  renderizarCamera();
  renderizarProcessamento();
  atualizarTelemetria();
}

function atualizarTelemetria() {
  const p = parametros();
  const pwm = pwmServo(estado.car.steer, p);
  const steerText = `${(estado.car.steer * 180 / Math.PI).toFixed(1)}°`;
  const speedText = `${estado.currentSpeed.toFixed(2)} m/s`;
  const errorText = `${estado.lastError.toFixed(3)} m`;
  const headingText = `${normalizarGraus(estado.car.heading).toFixed(1)}°`;
  interfaceUsuario.steerMetric.textContent = steerText;
  interfaceUsuario.pwmMetric.textContent = `${pwm.toFixed(0)} µs`;
  interfaceUsuario.speedMetric.textContent = speedText;
  interfaceUsuario.errorMetric.textContent = errorText;
  interfaceUsuario.headingMetric.textContent = headingText;
  document.getElementById("steerMetricMirror").textContent = steerText;
  document.getElementById("speedMetricMirror").textContent = speedText;
  document.getElementById("errorMetricMirror").textContent = errorText;
  document.getElementById("headingMetricMirror").textContent = headingText;
  interfaceUsuario.fpsMetric.textContent = `${estado.fps.toFixed(0)}`;
  interfaceUsuario.clock.textContent = `${estado.time.toFixed(3)}s`;
  interfaceUsuario.hz.textContent = `${estado.fps.toFixed(1)} Hz`;
  atualizarLeituraPlaca();
  document.body.dataset.simState = JSON.stringify({
    time: Number(estado.time.toFixed(3)),
    x: Number(estado.car.x.toFixed(2)),
    y: Number(estado.car.y.toFixed(2)),
    headingDeg: Number(normalizarGraus(estado.car.heading).toFixed(2)),
    steerDeg: Number((estado.car.steer * 180 / Math.PI).toFixed(2)),
    speedMps: Number(estado.currentSpeed.toFixed(2)),
    rightLineOffsetPx: arredondarOuNulo(estado.lastInput?.rightLineOffsetPx),
    leftLineOffsetPx: arredondarOuNulo(estado.lastInput?.leftLineOffsetPx),
    centerDashedOffsetPx: arredondarOuNulo(estado.lastInput?.centerDashedOffsetPx),
    confidence: Number(estado.confidence.toFixed(2)),
    sign: estado.activeSign ? { type: estado.activeSign.type, forward: arredondarOuNulo(estado.activeSign.forward), lateral: arredondarOuNulo(estado.activeSign.lateral) } : null,
    crossBarrierAhead: estado.lastInput?.crossBarrierAhead || null,
    laneSafetyFault: estado.lastInput?.laneSafetyFault || null,
    wrongLaneFault: estado.lastInput?.wrongLaneFault || null,
    maneuver: entradaManobraAtual(),
    plannedCorridor: resumoCorredorPlanejado(),
    controllerMode: interfaceUsuario.controllerMode.value,
    trackMode: interfaceUsuario.trackMode.value,
    directionMode: interfaceUsuario.directionMode.value
  });
}

function preverTrajetoria() {
  const p = parametros();
  const maneuver = entradaManobraAtual();
  const steerCommand = maneuver?.active ? maneuver.steerBias : estado.lastSteerCommand;
  const speed = maneuver?.active ? 0.22 : Math.max(estado.currentSpeed, 0.32);
  const pxPerMeter = escalaMundo();
  const dt = 0.12;
  const horizon = maneuver?.active ? 2.9 : 1.8;
  const pose = { ...estado.car };
  const points = [{ x: pose.x, y: pose.y, heading: pose.heading }];

  for (let t = 0; t < horizon; t += dt) {
    const steerRad = limitar(steerCommand, -1, 1) * p.maxSteer;
    const v = speed * pxPerMeter;
    pose.heading += (v / (p.wheelbase * pxPerMeter)) * Math.tan(steerRad) * dt;
    pose.x += Math.cos(pose.heading) * v * dt;
    pose.y += Math.sin(pose.heading) * v * dt;
    points.push({ x: pose.x, y: pose.y, heading: pose.heading });
  }
  return { points, maneuver, halfLane: 28 };
}

function resumoCorredorPlanejado() {
  const prediction = preverTrajetoria();
  const end = prediction.points[prediction.points.length - 1];
  return {
    active: Boolean(prediction.maneuver?.active),
    points: prediction.points.length,
    endX: Number(end.x.toFixed(1)),
    endY: Number(end.y.toFixed(1))
  };
}

function renderizarCorredorPlanejado() {
  const prediction = preverTrajetoria();
  if (prediction.points.length < 2) return;
  const alpha = prediction.maneuver?.active ? 0.9 : 0.42;
  desenharLinhaCorredor(ctxSimulacao, prediction.points, -prediction.halfLane, `rgba(36,208,196,${alpha})`, 3);
  desenharLinhaCorredor(ctxSimulacao, prediction.points, prediction.halfLane, `rgba(242,194,48,${alpha})`, 3);
}

function desenharLinhaCorredor(targetCtx, points, lateralOffset, color, width) {
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

function arredondarOuNulo(value) {
  return value === null || value === undefined ? null : Number(value.toFixed(2));
}

function quadro(ts) {
  if (!estado.lastTs) estado.lastTs = ts;
  const rawDt = Math.min(0.04, (ts - estado.lastTs) / 1000);
  estado.lastTs = ts;
  estado.fps = estado.fps * 0.9 + (1 / Math.max(rawDt, 0.001)) * 0.1;
  if (estado.running) atualizar(rawDt);
  renderizar();
  requestAnimationFrame(quadro);
}

function compilarCodigoPersonalizado() {
  const source = interfaceUsuario.codeEditor.value;
  if (pareceCodigoRoboPython(source)) {
    aplicarPerfilRoboPython(source);
    return;
  }
  try {
    controladorPersonalizado = new Function("input", "api", `${source}\nreturn control(input, api);`);
    controladorPersonalizado({
      confidence: 0,
      lastSteer: 0,
      rightLineOffsetPx: null,
      leftLineOffsetPx: null,
      centerDashedOffsetPx: null,
      rightLineSlope: 0,
      laneCenterTargetPx: null,
      sign: null,
      maneuver: null,
      searchSteer: interfaceUsuario.directionMode.value === "reverse" ? -0.18 : 0.18,
      trackMode: interfaceUsuario.trackMode.value
    }, { limitar, Math });
    interfaceUsuario.codeStatus.textContent = "Código aplicado.";
    interfaceUsuario.controllerMode.value = "customCode";
  } catch (error) {
    controladorPersonalizado = null;
    interfaceUsuario.codeStatus.textContent = `Erro: ${error.message}`;
  }
}

function placasPermitidasSelecionadas() {
  const boxes = [...document.querySelectorAll(".allowedSign")].filter((box) => box.checked);
  const values = boxes.map((box) => box.value);
  return values.length ? values : ["proceed_forward"];
}

function escolherPlaca(allowed) {
  const forced = interfaceUsuario.signType.value;
  if (forced !== "random" && allowed.includes(forced)) return forced;
  return allowed[Math.floor(Math.random() * allowed.length)] || "proceed_forward";
}

function adicionarGeradorPlaca(x, y) {
  const allowed = placasPermitidasSelecionadas();
  const generator = {
    id: estado.nextSignId++,
    x,
    y,
    allowed,
    approachHeading: estado.car.heading,
    type: escolherPlaca(allowed)
  };
  estado.signGenerators.push(generator);
  salvarGeradoresPlaca();
  atualizarListaPlacas();
  registrarLog(`Gerador ${generator.id} criado: ${TIPOS_PLACA[generator.type].label}`);
}

function sortearPlacas() {
  for (const generator of estado.signGenerators) {
    generator.type = escolherPlaca(generator.allowed);
  }
  salvarGeradoresPlaca();
  atualizarListaPlacas();
  registrarLog("Placas sorteadas");
}

function limparPlacas() {
  estado.signGenerators = [];
  estado.activeSign = null;
  salvarGeradoresPlaca();
  atualizarListaPlacas();
  registrarLog("Geradores de placa limpos");
}

function obterPlacasVisiveis() {
  const c = estado.car;
  const { forward, right } = referencialLocal();
  return estado.signGenerators
    .map((generator) => {
      const dx = generator.x - c.x;
      const dy = generator.y - c.y;
      const f = dx * forward.x + dy * forward.y;
      const l = dx * right.x + dy * right.y;
      const approachError = Number.isFinite(generator.approachHeading)
        ? Math.abs(normalizarRad(c.heading - generator.approachHeading))
        : 0;
      return { ...generator, forward: f, lateral: l, approachError, distance: Math.hypot(dx, dy) };
    })
    .filter((sign) => sign.forward > -18 && sign.forward < 165 && sign.lateral > 10 && sign.lateral < 118)
    .sort((a, b) => a.forward - b.forward);
}

function obterPlacaVisivel() {
  return obterPlacasVisiveis().find((sign) => placaAcionavel(sign) && !estado.handledSigns.has(sign.id)) || null;
}

function obterPlacaObservada() {
  return obterPlacasVisiveis().find((sign) => !estado.handledSigns.has(sign.id)) || null;
}

function placaAcionavel(sign) {
  const minForward = sign.type === "stop" ? 8 : 2;
  const maxForward = sign.type === "stop" ? 28 : 18;
  if (sign.type !== "stop") {
    return sign.forward > -18 && sign.forward < 0 && sign.lateral > 8 && sign.lateral < 102 && sign.approachError < 0.85;
  }
  return sign.forward > minForward && sign.forward < maxForward && sign.lateral > 8 && sign.lateral < 102 && sign.approachError < 0.85;
}

function atualizarListaPlacas() {
  if (!interfaceUsuario.signList) return;
  interfaceUsuario.signList.textContent = estado.signGenerators.length
    ? estado.signGenerators.map((g) => {
      const allowed = g.allowed.map((type) => TIPOS_PLACA[type]?.short || type).join(",");
      const dir = Number.isFinite(g.approachHeading) ? `${normalizarGraus(g.approachHeading).toFixed(0)}°` : "qualquer";
      return `#${g.id} ${TIPOS_PLACA[g.type].label}  x=${g.x.toFixed(0)} y=${g.y.toFixed(0)}  sentido=${dir}  permitidas=[${allowed}]`;
    }).join("\n")
    : "sem geradores";
}

function salvarGeradoresPlaca() {
  try {
    localStorage.setItem("ackermann.signGenerators", JSON.stringify(estado.signGenerators));
    localStorage.setItem("ackermann.nextSignId", String(estado.nextSignId));
  } catch (error) {
    registrarLog(`Nao foi possivel salvar geradores: ${error.message}`);
  }
}

function carregarGeradoresPlaca() {
  try {
    const saved = JSON.parse(localStorage.getItem("ackermann.signGenerators") || "[]");
    if (Array.isArray(saved)) {
      estado.signGenerators = saved
        .filter((g) => Number.isFinite(g.x) && Number.isFinite(g.y) && TIPOS_PLACA[g.type])
        .map((g) => ({
          id: Number(g.id) || estado.nextSignId++,
          x: Number(g.x),
          y: Number(g.y),
          allowed: Array.isArray(g.allowed) && g.allowed.length ? g.allowed.filter((type) => TIPOS_PLACA[type]) : ["proceed_forward"],
          approachHeading: Number.isFinite(g.approachHeading) ? Number(g.approachHeading) : null,
          type: TIPOS_PLACA[g.type] ? g.type : "proceed_forward"
        }));
    }
    const nextId = Number(localStorage.getItem("ackermann.nextSignId"));
    estado.nextSignId = Math.max(nextId || 1, ...estado.signGenerators.map((g) => g.id + 1), 1);
  } catch (error) {
    estado.signGenerators = [];
    registrarLog(`Nao foi possivel carregar geradores: ${error.message}`);
  }
}

function aplicarCenarioDaUrl() {
  const parametros = new URLSearchParams(window.location.search);
  const scenario = parametros.get("scenario");
  if (!scenario) return;
  estado.signGenerators = [];
  estado.nextSignId = 1;
  if (scenario === "left-sign") {
    estado.signGenerators.push({ id: estado.nextSignId++, x: 20, y: 780, type: "stop", allowed: ["stop"], approachHeading: -Math.PI / 2 });
  } else if (scenario === "far-stop") {
    estado.signGenerators.push({ id: estado.nextSignId++, x: 115, y: 610, type: "stop", allowed: ["stop"], approachHeading: -Math.PI / 2 });
  } else if (scenario === "right-stop") {
    estado.signGenerators.push({ id: estado.nextSignId++, x: 125, y: 820, type: "stop", allowed: ["stop"], approachHeading: -Math.PI / 2 });
  } else if (scenario === "right-left-turn") {
    estado.signGenerators.push({ id: estado.nextSignId++, x: 125, y: 820, type: "proceed_left", allowed: ["proceed_left"], approachHeading: -Math.PI / 2 });
  } else if (scenario === "city-left-sign") {
    interfaceUsuario.trackMode.value = "city";
    estado.signGenerators.push({ id: estado.nextSignId++, x: 398, y: 965, type: "stop", allowed: ["stop"], approachHeading: -Math.PI / 2 });
  } else if (scenario === "city-far-stop") {
    interfaceUsuario.trackMode.value = "city";
    estado.signGenerators.push({ id: estado.nextSignId++, x: 505, y: 770, type: "stop", allowed: ["stop"], approachHeading: -Math.PI / 2 });
  } else if (scenario === "city-right-stop") {
    interfaceUsuario.trackMode.value = "city";
    estado.signGenerators.push({ id: estado.nextSignId++, x: 505, y: 965, type: "stop", allowed: ["stop"], approachHeading: -Math.PI / 2 });
  } else if (scenario === "city-left-turn") {
    interfaceUsuario.trackMode.value = "city";
    estado.signGenerators.push({ id: estado.nextSignId++, x: 505, y: 870, type: "proceed_left", allowed: ["proceed_left"], approachHeading: -Math.PI / 2 });
  } else if (scenario === "city-right-turn") {
    interfaceUsuario.trackMode.value = "city";
    estado.signGenerators.push({ id: estado.nextSignId++, x: 505, y: 870, type: "proceed_right", allowed: ["proceed_right"], approachHeading: -Math.PI / 2 });
  } else if (scenario === "city-reverse-wrong-sign") {
    interfaceUsuario.trackMode.value = "city";
    interfaceUsuario.directionMode.value = "reverse";
    estado.signGenerators.push({ id: estado.nextSignId++, x: 360, y: 320, type: "proceed_left", allowed: ["proceed_left"], approachHeading: -Math.PI / 2 });
  } else if (scenario === "city-reverse-right-stop") {
    interfaceUsuario.trackMode.value = "city";
    interfaceUsuario.directionMode.value = "reverse";
    estado.signGenerators.push({ id: estado.nextSignId++, x: 360, y: 320, type: "stop", allowed: ["stop"], approachHeading: Math.PI / 2 });
  }
  atualizarListaPlacas();
  registrarLog(`Cenario carregado: ${scenario}`);
}

function atualizarLeituraPlaca() {
  const sign = estado.activeSign;
  if (!interfaceUsuario.signReadout) return;
  const maneuver = entradaManobraAtual();
  if (maneuver?.stop) {
    interfaceUsuario.signReadout.textContent = [
      "STOP em execução",
      `tempo restante: ${maneuver.remaining}s`,
      "placa já marcada como atendida"
    ].join("\n");
    return;
  }
  if (maneuver?.active) {
    if (maneuver.type === "lane_settle") {
      interfaceUsuario.signReadout.textContent = [
        "estabilizando saida da conversao",
        `rumo alvo: ${maneuver.targetHeadingDeg}°`,
        `volante: ${maneuver.steerBias.toFixed(2)}`,
        `tempo restante: ${maneuver.remaining}s`
      ].join("\n");
      return;
    }
    interfaceUsuario.signReadout.textContent = [
      `${TIPOS_PLACA[maneuver.type]?.label || maneuver.type}`,
      "manobra em execução",
      `volante: ${maneuver.steerBias.toFixed(2)}`,
      `tempo restante: ${maneuver.remaining}s`
    ].join("\n");
    return;
  }
  if (!sign) {
    if (estado.lastInput?.wrongLaneFault) {
      interfaceUsuario.signReadout.textContent = [
        "corrigindo faixa direita",
        "tracejada apareceu à direita da câmera",
        `dash: ${estado.lastInput.wrongLaneFault.dash}px`,
        "voltando para a faixa correta"
      ].join("\n");
      return;
    }
    if (estado.lastInput?.laneSafetyFault) {
      interfaceUsuario.signReadout.textContent = [
      "corrigindo faixa direita",
      "perto demais da tracejada/contramão",
      `erro: ${estado.lastInput.laneSafetyFault.error}px`,
      "recuperando para a direita"
      ].join("\n");
      return;
    }
    if (estado.lastInput?.crossBarrierAhead) {
      interfaceUsuario.signReadout.textContent = [
        "aguardando placa",
        "linha continua/bloqueio detectado",
        `distancia: ${estado.lastInput.crossBarrierAhead.distance}px`,
        "sem decisao valida: carro parado"
      ].join("\n");
      return;
    }
    interfaceUsuario.signReadout.textContent = "nenhuma placa no campo de visão";
    return;
  }
  const def = TIPOS_PLACA[sign.type];
  interfaceUsuario.signReadout.textContent = [
    `${def.label}`,
    `decisão: ${def.decision}`,
    `distância frontal: ${sign.forward.toFixed(1)} px`,
    `lateral: ${sign.lateral.toFixed(1)} px`,
    `gerador #${sign.id}`
  ].join("\n");
}

function renderizarPlacasNaPista() {
  for (const generator of estado.signGenerators) {
    ctxSimulacao.save();
    ctxSimulacao.globalAlpha = 0.95;
    ctxSimulacao.strokeStyle = "#f2c230";
    ctxSimulacao.lineWidth = 1;
    ctxSimulacao.setLineDash([4, 4]);
    ctxSimulacao.beginPath();
    ctxSimulacao.arc(generator.x, generator.y, 22, 0, Math.PI * 2);
    ctxSimulacao.stroke();
    ctxSimulacao.setLineDash([]);
    desenharSimboloPlaca(ctxSimulacao, generator.type, generator.x, generator.y, 28, 0);
    ctxSimulacao.fillStyle = "#f2f4f5";
    ctxSimulacao.font = "700 11px ui-sans-serif, system-ui";
    ctxSimulacao.fillText(`#${generator.id}`, generator.x + 18, generator.y - 18);
    ctxSimulacao.restore();
  }
}

function desenharSimboloPlaca(targetCtx, type, x, y, size, rotation) {
  const def = TIPOS_PLACA[type] || TIPOS_PLACA.proceed_forward;
  targetCtx.save();
  targetCtx.translate(x, y);
  targetCtx.rotate(rotation);
  targetCtx.lineWidth = Math.max(2, size * 0.09);
  targetCtx.textAlign = "center";
  targetCtx.textBaseline = "middle";
  if (type === "stop") {
    poligono(targetCtx, 8, size * 0.58);
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

function poligono(targetCtx, sides, radius) {
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

function pareceCodigoRoboPython(source) {
  return /\bimport\s+(cv2|RPi|numpy|sys|time)\b/.test(source)
    || /\bclass\s+AutonomousCar\b/.test(source)
    || /\bGPIO\./.test(source)
    || /\bdef\s+detect_lanes\b/.test(source);
}

function aplicarPerfilRoboPython(source) {
  const getNumber = (name, fallback) => {
    const escaped = name.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
    const match = source.match(new RegExp(`^\\s*${escaped}\\s*=\\s*(-?\\d+(?:\\.\\d+)?)`, "m"));
    return match ? Number(match[1]) : fallback;
  };

  const velBase = getNumber("VEL_BASE", 65);
  const servoMin = getNumber("SERVO_MIN_REAL", -0.5);
  const servoMax = getNumber("SERVO_MAX_REAL", 1.0);
  const servoCenter = getNumber("SERVO_CENTRO", 0.25);
  limiarBranco = getNumber("LINHA_BRANCA_THRESH", getNumber("white_threshold", 185));

  const kp = getNumber("self.KP", getNumber("KP", null));
  const ki = getNumber("self.KI", getNumber("KI", null));
  const kd = getNumber("self.KD", getNumber("KD", null));
  if (kp !== null) interfaceUsuario.kp.value = limitar(kp, Number(interfaceUsuario.kp.min), Number(interfaceUsuario.kp.max));
  if (ki !== null) interfaceUsuario.ki.value = limitar(ki, Number(interfaceUsuario.ki.min), Number(interfaceUsuario.ki.max));
  if (kd !== null) interfaceUsuario.kd.value = limitar(kd, Number(interfaceUsuario.kd.min), Number(interfaceUsuario.kd.max));

  interfaceUsuario.speed.value = limitar((velBase / 100) * 1.2, Number(interfaceUsuario.speed.min), Number(interfaceUsuario.speed.max)).toFixed(2);
  interfaceUsuario.pwmMin.value = Math.round(1500 + servoMin * 500);
  interfaceUsuario.pwmCenter.value = Math.round(1500 + servoCenter * 500);
  interfaceUsuario.pwmMax.value = Math.round(1500 + servoMax * 500);
  interfaceUsuario.controllerMode.value = "rightLine";
  controladorPersonalizado = null;
  sincronizarSaidasSliders();
  resetarCarro();
  interfaceUsuario.codeStatus.textContent = [
    "Python/RPi importado como perfil de simulação.",
    `threshold=${whiteThreshold}, VEL_BASE=${velBase}, servo=[${servoMin}, ${servoCenter}, ${servoMax}]`
  ].join(" ");
  registrarLog("Perfil Python/RPi importado");
}

function vincularSliders() {
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
  for (const [input, output, formatarValor] of pairs) {
    const sync = () => { interfaceUsuario[output].textContent = formatarValor(interfaceUsuario[input].value); };
    interfaceUsuario[input].addEventListener("input", sync);
  }
  sincronizarSaidasSliders();
}

function sincronizarSaidasSliders() {
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
  for (const [input, output, formatarValor] of pairs) {
    interfaceUsuario[output].textContent = formatarValor(interfaceUsuario[input].value);
  }
}

function aplicarPredefinicaoAmbiente() {
  const presets = {
    clean: { light: 1, contrast: 1, noise: 0, shadow: 0 },
    sun: { light: 1.28, contrast: 1.28, noise: 8, shadow: 0.22 },
    indoor: { light: 0.86, contrast: 1.12, noise: 12, shadow: 0.12 },
    dark: { light: 0.48, contrast: 0.82, noise: 24, shadow: 0.35 },
    noisy: { light: 0.88, contrast: 1.05, noise: 52, shadow: 0.18 }
  };
  const preset = presets[interfaceUsuario.environmentPreset.value] || presets.clean;
  interfaceUsuario.envLight.value = preset.light;
  interfaceUsuario.envContrast.value = preset.contrast;
  interfaceUsuario.envNoise.value = preset.noise;
  interfaceUsuario.envShadow.value = preset.shadow;
  sincronizarSaidasSliders();
  registrarLog(`Ambiente: ${ui.environmentPreset.options[ui.environmentPreset.selectedIndex].text}`);
}

interfaceUsuario.runBtn.addEventListener("click", () => { estado.running = true; registrarLog("Simulação rodando"); });
interfaceUsuario.pauseBtn.addEventListener("click", () => { estado.running = false; registrarLog("Simulação pausada"); });
interfaceUsuario.resetBtn.addEventListener("click", resetarCarro);
interfaceUsuario.trackMode.addEventListener("change", resetarCarro);
interfaceUsuario.directionMode.addEventListener("change", resetarCarro);
interfaceUsuario.environmentPreset.addEventListener("change", aplicarPredefinicaoAmbiente);
interfaceUsuario.randomizeSignsBtn.addEventListener("click", sortearPlacas);
interfaceUsuario.clearSignsBtn.addEventListener("click", limparPlacas);
interfaceUsuario.applyCodeBtn.addEventListener("click", compilarCodigoPersonalizado);
interfaceUsuario.restoreCodeBtn.addEventListener("click", () => {
  interfaceUsuario.codeEditor.value = CODIGO_CONTROLADOR_PADRAO;
  controladorPersonalizado = null;
  interfaceUsuario.controllerMode.value = "rightLine";
  resetarCarro();
  interfaceUsuario.codeStatus.textContent = "Controlador padrão restaurado.";
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
interfaceUsuario.trackInput.addEventListener("change", (ev) => {
  const file = ev.target.files?.[0];
  if (!file) return;
  const img = new Image();
  img.onload = () => {
    ctxPista.fillStyle = "#202421";
    ctxPista.fillRect(0, 0, telaPista.width, telaPista.height);
    const scale = Math.min(telaPista.width / img.width, telaPista.height / img.height);
    const iw = img.width * scale;
    const ih = img.height * scale;
    ctxPista.drawImage(img, (telaPista.width - iw) / 2, (telaPista.height - ih) / 2, iw, ih);
    atualizarPixelsPista();
    interfaceUsuario.trackName.textContent = file.name;
    resetarCarro();
    registrarLog(`Imagem carregada: ${file.name}`);
  };
  img.src = URL.createObjectURL(file);
});

telaSimulacao.addEventListener("click", (ev) => {
  const rect = telaSimulacao.getBoundingClientRect();
  const x = (ev.clientX - rect.left) * (telaSimulacao.width / rect.width);
  const y = (ev.clientY - rect.top) * (telaSimulacao.height / rect.height);
  if (interfaceUsuario.signEditMode.checked) {
    adicionarGeradorPlaca(x, y);
    return;
  }
  estado.car.x = x;
  estado.car.y = y;
  estado.path = [];
  registrarLog(`Carro reposicionado: ${estado.car.x.toFixed(0)}, ${estado.car.y.toFixed(0)}`);
});

window.addEventListener("keydown", (ev) => {
  if (ev.key.toLowerCase() === "a") estado.car.heading -= 0.08;
  if (ev.key.toLowerCase() === "d") estado.car.heading += 0.08;
});

window.__simDebug = {
  getState: () => JSON.parse(document.body.dataset.simState || "{}"),
  getInput: () => JSON.parse(JSON.stringify(estado.lastInput)),
  getCar: () => JSON.parse(JSON.stringify(estado.car)),
  snapshot: () => JSON.parse(JSON.stringify({
    time: estado.time,
    running: estado.running,
    car: estado.car,
    speed: estado.currentSpeed,
    activeSign: estado.activeSign,
    maneuver: entradaManobraAtual(),
    headingHold: estado.headingHold,
    lastInput: estado.lastInput,
    handledSigns: [...estado.handledSigns],
    signs: estado.signGenerators
  })),
  run: () => { estado.running = true; },
  pause: () => { estado.running = false; },
  reset: () => resetarCarro(),
  limparPlacas: () => limparPlacas(),
  addSign: (x, y, type = "stop", allowed = [type], approachHeading = estado.car.heading) => {
    estado.signGenerators.push({ id: estado.nextSignId++, x, y, type, allowed, approachHeading });
    salvarGeradoresPlaca();
    atualizarListaPlacas();
  },
  setEnvironment: (preset) => {
    interfaceUsuario.environmentPreset.value = preset;
    aplicarPredefinicaoAmbiente();
  }
};

interfaceUsuario.codeEditor.value = CODIGO_CONTROLADOR_PADRAO;
interfaceUsuario.controllerMode.value = "rightLine";
carregarGeradoresPlaca();
aplicarCenarioDaUrl();
carregarPistaOficial();
vincularSliders();
atualizarListaPlacas();
requestAnimationFrame(quadro);
