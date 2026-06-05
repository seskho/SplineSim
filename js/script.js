let calcPoints = [
    { x: 2.0, y: 3.0 },
    { x: 4.0, y: 1.0 },
    { x: 7.0, y: 5.0 },
    { x: 9.0, y: 2.0 }
];

let calculatedSplines = [];

const calcCanvas = document.getElementById('calculator-canvas');
const calcCtx = calcCanvas.getContext('2d');
let dragPointIndex = -1;
let hoverPointIndex = -1;

let xMin = 0, xMax = 10;
let yMin = 0, yMax = 6;

let droneMissionName = "1";
let droneWaypoints = [];
let droneObstacles = [];
let droneSplines = [];
let dronePathPoints = [];
let isDroneFlying = false;
let droneAnimIndex = 0;
let droneSpeedMultiplier = 3;
let lastFrameTime = 0;

const droneMapCanvas = document.getElementById('drone-map-canvas');
const droneMapCtx = droneMapCanvas ? droneMapCanvas.getContext('2d') : null;

const droneVelCanvas = document.getElementById('drone-vel-canvas');
const droneVelCtx = droneVelCanvas ? droneVelCanvas.getContext('2d') : null;

const droneAccCanvas = document.getElementById('drone-acc-canvas');
const droneAccCtx = droneAccCanvas ? droneAccCanvas.getContext('2d') : null;

// =========================================================================
// VARIABLES PARA MONTAÑA RUSA Y CLIMA
// =========================================================================
let isCoasterRunning = false;
let coasterTrackName = "1";
let coasterWaypoints = [];
let coasterSplines = [];
let coasterPathPoints = [];
let coasterAnimX = 0;
let coasterLastTime = 0;

const coasterMapCanvas = document.getElementById('coaster-map-canvas');
const coasterMapCtx = coasterMapCanvas ? coasterMapCanvas.getContext('2d') : null;

const coasterGCanvas = document.getElementById('coaster-g-canvas');
const coasterGCtx = coasterGCanvas ? coasterGCanvas.getContext('2d') : null;

const coasterJerkCanvas = document.getElementById('coaster-jerk-canvas');
const coasterJerkCtx = coasterJerkCanvas ? coasterJerkCanvas.getContext('2d') : null;

let weatherDatasetName = "1";
let weatherPoints = [];
let weatherSplines = [];

const weatherCanvas = document.getElementById('weather-canvas');
const weatherCtx = weatherCanvas ? weatherCanvas.getContext('2d') : null;

// Misiones predefinidas
const MISIONES_DB = {
    "1": {
        waypoints: [{x: 0, y: 2.0}, {x: 2.0, y: 5.5}, {x: 5.0, y: 1.5}, {x: 7.5, y: 6.0}, {x: 10.0, y: 3.0}],
        obstacles: [
            {x: 1.0, y: 1.2, r: 0.5}, // En el suelo, spline pasa a ~3.8 (por encima)
            {x: 3.5, y: 5.8, r: 0.5}, // En el techo, spline pasa a ~3.5 (por debajo)
            {x: 6.2, y: 1.2, r: 0.5}, // En el suelo, spline pasa a ~3.9 (por encima)
            {x: 8.8, y: 6.8, r: 0.5}  // En el techo, spline pasa a ~5.4 (por debajo) - evitamos colisión
        ]
    },
    "2": {
        waypoints: [{x: 0, y: 1.0}, {x: 2.5, y: 6.0}, {x: 5.0, y: 1.0}, {x: 7.5, y: 6.0}, {x: 10.0, y: 1.0}],
        obstacles: [
            {x: 1.25, y: 4.839, r: 0.4}, // Punto de cultivo 1 (la spline pasa exactamente a y=4.839)
            {x: 3.75, y: 3.232, r: 0.4}, // Punto de cultivo 2 (la spline pasa exactamente a y=3.232)
            {x: 6.25, y: 3.232, r: 0.4}, // Punto de cultivo 3 (la spline pasa exactamente a y=3.232)
            {x: 8.75, y: 4.839, r: 0.4}  // Punto de cultivo 4 (la spline pasa exactamente a y=4.839)
        ]
    },
    "3": {
        waypoints: [{x: 0, y: 3.0}, {x: 3.0, y: 5.0}, {x: 6.0, y: 2.0}, {x: 8.5, y: 4.5}, {x: 10.0, y: 2.5}],
        obstacles: [
            {x: 1.5, y: 4.749, r: 0.4},  // Punto de entrega 1 (la spline pasa exactamente a y=4.749)
            {x: 4.5, y: 3.128, r: 0.4},  // Punto de entrega 2 (la spline pasa exactamente a y=3.128)
            {x: 7.25, y: 3.398, r: 0.4} // Punto de entrega 3 (la spline pasa exactamente a y=3.398)
        ]
    }
};

// Paleta de colores sincronizada con style.css
const COLOR = {
    bg_main: "#0b0f19",
    bg_card: "#1e293b",
    fg_normal: "#e5e7eb",
    fg_muted: "#64748b",
    accent_indigo: "#6366f1",
    accent_blue: "#3b82f6",
    success: "#10b981",
    danger: "#ef4444",
    warning: "#f59e0b",
    grid: "rgba(255, 255, 255, 0.05)",
    axis: "rgba(255, 255, 255, 0.15)"
};

/**
 * Resuelve M * g = b mediante Eliminación Gaussiana con pivoteo parcial.
 */
function solveLinearSystem(M, b) {
    const n = b.length;
    const A = M.map(row => [...row]);
    const y = [...b];

    for (let i = 0; i < n; i++) {
        let maxEl = Math.abs(A[i][i]);
        let maxRow = i;
        for (let k = i + 1; k < n; k++) {
            if (Math.abs(A[k][i]) > maxEl) {
                maxEl = Math.abs(A[k][i]);
                maxRow = k;
            }
        }
        
        const tempRow = A[maxRow];
        A[maxRow] = A[i];
        A[i] = tempRow;

        const tempVal = y[maxRow];
        y[maxRow] = y[i];
        y[i] = tempVal;

        for (let k = i + 1; k < n; k++) {
            if (A[i][i] === 0) continue;
            const factor = -A[k][i] / A[i][i];
            for (let j = i; j < n; j++) {
                if (i === j) {
                    A[k][j] = 0;
                } else {
                    A[k][j] += factor * A[i][j];
                }
            }
            y[k] += factor * y[i];
        }
    }

    const g = new Array(n).fill(0);
    for (let i = n - 1; i >= 0; i--) {
        if (A[i][i] === 0) {
            g[i] = 0;
            continue;
        }
        g[i] = y[i] / A[i][i];
        for (let k = i - 1; k >= 0; k--) {
            y[k] -= A[k][i] * g[i];
        }
    }
    return g;
}

/**
 * Motor de Splines Cúbicos Naturales.
 */
function calculateCubicSplines(pts) {
    const sorted = [...pts].sort((a, b) => a.x - b.x);
    const n = sorted.length - 1;
    
    const h = [];
    for (let i = 0; i < n; i++) {
        h.push(sorted[i+1].x - sorted[i].x);
    }
    
    const v = [];
    for (let i = 0; i < n; i++) {
        v.push((sorted[i+1].y - sorted[i].y) / h[i]);
    }
    
    const b = new Array(n + 1).fill(0);
    for (let i = 1; i < n; i++) {
        b[i] = 6 * (v[i] - v[i-1]);
    }
    
    const M = Array.from({ length: n + 1 }, () => new Array(n + 1).fill(0));
    M[0][0] = 1;
    M[n][n] = 1;
    for (let i = 1; i < n; i++) {
        M[i][i-1] = h[i-1];
        M[i][i] = 2 * (h[i-1] + h[i]);
        M[i][i+1] = h[i];
    }
    
    const g = solveLinearSystem(M, b);
    
    const splines = [];
    for (let i = 0; i < n; i++) {
        const a_coef = sorted[i].y;
        const c_coef = g[i] / 2;
        const d_coef = (g[i+1] - g[i]) / (6 * h[i]);
        const b_coef = ((sorted[i+1].y - sorted[i].y) / h[i]) - (h[i] * (2 * g[i] + g[i+1]) / 6);
        
        splines.push({
            a: a_coef,
            b: b_coef,
            c: c_coef,
            d: d_coef,
            x0: sorted[i].x,
            x1: sorted[i+1].x
        });
    }
    
    return { h, v, b, M, g, splines, sorted };
}

/**
 * Evalúa las splines en un punto x particular.
 */
function evaluateSplinesAt(splines, x) {
    if (splines.length === 0) return null;
    
    let i = 0;
    for (let k = 0; k < splines.length; k++) {
        if (x >= splines[k].x0 && x <= splines[k].x1) {
            i = k;
            if (x < splines[k].x1) break;
        }
    }
    
    if (x < splines[0].x0) i = 0;
    if (x > splines[splines.length - 1].x1) i = splines.length - 1;
    
    const s = splines[i];
    const dx = x - s.x0;
    
    const y = s.a + s.b * dx + s.c * dx * dx + s.d * dx * dx * dx;
    const dy = s.b + 2 * s.c * dx + 3 * s.d * dx * dx;
    const ddy = 2 * s.c + 6 * s.d * dx;
    
    return { y, dy, ddy, tramoIndex: i, x0: s.x0 };
}

/**
 * Interpolación polinómica global de Lagrange
 */
function evaluateLagrangeAt(pts, x) {
    const n = pts.length;
    let sum = 0;
    for (let i = 0; i < n; i++) {
        let term = pts[i].y;
        for (let j = 0; j < n; j++) {
            if (i !== j) {
                term = term * (x - pts[j].x) / (pts[i].x - pts[j].x);
            }
        }
        sum += term;
    }
    return sum;
}

function mathToCanvas(mx, my, width, height) {
    const px = ((mx - xMin) / (xMax - xMin)) * width;
    const py = height - ((my - yMin) / (yMax - yMin)) * height;
    return { x: px, y: py };
}

function canvasToMath(cx, cy, width, height) {
    const mx = xMin + (cx / width) * (xMax - xMin);
    const my = yMin + ((height - cy) / height) * (yMax - yMin);
    return { x: mx, y: my };
}

function resizeCanvasToFit(canvas) {
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (w > 0 && h > 0) {
        canvas.width = w;
        canvas.height = h;
    } else {
        const parent = canvas.parentElement;
        canvas.width = parent.clientWidth || 300;
        canvas.height = parent.clientHeight || 150;
    }
}

function updatePointsTable() {
    const tbody = document.querySelector("#points-table tbody");
    tbody.innerHTML = "";
    
    calcPoints.forEach((p, idx) => {
        const tr = document.createElement("tr");
        tr.innerHTML = `
            <td>P<sub>${idx}</sub></td>
            <td>${p.x.toFixed(2)}</td>
            <td>${p.y.toFixed(2)}</td>
            <td><button class="row-delete-btn" onclick="deletePoint(${idx})">🗑</button></td>
        `;
        tbody.appendChild(tr);
    });
}

function addPointManual() {
    const xInput = document.getElementById("point-x");
    const yInput = document.getElementById("point-y");
    const xVal = parseFloat(xInput.value);
    const yVal = parseFloat(yInput.value);
    
    if (isNaN(xVal) || isNaN(yVal)) {
        alert("Por favor, ingresa números válidos.");
        return;
    }
    
    const duplicate = calcPoints.some(p => Math.abs(p.x - xVal) < 0.01);
    if (duplicate) {
        alert("Ya existe un punto en esa coordenada X. Por favor, usa una diferente.");
        return;
    }
    
    calcPoints.push({ x: xVal, y: yVal });
    calcPoints.sort((a, b) => a.x - b.x);
    
    xInput.value = "";
    yInput.value = "";
    
    recomputeAllCalculator();
}

function deletePoint(idx) {
    calcPoints.splice(idx, 1);
    recomputeAllCalculator();
}

function clearAllPoints() {
    calcPoints = [];
    recomputeAllCalculator();
}

function recomputeAllCalculator() {
    updatePointsTable();
    
    if (calcPoints.length > 0) {
        const xs = calcPoints.map(p => p.x);
        const ys = calcPoints.map(p => p.y);
        
        const minX = Math.min(...xs);
        const maxX = Math.max(...xs);
        const minY = Math.min(...ys);
        const maxY = Math.max(...ys);
        
        const dx = (maxX - minX) || 1.0;
        const dy = (maxY - minY) || 1.0;
        
        xMin = minX - dx * 0.15;
        xMax = maxX + dx * 0.15;
        yMin = minY - dy * 0.2;
        yMax = maxY + dy * 0.2;
        
        if (xMax - xMin < 2) { xMin -= 1; xMax += 1; }
        if (yMax - yMin < 2) { yMin -= 1; yMax += 1; }
    } else {
        xMin = 0; xMax = 10;
        yMin = 0; yMax = 6;
    }
    
    const output = document.getElementById("math-derivation-output");
    
    if (calcPoints.length < 3) {
        calculatedSplines = [];
        output.textContent = `Se requieren al menos 3 puntos de control para deducir Splines Cúbicos Naturales. Puntos actuales: ${calcPoints.length}.`;
        drawCalculator();
        return;
    }
    
    const result = calculateCubicSplines(calcPoints);
    calculatedSplines = result.splines;
    
    let txt = [];
    txt.push("=========================================================================");
    txt.push("   PASO A PASO: DEDUCCIÓN DE SPLINES CÚBICOS NATURALES (ÁLGEBRA LINEAL)  ");
    txt.push("=========================================================================\n");
    
    txt.push("1. DISTANCIAS LOCALES DE NODOS (h_k):");
    result.h.forEach((hVal, i) => {
        txt.push(`   Tramo [x_${i}, x_${i+1}]: h_${i} = ${result.sorted[i+1].x.toFixed(2)} - ${result.sorted[i].x.toFixed(2)} = ${hVal.toFixed(4)}`);
    });
    txt.push(`   Pendientes locales v_k: [${result.v.map(val => val.toFixed(4)).join(", ")}]\n`);
    
    txt.push("2. CONSTRUCCIÓN DE LA MATRIZ DE COEFICIENTES M (Tridiagonal):");
    txt.push("   Fronteras naturales obligan a: g_0 = 0  y  g_n = 0");
    txt.push("   Fórmula para nodos internos i: h_{i-1} * g_{i-1} + 2*(h_{i-1} + h_i)*g_i + h_i*g_{i+1} = 6 * (v_i - v_{i-1})");
    txt.push("   Matriz M:");
    result.M.forEach((row, i) => {
        txt.push(`      [ ` + row.map(v => v.toFixed(3).padStart(7)).join(" ") + ` ]`);
    });
    txt.push("");
    
    txt.push("3. VECTOR DE TÉRMINOS INDEPENDIENTES b:");
    txt.push(`   b = [ ` + result.b.map(val => val.toFixed(4)).join(", ") + ` ]^T\n`);
    
    txt.push("4. RESOLVER EL SISTEMA LINEAL M * g = b PARA g:");
    txt.push("   (Donde g_i representa la segunda derivada S''(x_i) en el nodo)");
    txt.push(`   g = [ ` + result.g.map((val, idx) => `g_${idx}=${val.toFixed(5)}`).join(", ") + ` ]^T\n`);
    
    txt.push("5. DEFINICIÓN DE LOS SPLINES POLINOMIALES POR INTERVALO:");
    txt.push("   S_i(x) = a_i + b_i*(x - x_i) + c_i*(x - x_i)^2 + d_i*(x - x_i)^3");
    result.splines.forEach((s, i) => {
        txt.push(`   • Tramo ${i} [${s.x0.toFixed(1)}, ${s.x1.toFixed(1)}]:`);
        txt.push(`     a_${i} = ${s.a.toFixed(5)}`);
        txt.push(`     b_${i} = ${s.b.toFixed(5)}`);
        txt.push(`     c_${i} = ${s.c.toFixed(5)}`);
        txt.push(`     d_${i} = ${s.d.toFixed(5)}`);
        txt.push(`     Ecuación: S_${i}(x) = ${s.a.toFixed(4)} + ${s.b.toFixed(4)}*(x - ${s.x0.toFixed(1)}) + ${s.c.toFixed(4)}*(x - ${s.x0.toFixed(1)})^2 + ${s.d.toFixed(4)}*(x - ${s.x0.toFixed(1)})^3`);
    });
    
    output.textContent = txt.join("\n");
    
    evaluateCustomX();
    drawCalculator();
}

function drawCalculator() {
    resizeCanvasToFit(calcCanvas);
    const w = calcCanvas.width;
    const h = calcCanvas.height;
    
    calcCtx.fillStyle = COLOR.bg_main;
    calcCtx.fillRect(0, 0, w, h);
    
    calcCtx.strokeStyle = COLOR.grid;
    calcCtx.lineWidth = 1;
    calcCtx.fillStyle = COLOR.fg_muted;
    calcCtx.font = "10px Outfit";
    
    const stepX = (xMax - xMin) / 10;
    for (let i = 0; i <= 10; i++) {
        const mx = xMin + i * stepX;
        const c = mathToCanvas(mx, 0, w, h);
        calcCtx.beginPath();
        calcCtx.moveTo(c.x, 0);
        calcCtx.lineTo(c.x, h);
        calcCtx.stroke();
        calcCtx.fillText(mx.toFixed(1), c.x + 2, h - 5);
    }
    
    const stepY = (yMax - yMin) / 8;
    for (let i = 0; i <= 8; i++) {
        const my = yMin + i * stepY;
        const c = mathToCanvas(0, my, w, h);
        calcCtx.beginPath();
        calcCtx.moveTo(0, c.y);
        calcCtx.lineTo(w, c.y);
        calcCtx.stroke();
        calcCtx.fillText(my.toFixed(1), 5, c.y - 4);
    }
    
    calcCtx.strokeStyle = COLOR.axis;
    calcCtx.lineWidth = 1.5;
    if (xMin < 0 && xMax > 0) {
        const c0 = mathToCanvas(0, 0, w, h);
        calcCtx.beginPath(); calcCtx.moveTo(c0.x, 0); calcCtx.lineTo(c0.x, h); calcCtx.stroke();
    }
    if (yMin < 0 && yMax > 0) {
        const c0 = mathToCanvas(0, 0, w, h);
        calcCtx.beginPath(); calcCtx.moveTo(0, c0.y); calcCtx.lineTo(w, c0.y); calcCtx.stroke();
    }
    
    if (calcPoints.length < 2) return;
    
    const showSpline = document.getElementById("show-spline-check").checked;
    const showLagrange = document.getElementById("show-lagrange-check").checked;
    
    if (showLagrange && calcPoints.length >= 2) {
        calcCtx.beginPath();
        calcCtx.strokeStyle = COLOR.danger;
        calcCtx.lineWidth = 1.5;
        calcCtx.setLineDash([5, 5]);
        
        for (let px = 0; px < w; px++) {
            const mx = canvasToMath(px, 0, w, h).x;
            if (mx >= calcPoints[0].x && mx <= calcPoints[calcPoints.length - 1].x) {
                const my = evaluateLagrangeAt(calcPoints, mx);
                const cy = mathToCanvas(mx, my, w, h).y;
                if (px === 0 || mx === calcPoints[0].x) {
                    calcCtx.moveTo(px, cy);
                } else {
                    calcCtx.lineTo(px, cy);
                }
            }
        }
        calcCtx.stroke();
        calcCtx.setLineDash([]);
    }
    
    if (showSpline && calculatedSplines.length > 0) {
        calcCtx.beginPath();
        calcCtx.strokeStyle = COLOR.accent_blue;
        calcCtx.lineWidth = 3;
        
        for (let px = 0; px < w; px++) {
            const mx = canvasToMath(px, 0, w, h).x;
            if (mx >= calcPoints[0].x && mx <= calcPoints[calcPoints.length - 1].x) {
                const evalRes = evaluateSplinesAt(calculatedSplines, mx);
                if (evalRes) {
                    const cy = mathToCanvas(mx, evalRes.y, w, h).y;
                    if (px === 0 || mx === calcPoints[0].x) {
                        calcCtx.moveTo(px, cy);
                    } else {
                        calcCtx.lineTo(px, cy);
                    }
                }
            }
        }
        calcCtx.stroke();
    }
    
    const evalXInput = document.getElementById("eval-x");
    const evalX = parseFloat(evalXInput.value);
    if (!isNaN(evalX) && calculatedSplines.length > 0 && evalX >= calcPoints[0].x && evalX <= calcPoints[calcPoints.length - 1].x) {
        const evalRes = evaluateSplinesAt(calculatedSplines, evalX);
        if (evalRes) {
            const cPos = mathToCanvas(evalX, evalRes.y, w, h);
            
            calcCtx.beginPath();
            calcCtx.arc(cPos.x, cPos.y, 10, 0, 2 * Math.PI);
            calcCtx.fillStyle = "rgba(245, 158, 11, 0.35)";
            calcCtx.fill();
            
            calcCtx.beginPath();
            calcCtx.arc(cPos.x, cPos.y, 5, 0, 2 * Math.PI);
            calcCtx.fillStyle = COLOR.warning;
            calcCtx.strokeStyle = "white";
            calcCtx.lineWidth = 1.5;
            calcCtx.fill();
            calcCtx.stroke();
        }
    }
    
    calcPoints.forEach((p, idx) => {
        const c = mathToCanvas(p.x, p.y, w, h);
        const isHover = (idx === hoverPointIndex);
        const isDragging = (idx === dragPointIndex);
        
        calcCtx.beginPath();
        calcCtx.arc(c.x, c.y, (isHover || isDragging) ? 8 : 6, 0, 2 * Math.PI);
        calcCtx.fillStyle = (isDragging) ? COLOR.accent_indigo : "white";
        calcCtx.strokeStyle = COLOR.accent_indigo;
        calcCtx.lineWidth = 2.5;
        calcCtx.fill();
        calcCtx.stroke();
        
        calcCtx.fillStyle = COLOR.fg_normal;
        calcCtx.fillText(`P${idx}`, c.x - 8, c.y - 12);
    });
}

function evaluateCustomX() {
    const input = document.getElementById("eval-x");
    const outY = document.getElementById("eval-res-y");
    const outDy = document.getElementById("eval-res-dy");
    const outDdy = document.getElementById("eval-res-ddy");
    const outEq = document.getElementById("eval-res-equation");
    
    const xVal = parseFloat(input.value);
    
    if (isNaN(xVal) || calculatedSplines.length === 0) {
        outY.textContent = "--";
        outDy.textContent = "--";
        outDdy.textContent = "--";
        outEq.textContent = "--";
        return;
    }
    
    if (xVal < calcPoints[0].x || xVal > calcPoints[calcPoints.length - 1].x) {
        outY.textContent = "Fuera de Rango";
        outY.style.color = COLOR.danger;
        outDy.textContent = "--";
        outDdy.textContent = "--";
        outEq.textContent = "El valor x debe estar en el intervalo [" + calcPoints[0].x.toFixed(2) + ", " + calcPoints[calcPoints.length - 1].x.toFixed(2) + "]";
        return;
    }
    
    outY.style.color = COLOR.success;
    const res = evaluateSplinesAt(calculatedSplines, xVal);
    
    outY.textContent = res.y.toFixed(5);
    outDy.textContent = res.dy.toFixed(5) + " m/s";
    outDdy.textContent = res.ddy.toFixed(5) + " m/s²";
    
    const s = calculatedSplines[res.tramoIndex];
    outEq.innerHTML = `S<sub>${res.tramoIndex}</sub>(x) = ${s.a.toFixed(3)} + ${s.b.toFixed(3)}*(x - ${s.x0.toFixed(1)}) + ${s.c.toFixed(3)}*(x - ${s.x0.toFixed(1)})² + ${s.d.toFixed(3)}*(x - ${s.x0.toFixed(1)})³`;
    
    drawCalculator();
}

// =========================================================================
// GESTIÓN DE EVENTOS DE MOUSE - DRAG & DROP & DOUBLE CLICK
// =========================================================================

calcCanvas.addEventListener('mousedown', (e) => {
    if (calcPoints.length === 0) return;
    
    const rect = calcCanvas.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;
    
    const w = calcCanvas.width;
    const h = calcCanvas.height;
    
    const hitRadius = 14;
    dragPointIndex = -1;
    
    calcPoints.forEach((p, idx) => {
        const c = mathToCanvas(p.x, p.y, w, h);
        const dist = Math.hypot(c.x - cx, c.y - cy);
        if (dist < hitRadius) {
            dragPointIndex = idx;
        }
    });
    
    if (dragPointIndex !== -1) {
        drawCalculator();
    }
});

calcCanvas.addEventListener('mousemove', (e) => {
    const rect = calcCanvas.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;
    
    const w = calcCanvas.width;
    const h = calcCanvas.height;
    
    const hitRadius = 14;
    let foundHover = -1;
    calcPoints.forEach((p, idx) => {
        const c = mathToCanvas(p.x, p.y, w, h);
        if (Math.hypot(c.x - cx, c.y - cy) < hitRadius) {
            foundHover = idx;
        }
    });
    
    if (foundHover !== hoverPointIndex) {
        hoverPointIndex = foundHover;
        calcCanvas.style.cursor = (hoverPointIndex !== -1) ? 'ns-resize' : 'default';
        drawCalculator();
    }
    
    if (dragPointIndex !== -1) {
        const mPos = canvasToMath(cx, cy, w, h);
        calcPoints[dragPointIndex].y = mPos.y;
        
        recomputeAllCalculator();
    }
});

calcCanvas.addEventListener('mouseup', () => {
    if (dragPointIndex !== -1) {
        dragPointIndex = -1;
        drawCalculator();
    }
});

calcCanvas.addEventListener('dblclick', (e) => {
    const rect = calcCanvas.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;
    
    const w = calcCanvas.width;
    const h = calcCanvas.height;
    
    const mPos = canvasToMath(cx, cy, w, h);
    
    const umbral = 0.25;
    const isClose = calcPoints.some(p => Math.abs(p.x - mPos.x) < umbral);
    
    if (!isClose) {
        calcPoints.push({ x: mPos.x, y: mPos.y });
        calcPoints.sort((a, b) => a.x - b.x);
        recomputeAllCalculator();
    }
});

// =========================================================================
// PESTAÑA 2: SIMULADOR DE DRON Y CONTINUIDAD C2
// =========================================================================

function loadSelectedMission() {
    isDroneFlying = false;
    document.getElementById("btn-drone-start").textContent = "▶ Iniciar Vuelo";
    document.getElementById("btn-drone-start").className = "btn btn-success";
    
    const mType = document.getElementById("mission-select").value;
    droneMissionName = mType;
    
    if (mType === "custom") {
        if (calcPoints.length < 3) {
            alert("Por favor, agrega al menos 3 puntos en la calculadora de la Pestaña 1 primero.");
            document.getElementById("mission-select").value = "1";
            loadSelectedMission();
            return;
        }
        droneWaypoints = calcPoints.map(p => ({ ...p }));
        droneObstacles = [];
    } else {
        const m = MISIONES_DB[mType];
        droneWaypoints = m.waypoints.map(p => ({ ...p }));
        droneObstacles = m.obstacles.map(o => ({ ...o }));
    }
    
    // Resolver spline de la ruta del dron
    const result = calculateCubicSplines(droneWaypoints);
    droneSplines = result.splines;
    
    // Generar 300 puntos finos para trazar las curvas y correr la animación
    dronePathPoints = [];
    const minX = droneWaypoints[0].x;
    const maxX = droneWaypoints[droneWaypoints.length - 1].x;
    
    const count = 300;
    const step = (maxX - minX) / (count - 1);
    
    for (let i = 0; i < count; i++) {
        const x = minX + i * step;
        const evalRes = evaluateSplinesAt(droneSplines, x);
        dronePathPoints.push({
            x: x,
            y: evalRes.y,
            dy: evalRes.dy,
            ddy: evalRes.ddy
        });
    }
    
    droneAnimIndex = 0;
    drawDroneMission();
}

function drawDroneMission() {
    resizeCanvasToFit(droneMapCanvas);
    const w = droneMapCanvas.width;
    const h = droneMapCanvas.height;
    
    droneMapCtx.fillStyle = COLOR.bg_main;
    droneMapCtx.fillRect(0, 0, w, h);
    
    const currentIdx = Math.min(droneAnimIndex, dronePathPoints.length - 1);
    const dPos = dronePathPoints[currentIdx];
    
    // Definir los límites del mapa del dron (0 a 10 en X, 0 a 7 en Y)
    const dXMin = -0.5, dXMax = 10.5;
    const dYMin = -0.5, dYMax = 7.5;
    
    function dMathToCanvas(mx, my) {
        const px = ((mx - dXMin) / (dXMax - dXMin)) * w;
        const py = h - ((my - dYMin) / (dYMax - dYMin)) * h;
        return { x: px, y: py };
    }
    
    // 1. Dibujar cuadricula del cielo
    droneMapCtx.strokeStyle = COLOR.grid;
    droneMapCtx.lineWidth = 1;
    for (let x = 0; x <= 10; x++) {
        const c1 = dMathToCanvas(x, dYMin);
        const c2 = dMathToCanvas(x, dYMax);
        droneMapCtx.beginPath(); droneMapCtx.moveTo(c1.x, c1.y); droneMapCtx.lineTo(c2.x, c2.y); droneMapCtx.stroke();
    }
    for (let y = 0; y <= 7; y++) {
        const c1 = dMathToCanvas(dXMin, y);
        const c2 = dMathToCanvas(dXMax, y);
        droneMapCtx.beginPath(); droneMapCtx.moveTo(c1.x, c1.y); droneMapCtx.lineTo(c2.x, c2.y); droneMapCtx.stroke();
    }
    
    // 2. Dibujar obstáculos y zonas objetivo
    droneObstacles.forEach(obs => {
        const cCenter = dMathToCanvas(obs.x, obs.y);
        const cRadius = (obs.r / (dXMax - dXMin)) * w;
        
        let strokeColor = COLOR.danger;
        let fillColor = "rgba(239, 68, 68, 0.15)";
        let emoji = "⚠️";
        
        if (droneMissionName === "2") {
            strokeColor = COLOR.success;
            fillColor = "rgba(16, 185, 129, 0.15)";
            emoji = "🌱";
        } else if (droneMissionName === "3") {
            strokeColor = COLOR.accent_blue;
            fillColor = "rgba(59, 130, 246, 0.15)";
            emoji = "📦";
        }
        
        // Círculo base
        droneMapCtx.beginPath();
        droneMapCtx.arc(cCenter.x, cCenter.y, cRadius, 0, 2 * Math.PI);
        droneMapCtx.fillStyle = fillColor;
        droneMapCtx.strokeStyle = strokeColor;
        droneMapCtx.lineWidth = 2;
        droneMapCtx.fill();
        droneMapCtx.stroke();
        
        // Icono Emoji central
        droneMapCtx.save();
        droneMapCtx.font = "16px Outfit, sans-serif";
        droneMapCtx.textAlign = "center";
        droneMapCtx.textBaseline = "middle";
        droneMapCtx.fillText(emoji, cCenter.x, cCenter.y);
        droneMapCtx.restore();
        
        // Badge de completado (check verde) para cultivos y entregas
        const isPassed = dPos && dPos.x >= obs.x;
        if ((droneMissionName === "2" || droneMissionName === "3") && isPassed) {
            const badgeX = cCenter.x + cRadius * 0.7;
            const badgeY = cCenter.y - cRadius * 0.7;
            
            droneMapCtx.beginPath();
            droneMapCtx.arc(badgeX, badgeY, 6, 0, 2 * Math.PI);
            droneMapCtx.fillStyle = COLOR.success;
            droneMapCtx.fill();
            
            droneMapCtx.fillStyle = "white";
            droneMapCtx.font = "bold 9px Outfit, sans-serif";
            droneMapCtx.textAlign = "center";
            droneMapCtx.textBaseline = "middle";
            droneMapCtx.fillText("✓", badgeX, badgeY);
        }
    });
    
    // 3. Dibujar camino completo del spline (gris/indigo tenue)
    if (dronePathPoints.length > 1) {
        droneMapCtx.beginPath();
        droneMapCtx.strokeStyle = "rgba(99, 102, 241, 0.35)"; // Indigo translúcido
        droneMapCtx.lineWidth = 3.5;
        
        const cStart = dMathToCanvas(dronePathPoints[0].x, dronePathPoints[0].y);
        droneMapCtx.moveTo(cStart.x, cStart.y);
        
        for (let i = 1; i < dronePathPoints.length; i++) {
            const cNode = dMathToCanvas(dronePathPoints[i].x, dronePathPoints[i].y);
            droneMapCtx.lineTo(cNode.x, cNode.y);
        }
        droneMapCtx.stroke();
    }
    
    // 4. Dibujar camino recorrido (verde success)
    if (droneAnimIndex > 0 && dronePathPoints.length > 1) {
        droneMapCtx.beginPath();
        droneMapCtx.strokeStyle = COLOR.success;
        droneMapCtx.lineWidth = 3.5;
        
        const cStart = dMathToCanvas(dronePathPoints[0].x, dronePathPoints[0].y);
        droneMapCtx.moveTo(cStart.x, cStart.y);
        
        const limit = Math.min(droneAnimIndex, dronePathPoints.length - 1);
        for (let i = 1; i <= limit; i++) {
            const cNode = dMathToCanvas(dronePathPoints[i].x, dronePathPoints[i].y);
            droneMapCtx.lineTo(cNode.x, cNode.y);
        }
        droneMapCtx.stroke();
    }
    
    // 5. Dibujar los Nódulos de la Ruta (Waypoints)
    droneWaypoints.forEach((wp, idx) => {
        const c = dMathToCanvas(wp.x, wp.y);
        droneMapCtx.beginPath();
        droneMapCtx.arc(c.x, c.y, 6, 0, 2 * Math.PI);
        droneMapCtx.fillStyle = "white";
        droneMapCtx.strokeStyle = COLOR.accent_indigo;
        droneMapCtx.lineWidth = 2.5;
        droneMapCtx.fill();
        droneMapCtx.stroke();
        
        droneMapCtx.fillStyle = COLOR.fg_normal;
        droneMapCtx.font = "bold 9px Outfit";
        droneMapCtx.fillText(`W${idx}`, c.x - 7, c.y - 11);
    });
    
    // 6. Dibujar el Dron
    if (dPos) {
        const cDrone = dMathToCanvas(dPos.x, dPos.y);
        drawDroneIcon(droneMapCtx, cDrone.x, cDrone.y, 16);
        drawDroneKinematics(dPos.x);
    }
}

function drawDroneIcon(ctx, cx, cy, size) {
    ctx.strokeStyle = "white";
    ctx.lineWidth = 3.5;
    ctx.beginPath();
    ctx.moveTo(cx - size, cy - size/3);
    ctx.lineTo(cx + size, cy + size/3);
    ctx.moveTo(cx - size, cy + size/3);
    ctx.lineTo(cx + size, cy - size/3);
    ctx.stroke();
    
    const rotWidth = 6;
    ctx.lineWidth = 2.5;
    
    ctx.strokeStyle = COLOR.danger;
    ctx.beginPath();
    ctx.moveTo(cx - size - rotWidth, cy - size/3); ctx.lineTo(cx - size + rotWidth, cy - size/3);
    ctx.moveTo(cx - size - rotWidth, cy + size/3); ctx.lineTo(cx - size + rotWidth, cy + size/3);
    ctx.stroke();
    
    ctx.strokeStyle = COLOR.success;
    ctx.beginPath();
    ctx.moveTo(cx + size - rotWidth, cy - size/3); ctx.lineTo(cx + size + rotWidth, cy - size/3);
    ctx.moveTo(cx + size - rotWidth, cy + size/3); ctx.lineTo(cx + size + rotWidth, cy + size/3);
    ctx.stroke();
    
    ctx.beginPath();
    ctx.arc(cx, cy, 6, 0, 2 * Math.PI);
    ctx.fillStyle = COLOR.accent_blue;
    ctx.strokeStyle = "white";
    ctx.lineWidth = 1.5;
    ctx.fill();
    ctx.stroke();
}

/**
 * Renderiza los gráficos cinemáticos dinámicos laterales
 */
function drawDroneKinematics(currentMathX) {
    resizeCanvasToFit(droneVelCanvas);
    resizeCanvasToFit(droneAccCanvas);
    
    const w = droneVelCanvas.width;
    const h1 = droneVelCanvas.height;
    const h2 = droneAccCanvas.height;
    
    const minX = droneWaypoints[0].x;
    const maxX = droneWaypoints[droneWaypoints.length - 1].x;
    
    // Encontrar límites de velocidad (dy) y aceleración (ddy) para escalar
    const slopes = dronePathPoints.map(p => p.dy);
    const curvatures = dronePathPoints.map(p => p.ddy);
    
    const minVel = Math.min(...slopes, -1.0) - 0.2;
    const maxVel = Math.max(...slopes, 1.0) + 0.2;
    
    const minAcc = Math.min(...curvatures, -1.0) - 0.2;
    const maxAcc = Math.max(...curvatures, 1.0) + 0.2;
    
    function scaleX(mx) {
        return ((mx - minX) / (maxX - minX)) * w;
    }
    
    function scaleVel(my) {
        return h1 - ((my - minVel) / (maxVel - minVel)) * h1;
    }
    
    function scaleAcc(my) {
        return h2 - ((my - minAcc) / (maxAcc - minAcc)) * h2;
    }
    
    // ------------------------------------
    // DIBUJAR VELOCIDAD Y'(X)
    // ------------------------------------
    droneVelCtx.fillStyle = COLOR.bg_main;
    droneVelCtx.fillRect(0, 0, w, h1);
    
    // Grilla velocidad
    droneVelCtx.strokeStyle = COLOR.grid;
    droneVelCtx.lineWidth = 1;
    for (let val = Math.ceil(minVel); val <= maxVel; val += (maxVel-minVel)/4) {
        const cy = scaleVel(val);
        droneVelCtx.beginPath(); droneVelCtx.moveTo(0, cy); droneVelCtx.lineTo(w, cy); droneVelCtx.stroke();
    }
    
    // Línea horizontal Y = 0 (Eje neutro)
    droneVelCtx.strokeStyle = "rgba(255, 255, 255, 0.15)";
    const cVel0 = scaleVel(0);
    droneVelCtx.beginPath(); droneVelCtx.moveTo(0, cVel0); droneVelCtx.lineTo(w, cVel0); droneVelCtx.stroke();
    
    // Trazar curva de velocidad (Verde esmeralda)
    if (dronePathPoints.length > 1) {
        droneVelCtx.beginPath();
        droneVelCtx.strokeStyle = COLOR.success;
        droneVelCtx.lineWidth = 2;
        droneVelCtx.moveTo(scaleX(dronePathPoints[0].x), scaleVel(dronePathPoints[0].dy));
        for (let i = 1; i < dronePathPoints.length; i++) {
            droneVelCtx.lineTo(scaleX(dronePathPoints[i].x), scaleVel(dronePathPoints[i].dy));
        }
        droneVelCtx.stroke();
    }
    
    // Línea vertical móvil (Posición actual del dron)
    const pxMarker = scaleX(currentMathX);
    droneVelCtx.strokeStyle = COLOR.warning;
    droneVelCtx.lineWidth = 1.5;
    droneVelCtx.setLineDash([3, 3]);
    droneVelCtx.beginPath(); droneVelCtx.moveTo(pxMarker, 0); droneVelCtx.lineTo(pxMarker, h1); droneVelCtx.stroke();
    droneVelCtx.setLineDash([]);
    
    // ------------------------------------
    // DIBUJAR ACELERACIÓN Y''(X)
    // ------------------------------------
    droneAccCtx.fillStyle = COLOR.bg_main;
    droneAccCtx.fillRect(0, 0, w, h2);
    
    // Grilla aceleración
    droneAccCtx.strokeStyle = COLOR.grid;
    droneAccCtx.lineWidth = 1;
    for (let val = Math.ceil(minAcc); val <= maxAcc; val += (maxAcc-minAcc)/4) {
        const cy = scaleAcc(val);
        droneAccCtx.beginPath(); droneAccCtx.moveTo(0, cy); droneAccCtx.lineTo(w, cy); droneAccCtx.stroke();
    }
    
    // Línea horizontal Y = 0 (Eje neutro)
    droneAccCtx.strokeStyle = "rgba(255, 255, 255, 0.15)";
    const cAcc0 = scaleAcc(0);
    droneAccCtx.beginPath(); droneAccCtx.moveTo(0, cAcc0); droneAccCtx.lineTo(w, cAcc0); droneAccCtx.stroke();
    
    // Trazar curva de aceleración (Rojo carmesí - Poligonal continua)
    if (dronePathPoints.length > 1) {
        droneAccCtx.beginPath();
        droneAccCtx.strokeStyle = COLOR.danger;
        droneAccCtx.lineWidth = 2;
        droneAccCtx.moveTo(scaleX(dronePathPoints[0].x), scaleAcc(dronePathPoints[0].ddy));
        for (let i = 1; i < dronePathPoints.length; i++) {
            droneAccCtx.lineTo(scaleX(dronePathPoints[i].x), scaleAcc(dronePathPoints[i].ddy));
        }
        droneAccCtx.stroke();
    }
    
    // Línea vertical móvil (Posición actual del dron)
    droneAccCtx.strokeStyle = COLOR.warning;
    droneAccCtx.lineWidth = 1.5;
    droneAccCtx.setLineDash([3, 3]);
    droneAccCtx.beginPath(); droneAccCtx.moveTo(pxMarker, 0); droneAccCtx.lineTo(pxMarker, h2); droneAccCtx.stroke();
    droneAccCtx.setLineDash([]);
}

function toggleDroneFlight() {
    const btn = document.getElementById("btn-drone-start");
    
    if (isDroneFlying) {
        isDroneFlying = false;
        btn.textContent = "▶ Continuar Vuelo";
        btn.className = "btn btn-success";
    } else {
        isDroneFlying = true;
        btn.textContent = "⏸ Pausar Vuelo";
        btn.className = "btn btn-warning";
        lastFrameTime = performance.now();
        requestAnimationFrame(droneFlightLoop);
    }
}

function resetDroneFlight() {
    isDroneFlying = false;
    droneAnimIndex = 0;
    
    const btn = document.getElementById("btn-drone-start");
    btn.textContent = "▶ Iniciar Vuelo";
    btn.className = "btn btn-success";
    
    const p0 = dronePathPoints[0];
    document.getElementById("hud-pos").textContent = `X: ${p0.x.toFixed(2)}, Y: ${p0.y.toFixed(2)}`;
    document.getElementById("hud-vel").textContent = `${p0.dy.toFixed(2)} m/s`;
    document.getElementById("hud-acc").textContent = `${p0.ddy.toFixed(2)} m/s²`;
    
    drawDroneMission();
}

function updateDroneSpeed() {
    const slider = document.getElementById("drone-speed-slider");
    droneSpeedMultiplier = parseInt(slider.value);
}

function droneFlightLoop(timestamp) {
    if (!isDroneFlying) return;
    
    droneAnimIndex += droneSpeedMultiplier;
    
    if (droneAnimIndex >= dronePathPoints.length) {
        droneAnimIndex = dronePathPoints.length - 1;
        drawDroneMission();
        
        isDroneFlying = false;
        const btn = document.getElementById("btn-drone-start");
        btn.textContent = "▶ Re-iniciar Vuelo";
        btn.className = "btn btn-success";
        
        alert("¡Misión Completada! El dron ha llegado a su destino con aterrizaje suave.");
        droneAnimIndex = 0;
        return;
    }
    
    const dPos = dronePathPoints[droneAnimIndex];
    document.getElementById("hud-pos").textContent = `X: ${dPos.x.toFixed(2)}, Y: ${dPos.y.toFixed(2)}`;
    document.getElementById("hud-vel").textContent = `${dPos.dy.toFixed(2)} m/s`;
    document.getElementById("hud-acc").textContent = `${dPos.ddy.toFixed(2)} m/s²`;
    
    drawDroneMission();
    
    requestAnimationFrame(droneFlightLoop);
}

// =========================================================================
// DATABASES DE MONTAÑA RUSA Y CLIMA
// =========================================================================
const TRACKS_DB = {
    "1": { // Camelback
        waypoints: [{x: 0, y: 6.5}, {x: 2.2, y: 1.0}, {x: 5.0, y: 4.8}, {x: 7.8, y: 1.2}, {x: 10.0, y: 3.5}]
    },
    "2": { // Looping (Simulado por cresta pronunciada)
        waypoints: [{x: 0, y: 6.8}, {x: 2.2, y: 1.2}, {x: 4.5, y: 5.5}, {x: 6.8, y: 1.2}, {x: 10.0, y: 2.5}]
    },
    "3": { // Suave
        waypoints: [{x: 0, y: 4.5}, {x: 3.0, y: 2.2}, {x: 6.0, y: 4.8}, {x: 8.5, y: 1.8}, {x: 10.0, y: 2.5}]
    }
};

const WEATHER_DB = {
    "1": {
        points: [
            {x: 0, y: 12.0, note: "Medianoche - Frío estable"},
            {x: 4, y: 8.0, note: "Madrugada - Mínima diaria"},
            {x: 8, y: 15.0, note: "Mañana - Calentamiento rápido"},
            {x: 12, y: 24.0, note: "Mediodía - Máxima térmica"},
            {x: 16, y: 22.0, note: "Tarde - Descenso paulatino"},
            {x: 20, y: 16.0, note: "Noche - Radiación terrestre"},
            {x: 24, y: 11.5, note: "Medianoche - Cierre de ciclo"}
        ]
    },
    "2": {
        points: [
            {x: 0, y: 18.0, note: "Madrugada templada"},
            {x: 4, y: 17.2, note: "Pre-frontal nublado"},
            {x: 8, y: 15.0, note: "Llegada del viento sur"},
            {x: 12, y: 9.0, note: "Lluvia intensa y caída brusca"},
            {x: 16, y: 5.0, note: "Helada vespertina"},
            {x: 20, y: 2.0, note: "Noche gélida"},
            {x: 24, y: 0.5, note: "Congelación superficial"}
        ]
    },
    "3": {
        points: [
            {x: 0, y: 16.0, note: "Ambiente controlado"},
            {x: 4, y: 15.4, note: "Leve oscilación fría"},
            {x: 8, y: 16.2, note: "Ventilación activa"},
            {x: 12, y: 16.8, note: "Temperatura objetivo"},
            {x: 16, y: 16.5, note: "Estabilidad térmica"},
            {x: 20, y: 16.0, note: "Cierre de escotillas"},
            {x: 24, y: 15.8, note: "Noche estable"}
        ]
    }
};

// =========================================================================
// GESTOR DE NAVEGACIÓN Y SUB-APLICACIONES
// =========================================================================
function switchTab(tabId) {
    document.querySelectorAll(".tab-section").forEach(s => s.classList.remove("active"));
    document.querySelectorAll(".tab-btn").forEach(b => b.classList.remove("active"));
    
    isDroneFlying = false;
    isCoasterRunning = false;
    
    if (tabId === 'calculadora') {
        document.getElementById("tab-calculadora").classList.add("active");
        document.getElementById("tab-calc-btn").classList.add("active");
        setTimeout(() => recomputeAllCalculator(), 10);
    } else {
        document.getElementById("tab-aplicaciones").classList.add("active");
        document.getElementById("tab-apps-btn").classList.add("active");
        setTimeout(() => switchSubApp('dron'), 10);
    }
}

function switchSubApp(appId) {
    document.querySelectorAll(".sub-app-container").forEach(c => c.classList.remove("active"));
    document.querySelectorAll(".app-sub-btn").forEach(b => b.classList.remove("active"));
    
    isDroneFlying = false;
    isCoasterRunning = false;
    
    if (appId === 'dron') {
        document.getElementById("sub-app-dron").classList.add("active");
        document.getElementById("btn-sub-dron").classList.add("active");
        setTimeout(() => loadSelectedMission(), 10);
    } else if (appId === 'coaster') {
        document.getElementById("sub-app-coaster").classList.add("active");
        document.getElementById("btn-sub-coaster").classList.add("active");
        setTimeout(() => loadSelectedTrack(), 10);
    } else if (appId === 'clima') {
        document.getElementById("sub-app-clima").classList.add("active");
        document.getElementById("btn-sub-clima").classList.add("active");
        setTimeout(() => loadSelectedWeather(), 10);
    }
}

// =========================================================================
// CONTROLADORES DE SIMULACIÓN DE MONTAÑA RUSA
// =========================================================================
function loadSelectedTrack() {
    isCoasterRunning = false;
    document.getElementById("btn-coaster-start").textContent = "▶ Iniciar Recorrido";
    document.getElementById("btn-coaster-start").className = "btn btn-success";
    
    const trackVal = document.getElementById("track-select").value;
    coasterTrackName = trackVal;
    
    const track = TRACKS_DB[trackVal];
    coasterWaypoints = track.waypoints.map(p => ({ ...p }));
    
    const result = calculateCubicSplines(coasterWaypoints);
    coasterSplines = result.splines;
    
    // Generar puntos densos del recorrido
    coasterPathPoints = [];
    const minX = coasterWaypoints[0].x;
    const maxX = coasterWaypoints[coasterWaypoints.length - 1].x;
    const count = 300;
    const step = (maxX - minX) / (count - 1);
    
    for (let i = 0; i < count; i++) {
        const x = minX + i * step;
        const evalRes = evaluateSplinesAt(coasterSplines, x);
        
        // Curvatura: k = y'' / (1 + y'^2)^1.5
        const dy = evalRes.dy;
        const ddy = evalRes.ddy;
        const curvature = ddy / Math.pow(1 + dy * dy, 1.5);
        
        coasterPathPoints.push({
            x: x,
            y: evalRes.y,
            dy: dy,
            ddy: ddy,
            curvature: curvature
        });
    }
    
    coasterAnimX = minX;
    coasterLastTime = 0;
    resetCoasterAnimation();
}

function resetCoasterAnimation() {
    isCoasterRunning = false;
    coasterAnimX = coasterWaypoints[0].x;
    coasterLastTime = 0;
    
    const btn = document.getElementById("btn-coaster-start");
    btn.textContent = "▶ Iniciar Recorrido";
    btn.className = "btn btn-success";
    
    const startY = coasterWaypoints[0].y;
    document.getElementById("hud-coaster-height").textContent = `${startY.toFixed(2)} m`;
    document.getElementById("hud-coaster-speed").textContent = `0.00 m/s`;
    document.getElementById("hud-coaster-gforce").textContent = `1.00 G`;
    document.getElementById("hud-coaster-jerk").textContent = `0.00 m/s³`;
    
    drawCoaster();
}

function toggleCoasterAnimation() {
    const btn = document.getElementById("btn-coaster-start");
    if (isCoasterRunning) {
        isCoasterRunning = false;
        btn.textContent = "▶ Continuar Recorrido";
        btn.className = "btn btn-success";
    } else {
        isCoasterRunning = true;
        btn.textContent = "⏸ Pausar Recorrido";
        btn.className = "btn btn-warning";
        coasterLastTime = performance.now();
        requestAnimationFrame(coasterAnimationLoop);
    }
}

function coasterAnimationLoop(timestamp) {
    if (!isCoasterRunning) return;
    
    if (!coasterLastTime) coasterLastTime = timestamp;
    let dt = (timestamp - coasterLastTime) / 1000;
    // Limitar dt para evitar saltos enormes por pérdida de foco
    if (dt > 0.1) dt = 0.1;
    coasterLastTime = timestamp;
    
    const minX = coasterWaypoints[0].x;
    const maxX = coasterWaypoints[coasterWaypoints.length - 1].x;
    
    const evalRes = evaluateSplinesAt(coasterSplines, coasterAnimX);
    if (!evalRes) return;
    
    // Física del Carrito: Conservación de Energía
    // v = sqrt(v_start^2 + 2*g*(y_start - y))
    const yStart = coasterWaypoints[0].y;
    const yVal = evalRes.y;
    const g = 9.81;
    const vStart = 3.0; // Velocidad de impulso inicial
    let speed = Math.max(2.0, Math.sqrt(vStart * vStart + 2 * g * (yStart - yVal)));
    
    // Avanzar a lo largo del arco de la curva
    // ds = v * dt
    // dx = ds / sqrt(1 + y'^2)
    const dy = evalRes.dy;
    const ds = speed * dt;
    const dx = ds / Math.sqrt(1 + dy * dy);
    
    coasterAnimX += dx;
    
    if (coasterAnimX >= maxX) {
        coasterAnimX = maxX;
        drawCoaster();
        isCoasterRunning = false;
        const btn = document.getElementById("btn-coaster-start");
        btn.textContent = "▶ Re-iniciar Recorrido";
        btn.className = "btn btn-success";
        alert("¡Recorrido terminado! Frenado suave magnético activado.");
        coasterAnimX = minX;
        return;
    }
    
    // Calcular Fuerza G vertical sentida por el pasajero
    // G_y = cos(theta) + a_centripeta/g
    // cos(theta) = 1 / sqrt(1 + y'^2)
    // a_c = v^2 * curvature
    const cosTheta = 1.0 / Math.sqrt(1 + dy * dy);
    const ddy = evalRes.ddy;
    const curvature = ddy / Math.pow(1 + dy * dy, 1.5);
    const centripetalAcc = speed * speed * curvature;
    const gForce = cosTheta + (centripetalAcc / g);
    
    // Jerk (Numérico): tasa de cambio de aceleración vertical (Fuerza G * g)
    // Usamos el paso previo de la curva para deducir y'''
    const stepOffset = 0.05;
    const nextEval = evaluateSplinesAt(coasterSplines, Math.min(maxX, coasterAnimX + stepOffset));
    const nextCurvature = nextEval.ddy / Math.pow(1 + nextEval.dy * nextEval.dy, 1.5);
    const nextSpeed = Math.max(2.0, Math.sqrt(vStart * vStart + 2 * g * (yStart - nextEval.y)));
    const nextGForce = (1.0 / Math.sqrt(1 + nextEval.dy * nextEval.dy)) + (nextSpeed * nextSpeed * nextCurvature / g);
    
    const deltaG = (nextGForce - gForce) * g; // en m/s²
    const dtSpacing = stepOffset / (speed * cosTheta);
    const jerk = dtSpacing > 0 ? Math.abs(deltaG / dtSpacing) : 0;
    
    document.getElementById("hud-coaster-height").textContent = `${yVal.toFixed(2)} m`;
    document.getElementById("hud-coaster-speed").textContent = `${(speed * 3.6).toFixed(1)} km/h`;
    document.getElementById("hud-coaster-gforce").textContent = `${gForce.toFixed(2)} G`;
    document.getElementById("hud-coaster-jerk").textContent = `${jerk.toFixed(2)} m/s³`;
    
    // Color de alarma para fuerzas G extremas
    const gValSpan = document.getElementById("hud-coaster-gforce");
    if (gForce > 3.5 || gForce < -0.5) {
        gValSpan.style.color = COLOR.danger;
    } else if (gForce > 2.5 || gForce < 0.2) {
        gValSpan.style.color = COLOR.warning;
    } else {
        gValSpan.style.color = COLOR.success;
    }
    
    drawCoaster();
    requestAnimationFrame(coasterAnimationLoop);
}

function drawCoaster() {
    if (!coasterMapCanvas) return;
    
    resizeCanvasToFit(coasterMapCanvas);
    const w = coasterMapCanvas.width;
    const h = coasterMapCanvas.height;
    
    coasterMapCtx.fillStyle = COLOR.bg_main;
    coasterMapCtx.fillRect(0, 0, w, h);
    
    const cXMin = -0.5, cXMax = 10.5;
    const cYMin = -0.5, cYMax = 7.5;
    
    function cMathToCanvas(mx, my) {
        const px = ((mx - cXMin) / (cXMax - cXMin)) * w;
        const py = h - ((my - cYMin) / (cYMax - cYMin)) * h;
        return { x: px, y: py };
    }
    
    // 1. Grilla
    coasterMapCtx.strokeStyle = COLOR.grid;
    coasterMapCtx.lineWidth = 1;
    for (let x = 0; x <= 10; x++) {
        const c1 = cMathToCanvas(x, cYMin);
        const c2 = cMathToCanvas(x, cYMax);
        coasterMapCtx.beginPath(); coasterMapCtx.moveTo(c1.x, c1.y); coasterMapCtx.lineTo(c2.x, c2.y); coasterMapCtx.stroke();
    }
    for (let y = 0; y <= 7; y++) {
        const c1 = cMathToCanvas(cXMin, y);
        const c2 = cMathToCanvas(cXMax, y);
        coasterMapCtx.beginPath(); coasterMapCtx.moveTo(c1.x, c1.y); coasterMapCtx.lineTo(c2.x, c2.y); coasterMapCtx.stroke();
    }
    
    // 2. Dibujar las Columnas de Soporte Estructurales de la Vía
    coasterMapCtx.strokeStyle = "rgba(255, 255, 255, 0.08)";
    coasterMapCtx.lineWidth = 2;
    for (let x = 0.5; x < 10; x += 0.8) {
        const evalRes = evaluateSplinesAt(coasterSplines, x);
        if (evalRes) {
            const trackPos = cMathToCanvas(x, evalRes.y);
            const groundPos = cMathToCanvas(x, 0);
            
            // Columna vertical
            coasterMapCtx.beginPath();
            coasterMapCtx.moveTo(trackPos.x, trackPos.y);
            coasterMapCtx.lineTo(groundPos.x, groundPos.y);
            coasterMapCtx.stroke();
            
            // Vigas de soporte cruzadas
            coasterMapCtx.beginPath();
            coasterMapCtx.moveTo(trackPos.x - 12, trackPos.y + 15);
            coasterMapCtx.lineTo(trackPos.x + 12, trackPos.y + 15);
            coasterMapCtx.stroke();
        }
    }
    
    // 3. Dibujar el riel de acero (Spline)
    if (coasterPathPoints.length > 1) {
        // Riel secundario (sombra de soporte inferior)
        coasterMapCtx.beginPath();
        coasterMapCtx.strokeStyle = "#1e293b";
        coasterMapCtx.lineWidth = 7;
        const cStartLower = cMathToCanvas(coasterPathPoints[0].x, coasterPathPoints[0].y - 0.1);
        coasterMapCtx.moveTo(cStartLower.x, cStartLower.y);
        for (let i = 1; i < coasterPathPoints.length; i++) {
            const cNode = cMathToCanvas(coasterPathPoints[i].x, coasterPathPoints[i].y - 0.1);
            coasterMapCtx.lineTo(cNode.x, cNode.y);
        }
        coasterMapCtx.stroke();
        
        // Riel principal brillante
        coasterMapCtx.beginPath();
        coasterMapCtx.strokeStyle = COLOR.accent_indigo;
        coasterMapCtx.lineWidth = 3.5;
        const cStart = cMathToCanvas(coasterPathPoints[0].x, coasterPathPoints[0].y);
        coasterMapCtx.moveTo(cStart.x, cStart.y);
        for (let i = 1; i < coasterPathPoints.length; i++) {
            const cNode = cMathToCanvas(coasterPathPoints[i].x, coasterPathPoints[i].y);
            coasterMapCtx.lineTo(cNode.x, cNode.y);
        }
        coasterMapCtx.stroke();
    }
    
    // 4. Dibujar los Nodos/Puntos de Control
    coasterWaypoints.forEach((wp, idx) => {
        const c = cMathToCanvas(wp.x, wp.y);
        coasterMapCtx.beginPath();
        coasterMapCtx.arc(c.x, c.y, 5, 0, 2 * Math.PI);
        coasterMapCtx.fillStyle = "white";
        coasterMapCtx.strokeStyle = COLOR.accent_indigo;
        coasterMapCtx.lineWidth = 2;
        coasterMapCtx.fill();
        coasterMapCtx.stroke();
        
        coasterMapCtx.fillStyle = COLOR.fg_muted;
        coasterMapCtx.font = "bold 9px Outfit";
        coasterMapCtx.fillText(`W${idx}`, c.x - 7, c.y - 10);
    });
    
    // 5. Dibujar el Carro de la Montaña Rusa
    const evalRes = evaluateSplinesAt(coasterSplines, coasterAnimX);
    if (evalRes) {
        const cCart = cMathToCanvas(coasterAnimX, evalRes.y);
        const angle = Math.atan(evalRes.dy);
        
        coasterMapCtx.save();
        coasterMapCtx.translate(cCart.x, cCart.y);
        coasterMapCtx.rotate(-angle); // Canvas y invertido
        
        // Cabina
        coasterMapCtx.fillStyle = COLOR.danger;
        coasterMapCtx.strokeStyle = "white";
        coasterMapCtx.lineWidth = 1.5;
        coasterMapCtx.beginPath();
        coasterMapCtx.roundRect(-14, -10, 28, 14, 3);
        coasterMapCtx.fill();
        coasterMapCtx.stroke();
        
        // Pasajeros (cabeza simplificada)
        coasterMapCtx.fillStyle = COLOR.warning;
        coasterMapCtx.beginPath();
        coasterMapCtx.arc(-5, -13, 3, 0, 2 * Math.PI);
        coasterMapCtx.arc(5, -13, 3, 0, 2 * Math.PI);
        coasterMapCtx.fill();
        
        // Ruedas
        coasterMapCtx.fillStyle = "#f9fafb";
        coasterMapCtx.beginPath();
        coasterMapCtx.arc(-8, 6, 4, 0, 2 * Math.PI);
        coasterMapCtx.arc(8, 6, 4, 0, 2 * Math.PI);
        coasterMapCtx.fill();
        
        coasterMapCtx.restore();
    }
    
    drawCoasterCharts(coasterAnimX);
}

function drawCoasterCharts(currentMathX) {
    if (!coasterGCanvas || !coasterJerkCanvas) return;
    
    resizeCanvasToFit(coasterGCanvas);
    resizeCanvasToFit(coasterJerkCanvas);
    
    const w = coasterGCanvas.width;
    const h1 = coasterGCanvas.height;
    const h2 = coasterJerkCanvas.height;
    
    const minX = coasterWaypoints[0].x;
    const maxX = coasterWaypoints[coasterWaypoints.length - 1].x;
    
    // Obtener valores dinámicos
    const yStart = coasterWaypoints[0].y;
    const g = 9.81;
    const vStart = 3.0;
    
    const pathStats = coasterPathPoints.map(p => {
        const speed = Math.max(2.0, Math.sqrt(vStart * vStart + 2 * g * (yStart - p.y)));
        const cosTheta = 1.0 / Math.sqrt(1 + p.dy * p.dy);
        const gForce = cosTheta + (speed * speed * p.curvature / g);
        return { x: p.x, gForce: gForce };
    });
    
    // Calcular Jerk espacial en cada punto para graficar
    const jerkStats = [];
    for (let i = 0; i < pathStats.length; i++) {
        let jerk = 0;
        if (i < pathStats.length - 1) {
            const p = coasterPathPoints[i];
            const pNext = coasterPathPoints[i+1];
            const dxVal = pNext.x - p.x;
            const speed = Math.max(2.0, Math.sqrt(vStart * vStart + 2 * g * (yStart - p.y)));
            const cosTheta = 1.0 / Math.sqrt(1 + p.dy * p.dy);
            
            const gForce = pathStats[i].gForce;
            const nextGForce = pathStats[i+1].gForce;
            const deltaG = (nextGForce - gForce) * g;
            const dtSpacing = dxVal / (speed * cosTheta);
            jerk = dtSpacing > 0 ? Math.abs(deltaG / dtSpacing) : 0;
        } else {
            jerk = jerkStats[jerkStats.length - 1] || 0;
        }
        jerkStats.push(jerk);
    }
    
    const minG = -1.5, maxG = 5.0;
    const maxJerkVal = Math.max(...jerkStats, 10.0) + 1.0;
    
    function scaleX(mx) {
        return ((mx - minX) / (maxX - minX)) * w;
    }
    function scaleG(my) {
        return h1 - ((my - minG) / (maxG - minG)) * h1;
    }
    function scaleJerk(my) {
        return h2 - (my / maxJerkVal) * h2;
    }
    
    // --- G-FORCE CHART ---
    coasterGCtx.fillStyle = COLOR.bg_main;
    coasterGCtx.fillRect(0, 0, w, h1);
    
    coasterGCtx.strokeStyle = COLOR.grid;
    coasterGCtx.lineWidth = 1;
    // Líneas horizontales de referencia (1G, 3G, 0G)
    [0, 1, 3].forEach(val => {
        const cy = scaleG(val);
        coasterGCtx.beginPath(); coasterGCtx.moveTo(0, cy); coasterGCtx.lineTo(w, cy); coasterGCtx.stroke();
        coasterGCtx.fillStyle = COLOR.fg_muted;
        coasterGCtx.font = "8px Outfit";
        coasterGCtx.fillText(`${val} G`, 5, cy - 2);
    });
    
    coasterGCtx.beginPath();
    coasterGCtx.strokeStyle = COLOR.warning;
    coasterGCtx.lineWidth = 2;
    coasterGCtx.moveTo(scaleX(pathStats[0].x), scaleG(pathStats[0].gForce));
    for (let i = 1; i < pathStats.length; i++) {
        coasterGCtx.lineTo(scaleX(pathStats[i].x), scaleG(pathStats[i].gForce));
    }
    coasterGCtx.stroke();
    
    // Cursor móvil
    const pxMarker = scaleX(currentMathX);
    coasterGCtx.strokeStyle = COLOR.danger;
    coasterGCtx.setLineDash([3, 3]);
    coasterGCtx.beginPath(); coasterGCtx.moveTo(pxMarker, 0); coasterGCtx.lineTo(pxMarker, h1); coasterGCtx.stroke();
    coasterGCtx.setLineDash([]);
    
    // --- JERK CHART ---
    coasterJerkCtx.fillStyle = COLOR.bg_main;
    coasterJerkCtx.fillRect(0, 0, w, h2);
    
    coasterJerkCtx.strokeStyle = COLOR.grid;
    coasterJerkCtx.lineWidth = 1;
    // Línea de peligro Jerk > 15 m/s3 (límite del cuerpo humano)
    const cyDanger = scaleJerk(15.0);
    coasterJerkCtx.strokeStyle = "rgba(239, 68, 68, 0.25)";
    coasterJerkCtx.lineWidth = 1.5;
    coasterJerkCtx.beginPath(); coasterJerkCtx.moveTo(0, cyDanger); coasterJerkCtx.lineTo(w, cyDanger); coasterJerkCtx.stroke();
    coasterJerkCtx.fillStyle = COLOR.danger;
    coasterJerkCtx.fillText("15 m/s³ (LÍMITE SEGURO)", 5, cyDanger - 3);
    
    coasterJerkCtx.beginPath();
    coasterJerkCtx.strokeStyle = COLOR.danger;
    coasterJerkCtx.lineWidth = 2;
    coasterJerkCtx.moveTo(scaleX(coasterPathPoints[0].x), scaleJerk(jerkStats[0]));
    for (let i = 1; i < coasterPathPoints.length; i++) {
        coasterJerkCtx.lineTo(scaleX(coasterPathPoints[i].x), scaleJerk(jerkStats[i]));
    }
    coasterJerkCtx.stroke();
    
    coasterJerkCtx.strokeStyle = COLOR.warning;
    coasterJerkCtx.setLineDash([3, 3]);
    coasterJerkCtx.beginPath(); coasterJerkCtx.moveTo(pxMarker, 0); coasterJerkCtx.lineTo(pxMarker, h2); coasterJerkCtx.stroke();
    coasterJerkCtx.setLineDash([]);
}

// =========================================================================
// CONTROLADORES DE MODELADO DE CLIMA
// =========================================================================
function loadSelectedWeather() {
    const weatherVal = document.getElementById("weather-select").value;
    weatherDatasetName = weatherVal;
    
    const data = WEATHER_DB[weatherVal];
    weatherPoints = data.points.map(p => ({ ...p }));
    
    // Rellenar tabla en el HTML
    const tbody = document.querySelector("#weather-table tbody");
    tbody.innerHTML = "";
    weatherPoints.forEach(p => {
        const tr = document.createElement("tr");
        tr.innerHTML = `
            <td style="font-weight: 700; color: var(--accent-indigo);">${p.x.toString().padStart(2, '0')}:00</td>
            <td style="font-family: var(--font-mono); font-weight: 600;">${p.y.toFixed(1)} °C</td>
            <td style="font-size: 0.75rem; color: var(--fg-muted);">${p.note}</td>
        `;
        tbody.appendChild(tr);
    });
    
    // Calcular spline del clima
    const result = calculateCubicSplines(weatherPoints);
    weatherSplines = result.splines;
    
    drawWeather();
}

function drawWeather() {
    if (!weatherCanvas) return;
    
    resizeCanvasToFit(weatherCanvas);
    const w = weatherCanvas.width;
    const h = weatherCanvas.height;
    
    weatherCtx.fillStyle = COLOR.bg_main;
    weatherCtx.fillRect(0, 0, w, h);
    
    const wXMin = -1.0, wXMax = 25.0;
    
    // Encontrar límites de temperatura del dataset para ajustar escala vertical
    const ys = weatherPoints.map(p => p.y);
    const wYMin = Math.min(...ys, 0) - 2;
    const wYMax = Math.max(...ys, 20) + 4;
    
    function wMathToCanvas(mx, my) {
        const px = ((mx - wXMin) / (wXMax - wXMin)) * w;
        const py = h - ((my - wYMin) / (wYMax - wYMin)) * h;
        return { x: px, y: py };
    }
    function wCanvasToMath(cx, cy) {
        const mx = wXMin + (cx / w) * (wXMax - wXMin);
        const my = wYMin + ((h - cy) / h) * (wYMax - wYMin);
        return { x: mx, y: my };
    }
    
    // Grid horizontal (Tiempo horas)
    weatherCtx.strokeStyle = COLOR.grid;
    weatherCtx.lineWidth = 1;
    weatherCtx.fillStyle = COLOR.fg_muted;
    weatherCtx.font = "8px Outfit";
    for (let hour = 0; hour <= 24; hour += 4) {
        const c = wMathToCanvas(hour, 0);
        weatherCtx.beginPath(); weatherCtx.moveTo(c.x, 0); weatherCtx.lineTo(c.x, h); weatherCtx.stroke();
        weatherCtx.fillText(`${hour}h`, c.x - 6, h - 8);
    }
    
    // Grid vertical (Temperaturas)
    for (let temp = Math.floor(wYMin); temp <= wYMax; temp += 5) {
        const c = wMathToCanvas(0, temp);
        weatherCtx.beginPath(); weatherCtx.moveTo(0, c.y); weatherCtx.lineTo(w, c.y); weatherCtx.stroke();
        weatherCtx.fillText(`${temp}°C`, 5, c.y - 3);
    }
    
    const showLinear = document.getElementById("show-weather-linear").checked;
    const showLagrange = document.getElementById("show-weather-lagrange").checked;
    const showSpline = document.getElementById("show-weather-spline").checked;
    
    // 1. Curva Interpolación Lineal (Muted Grey / Segmentos rectos)
    if (showLinear && weatherPoints.length > 1) {
        weatherCtx.beginPath();
        weatherCtx.strokeStyle = "rgba(156, 163, 175, 0.45)";
        weatherCtx.lineWidth = 1.5;
        const cStart = wMathToCanvas(weatherPoints[0].x, weatherPoints[0].y);
        weatherCtx.moveTo(cStart.x, cStart.y);
        for (let i = 1; i < weatherPoints.length; i++) {
            const cNode = wMathToCanvas(weatherPoints[i].x, weatherPoints[i].y);
            weatherCtx.lineTo(cNode.x, cNode.y);
        }
        weatherCtx.stroke();
    }
    
    // 2. Curva Polinomio Global de Lagrange (Danger Red - divergencia extrema por Runge)
    if (showLagrange && weatherPoints.length >= 2) {
        weatherCtx.beginPath();
        weatherCtx.strokeStyle = COLOR.danger;
        weatherCtx.lineWidth = 1.5;
        weatherCtx.setLineDash([5, 5]);
        
        let started = false;
        for (let px = 0; px < w; px++) {
            const mx = wCanvasToMath(px, 0).x;
            if (mx >= weatherPoints[0].x && mx <= weatherPoints[weatherPoints.length - 1].x) {
                const my = evaluateLagrangeAt(weatherPoints, mx);
                // Evitar dibujar si se sale de la pantalla de forma descomunal
                if (my >= wYMin - 20 && my <= wYMax + 20) {
                    const cy = wMathToCanvas(mx, my).y;
                    if (!started) {
                        weatherCtx.moveTo(px, cy);
                        started = true;
                    } else {
                        weatherCtx.lineTo(px, cy);
                    }
                }
            }
        }
        weatherCtx.stroke();
        weatherCtx.setLineDash([]);
    }
    
    // 3. Curva Spline Cúbico Natural (Success Green - suavidad y estabilidad física)
    if (showSpline && weatherSplines.length > 0) {
        weatherCtx.beginPath();
        weatherCtx.strokeStyle = COLOR.success;
        weatherCtx.lineWidth = 3;
        
        let started = false;
        for (let px = 0; px < w; px++) {
            const mx = wCanvasToMath(px, 0).x;
            if (mx >= weatherPoints[0].x && mx <= weatherPoints[weatherPoints.length - 1].x) {
                const evalRes = evaluateSplinesAt(weatherSplines, mx);
                if (evalRes) {
                    const cy = wMathToCanvas(mx, evalRes.y).y;
                    if (!started) {
                        weatherCtx.moveTo(px, cy);
                        started = true;
                    } else {
                        weatherCtx.lineTo(px, cy);
                    }
                }
            }
        }
        weatherCtx.stroke();
    }
    
    // 4. Dibujar los puntos del sensor
    weatherPoints.forEach(p => {
        const c = wMathToCanvas(p.x, p.y);
        weatherCtx.beginPath();
        weatherCtx.arc(c.x, c.y, 6, 0, 2 * Math.PI);
        weatherCtx.fillStyle = "white";
        weatherCtx.strokeStyle = COLOR.accent_indigo;
        weatherCtx.lineWidth = 2.5;
        weatherCtx.fill();
        weatherCtx.stroke();
    });
}

// =========================================================================
// INICIALIZADORES AL CARGAR LA PÁGINA
// =========================================================================
window.addEventListener('load', () => {
    recomputeAllCalculator();
    
    window.addEventListener('resize', () => {
        if (document.getElementById("tab-calculadora").classList.contains("active")) {
            drawCalculator();
        } else {
            // Redibujar la sub-aplicación que esté activa actualmente
            if (document.getElementById("sub-app-dron").classList.contains("active")) {
                drawDroneMission();
            } else if (document.getElementById("sub-app-coaster").classList.contains("active")) {
                drawCoaster();
            } else if (document.getElementById("sub-app-clima").classList.contains("active")) {
                drawWeather();
            }
        }
    });
});
