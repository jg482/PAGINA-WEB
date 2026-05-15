// --- PARÁMETROS FIJOS DE LA PLANTA ---
const Kp1 = 0.2, T1 = 2;   // Planta 1 (Rápida)
const Kp2 = 0.8, T2 = 10;  // Planta 2 (Lenta)
const umin = 0, umax = 100;
const Ts = 0.1, t_fin = 60;
const ru = 50, u1_0 = 50, u2_0 = 50, ry_ini = 50;
const t_step = 5;
const b1 = 0, b2 = 0; // Set-point weighting

let chartOutput, chartValves;

// Función principal de simulación matemática
function simulate() {
    // 1. Leer valores de los sliders
    const Kc1 = parseFloat(document.getElementById('kc1').value);
    const Ti1 = parseFloat(document.getElementById('ti1').value);
    const Kc2 = parseFloat(document.getElementById('kc2').value);
    const Ti2 = parseFloat(document.getElementById('ti2').value);
    const ry_fin = parseFloat(document.getElementById('ry_fin').value);

    // 2. Actualizar el texto que muestra el número junto al slider
    document.getElementById('val-kc1').innerText = Kc1.toFixed(2);
    document.getElementById('val-ti1').innerText = Ti1.toFixed(2);
    document.getElementById('val-kc2').innerText = Kc2.toFixed(2);
    document.getElementById('val-ti2').innerText = Ti2.toFixed(1);
    document.getElementById('val-ry').innerText = ry_fin;

    // 3. Pre-cálculos
    const N = Math.floor(t_fin / Ts);
    const Kf = -Kp2 / Kp1;
    const Kaw1 = 1 / Ti1;
    const Kaw2 = 1 / Ti2;

    // Vectores para las gráficas
    let t_data = [], ry_data = [], y_data = [], u1_data = [], u2_data = [];
    
    // Estados iniciales en régimen estacionario
    let y1 = Kp1 * u1_0, dy1 = 0;
    let y2 = Kp2 * u2_0, dy2 = 0;
    let I1 = (u1_0 - Kf * u2_0) - Kc1 * (b1 * ry_ini - (y1 + y2));
    let I2 = u2_0 - Kc2 * (u1_0 - b2 * ru);

    // 4. Bucle de Simulación
    for (let k = 0; k <= N; k++) {
        let t = k * Ts;
        let ry = (t < t_step) ? ry_ini : ry_fin; // Salto escalón
        let y = y1 + y2;

        // --- Lógica del Controlador MRC ---
        let error1 = ry - y;
        let error1P = b1 * ry - y;
        let u1_base = Kc1 * error1P + I1;

        // Resolución del bucle algebraico
        let divisor = 1 - (Kf * Kc2);
        let u1_ideal = (u1_base + Kf * I2 - Kf * Kc2 * b2 * ru) / divisor;

        // Válvula lenta (C2)
        let error2P_ideal = u1_ideal - b2 * ru;
        let u2_unsat = Kc2 * error2P_ideal + I2;
        let u2_final = Math.max(umin, Math.min(u2_unsat, umax));
        let diff_sat2 = u2_final - u2_unsat;

        // Válvula rápida (C1)
        let u1_total = u1_base + Kf * u2_final;
        let u1_final = Math.max(umin, Math.min(u1_total, umax));
        let diff_sat1 = u1_final - u1_total;

        // Actualizar integradores (Back-calculation para Anti-Windup)
        I1 += ((Kc1 * Ts / Ti1) * error1 + (Kaw1 * diff_sat1 * Ts));
        I2 += ((Kc2 * Ts / Ti2) * (u1_final - ru) + (Kaw2 * diff_sat2 * Ts));

        // --- Integración de las Plantas (Euler) ---
        // Ecuación diferencial 2do orden para y1
        let ddy1 = (Kp1 * u1_final - y1 - 2 * T1 * dy1) / (T1 * T1);
        dy1 += ddy1 * Ts;
        y1 += dy1 * Ts;

        // Ecuación diferencial 2do orden para y2
        let ddy2 = (Kp2 * u2_final - y2 - 2 * T2 * dy2) / (T2 * T2);
        dy2 += ddy2 * Ts;
        y2 += dy2 * Ts;

        // 5. Guardar los datos del instante actual para graficarlos
        t_data.push(t.toFixed(1));
        ry_data.push(ry);
        y_data.push(y);
        u1_data.push(u1_final);
        u2_data.push(u2_final);
    }

    // Enviar datos al motor gráfico
    updateCharts(t_data, ry_data, y_data, u1_data, u2_data);
}

// Función para actualizar las líneas de la gráfica
function updateCharts(t, ry, y, u1, u2) {
    chartOutput.data.labels = t;
    chartOutput.data.datasets[0].data = ry;
    chartOutput.data.datasets[1].data = y;
    chartOutput.update('none'); // 'none' evita animaciones lentas en tiempo real

    chartValves.data.labels = t;
    chartValves.data.datasets[0].data = u1;
    chartValves.data.datasets[1].data = u2;
    chartValves.update('none');
}

// Función para crear las gráficas en blanco al abrir la página
function initCharts() {
    const ctxOut = document.getElementById('chartOutput').getContext('2d');
    chartOutput = new Chart(ctxOut, {
        type: 'line',
        data: {
            labels: [],
            datasets: [
                { label: 'Referencia (ry)', data: [], borderColor: 'black', borderDash: [5, 5], fill: false, pointRadius: 0 },
                { label: 'Salida (y)', data: [], borderColor: 'blue', fill: false, pointRadius: 0 }
            ]
        },
        options: { 
            responsive: true, 
            maintainAspectRatio: false, 
            animation: false,
            scales: { y: { min: 45, max: 75 } }
        }
    });

    const ctxVal = document.getElementById('chartValves').getContext('2d');
    chartValves = new Chart(ctxVal, {
        type: 'line',
        data: {
            labels: [],
            datasets: [
                { label: 'u1 (Rápida)', data: [], borderColor: 'red', fill: false, pointRadius: 0 },
                { label: 'u2 (Lenta)', data: [], borderColor: 'magenta', fill: false, pointRadius: 0 }
            ]
        },
        options: { 
            responsive: true, 
            maintainAspectRatio: false, 
            animation: false,
            scales: { y: { min: 0, max: 100 } }
        }
    });
}

// --- ARRANQUE DE LA APLICACIÓN ---
initCharts();
simulate();

// Escuchar cambios: cada vez que se mueve un slider, se ejecuta simulate()
document.querySelectorAll('input').forEach(input => {
    input.addEventListener('input', simulate);
});