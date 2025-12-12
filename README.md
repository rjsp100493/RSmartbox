# Pump RSmartbox

Firmware y panel web para la LilyGO T-Call SIM7000G midiendo:
- Corriente con SCT013 (100A/1V) en GPIO36.
- Voltaje con ZMPT101B en GPIO39.

## Wiring resumen
- **SCT013**: un hilo a bias (ver ZMPT si compartes nodo), otro a GND. Bias del SCT (como ya tienes) centrado en ~1.65 V con 2×10 kΩ y RC (100 nF + 100 µF) a GND. GPIO36 en modo ADC.
- **ZMPT101B**:
  - Alimenta a 5 V (recomendado) o 3.3 V si la señal es suficiente.
  - OUT → resistencia serie 100–220 Ω → GPIO39.
  - Divisor 1:1 alto (p. ej. 100 kΩ/100 kΩ) en OUT si alimentas a 5 V, nodo al GPIO39, para no superar 3.3 V.
  - GND común. Ajusta el pot del módulo para que OUT sin AC quede ~Vcc/2 (≈2.5 V si Vcc=5 V) y con AC la señal pico no pase 3.3 V en el ADC.
  - Opcional: diodos de clamp en el pin (ánodo pin–cátodo 3.3 V y ánodo GND–cátodo pin).

## Calibración
- Corriente: `irms = vrms_adc / 0.01` (SCT 100A/1V). Se recorta a 0 si < 2 A; ajusta a tu gusto en `publishCurrentAndVoltage()`.
- Voltaje: ajusta `voltageScale` en `src/main.cpp` con una referencia conocida:  
  `voltageScale = V_real / Vrms_adcV` (observa `Vrms_adcV` en el monitor serie con 120 V AC). Valor actual: 216.0.
- Ventana RMS: 1200 ms. Reduce si quieres respuesta más rápida; aumenta si quieres menos ruido.

## Firmware
- PlatformIO (ESP32 Dev Module). MQTT por TCP a `test.mosquitto.org:1883` (cámbialo en `main.cpp` y panel si usas otro broker).
- Publica:
  - `sim7000g/corriente` (A, 2 decimales).
  - `sim7000g/voltaje` (V, 1 decimal por panel; 3 decimales en payload).
  - `sim7000g/adc_v` (bias de corriente, informativo).

Build & Upload desde PlatformIO. Monitor serie a 115200 para ver `Vrms_adcV` y depurar.

## Panel web
`panel_streamlit/mqtt_dashboard.html` (usa MQTT sobre WebSocket):
- Campos para broker WS/WSS y topics (`sim7000g/corriente` y `sim7000g/voltaje`).
- Métricas e historiales separados (corriente y voltaje).

Servir localmente (misma LAN que el móvil):
```
cd panel_streamlit
python -m http.server 8000
```
Abre en el navegador: `http://<IP_PC>:8000/mqtt_dashboard.html` y configura el broker (ej. `wss://test.mosquitto.org:8081`). También puedes abrir el HTML directamente en el móvil si apuntas al mismo broker con WS.

## Notas
- Evita dejar GPIO39 al aire: el ADC flota y verás lecturas falsas. Siempre referencia el pin con el divisor/bias y GND común.
- Si alimentas ZMPT a 3.3 V y la señal es muy baja/inestable, aliméntalo a 5 V y atenúa la salida al ADC (divisor 1:1 alto + serie/clamps).
- Para lecturas estables, calibra con carga/tensión conocidas después de ajustar el potenciómetro.
