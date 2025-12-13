from flask import Flask, send_from_directory, abort
import threading
import collector_api  # reutiliza la app y el hilo MQTT->SQLite
from pathlib import Path

# Usamos la app de collector_api
app = collector_api.app
PANEL_DIR = Path(__file__).parent / "panel_streamlit"


@app.route("/")
@app.route("/mqtt_dashboard.html")
def panel():
    target = PANEL_DIR / "mqtt_dashboard.html"
    print("panel route hit, target:", target, "exists:", target.exists(), flush=True)
    if target.exists():
        return target.read_text(encoding="utf-8")
    abort(404)

# Ruta para servir otros assets estáticos dentro de panel_streamlit si fuera necesario
@app.route("/panel_streamlit/<path:filename>")
def panel_static(filename):
    target = PANEL_DIR / filename
    if target.exists():
        return target.read_bytes()
    abort(404)


if __name__ == "__main__":
    # Arranca el hilo MQTT (si no se ha iniciado ya)
    if not getattr(collector_api, "mqtt_running", False):
        threading.Thread(target=collector_api.mqtt_thread, daemon=True).start()
        collector_api.mqtt_running = True
    # Modo desarrollo con recarga automática al detectar cambios
    app.run(host="0.0.0.0", port=5000, debug=True, use_reloader=True)
