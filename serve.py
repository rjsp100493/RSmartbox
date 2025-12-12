from flask import Flask, send_from_directory
import threading
import collector_api  # reutiliza la app y el hilo MQTT->SQLite

# Usamos la app de collector_api
app = collector_api.app


@app.route("/")
@app.route("/mqtt_dashboard.html")
def panel():
    return send_from_directory("panel_streamlit", "mqtt_dashboard.html")


if __name__ == "__main__":
    # Arranca el hilo MQTT (si no se ha iniciado ya)
    if not getattr(collector_api, "mqtt_running", False):
        threading.Thread(target=collector_api.mqtt_thread, daemon=True).start()
        collector_api.mqtt_running = True
    app.run(host="0.0.0.0", port=5000)
