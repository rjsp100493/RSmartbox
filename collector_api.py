import time
import threading
import sqlite3
import json
import os
from pathlib import Path
from flask import Flask, request, jsonify
from flask_cors import CORS
import paho.mqtt.client as mqtt

BASE_DIR = Path(__file__).resolve().parent
DB = str(BASE_DIR / "lecturas.db")
BROKER = "test.mosquitto.org"

# --- DB init ---
conn = sqlite3.connect(DB, check_same_thread=False, timeout=30)
cur = conn.cursor()
cur.execute("PRAGMA journal_mode=WAL;")
cur.execute("PRAGMA synchronous=NORMAL;")
cur.execute(
    """CREATE TABLE IF NOT EXISTS lecturas(
    ts INTEGER,
    corriente REAL,
    voltaje REAL,
    estado TEXT,
    t_trabajo_min REAL,
    t_recuperando_min REAL,
    t_rec_total_min REAL,
    corriente_falla REAL,
    voltaje_falla REAL
)"""
)
conn.commit()
# Asegura columnas por si el esquema previo no las tenía
for col in [
    ("estado", "TEXT"),
    ("t_trabajo_min", "REAL"),
    ("t_recuperando_min", "REAL"),
    ("t_rec_total_min", "REAL"),
    ("corriente_falla", "REAL"),
    ("voltaje_falla", "REAL"),
]:
    try:
        cur.execute(f"ALTER TABLE lecturas ADD COLUMN {col[0]} {col[1]}")
    except Exception:
        pass
conn.commit()

state = {"corriente": None, "voltaje": None}
mqtt_running = False


# --- MQTT callback ---
def on_message(client, userdata, msg):
    try:
        ts = int(time.time())
        t = msg.topic.lower()
        payload = msg.payload

        # Si es JSON, parseamos el paquete de estado
        if "estado" in t and payload.startswith(b"{"):
            data = json.loads(payload.decode("utf-8"))
            ts = int(data.get("ts", ts))
            state["corriente"] = float(data.get("corriente", state["corriente"] or 0.0))
            state["voltaje"] = float(data.get("voltaje", state["voltaje"] or 0.0))
            estado = data.get("estado")
            t_trab = data.get("t_trabajo_min")
            t_rec = data.get("t_recuperando_min")
            t_rec_total = data.get("t_rec_total_min")
            corr_falla = data.get("corriente_falla")
            volt_falla = data.get("voltaje_falla")
        else:
            # Solo procesamos el topic estado; ignoramos corriente/voltaje/gps/adc simples
            if "estado" not in t:
                return
            text = payload.decode(errors="ignore").strip()
            parts = dict(
                kv.split("=", 1) for kv in text.replace(",", " ").split() if "=" in kv
            )
            estado = parts.get("estado")
            corr_falla = float(parts["Ifalla"]) if "Ifalla" in parts else None
            volt_falla = float(parts["Vfalla"]) if "Vfalla" in parts else None
            t_trab = float(parts["tTrabajo"]) if "tTrabajo" in parts else None
            t_rec = float(parts["tRec"]) if "tRec" in parts else None
            t_rec_total = float(parts["tRecTotal"]) if "tRecTotal" in parts else None
            if "I" in parts:
                state["corriente"] = float(parts["I"])
            if "V" in parts:
                state["voltaje"] = float(parts["V"])

        if state["corriente"] is None and state["voltaje"] is None and not estado:
            return  # nada útil para guardar

        # Normaliza timestamp: si viene muy bajo (no epoch), usa ahora
        if ts < 1000000000:
            ts = int(time.time())

        # Guarda cada mensaje usando el último valor conocido de ambas magnitudes.
        cur.execute(
            "INSERT INTO lecturas(ts,corriente,voltaje,estado,t_trabajo_min,t_recuperando_min,t_rec_total_min,corriente_falla,voltaje_falla) VALUES (?,?,?,?,?,?,?,?,?)",
            (
                ts,
                state["corriente"] if state["corriente"] is not None else 0.0,
                state["voltaje"] if state["voltaje"] is not None else 0.0,
                estado,
                t_trab,
                t_rec,
                t_rec_total,
                corr_falla,
                volt_falla,
            ),
        )
        conn.commit()
        # Log rápido para ver que está llegando todo
        print(f"INS {ts}: I={state['corriente']} V={state['voltaje']} estado={estado} topic={msg.topic}")
    except Exception as e:
        print("Error guardando:", e)


def mqtt_thread():
    global mqtt_running
    mqtt_running = True
    c = mqtt.Client()
    c.on_message = on_message
    c.connect(BROKER, 1883, 30)
    # Suscribimos a todos los subtopics por si el firmware usa sufijos distintos
    c.subscribe("sim7000g/#")
    print("MQTT->SQLite escuchando en", BROKER)
    c.loop_forever()


# --- API Flask ---
app = Flask(__name__)
CORS(app)


@app.route("/health")
def health():
    cur2 = conn.cursor()
    cur2.execute("SELECT COUNT(*), MAX(ts) FROM lecturas")
    rows, last_ts = cur2.fetchone()
    return jsonify(
        {
            "db": DB,
            "cwd": os.getcwd(),
            "rows": rows,
            "last_ts": last_ts,
            "mqtt_running": bool(mqtt_running),
        }
    )


@app.route("/lecturas")
def lecturas():
    ts0 = int(request.args.get("start", 0))
    ts1 = int(request.args.get("end", time.time()))
    cur2 = conn.cursor()
    cur2.execute(
        """SELECT ts,corriente,voltaje,estado,t_trabajo_min,t_recuperando_min,t_rec_total_min,corriente_falla,voltaje_falla FROM lecturas
           WHERE ts BETWEEN ? AND ?
           ORDER BY ts""",
        (ts0, ts1),
    )
    data = [
        {
            "ts": r[0],
            "corriente": r[1],
            "voltaje": r[2],
            "estado": r[3],
            "t_trabajo_min": r[4],
            "t_recuperando_min": r[5],
            "t_rec_total_min": r[6],
            "corriente_falla": r[7],
            "voltaje_falla": r[8],
        }
        for r in cur2.fetchall()
    ]
    return jsonify(data)


if __name__ == "__main__":
    threading.Thread(target=mqtt_thread, daemon=True).start()
    app.run(host="0.0.0.0", port=5000)
