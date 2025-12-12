"""
Panel rápido en Streamlit para ver estado GPIO y voltaje publicados por el SIM7000G.

Uso:
- pip install -r requirements.txt
- streamlit run app.py

Por defecto se conecta a test.mosquitto.org:1883 y se suscribe a:
  - sim7000g/estado
  - sim7000g/voltaje
Puedes cambiarlos en la barra lateral.
"""

import queue
import time
from datetime import datetime

import paho.mqtt.client as mqtt
import streamlit as st

DEFAULT_BROKER = "test.mosquitto.org"
DEFAULT_PORT = 1883
DEFAULT_TOPIC_STATE = "sim7000g/estado"
DEFAULT_TOPIC_VOLT = "sim7000g/voltaje"
DEFAULT_TOPIC_LED_CMD = "sim7000g/led/cmd"


@st.cache_resource(show_spinner=False)
def get_mqtt_client(broker: str, port: int, user: str, password: str, topics: list[str]):
    q: queue.Queue = queue.Queue()

    def on_connect(client, userdata, flags, rc):
        q.put({"type": "info", "msg": f"Conectado al broker (rc={rc})"})
        for t in topics:
            client.subscribe(t, qos=1)

    def on_message(client, userdata, msg):
        payload = msg.payload.decode(errors="ignore")
        q.put(
            {
                "type": "msg",
                "topic": msg.topic,
                "payload": payload,
                "ts": time.time(),
            }
        )

    client = mqtt.Client()
    if user:
        client.username_pw_set(user, password or "")
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, port, keepalive=30)
    client.loop_start()
    return client, q


def main():
    st.set_page_config(page_title="SIM7000G Panel", layout="centered")
    st.title("Estado SIM7000G")

    with st.sidebar:
        st.header("Conexión MQTT")
        broker = st.text_input("Broker", value=DEFAULT_BROKER)
        port = st.number_input("Puerto", value=DEFAULT_PORT, step=1)
        user = st.text_input("Usuario", value="", type="default")
        password = st.text_input("Contraseña", value="", type="password")
        topic_state = st.text_input("Topic estado", value=DEFAULT_TOPIC_STATE)
        topic_volt = st.text_input("Topic voltaje", value=DEFAULT_TOPIC_VOLT)
        topic_led = st.text_input("Topic LED cmd", value=DEFAULT_TOPIC_LED_CMD)
        st.caption("Se muestran el estado GPIO y el voltaje del potenciómetro. Pulsador: envía on/off al LED.")

    client, q = get_mqtt_client(broker, int(port), user, password, [topic_state, topic_volt])

    if "last_state" not in st.session_state:
        st.session_state.last_state = "desconocido"
    if "last_volt" not in st.session_state:
        st.session_state.last_volt = "—"
    if "last_ts" not in st.session_state:
        st.session_state.last_ts = None

    # Drena mensajes pendientes
    drained = []
    while True:
        try:
            drained.append(q.get_nowait())
        except queue.Empty:
            break

    for evt in drained:
        if evt["type"] == "msg":
            if evt["topic"] == topic_state:
                st.session_state.last_state = evt["payload"]
                st.session_state.last_ts = evt["ts"]
            if evt["topic"] == topic_volt:
                st.session_state.last_volt = evt["payload"]
                st.session_state.last_ts = evt["ts"]

    state = st.session_state.last_state
    volt = st.session_state.last_volt
    last_ts_text = (
        datetime.fromtimestamp(st.session_state.last_ts).strftime("%H:%M:%S")
        if st.session_state.last_ts
        else "—"
    )

    # Tarjeta de estado
    color = "#7f8c8d"
    if isinstance(state, str):
        low = state.lower()
        if "encendido" in low or "on" in low:
            color = "#2ecc71"
        elif "apagado" in low or "off" in low:
            color = "#e74c3c"

    card_state = f"""
    <div style="background:#0f172a;padding:24px 20px;border-radius:16px;border:1px solid #1f2937;box-shadow:0 10px 25px rgba(0,0,0,0.3);color:#e5e7eb;">
      <div style="display:flex;align-items:center;gap:14px;">
        <div style="width:18px;height:18px;border-radius:50%;background:{color};box-shadow:0 0 18px {color};"></div>
        <div>
          <div style="font-size:14px;text-transform:uppercase;letter-spacing:1px;color:#9ca3af;">Estado GPIO</div>
          <div style="font-size:26px;font-weight:700;color:#e5e7eb;">{state}</div>
        </div>
      </div>
      <div style="margin-top:12px;font-size:12px;color:#9ca3af;">Última actualización: {last_ts_text}</div>
    </div>
    """
    st.markdown(card_state, unsafe_allow_html=True)

    # Tarjeta de voltaje
    card_volt = f"""
    <div style="margin-top:16px;background:#0f172a;padding:20px;border-radius:16px;border:1px solid #1f2937;box-shadow:0 8px 20px rgba(0,0,0,0.25);color:#e5e7eb;">
      <div style="font-size:14px;text-transform:uppercase;letter-spacing:1px;color:#9ca3af;">Voltaje ADC (V)</div>
      <div style="font-size:30px;font-weight:700;color:#e5e7eb;">{volt}</div>
    </div>
    """
    st.markdown(card_volt, unsafe_allow_html=True)

    st.divider()
    st.subheader("Control LED")
    if st.button("Encender LED"):
        client.publish(topic_led, "on", qos=1, retain=False)
        st.success("Comando enviado: on")
    if st.button("Apagar LED"):
        client.publish(topic_led, "off", qos=1, retain=False)
        st.info("Comando enviado: off")

    st.caption("Pantalla minimal: se refresca sola cada 3 segundos.")
    try:
        st_autorefresh = getattr(st, "autorefresh", None)
        if st_autorefresh:
            st_autorefresh(interval=3000, key="refresh")
        else:
            time.sleep(3)
            if hasattr(st, "rerun"):
                st.rerun()
            elif hasattr(st, "experimental_rerun"):
                st.experimental_rerun()
    except Exception as e:
        st.warning(f"No se pudo refrescar automáticamente: {e}")


if __name__ == "__main__":
    main()
