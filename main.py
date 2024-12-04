import network
import time
import json
from machine import Pin, ADC, time_pulse_us
from WebSocketClient import WebSocketClient
import gc
import urequests
from umqtt.simple import MQTTClient

# Configuration des broches
TRIG_PIN = Pin(5, Pin.OUT)
ECHO_PIN = Pin(18, Pin.IN)
SOUND_SENSOR_PIN = ADC(Pin(34))
SOUND_SENSOR_PIN.atten(ADC.ATTN_11DB)
LED_PIN = Pin(23, Pin.OUT)

# Configuration WiFi
WIFI_SSID = "Cudy-EFFC"
WIFI_PASSWORD = "33954721"

# Configuration WebSocket
WEBSOCKET_URL = "ws://192.168.10.244:8080/step1"

# Configuration seuil
SOUND_SPEED = 340
TRIG_PULSE_DURATION_US = 10
SOUND_THRESHOLD = 300
SEND_INTERVAL = 1000
last_send_time = 0

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    print(f'Connexion au réseau {WIFI_SSID}...')
    
    if not wlan.isconnected():
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        max_wait = 10
        while max_wait > 0:
            if wlan.isconnected():
                break
            max_wait -= 1
            print('Attente de connexion...')
            time.sleep(1)
            
    if wlan.isconnected():
        print('Connexion WiFi réussie!')
        print('Adresse IP:', wlan.ifconfig()[0])
        return True
    else:
        print('Échec de connexion WiFi')
        return False

def measure_distance():
    TRIG_PIN.value(0)
    time.sleep_us(2)
    TRIG_PIN.value(1)
    time.sleep_us(TRIG_PULSE_DURATION_US)
    TRIG_PIN.value(0)

    duration = time_pulse_us(ECHO_PIN, 1)
    distance_cm = (duration * SOUND_SPEED) / (2 * 10000)
    return distance_cm

def send_data(ws, distance, sound_level):
    try:
        data = {
            "distance": distance,
            "sound_level": sound_level,
            "timestamp": time.time()
        }
        message = json.dumps(data)
        if ws.send(message):
            print("Données envoyées:", message)
        else:
            print("Erreur d'envoi du message WebSocket")
    except Exception as e:
        print("Erreur lors de l'envoi:", e)

def main():
    global last_send_time  # Ajout de cette ligne
    gc.collect()

    if not connect_wifi():
        print("Impossible de continuer sans connexion WiFi")
        return

    ws = WebSocketClient(WEBSOCKET_URL)

    try:
        if ws.connect():
            print("Connecté au serveur WebSocket")
            ws.socket.setblocking(False)
            
            # Envoi du message de connexion
            connect_message = json.dumps({"message": "ESP32 connecté", "timestamp": time.time()})
            if ws.send(connect_message):
                print(f"Message de connexion envoyé: {connect_message}")

            while True:
                distance = measure_distance()
                sound_level = SOUND_SENSOR_PIN.read()
                
                if time.ticks_ms() - last_send_time > SEND_INTERVAL:
                    send_data(ws, sound_level, distance)
                    last_send_time = time.ticks_ms()

                if distance < 15:
                    print("Objet détecté à proximité!")
                    json.dumps({"message": "Metal détecté", "timestamp": time.time()})
                    LED_PIN.value(1)
                    time.sleep(1)
                    LED_PIN.value(0)
                    # time.sleep(5)

                if sound_level > SOUND_THRESHOLD:
                    print("Souffle détecté!", sound_level)
                    LED_PIN.value(1)
                    time.sleep(1)
                    LED_PIN.value(0)

                time.sleep(0.1)

    except KeyboardInterrupt:
        print("Arrêt demandé par l'utilisateur")
    except Exception as e:
        print(f"Erreur: {e}")
    finally:
        if ws:
            ws.close()
            print("Connexion WebSocket fermée")

if __name__ == "__main__":
    main()
