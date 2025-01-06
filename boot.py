import network
import time
import json
from machine import Pin, ADC, time_pulse_us
from neopixel import NeoPixel
from WebSocketClient import WebSocketClient
import gc
import random
import uasyncio as asyncio

# Configuration des broches
TRIG_PIN = Pin(5, Pin.OUT)
ECHO_PIN = Pin(18, Pin.IN)
SOUND_SENSOR_PIN = ADC(Pin(34))
SOUND_SENSOR_PIN.atten(ADC.ATTN_11DB)

# Configuration du bandeau LED WS2812
NUM_LEDS = 20  # Nombre de LEDs
NEOPIXEL_PIN = Pin(13)  # GPIO connecté au bandeau LED
led_strip = NeoPixel(NEOPIXEL_PIN, NUM_LEDS)

# Configuration WiFi
WIFI_SSID = "Cudy-EFFC"
WIFI_PASSWORD = "33954721"

# Configuration WebSocket
WEBSOCKET_URL = "ws://192.168.10.31:8080/step1"

# Configuration seuil
SOUND_SPEED = 340
TRIG_PULSE_DURATION_US = 10
SOUND_THRESHOLD = 900
SEND_INTERVAL = 1000
last_send_time = 0

# Flag pour contrôler l'effet de feu
fire_running = False


async def connect_wifi():
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
            await asyncio.sleep(1)
            
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


async def fire_effect(strip, num_leds):
    """
    Simule un effet de feu rouge/orange sur un bandeau LED.
    :param strip: Instance de NeoPixel.
    :param num_leds: Nombre de LEDs dans le bandeau.
    """
    global fire_running
    while fire_running:
        for i in range(num_leds):
            # Génération d'une couleur aléatoire dans les tons rouge/orange
            r = random.randint(180, 255)  # Rouge intense
            g = random.randint(50, 100)  # Orange
            b = 0  # Pas de bleu pour éviter toute nuance jaune ou autre

            # Simuler un crépitement en éteignant aléatoirement certaines LEDs
            if random.random() > 0.8:  # 20% de chance de simuler un crépitement
                r = g = b = 0

            strip[i] = (r, g, b)

        # Mise à jour du bandeau
        strip.write()
        await asyncio.sleep(0.05)  # Pause pour créer un effet de crépitement


async def main():
    global last_send_time, fire_running
    gc.collect()
    count = 0

    if not await connect_wifi():
        print("Impossible de continuer sans connexion WiFi")
        return

    ws = WebSocketClient(WEBSOCKET_URL)

    try:
        if ws.connect():
            print("Connecté au serveur WebSocket")
            ws.socket.setblocking(False)
            
            # Envoi du message de connexion
            connect_message = "STEP 1 - ESP32 connecté"
            if ws.send(connect_message):
                print(f"Message de connexion envoyé: {connect_message}")

            while True:
                distance = measure_distance()
                sound_level = SOUND_SENSOR_PIN.read()
                
                if time.ticks_ms() - last_send_time > SEND_INTERVAL:
                    # send_data(ws, distance, sound_level)
                    print(distance, sound_level)
                    last_send_time = time.ticks_ms()

                if 0 < distance < 25:
                    print("Objet détecté à proximité!")
                    ws.send("STEP 1 - Objet détecté")
                    

                if sound_level > SOUND_THRESHOLD:
                    count = count + 1
                    print("Souffle détecté!", sound_level)
                    ws.send("STEP 1 - Souffle détecté")
                    
                    if count == 3:
                        # Démarrer l'effet de feu en arrière-plan
                        if not fire_running:
                            fire_running = True
                            asyncio.create_task(fire_effect(led_strip, NUM_LEDS))

                await asyncio.sleep(0.1)

    except KeyboardInterrupt:
        print("Arrêt demandé par l'utilisateur")
        # Éteindre le bandeau à la fin
        for i in range(NUM_LEDS):
            led_strip[i] = (0, 0, 0)
        led_strip.write()
    except Exception as e:
        print(f"Erreur: {e}")
    finally:
        if ws:
            ws.close()
            print("Connexion WebSocket fermée")
        # Éteindre le bandeau à la fin
        for i in range(NUM_LEDS):
            led_strip[i] = (0, 0, 0)
        led_strip.write()


# Lancer le programme principal avec uasyncio
try:
    asyncio.run(main())
finally:
    asyncio.new_event_loop()
