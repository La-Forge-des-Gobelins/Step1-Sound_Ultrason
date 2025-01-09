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
NUM_LEDS = 180
NEOPIXEL_PIN = Pin(13)
led_strip = NeoPixel(NEOPIXEL_PIN, NUM_LEDS)

# Configuration WiFi
WIFI_SSID = "Cudy-EFFC"
WIFI_PASSWORD = "33954721"

# Configuration WebSocket
WEBSOCKET_URL = "ws://192.168.10.31:8080/step1-Four"

# Configuration seuil
SOUND_SPEED = 300
TRIG_PULSE_DURATION_US = 10
SOUND_THRESHOLD = 200
SEND_INTERVAL = 1000
last_send_time = 0

previous_distance = 0
previous_distance_epee = 0

# Flag pour contrôler l'effet de feu
fire_running = False

# Configuration du debug
DEBUG = False  # Mettre à True pour voir tous les messages de debug


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
    if duration < 0:
        return float('inf')
    distance_cm = (duration * SOUND_SPEED) / (2 * 10000)
    return distance_cm


async def fire_effect(strip, num_leds):
    global fire_running
    while fire_running:
        for i in range(num_leds):
            r = random.randint(180, 255)
            g = random.randint(50, 100)
            b = 0
            if random.random() > 0.8:
                r = g = b = 0
            strip[i] = (r, g, b)
        strip.write()
        await asyncio.sleep(0.05)

async def listen_websocket(ws):
    global last_send_time, fire_running, previous_distance, previous_distance_epee
    last_message_time = 0
    WEBSOCKET_CHECK_INTERVAL = 100  # Vérifier les messages toutes les 100ms
    
    while True:
        try:
            current_time = time.ticks_ms()
            if time.ticks_diff(current_time, last_message_time) >= WEBSOCKET_CHECK_INTERVAL:
                msg = ws.receive()
                
                if msg != None:
                    print("Message reçu :", msg)
                    
                last_message_time = current_time
                
                if msg and msg != "ping":  # N'afficher que les messages non-ping
                    print(f"Message reçu: {msg}")
                
                if msg == "ping":
                    ws.send("Four - pong")
                
                if msg == "start_led":
                    print("allumer les led")
                    if not fire_running:
                        fire_running = True
                        asyncio.create_task(fire_effect(led_strip, NUM_LEDS))
                    
        except Exception as e:
            print(f"WebSocket error: {e}")
            break
            
        await asyncio.sleep_ms(WEBSOCKET_CHECK_INTERVAL)

async def run_main_loop(ws):
    global last_send_time, fire_running, previous_distance, previous_distance_epee
    gc.collect()
    count = 0

    try:
        if ws.connect():
            ws.socket.setblocking(False)
            
            ws.send("connect")
            print("ESP32 connecté et prêt")

            while True:
                distance = measure_distance()
                sound_level = SOUND_SENSOR_PIN.read()
                
                current_time = time.ticks_ms()
                if time.ticks_diff(current_time, last_send_time) > SEND_INTERVAL:
                    last_send_time = current_time

                if 2 < distance < 17.4:
                    if abs(distance - previous_distance) > 0.7:
                        print("Metal détecté", distance)
                        ws.send("Metal détecté")
                    previous_distance = distance
                elif 17.5 < distance < 22:
                    if abs(distance - previous_distance_epee) > 0.7:
                        print("Epee détecté", distance)
                        ws.send("Epee détectée")
                    previous_distance_epee = distance
                    
                    
                if sound_level > SOUND_THRESHOLD:
                    count += 1
                    print(f"Souffle détecté! ({count}/3)")
                    
                    if count >= 1:
                        if not fire_running:
                            fire_running = True
                            ws.send("Souffle détecté")
                            asyncio.create_task(fire_effect(led_strip, NUM_LEDS))
                
                await asyncio.sleep_ms(100)

    except Exception as e:
        print(f"Error in main loop: {e}")

async def main():    
    if not await connect_wifi():
        print("Impossible de continuer sans connexion WiFi")
        return

    ws = WebSocketClient(WEBSOCKET_URL)
    
    if not ws:
        print("Failed to start - no connection")
        return
    
    try:
        await asyncio.gather(
            run_main_loop(ws),
            listen_websocket(ws)
        )
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if ws:
            ws.close()
            print("Connexion WebSocket fermée")
        for i in range(NUM_LEDS):
            led_strip[i] = (0, 0, 0)
        led_strip.write()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Programme interrompu par l'utilisateur")
        for i in range(NUM_LEDS):
            led_strip[i] = (0, 0, 0)
        led_strip.write()
