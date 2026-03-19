import serial
import time
import ctypes
import os
import struct
import socket

# --- CONFIGURATION ---
# Motor Constants
CURRENT_SCALE = 2048.0   
VOLTAGE_SCALE = 512.0    
LEFT_SIDE_IDS = [1, 2, 3]
RIGHT_SIDE_IDS = [4, 5, 6]

# Communication Ports
RADIO_PORT = '/dev/ttyUSB0'
RADIO_BAUD = 9600

# Arduino Ports (Prioritizing the specific CH341 paths)
ARDUINO_PORTS = [
    '/dev/ttyCH341USB0', 
    '/dev/ttyCH341USB1', 
]
ARDUINO_BAUD = 9600

# --- SECTION 1: SYSTEM INITIALIZATION ---

# A. Configure CAN Interface
print("\n[INIT] Configuring CAN interface...")
os.system("sudo ip link set can1 down")
time.sleep(0.1)
# Auto-restart handles noise/buffer errors
os.system("sudo ip link set can1 up type can bitrate 1000000 restart-ms 100")
time.sleep(0.1)

# B. Initialize Spark Max Bridge
print("[INIT] Initializing Spark Max Bridge...")
lib = None
try:
    lib = ctypes.CDLL("/home/urc/sparkcan/build/libspark_bridge.so")
    # Initialize all 6 motors
    for mid in LEFT_SIDE_IDS + RIGHT_SIDE_IDS:
        lib.init_motor(b"can1", mid)
    print(" -> 6 Motors Initialized.")
except Exception as e:
    print(f" -> BRIDGE ERROR: {e}")
    print("    (Motor control will fail, but LEDs might still work)")

# C. Connect to Arduino (LEDs)
print("[INIT] Searching for Arduino (LEDs)...")
arduino = None
for port in ARDUINO_PORTS:
    try:
        candidate = serial.Serial(port, ARDUINO_BAUD, timeout=1)
        arduino = candidate
        print(f" -> Arduino FOUND on {port}")
        break
    except: pass

if not arduino:
    print(" -> WARNING: Arduino not found. LEDs will not function.")
else:
    time.sleep(2) # Wait for Arduino reset

# D. Connect to Radio
print("[INIT] Connecting to Radio...")
ser = None
try:
    ser = serial.Serial(RADIO_PORT, RADIO_BAUD, timeout=0.1)
    print(f" -> Radio Connected on {RADIO_PORT}")
except:
    print(" -> WARNING: No Radio Found. Rover is in passive mode.")

# E. Setup Telemetry Listener
can_sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
try:
    can_sock.bind(('can1',))
    can_sock.setblocking(False)
    print("[INIT] Telemetry Listener Active.")
except Exception as e:
    print(f" -> SOCKET ERROR: {e}")

# --- SECTION 2: RUNTIME VARIABLES ---
left_power = 0.0
right_power = 0.0
last_packet_time = time.time()
last_telemetry_time = time.time()
motor_data = { mid: {'amps': 0.0} for mid in LEFT_SIDE_IDS + RIGHT_SIDE_IDS }

def parse_telemetry(can_id, data):
    """Parses incoming CAN frames to extract Motor Current."""
    if len(data) < 8: return
    device_id = can_id & 0x3F
    api_id = (can_id >> 6) & 0x3FF
    
    # Class 46 (0x2E0) Status 1: Contains Current & Voltage
    if api_id == 0x2E0: 
        raw_curr = struct.unpack('<h', data[0:2])[0]
        if device_id in motor_data:
            motor_data[device_id]['amps'] = abs(raw_curr / CURRENT_SCALE)

print("\n--- ROVER SYSTEMS ONLINE ---")

# --- SECTION 3: MAIN LOOP ---
try:
    while True:
        # 1. READ CAN TELEMETRY (Drain buffer)
        try:
            while True:
                frame = can_sock.recv(16)
                can_id, can_dlc, data = struct.unpack("<IB3x8s", frame)
                parse_telemetry(can_id & 0x1FFFFFFF, data)
        except BlockingIOError: pass 

        # 2. READ RADIO COMMANDS
        # Expecting: <L_VAL, R_VAL, LED_CHAR> e.g., <0.5, -0.5, R>
        if ser and ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('<') and line.endswith('>'):
                    parts = line[1:-1].split(',')
                    
                    # Check if we have at least Motor Data
                    if len(parts) >= 2:
                        left_power = float(parts[0])
                        right_power = float(parts[1])
                        last_packet_time = time.time()
                        
                        # Check if we ALSO have LED Data (Index 2)
                        if len(parts) >= 3 and arduino:
                            led_cmd = parts[2].strip()
                            arduino.write(led_cmd.encode())
                            
            except ValueError: pass

        # 3. SAFETY TIMEOUT (Stop motors if radio silence > 0.5s)
        if time.time() - last_packet_time > 0.5:
            left_power = 0.0
            right_power = 0.0

        # 4. WRITE POWER TO MOTORS
        if lib:
            # Left Side (Inverted)
            l_final = left_power * 1.0
            for mid in LEFT_SIDE_IDS:
                lib.set_power(mid, ctypes.c_float(l_final))
            
            # Right Side (Standard)
            r_final = right_power * -1.0 
            for mid in RIGHT_SIDE_IDS:
                lib.set_power(mid, ctypes.c_float(r_final))

        # 5. SEND TELEMETRY TO LAPTOP (10Hz)
        if time.time() - last_telemetry_time > 0.1:
            packet_data = []
            for i in sorted(motor_data.keys()):
                packet_data.append(f"M{i}:{motor_data[i]['amps']:.2f}")
            
            packet_str = "[" + ",".join(packet_data) + "]\n"
            
            if ser:
                ser.write(packet_str.encode('utf-8'))
            
            last_telemetry_time = time.time()

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\n[SHUTDOWN] Stopping...")
finally:
    # Safe Shutdown
    if lib:
        for mid in LEFT_SIDE_IDS + RIGHT_SIDE_IDS:
            lib.set_power(mid, ctypes.c_float(0.0))
    if arduino:
        arduino.close()
    os.system("sudo ip link set can1 down")


