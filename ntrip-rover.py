import serial
import time
import socket
import base64
import threading
import sys

# --- CONFIGURATION ---
CAST_HOST = "rtk2go.com"
CAST_PORT = 2101
USER_EMAIL = "voukich@gmail.com"
SERIAL_PORT = "/dev/ttyACM0"  # Change to match your SparkFun port
BAUDRATE = 115200             # Standard SparkFun/u-blox baud rate
MOUNTPOINT = "CA_SanJose_ML_X5"
# ---------------------

# Global variable to share the latest GGA sentence from the SparkFun board to the NTRIP socket
latest_gga = ""
gga_lock = threading.Lock()

def rtk_status_string(fix_quality):
    mapping = {
        "0": "No Fix",
        "1": "GNSS Fix (Standard)",
        "2": "DGNSS Fix",
        "4": "RTK FIXED (Centimeter Lock!)",
        "5": "RTK FLOAT (Acquiring...)"
    }
    return mapping.get(fix_quality, f"Unknown status ({fix_quality})")

def serial_reader_and_caster_uploader(ser, sock):
    """Reads from SparkFun board, prints RTK status, and sends updates back to the Caster."""
    global latest_gga
    print("Started Serial Reader thread...")
    
    last_caster_update = 0
    
    while ser.is_open:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                # Check for the GGA sentence which contains fix status
                if line.startswith(("$GNGGA", "$GPGGA")):
                    parts = line.split(',')
                    if len(parts) > 6:
                        fix_quality = parts[6]
                        satellites = parts[7] if len(parts) > 7 else "Unknown"
                        
                        print(f"[STATUS] Fix: {rtk_status_string(fix_quality)} | Sats used: {satellites}")
                        
                        # Store it so the caster can see our position if needed
                        with gga_lock:
                            latest_gga = line + "\r\n"
                        
                        # Send position back to RTK2GO every 5 seconds to keep the connection alive
                        current_time = time.time()
                        if current_time - last_caster_update > 5:
                            try:
                                sock.sendall(latest_gga.encode('utf-8'))
                                last_caster_update = current_time
                            except socket.error:
                                # Socket handler in main thread will pick up the disconnect
                                pass
        except Exception as e:
            print(f"[Serial Thread Error]: {e}")
            break
        time.sleep(0.1)

def connect_ntrip():
    global latest_gga
    
    # Initialize Serial connection to SparkFun board
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    ser.flushInput()
    ser.flushOutput()
    
    # Create socket connection to RTK2GO
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((CAST_HOST, CAST_PORT))
    
    # Encode credentials
    credentials = f"{USER_EMAIL}:none"
    encoded_creds = base64.b64encode(credentials.encode('utf-8')).decode('utf-8')
    
    # Format Ntrip request header
    req =  f"GET /{MOUNTPOINT} HTTP/1.0\r\n"
    req += f"User-Agent: NTRIP PythonClient/1.0\r\n"
    req += f"Authorization: Basic {encoded_creds}\r\n"
    req += f"Ntrip-Version: Ntrip/2.0\r\n"
    req += f"Connection: close\r\n\r\n"
    
    s.sendall(req.encode('utf-8'))
    
    # Read the initial caster response
    response = s.recv(1024).decode('utf-8', errors='ignore')
    
    if any(status in response for status in ["ICY 200 OK", "HTTP/1.0 200 OK", "HTTP/1.1 200 OK"]):
        print(f"Connected successfully to {MOUNTPOINT}! Starting threads...")
        
        # Spin up the background thread to handle reading the SparkFun serial responses
        t = threading.Thread(target=serial_reader_and_caster_uploader, args=(ser, s), daemon=True)
        t.start()
        
        # Main thread loop: Blocking network reads -> pipe directly to serial
        while True:
            data = s.recv(2048)
            if not data:
                print("Connection closed by server.")
                break
            ser.write(data)  # Push RTCM data straight to SparkFun board
    else:
        print("Connection failed. Response from caster:", response)
        
    s.close()
    ser.close()

if __name__ == "__main__":
    delay = 5
    while True:
        try:
            connect_ntrip()
            delay = 5 
        except Exception as e:
            print(f"Error: {e}. Reconnecting in {delay} seconds...")
            time.sleep(delay)
            delay = min(delay * 2, 60)