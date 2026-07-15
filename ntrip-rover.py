import serial
import time
import socket
import base64
import sys

# --- CONFIGURATION ---
CAST_HOST = "rtk2go.com"
CAST_PORT = 2101
USER_EMAIL = "voukich@gmail.com"
SERIAL_PORT = "/dev/ttyACM0"  # Change to match your SparkFun port
BAUDRATE = 115200             # Standard SparkFun/u-blox baud rate
# ---------------------

def connect_ntrip(mountpoint="CA_SanJose_ML_X5"):
    # Initialize Serial connection to SparkFun board
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    
    # Create socket connection to RTK2GO
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((CAST_HOST, CAST_PORT))
    
    # 1. Properly format and Base64 encode the credentials (username:password)
    # RTK2GO uses your email as username and doesn't require a password (we'll use 'none')
    credentials = f"{USER_EMAIL}:none"
    encoded_creds = base64.b64encode(credentials.encode('utf-8')).decode('utf-8')
    
    # 2. Formulate Ntrip request header with standard protocol versions
    req =  f"GET /{mountpoint} HTTP/1.0\r\n"  # HTTP/1.0 is preferred by many NTRIP casters
    req += f"User-Agent: NTRIP PythonClient/1.0\r\n"
    req += f"Authorization: Basic {encoded_creds}\r\n"  # <-- Use encoded credentials
    req += f"Ntrip-Version: Ntrip/2.0\r\n"             # <-- Tells the caster you are an NTRIP client
    req += f"Connection: close\r\n\r\n"
    
    s.sendall(req.encode('utf-8'))
    
    # Read the initial caster response
    response = s.recv(1024).decode('utf-8', errors='ignore')
    
    # RTK2GO will usually respond with "ICY 200 OK" on success
    if "ICY 200 OK" in response or "HTTP/1.0 200 OK" in response or "HTTP/1.1 200 OK" in response:
        print(f"Connected successfully to {mountpoint}! Streaming RTK data...")
        
        # Continuous loop to pipe network data to serial port
        while True:
            data = s.recv(2048)
            if not data:
                print("Connection closed by server.")
                break
            ser.write(data)  # Push RTCM data straight to SparkFun board
    else:
        print("Connection failed. Response:", response)
        
    s.close()
    ser.close()

if __name__ == "__main__":
    delay = 10
    while True:
        try:
            connect_ntrip()
            delay = 10 # reset delay on successful connection 
        except Exception as e:
            print(f"Error: {e}. Reconnecting in {delay} seconds...")
            delay = min(delay * 2, 300)