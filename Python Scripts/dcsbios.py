import socket
import struct
import serial
import threading
import time
from dcsb_parser import DcsbiosFramer

# === CONFIGURATION ===
DCS_PC_IP = "192.168.1.2" # Change to the IP address of the PC running DCS
UDP_IP = "0.0.0.0" # Listen on all interfaces
UDP_PORT = 5010 # DCS-BIOS default UDP port
UDP_DEST_PORT = 7778 # Port to send serial data to DCS-BIOS
MULTICAST_GROUP = "239.255.50.10" # DCS-BIOS default multicast group
BAUDRATE = 250000 # This can be changed if required, eg "baudrate": 115200

# Note that your ISP may block multicast traffic.  In that case, you may try changing to the default VOD streaming multicast address of 224.0.0.251.

# --- Define your serial devices as a list of dictionaries ---
SERIAL_DEVICES = [
    {
        "name": "AFCS", # Name of the device for logging purposes
        "port": "/dev/ttyACM0", # This is the serial port for the AFCS device
        "baudrate": BAUDRATE, # This can be changed if required, eg "baudrate": 115200
        "enabled": True # Set to False to disable this device
    },
    {
        "name": "ICS",
        "port": "/dev/ttyACM1",
        "baudrate": BAUDRATE,
        "enabled": True
    },
    {
         "name": "FUEL",
         "port": "/dev/ttyACM2",
         "baudrate": BAUDRATE,
         "enabled": True
    },
    {
         "name": "ENGINE_START",
         "port": "/dev/ttyACM3",
         "baudrate": BAUDRATE,
         "enabled": True
    },
    {
         "name": "VOR/ILS",
         "port": "/dev/ttyACM4",
         "baudrate": BAUDRATE,
         "enabled": True
    },
    {
         "name": "O2",
         "port": "/dev/ttyACM5",
         "baudrate": BAUDRATE,
         "enabled": False
    },  
    {
         "name": "UTILITY_PANEL",
         "port": "/dev/ttyACM6",
         "baudrate": BAUDRATE,
         "enabled": False
    },
    {
         "name": "OUTBOARD_THROTTLE_PANEL",
         "port": "/dev/ttyACM7",
         "baudrate": BAUDRATE,
         "enabled": False
    },
    {
         "name": "CMS",
         "port": "/dev/ttyACM8",
         "baudrate": BAUDRATE,
         "enabled": False
    },
    {
         "name": "LEFT_SUBPANEL",
         "port": "/dev/ttyACM9",
         "baudrate": BAUDRATE,
         "enabled": False
    },
    {
         "name": "RANDOM_NANO",
         "port": "/dev/ttyUSB0",
         "baudrate": BAUDRATE,
         "enabled": False
    },
    {
        "name": "Nano2",
        "port": "/dev/ttyUSB1",
        "baudrate": BAUDRATE,
        "enabled": False
    },
    # Add more devices here
    # {
    #     "name": "ANOTHER_DEVICE",
    #     "port": "/dev/ttyUSB2",
    #     "baudrate": 152000, # Example of different baudrate for Nano or Mega
    #     "enabled": False
    # },
]

# === SETUP UDP SOCKET ===
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
udp_sock.bind((UDP_IP, UDP_PORT))

mreq = struct.pack("=4sl", socket.inet_aton(MULTICAST_GROUP), socket.INADDR_ANY)
udp_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

# === SERIAL HANDLERS (modified to take device config) ===
def serial_to_udp(device_config):
    serial_port_name = device_config["port"]
    baudrate = device_config["baudrate"]
    ser = None

    if not device_config["enabled"]:
        print(f"Serial connection for {device_config['name']} ({serial_port_name}) disabled. Skipping thread.")
        return

    print(f"Attempting to connect to serial port for {device_config['name']}: {serial_port_name}")
    while ser is None:
        try:
            ser = serial.Serial(serial_port_name, baudrate, timeout=0.1)
            print(f"Connected to serial for {device_config['name']}: {serial_port_name} at {baudrate} baud")
        except serial.SerialException as e:
            print(f"Serial error for {device_config['name']}: {e}. Retrying in 3 seconds...")
            time.sleep(3)
        except Exception as e:
            print(f"[UNEXPECTED SERIAL SETUP ERROR for {device_config['name']}] {e}. Retrying in 5 seconds...")
            time.sleep(5)

    while True:
        try:
            if ser and ser.is_open and ser.in_waiting:
                data = ser.read(ser.in_waiting)
                if data:
                    clean_data = data.replace(b'\r\n', b'\n').replace(b'\r', b'\n')
                    print(f"[{device_config['name']} SERIAL -> UDP] {clean_data.decode(errors='ignore')}", end='')
                    udp_sock.sendto(clean_data, (DCS_PC_IP, UDP_DEST_PORT))
            else:
                time.sleep(0.005)
        except (serial.SerialException, PermissionError) as e:
            print(f"[{device_config['name']} SERIAL READ ERROR] {e}. Attempting to reopen serial port...")
            try:
                if ser and ser.is_open:
                    ser.close()
            except Exception:
                pass
            time.sleep(3)
            try:
                ser = serial.Serial(serial_port_name, baudrate, timeout=0.1)
                print(f"[{device_config['name']} SERIAL RECOVERY] Reconnected to {serial_port_name}")
            except Exception as e:
                print(f"[{device_config['name']} SERIAL RECOVERY FAILED] {e}")
                time.sleep(5)
        except Exception as e:
            print(f"[UNEXPECTED SERIAL ERROR for {device_config['name']}] {e}")
            time.sleep(5)

def is_dcsbios_export_packet(data):
    return len(data) >= 4 and data[0] == 0x55 and data[1] == 0x55 and data[2] == 0x55 and data[3] == 0x55

# udp_to_serial now needs to manage multiple serial ports
# This version sends DCS-BIOS data to ALL enabled serial ports
# If you need specific DCS-BIOS data to go to specific devices,
# you'll need a more sophisticated parsing and routing mechanism.
active_serial_ports = []
def udp_to_serial():
    print(f"UDP listener started. Forwarding DCS-BIOS data to all active serial ports.")

    # Initialize active_serial_ports list with open serial connections
    for device_config in SERIAL_DEVICES:
        if device_config["enabled"]:
            serial_port_name = device_config["port"]
            baudrate = device_config["baudrate"]
            try:
                ser = serial.Serial(serial_port_name, baudrate, timeout=0.1)
                active_serial_ports.append({
                    "name": device_config["name"],
                    "port": ser
                })
                print(f"Successfully opened serial port for UDP forwarding: {device_config['name']} on {serial_port_name}")
            except serial.SerialException as e:
                print(f"Could not open serial port for UDP forwarding {device_config['name']} ({serial_port_name}): {e}. This device will not receive DCS-BIOS data.")
            except Exception as e:
                print(f"[UNEXPECTED SERIAL OPEN ERROR for UDP forwarding {device_config['name']}] {e}")

    while True:
        try:
            data, addr = udp_sock.recvfrom(1024)
            # Ignore packets that are not from configured DCS PC
            if addr[0] != DCS_PC_IP:
                continue

            # Feed incoming UDP bytes into the framer which will call our
            # callback for each complete DCS-BIOS framed export packet.
            framer.feed(data)
                        # You might want to remove this port from active_serial_ports
                        # or attempt to re-establish connection here.
        except Exception as e:
            print(f"[UDP RECEPTION ERROR] {e}")
            time.sleep(1)

# --- DCS-BIOS framer callback ---
def on_dcs_packet(packet: bytes, ts: float):
    # forward framed packet to all active serial ports
    for device_entry in active_serial_ports:
        ser = device_entry["port"]
        if ser and ser.is_open:
            try:
                ser.write(packet)
            except Exception as e:
                print(f"[{device_entry['name']}] write error: {e}")

# Instantiate the framer used by udp_to_serial()
framer = DcsbiosFramer(on_dcs_packet)

# === START THREADS ===
udp_thread = threading.Thread(target=udp_to_serial, daemon=True)
udp_thread.start()

serial_to_udp_threads = []
for device in SERIAL_DEVICES:
    if device["enabled"]:
        thread = threading.Thread(target=serial_to_udp, args=(device,), daemon=True)
        serial_to_udp_threads.append(thread)
        thread.start()

# Keep main thread running
print(f"Setup complete. Waiting for data from DCS at {DCS_PC_IP}...")
while True:
    try:
        time.sleep(1)
    except KeyboardInterrupt:
        print("\nExiting script. Closing all serial ports.")
        for device_entry in active_serial_ports:
            ser = device_entry["port"]
            if ser and ser.is_open:
                ser.close()
        # You might also want to terminate serial_to_udp threads gracefully,
        # but for daemon threads, exiting the main script usually handles this.
        break
