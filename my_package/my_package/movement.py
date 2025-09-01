from pymavlink import mavutil
from pynput import keyboard
import time

# Συνδέσου με MAVLink
print("🔌 Σύνδεση με drone μέσω MAVLink...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
print("✅ Συνδέθηκε! System ID:", master.target_system)

def get_current_yaw():
    msg = master.recv_match(type='ATTITUDE', blocking=True)
    return msg.yaw  # σε rad (radians)



initial_yaw = get_current_yaw()


# Λειτουργία: στέλνει velocity εντολές στο NED frame
def send_velocity(vx, vy, vz, duration=1):
    msg = master.mav.set_position_target_local_ned_encode(
        0,  # time_boot_ms
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111001111,  # ✅ Χρησιμοποιεί ΜΟΝΟ velocity, αγνοεί yaw και yaw_rate
        0, 0, 0,             # position (ignored)
        vx, vy, vz,          # velocity in m/s
        0, 0, 0,             # acceleration (ignored)
        0, 0                 # yaw, yaw_rate (ignored λόγω bitmask)
    )

    for _ in range(duration * 10):
        master.mav.send(msg)
        time.sleep(0.1)




# Τι κάνει κάθε πλήκτρο
def on_press(key):
    try:
        if key == keyboard.Key.up:
            print("⬆️ Μπροστά")
            send_velocity(1, 0, 0)
        elif key == keyboard.Key.down:
            print("⬇️ Πίσω")
            send_velocity(-1, 0, 0)
        elif key == keyboard.Key.left:
            print("⬅️ Αριστερά")
            send_velocity(0, -1, 0)
        elif key == keyboard.Key.right:
            print("➡️ Δεξιά")
            send_velocity(0, 1, 0)
        elif key.char == 'q':
            send_velocity(0, 0, 1)
        elif key.char == 'v':
            send_velocity(0, 0,-1)
        else:    return False
    except AttributeError:
        pass

listener = keyboard.Listener(on_press=on_press)
listener.start()

print("🎮 Χειρίσου το drone με τα arrow keys (τερματικό). Πάτα 'q' για προσγείωση.")
listener.join()
# Ακροατής πληκτρολογίου