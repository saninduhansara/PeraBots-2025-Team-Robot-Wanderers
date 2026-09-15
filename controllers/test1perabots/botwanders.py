import subprocess
import threading
import time
from itertools import count

import numpy as np
import re
import os

ERROR_LOG = "error.txt"
PID_LOG = "pid_constants.txt"
KU = 0.5  # Ultimate gain (manual or auto)

import subprocess
import os
import time
count=0
def launch_webots():
    cmd = [
        r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe",
        "--stdout",
        "D:/Maze Solver/New folder/FINAL/Undergraduate_exampleArena_new - Copy - Copy FINAL (2)/Undergraduate_exampleArena_new - Copy - Copy FINAL/worlds/pera bots.wbt"
    ]
    # Redirect Webots output to DEVNULL (no terminal clutter)
    subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def read_error_log():
    times, errors = [], []
    try:
        with open(ERROR_LOG, "r") as f:
            for line in f:
                #print(f"[DEBUG] Line: {line.strip()}")
                match = re.match(r"Time: ([\d.]+), Error: ([\d.-]+)", line)
                if match:
                    t = float(match.group(1))
                    e = float(match.group(2))
                    #print(f"[DEBUG] Matched - Time: {t}, Error: {e}")
                    times.append(t)
                    errors.append(e)
                else:
                    print("[DEBUG] No match")
    except FileNotFoundError:
        return [], []

    return times, errors

def detect_zero_crossings(times, errors):
    zero_crossings = []
    for i in range(1, len(errors)):
        if errors[i - 1] * errors[i] < 0:
            t1, t2 = times[i - 1], times[i]
            e1, e2 = errors[i - 1], errors[i]
            crossing_time = t1 + (0 - e1) * (t2 - t1) / (e2 - e1)
            zero_crossings.append(crossing_time)
    return zero_crossings

def calculate_tu(zero_crossings):
    if len(zero_crossings) < 3:
        return None
    periods = [zero_crossings[i + 2] - zero_crossings[i] for i in range(len(zero_crossings) - 2)]
    return np.mean(periods)

def calculate_pid_constants(ku, tu):
    kp = 0.6 * ku
    ki = 1.2 * ku / tu
    kd = 0.075 * ku * tu
    return kp, ki, kd

def save_pid_constants(kp, ki, kd):
    with open(PID_LOG, 'w') as f:
        f.write(f"Kp: {kp:.4f}\nKi: {ki:.4f}\nKd: {kd:.4f}\n")
    print(f"[Python] Updated PID: Kp={kp:.4f}, Ki={ki:.4f}, Kd={kd:.4f}")

def monitor_and_update():
    count = 0
    while count<100:
        time.sleep(5)
        times, errors = read_error_log()
        print("iii")
        if len(times) < 5:
            continue
        zero_crossings = detect_zero_crossings(times, errors)
        tu = calculate_tu(zero_crossings)
        if tu:
            kp, ki, kd = calculate_pid_constants(KU, tu)
            save_pid_constants(kp, ki, kd)
        else:
            print("[!] Not enough zero crossings yet.")
        count+=1
        print(times,zero_crossings)

# --------- MAIN ---------
if __name__ == "__main__":
    if os.path.exists(PID_LOG):
        os.remove(PID_LOG)
    if os.path.exists(ERROR_LOG):
        os.remove(ERROR_LOG)

    print("[*] Starting Webots...")
    launch_webots()

    print("[*] Monitoring and auto-tuning PID...")
    monitor_and_update()
    print("LLL")

