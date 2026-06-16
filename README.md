# Pixhawk Validator

A combined validation interface for Pixhawk flight controllers that checks IMU orientation, PWM output, UART ports, and GPS in a single UI. Designed to make Pixhawk validation fast, repeatable, and operator-independent at scale.

---

## What it validates

| Check | How |
|---|---|
| IMU orientation (RMSE) | Live quaternion comparison between two Pixhawks |
| PWM outputs | Teensy jig steps servos through 5 PWM positions and verifies each channel |
| UART ports (Telem1 / Telem2) | Lua script commands via Serial1/Serial2, Teensy reads back the signal |
| GPS1 / GPS2 presence | Checks both GPS ports on Pixhawk 1 for connected devices and fix status |

---

## Hardware required

- **2× Pixhawk** flight controllers (one blue "reference" unit, one under test)
- **1× Teensy 4.1** validation jig — wired to the blue Pixhawk's Telem1, Telem2, FMU PWM, IO PWM, GPS1, and GPS2 ports
- **1× Laptop** running Windows, macOS, or Linux with Node.js 18+

---

## Firmware & files required

| File | Where to load it |
|---|---|
| `arduplane (1).apj` — ArduPlane 4.6.3 | Flash to **both** Pixhawks via Mission Planner |
| `PWM_Validation_Params.param` | Load via Mission Planner → Full Parameter List → Load from file |
| `step_and_verify.lua` | Place in `/APM/scripts/` on Pixhawk 1's SD card |
| `teensy_step_verify.ino` | Flash to the Teensy 4.1 via Arduino IDE |

---

## ArduPilot parameter setup

After flashing firmware and loading the param file, verify the following on **Pixhawk 1**:

| Parameter | Value | Purpose |
|---|---|---|
| `SERVO_FUNCTION` / `SERVO_TRIM` | Per param file | Sets servo output functions and neutral trim |
| `BLH_SERVO_MASKS` | `0` | Disables BLHeli passthrough (required for PWM test) |
| `SERIAL1_PROTOCOL` | `28` | Enables Scripting on Telem1 |
| `SERIAL2_PROTOCOL` | `28` | Enables Scripting on Telem2 |
| `GPS2_TYPE` | `1` (Auto) | Enables detection of the second GPS port |
| `SCR_ENABLE` | `1` | Enables Lua scripting |

> After changing `SCR_ENABLE`, reboot the Pixhawk and confirm the `step_and_verify.lua` script is running (the scripting LED should blink).

---

## Software installation

**Requires Node.js 18 or later.** Download from [nodejs.org](https://nodejs.org).

```bash
# Clone the repository
git clone https://github.com/AirboundInc/Pixhawk-Validator.git
cd Pixhawk-Validator

# Install dependencies
npm install

# Start the server
node main.js
```

The server starts on **http://localhost:3000** — open this in any browser (Chrome recommended).

---

## Wiring the Teensy jig

Connect the Teensy jig to the **blue Pixhawk (Pixhawk 1)** only:

| Pixhawk 1 port | Teensy jig connector |
|---|---|
| Telem1 | UART input A |
| Telem2 | UART input B |
| FMU PWM (outputs 1–8) | PWM input bank A |
| IO PWM (outputs 1–8) | PWM input bank B |
| GPS1 | GPS1 passthrough |
| GPS2 | GPS2 passthrough |

Connect the Teensy itself to the laptop via USB.

---

## Using the interface

### 1. Connect the Pixhawks

- Plug both Pixhawks into the laptop via USB.
- Click **Refresh** in the Connection card to discover COM ports.
- Set **Pixhawk 1** to the port for the blue Pixhawk (the one connected to the Teensy jig).
- Set **Pixhawk 2** to the second Pixhawk.
- Select baud rate (default **115200**) and click **Start**.

Both 3D model blocks will begin moving in real time and the orientation graphs will start plotting.

### 2. Connect the Teensy

- In the **Step + Verify** card, select the Teensy's COM port from the dropdown.
- Click **Connect** — the Teensy badge turns green.

### 3. Check GPS status

The **GPS (PX1)** card shows the status of both GPS ports on Pixhawk 1 automatically:

| Badge | Meaning |
|---|---|
| Gray — | Waiting for data |
| Gray **No device** | No GPS unit detected on this port (after 5 s timeout) |
| Gray **No GPS** | Port configured but hardware not responding |
| Yellow **No Fix** | GPS unit connected, searching for satellites |
| Yellow **2D Fix** | Partial fix |
| Green **3D Fix** | Full fix — GPS healthy |
| Cyan **RTK Float / Fixed** | RTK precision fix |

### 4. Run the PWM + UART verification

- Click the **Verify** button in the Step + Verify card.
- The interface sends `STEST_ENABLE=1` to Pixhawk 1 which triggers the Lua script.
- The pipeline stages advance automatically as the Teensy reports progress:
  1. **Start ACK** — Telem1 and Telem2 acknowledge the start command
  2. **Preamble** — 2000 µs baseline signal for 1 s
  3. **Stepping** — servo outputs step through 1000 → 1250 → 1500 → 1750 → 2000 µs
  4. **Done ACK** — both UART ports acknowledge completion
  5. **Check** — Teensy reports PASS/FAIL with channel counts
  6. **Complete** — overall result shown in green (pass) or red (fail)

A failed result lists the specific channels that read 0 µs (disconnected or shorted).

### 5. Record orientation RMSE

- With both Pixhawks streaming, click **Record**.
- Move both Pixhawks through their range of motion.
- Click **Stop Rec** — the results card shows per-axis RMSE and variance for Roll, Pitch, and Yaw.

A low RMSE (< 2°) indicates the two IMUs agree well. The acceptance threshold will be defined once a mechanical jig is in place.

---

## Understanding the results

| Metric | Good | Review |
|---|---|---|
| Angular RMSE | < 2° | > 5° |
| GPS status | 3D Fix on both ports | No device / No GPS |
| PWM / UART | PASS 16/16 channels | Any FAIL |

A complete validation run covers all three checks. The interface is designed so a single operator can validate one Pixhawk in under two minutes without interpreting raw data.

---

## Repository structure

```
├── main.js          # Node.js server — serial ports, MAVLink, WebSocket
├── mavlink.js       # MAVLink v1/v2 parser and frame builder
├── package.json     # Node.js dependencies (serialport, ws)
└── public/
    ├── index.html   # UI layout
    ├── app.js       # Browser application — Three.js, charts, WebSocket client
    └── style.css    # Dark-theme stylesheet
```

---

## Troubleshooting

**Pixhawk not detected**
- Try a different USB cable (data cable, not charge-only).
- On Windows, install the [CP210x](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers) or [CH340](https://www.wch-ic.com/downloads/CH341SER_EXE.html) driver for your board.

**GPS2 stays "No device"**
- Confirm `GPS2_TYPE` is set to `1` (Auto) and the Pixhawk has been rebooted.
- Confirm the GPS cable is seated on the GPS2 port (not GPS1).

**Verify button does nothing**
- Pixhawk 1 must be streaming before Verify is available.
- Confirm `SCR_ENABLE=1` and that `step_and_verify.lua` is on the SD card in `/APM/scripts/`.
- Confirm the Teensy is connected and its badge shows green.

**PWM channels show FAIL**
- Check the physical wiring between the Pixhawk PWM outputs and the Teensy jig.
- Confirm `BLH_SERVO_MASKS=0` — BLHeli passthrough will prevent normal PWM output.
