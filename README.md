# cs-motomed-controller

cs-motomed-controller is a custom controller for a rehabilitation exercise bike based on Arduino UNO R4 WiFi. The project repurposes an Cinesport RECK MOTOmed Viva 1
physiotherapy bicycle by bypassing its original control electronics and instead using a
24 V Dunkermotoren GR63×55 DC motor, driven by a BTS7960 H-bridge motor driver. Motor current is measured with a single ACS758-050B Hall-effect current sensor so the firmware can detect muscle spasms and react automatically to protect the rider.

> ⚠️ **Safety first** – Modifying medical or rehabilitation hardware carries
> real risk. Double-check your wiring, verify the firmware on a bench before a
> person ever uses the bike, and consult a clinician or biomedical engineer if
> you are unsure about any change. You are responsible for the safe operation of
> your equipment.

## Key Features

- Bidirectional speed control for the 24 V DC motor via the BTS7960 driver.
- Continuous motor-current monitoring using an ACS758 sensor with exponential
  smoothing to reject noise.
- Multi-step spasm recovery sequence that stops the bike, attempts a gentle
  restart, and reverses direction if the load spike persists.
- Countdown session timer that halts the motor when the scheduled ride ends.
- Wi-Fi web interface served directly from the UNO R4 for starting/stopping
  rides, adjusting speed and direction, and monitoring live telemetry.
- On-device current-sensor calibration workflow so the ACS758 offsets can be
  re-zeroed without reflashing the board.

## Hardware Overview

| Component                                     | Notes                                                                          |
| --------------------------------------------- | ------------------------------------------------------------------------------ |
| Arduino UNO R4 WiFi                           | Renesas RA4M1-based MCU with on-board Wi-Fi (WiFiS3).                          |
| BTS7960 43 A H-bridge                         | Drives the 24 V brushed DC motor in both directions.                           |
| ACS758-050B (50 A)                            | Linear Hall-effect current sensor with galvanic isolation. One sensor is used. |
| 24 V Dunkermotoren GR63×55 DC Motor           | The original MOTOmed Viva 1 traction motor.                                    |
| DC25A 600W Buck Constant Current Power Module | DC12-75V DC Buck Converter Adjustable Regulator LED Driver for power supply.   |
| PLR DD4012SA 1A DC 5-40V to 12V Regulator     | DC-DC Step-Down Buck Converter Module Board for voltage regulation.            |
| Bridge Rectifier (10pcs GBJ3510 ZIP)          | 35A 1000V new original bridge rectifier for AC to DC conversion.               |
| 50V 4700UF Capacitors                         | Used in the power circuit for smoothing and filtering.                         |
| 100k Resistors                                | Used in various circuit configurations.                                        |
| Aluminium Alloy Casing                        | Enclosure for the electronics with 3D printed hold supports inside for boards. |
| Power Supply                                  | 24 V supply sized for peak motor current.                                      |
| Misc. wiring & protection                     | Fuses, emergency stop, chassis bonding, etc.                                   |

Adapt the wiring harness to your bike while keeping galvanic isolation between
low-voltage control electronics and the 24 V power stage.

## Repository Layout

```
cs-motomed-controller/
├── cs-motomed-controller.ino   # Main Arduino sketch
├── README.md                  # Project documentation (this file)
├── arduino_secrets.h.example  # Template for Wi-Fi credentials (not used directly)
├── sketch.json                # Arduino CLI/IDE board settings
└── thingProperties.h          # Legacy Arduino IoT Cloud stub (unused but retained)
```

## Getting Started

1. **Clone the repository**

   ```bash
   git clone https://github.com/<your-account>/cs-motomed-controller.git
   ```

2. **Install the board support**
   - Add the Arduino UNO R4 packages in the Arduino IDE or with `arduino-cli
board install arduino:renesas_uno`.
   - Install the `WiFiS3` library if it is not already bundled.

3. **Create your Wi-Fi secrets file**
   - Copy the template so the actual credentials stay out of Git history:
     ```bash
     cp arduino_secrets.h.example arduino_secrets.h
     ```
   - Edit `arduino_secrets.h` locally and replace the placeholder values with
     your network SSID (e.g. `Plus Innovative`) and password.
   - Keep `arduino_secrets.h` untracked – a `.gitignore` entry is provided so you
     cannot accidentally commit your credentials.

4. **Wire the hardware**
   - Connect the UNO R4 PWM pins `RPWM`/`LPWM` and enable pins `R_EN`/`L_EN` to
     the BTS7960 driver.
   - Route the motor supply through the ACS758 so the sensor measures the motor
     current. Feed the sensor outputs into `A0` (forward) and `A1` (reverse).
   - Add emergency-stop hardware and ensure the chassis is properly grounded.

5. **Flash the firmware**
   - Open `cs-motomed-controller.ino` in the Arduino IDE and select the UNO R4 WiFi target.
   - Upload the sketch. Use the serial monitor at `115200` baud to watch the
     boot log and note the IP address once the board joins Wi-Fi.

6. **Calibrate the current sensor**
   - With the motor unloaded, open the web interface at
     `http://<board-ip-address>/` and click **Calibrate Sensors**.
   - The firmware samples the ACS758 output and stores the offsets so spasm
     detection uses the correct baseline.

7. **Ride safely**
   - Start a session from the web UI, set your desired speed/direction, and
     monitor the live current draw and countdown timer.

## Web Interface

The onboard web UI exposes three endpoints:

- `/` – Single-page control dashboard served from flash (no external assets).
- `/status` – JSON payload with current telemetry (useful for automation).
- `/control` – Query-string API for integrating external clients, e.g.:
  `GET /control?action=start&speed=45&direction=forward`.

Every 1.5 seconds the page refreshes the status pane so you can confirm motor
state, session timer, Wi-Fi link, and latest current readings.

## Contributing & Workflow

- **Do not commit secrets.** Keep `arduino_secrets.h` local; the template file is
  provided for contributors.
- **Format & lint.** Match the existing C++ style (2 spaces, concise comments).
- **Test on hardware.** Simulate spasm events by loading the motor before asking
  for pull requests.
- **Document changes.** Update this README (or add docs in `docs/`) whenever you
  adjust the wiring, firmware flow, or safety assumptions.

Feel free to open issues describing enhancements, bug reports, or alternative
recovery profiles for different therapeutic needs.

## Safety & Legal Notice

- Disabling or bypassing the original MOTOmed control system voids any warranty
  and shifts responsibility for safety and regulatory compliance to you.
- Ensure the rider can always reach a physical emergency-stop switch that
  removes motor power regardless of firmware state.
- Log real sessions and consult with the rider’s clinician before deploying new
  firmware.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for the full text.
