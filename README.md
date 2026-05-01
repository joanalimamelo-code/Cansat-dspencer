# CANSAT 2024 - Team D-spencer (v1.3)

This repository contains the flight firmware for team D-spencer, developed for the CanSat 2024 competition. The system was designed for real-time sensor data collection, telemetry transmission, and automated execution of the Secondary Mission.

---

## Hardware Used

The system is based on the Arduino Nano architecture and integrates the following components:

| Component        | Function                                              |
|------------------|-------------------------------------------------------|
| Arduino Nano     | Main Microcontroller                                  |
| Adafruit BMP280  | Pressure and Temperature (Altitude Calculation)       |
| NEO-7M GPS       | Latitude and Longitude Tracking                       |
| SG90 Servo       | Secondary Mission Actuator (Compartment Opening)      |
| APC 220          | Radio Antenna for Serial Telemetry                    |

---

## Secondary Mission: Agriculture in Space

The D-spencer secondary mission focuses on space agriculture, testing the viability of biofertilizers under extreme flight conditions.

### Scientific Objective

The goal is to analyse how microorganisms essential to agriculture react to the stress caused by gravitational acceleration during launch and flight. The study focuses on:

- **Yeasts (Saccharomyces boulardii):** Probiotics tested for their resilience.
- **Bacteria (Bacillus velezensis):** Used as biofertilizers to promote plant growth and protection against pathogens.

### Technical Execution (Trigger)

The code monitors descent and altitude to ensure sample integrity and experiment completion:

1. **Fall Detection:** The system confirms the CanSat is in stable descent (velocity < -1.0 m/s for at least 2 seconds).
2. **Altitude Activation:** When altitude $H$ drops below 100 metres (`ALTITUDE_M_TO_OPEN_DOOR`), the Arduino activates the SG90 Servo.
3. **Mechanical Action:** The servo rotates to 90°, opening the compartment door. This can be used to release the samples or prepare the CanSat for immediate post-landing recovery.
4. **Recovery:** After opening, the system enters intensive GPS transmission mode so the team can quickly locate the biological samples.

---

## Flight Logic (State Machine)

The firmware uses a Finite State Machine (FSM) to manage the mission phases:

| State                       | Description                                                                 |
|-----------------------------|-----------------------------------------------------------------------------|
| `Starting`                  | Calibrates sensors and defines the zero level ($P_0$ and $T_0$) at ground. |
| `InFlight`                  | Monitors ascent and apogee.                                                 |
| `RelevantDescent`           | Detects that the CanSat has begun falling at a steady rate.                 |
| `TestingAltitudeAndOpenDoor`| Critical phase where the system waits to reach 100 m to trigger the secondary mission. |
| `AlreadyOpenedDoor`         | Final phase focused on transmitting precise coordinates for recovery.       |
| `SendDataWithGPSInfo`       | Continues GPS coordinate transmission to support recovery operations.       |

---

## Altitude Calculations

Altitude ($H$) is calculated in real time using the barometric formula:

$$H = \frac{T_0 + 273.15}{z} \cdot \left(1 - \left(\frac{P}{P_0}\right)^{\frac{z \cdot R}{g}}\right)$$

Where:

- $z$: Thermal lapse rate ($0.0065 \, K/m$)
- $R$: Gas constant ($287.06 \, J/kg \cdot K$)
- $g$: Gravitational acceleration ($9.81 \, m/s^2$)

---

## Telemetry Format

Data is transmitted every 1000 ms via APC 220 in the following format:

```
t:[Time] ; T:[Temperature] ; P:[Pressure] ; H:[Altitude] ; Lt:[Lat] ; Lg:[Lng]
```

---

## Team D-spencer

- Antonio Cruz
- Constanca Vasco
- Duarte Carvalho
- Joana Melo
- Joao Saltao
- Matilde Mendes
