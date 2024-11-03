# Line Following Robot (2022-2023)

Line following robot project developed during my first year at IUT de Cachan. The robot participated in the annual line-following competition where robots must follow white lines on a dark blue carpet as fast as possible.

⚠️ Note: I no longer have access to the full project files, including the schematics and PCB designs in KiCad. The only available resource is the main code. However, I can confirm that the sensor card utilized the CNY70 sensors.


## Project Overview

### Technical Implementation
- State machine architecture with 7 distinct states for line following
- 5-sensor array with multiplexed reading
- Real-time motor control with speed adjustment
- Button and jack interface for control
- LCD display for timing and debugging

### State Machine Logic
The robot uses a sophisticated state machine for navigation:
- `etat_td`: Straight line following
- `etat_corG/etat_corD`: Small trajectory corrections
- `etat_virG/etat_virD`: Sharp turn detection
- `etat_sorG/etat_sorD`: Turn exit management

### Code Structure
```cpp
// Main loop
while(true) {
    lireCapteur();      // Read sensors
    automateBP();       // Handle buttons
    automateSuivi();    // State machine
    commandeMoteur();   // Motor control
}
```
## Video Demo


https://github.com/user-attachments/assets/56713101-6999-4cbb-93db-9e9f86febb86


This demo video is provided by [the original source](https://tinyurl.com/36ry6fmx).


