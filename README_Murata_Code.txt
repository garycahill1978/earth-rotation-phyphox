Murata SCH16T-K01 + ESP32 + phyphox
Earth-rotation ABBA experiment
========================================

Main sketch
-----------
phyphoxMuRatav3.ino

This is the working ESP32 sketch used for the guided Murata SCH16T-K01
Earth-rotation experiment.

What the sketch does
--------------------
- Reads the Murata SCH16T-K01 over SPI.
- Uses 50 Hz scientific acquisition.
- Averages 5 samples for the ~10 Hz live BLE display.
- Runs a guided ABBA sequence:
    A1 -> B1 -> B2 -> A2
- Uses 5 s settling, 10 s measurement blocks, and 10 s rotation windows.
- Calculates the Earth-rotation result and sends the result to phyphox.
- Builds the phyphox experiment directly on the ESP32, so a separate
  .phyphox experiment file is not required for this version.

ESP32 -> Murata wiring
----------------------
Murata MISO  -> ESP32 GPIO 19
Murata CS    -> ESP32 GPIO 5
Murata SCK   -> ESP32 GPIO 18
Murata MOSI  -> ESP32 GPIO 23
Murata GND   -> ESP32 GND
Murata 3V3   -> ESP32 3V3
Murata VDDIO -> ESP32 3V3
RESET is not used in the sketch (RESET_PIN = -1).

Arduino requirements
--------------------
1. ESP32 board support in Arduino IDE:
   "esp32 by Espressif Systems"

2. SCH16T library:
   smplboards-SCH16T
   https://github.com/HTY2003/smplboards-SCH16T

3. phyphox BLE Arduino library:
   https://github.com/phyphox/phyphox-arduino

4. SPI
   Included with the ESP32 Arduino core.

Important reproducibility note
------------------------------
Record the exact ESP32 core, smplboards-SCH16T, and phyphox BLE library
versions used for the final publication build. The working SCH16T library
copy used in the experiment should be preserved if it contains local fixes.

Basic use
---------
1. Wire the SCH16T-K01 board to the ESP32.
2. Install the ESP32 board package and the two libraries above.
3. Open phyphoxMuRatav3.ino in Arduino IDE.
4. Select the correct ESP32 board and COM port.
5. Compile and upload.
6. Open Serial Monitor at 115200 baud.
7. Check for:
       *** SCH16T READY ***
8. Open phyphox on the phone and connect to the BLE device "Murata1".
9. In the guided experiment:
       Delete data
       place Y+ North
       Start
       follow the on-screen rotations
       Pause when finished to inspect/export results.

Measurement configuration in this sketch
-----------------------------------------
Sampling:        50 Hz
Live BLE rate:   ~10 Hz (5-sample average)
Settling:        5 s
Measurement:     10 s per ABBA block
Rotation window: 10 s
Primary axis:    Y
BLE name:        Murata1

Licensing / attribution
-----------------------
This package does NOT change the licence of third-party libraries.
Keep the original licence and copyright notices supplied with
smplboards-SCH16T and phyphox-arduino.

No separate licence has been assigned here to Gary Cahill's experiment
sketch. Add one if you want others to be explicitly permitted to reuse
or modify the sketch.

Website files
-------------
For the Murata page, keep these files in the same GitHub Pages folder:
- murata.html
- phyphoxMuRatav3.ino
- README_Murata_Code.txt
- Wiring_muRata_1.png
- Murata_ABBA_runs.png
- Murata_Allan.png
