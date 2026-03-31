# Dummies Firmware
This repository is a submodule of the larger **TEIDESAT Dummies system**. It contains the C++ firmware designed to run physically on *ESP32 WROOM 32* microcontrollers.

- The purpose of these "dummies" is to act as hardware simulators to **validate optical communication protocols** using light pulses before the actual satellite launch.

You **cannot run this code directly** on a standard PC (Windows, Mac, or Linux). Because this is microcontroller firmware, it interacts with physical hardware pins and requires an ESP32 architecture.

- **To run the code:** You must compile it and flash it onto physical ESP32 boards using PlatformIO.
- **To test without hardware:** If you are developing locally and do not have the ESP32 boards or electronic components wired up, do not use this repository. We highly recommend using the `firmware_emulator.py` located in the [dummies-server](https://github.com/Teidesat/dummies-server) repository, which generates mock HTTP requests and fakes the optical packages.

## Hardware configuration
The system is split into two independent PlatformIO projects:

1. **Transmitter Dummy (transmitter-dummy).** Polls the API Server for messages and translates them into optical signals.
   - Pin 18 (LIGHT_PIN): Connect to the transmitting LED.
   - Pin 19 (DEBUG_LED_PIN): Optional debug indicator.

2. **Receiver Dummy (receiver-dummy).** Reads the optical pulses at 1MHz using a circular buffer and sends the data to the Data Server.
   - Pin 5 (SIGNAL_PIN): Connect to the photoresistor/photodiode.

## Technical architecture
### Receiver
The receiver firmware uses FreeRTOS to divide labor across the ESP32's two cores:
* **Core 1 (Main Loop):** Continuously runs `readPhotorresistor()`, sampling GPIO 5 at 1MHz and pushing the bits into a 4096-element circular buffer.
* **Core 0 (Network Task):** A dedicated FreeRTOS task (`sendBuffer`) monitors the `packageCounter`. Whenever 1024 samples are ready, it packages them, calculates a checksum, and dispatches an HTTP POST request to the Data Server.

### Transmitter
The transmitter utilizes FreeRTOS task management and hardware mutexes to ensure stable operation:
* **Core 1 (Supervisor):** The main `loop()` continuously polls the API server for the current firmware state.
* **Core 0 (Transmission Worker):** When instructed to send, it spawns the `sendLoop()` task on Core 0. This task fetches the payload and frequency, converting the ASCII string into binary light pulses (MSB first) via GPIO 18.
* **Mutex Locks:** A `std::mutex` secures the Wi-Fi hardware stack, preventing collisions when both cores attempt to execute HTTP GET requests simultaneously.

## Secrets configuration

Both dummies-server and dummies-firmware projects read their Wi-Fi credentials and backend endpoints from `include/secrets.h`, which must never be committed. Follow these steps on every machine where the firmware will be compiled or flashed:

1. Navigate to the desired dummy (transmitter or receiver) and copy the template:
   ```bash
   cd dummies-firmware/<dummy-name>/include
   cp secrets.h.example secrets.h
   ```
2. Edit the fresh `secrets.h` with the real values:
   - `WIFI_SSID` / `WIFI_PASSWORD`
   - `API_SERVER_BASE_URL` for the transmitter dummy
   - `SERVER_HOSTNAME` and `SERVER_PORT` for the receiver dummy
3. Build or upload the firmware as usual. The `.gitignore` rules in each project ensure that `secrets.h` stays local.

If a populated `secrets.h` ever gets added to Git history, remove it with `git rm --cached include/secrets.h`, rotate the exposed credentials, and recommit.
