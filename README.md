# Dummies Firmware

This submodule contains the PlatformIO projects for both transmitter and receiver dummies. Each dummy ships its own `src`, `include`, and configuration files, so they can be flashed independently onto an ESP32 board.

## Secrets configuration

Both projects read their Wi-Fi credentials and backend endpoints from `include/secrets.h`, which must never be committed. Follow these steps on every machine where the firmware will be compiled or flashed:

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
