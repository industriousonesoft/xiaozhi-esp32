# Audio Debug Dashboard

This tool receives ESP32-S3 Korvo-1 audio telemetry over UDP and serves a
local browser dashboard.

```bash
python3 -m pip install -r scripts/audio_debug_dashboard/requirements.txt
PYTHONPATH=scripts/audio_debug_dashboard \
  python3 -m audio_debug_dashboard
```

Open `http://127.0.0.1:8000`, enter the ESP32 IP address, and connect.
The computer firewall must allow inbound UDP traffic on port `8002`.

Firmware requirements:

- Build the `esp32s3-korvo-1` board.
- Enable `CONFIG_USE_AUDIO_DEBUG_DASHBOARD`.
- The default device control port is UDP `8001`.

Recordings are written under `recordings/` in the current directory.
