# Device Simulators

Utility scripts that emulate INTI field robots using locally attached XBee radios.

## Requirements

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Usage

```bash
python3 src/device_simulator.py
```

The simulator:

- loads robot metadata from `it-gateway/robot-data/robots.json` (or the local copy under `data/robots.json`);
- pulls message definitions from `it-gateway/config/availableMessages.json`;
- discovers each connected `/dev/ttyUSB*` or `/dev/ttyACM*` XBee and links it to a robot by MAC address;
- opens listeners for all detected robots and replies with the REP sequences declared in `availableMessages.json`;
- reports the current horizontal-plane solar irradiance using the configured site parameters (when enabled).
- reports the current horizontal-plane solar irradiance using the configuration supplied for Santiago, Chile (customisable).

Useful flags:

- `--scan-only` prints matched devices without opening listeners.
- `--response-delay <seconds>` adjusts the pause between REP frames (default: `3.0`).
- `--verbose` enables debug logs for troubleshooting.
- `--solar-interval <seconds>` controls how often the irradiance is reported (default: `10` seconds).

### Response sequences

If you need the simulator to send automated replies, declare the sequences directly in `availableMessages.json` under a `RESPONSE_SEQUENCES` section. For example:

```json
"RESPONSE_SEQUENCES": {
  "SST": {
    "play": ["success", "begining-cycle", "half-cycle", "cycle-end", "stand-by"]
  }
}
```

Each item refers to the existing `REQ`/`REP` keys, so future message updates can stay in that single JSON file without touching the simulator code.

## Configuration

Key file: `config/device_simulator.json`

```json
{
  "availableMessagesPath": "../it-gateway/config/availableMessages.json",
  "solar": {
    "latitudeDegrees": -33.4489,
    "longitudeDegrees": -70.6693,
    "clearnessFactor": 0.7,
    "logIntervalSeconds": 10,
    "enabled": true,
    "noiseStdDev": 0.1
  },
  "messageLogging": {
    "enabled": true
  }
}
```

- `availableMessagesPath` accepts an absolute path or a path relative to the simulator project root. Update this value if the control-system repo moves or the file is relocated.
- `solar.latitudeDegrees` / `solar.longitudeDegrees` specify the location used when deriving solar time and declination (defaults correspond to Santiago, Chile).
- `solar.clearnessFactor` sets the atmospheric attenuation factor `k_t` (0.3–0.8 typical).
- `solar.logIntervalSeconds` defines the default cadence (in seconds) for irradiance logging; the `--solar-interval` flag overrides it per run.
- `solar.enabled` toggles console logging; sampling continues so the latest irradiance data remains available even when the output is muted.
- `solar.noiseStdDev` adjusts the standard deviation for the Gaussian noise applied to the irradiance factor (default 0.1).
- `messageLogging.enabled` turns detailed RX/TX message logging on/off while keeping the simulator operational.
