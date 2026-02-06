# TRH / Absolute Humidity Monitor (Python)

This repository contains a small Python application used during my thesis to **monitor temperature, relative humidity (RH), and absolute humidity (AH)** in real time.

The app:
- reads a serial sensor stream (default: `COM3`, 9600 baud)
- computes **absolute humidity (g/m³)** from temperature and RH using an IAPWS-based saturation vapor pressure formulation
- shows a live plot + large numeric display (Tkinter + Matplotlib)
- allows **CSV export** of the session data
- prevents Windows sleep during acquisition

> Tested on **Windows**. Tkinter is included with standard Python on Windows.

---

## Quick start

### 1) Create and activate a virtual environment (recommended)

```bash
python -m venv .venv
# Windows:
.venv\Scripts\activate
# macOS/Linux:
source .venv/bin/activate
```

### 2) Install dependencies

```bash
pip install -r requirements.txt
```

### 3) Configure the serial port

Open `Humidity_chamber_display_code.py` and edit these lines near the top:

```python
PORT = "COM3"
BAUD = 9600
```

### 4) Run

```bash
python Humidity_chamber_display_code.py
```

---

## Output CSV

Use the **“Enregistrer CSV…”** button to export a file with columns:

- `timestamp`
- `temp_C`
- `RH_pct`
- `AH_gm3`
- `temp_raw`
- `rh_raw`

---

## Notes / troubleshooting

- **COM port**: if you don’t know the port, check Windows Device Manager → “Ports (COM & LPT)”.
- If your adapter/sensor requires it, the code attempts to set **DTR/RTS** to wake the module.
- The script currently assumes the sensor frames are in the historical format used in the lab.
  If you change the sensor, you may need to update the parsing functions:
  `read_one_abab_temperature()` and `read_one_abab_humidity()`.

---

## License

See `LICENSE`.
