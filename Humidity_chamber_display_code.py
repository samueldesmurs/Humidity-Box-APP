import serial, struct, csv, time, math, threading, queue, sys, os
import tkinter as tk
from matplotlib.dates import DateFormatter, AutoDateLocator
from tkinter import ttk, filedialog, messagebox
from datetime import datetime
from collections import deque
from typing import Optional

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import ctypes

# =======================
#   Empêche la veille Windows pendant l'acquisition
# =======================
ES_CONTINUOUS        = 0x80000000
ES_SYSTEM_REQUIRED   = 0x00000001
ES_DISPLAY_REQUIRED  = 0x00000002

# =======================
#   Paramètres série
# =======================
PORT = "COM3"
BAUD = 9600

# --- Trames humidité (8N1) ---
HDR_AB = b"\xAB\xAB"
EOT    = b"\x0D\x0A"
WINDOW_SEC_AB = 1.0  # fenêtre de lecture d'une trame ABAB

# --- Trames température (7E1) ---
STX_PLUS = 0x2B  # '+'
WINDOW_SEC_PLUS = 1.0  # fenêtre de lecture d'une trame '+ ...'

# Fenêtre du graphe (None = tout garder)
PLOT_WINDOW_SECONDS = None  # ex: 15*60 pour 15 min

# =======================
#   Physique (IAPWS) — Wagner & Pruss
# =======================
def absolute_humidity_gm3(T_c: float, RH_pct: float) -> float:
    """Humidité absolue (g/m³) via IAPWS (Wagner & Pruss) + gaz parfaits."""
    Tk = T_c + 273.15
    theta = 1.0 - (Tk / 647.096)
    expo = (
        -7.85951783 * theta
        + 1.84408259 * (theta ** 1.5)
        - 11.7866497 * (theta ** 3.0)
        + 22.6807411 * (theta ** 3.5)
        - 15.9618719 * (theta ** 4.0)
        + 1.80122502 * (theta ** 7.5)
    )
    p_ws = 22064000.0 * math.exp((647.096 / Tk) * expo)  # Pa
    phi = RH_pct / 100.0
    rho_v_g_m3 = 1000.0 * ((p_ws * phi) / (461.5 * Tk))  # g/m³
    return rho_v_g_m3

# =======================
#   Ouverture des ports
# =======================
def open_serial_7E1():
    """Port pour les trames température '+' (7E1)."""
    return serial.Serial(
        PORT, BAUD,
        timeout=0.3, write_timeout=0.3,
        bytesize=serial.SEVENBITS, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE,
        xonxoff=False, rtscts=False, dsrdtr=False,
    )

def open_serial_8N1():
    """Port pour les trames humidité 'ABAB' (8N1)."""
    return serial.Serial(
        PORT, BAUD,
        timeout=0.3, write_timeout=0.3,
        bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
        xonxoff=False, rtscts=False, dsrdtr=False,
    )

# =======================
#   Lecture des trames
# =======================
def _read_one_abab_frame(ser, buf: bytearray) -> Optional[bytes]:
    """
    Retourne 1 trame complète AB AB ... chk 0D 0A ou None.
    Vérifie la checksum (octet juste avant 0D 0A).
    """
    buf += ser.read(256)

    i = buf.find(HDR_AB)
    if i < 0:
        if len(buf) > 4096:
            del buf[:-1024]
        return None
    if i > 0:
        del buf[:i]

    j = buf.find(EOT, 3)
    if j < 0:
        return None

    frame = bytes(buf[: j + 2])
    del buf[: j + 2]

    if not frame.startswith(HDR_AB) or not frame.endswith(EOT) or len(frame) < 6:
        return None

    chk = frame[-3]
    if (sum(frame[:-3]) & 0xFF) != chk:
        return None

    return frame

def read_one_abab_humidity():
    """
    Lit RH en 8N1 ('ABAB' ... chk CRLF).
    ⚠️ Mapping historique conservé : RH_raw = 2 premiers octets après AB AB.
    RH = rh_raw / 10.0
    Renvoie (rh_pct, rh_raw) ou (None, None).
    """
    deadline = time.time() + WINDOW_SEC_AB
    ser = open_serial_8N1()
    buf = bytearray()
    try:
        while time.time() < deadline:
            frame = _read_one_abab_frame(ser, buf)
            if frame is None:
                continue

            payload = frame[2:-3]  # tout entre l'entête et le checksum
            if len(payload) < 2:
                continue

            rh_raw = struct.unpack_from(">H", payload, 0)[0]
            rh = round(rh_raw / 10.0, 2)
            return rh, rh_raw
        return None, None
    finally:
        ser.close()

def open_serial_7E1():
    """Port pour les trames température '+' (7E1)."""
    ser = serial.Serial(
        PORT, BAUD,
        timeout=0.3, write_timeout=0.3,
        bytesize=serial.SEVENBITS, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE,
        xonxoff=False, rtscts=False, dsrdtr=False,
    )
    # Certains dongles ont besoin de DTR/RTS levés pour réveiller le module
    try:
        ser.setDTR(True)
        ser.setRTS(True)
    except Exception:
        pass
    return ser


def read_one_abab_temperature():
    """
    Lit T en 8N1 ('ABAB' ... chk CRLF).
    Structure trame: AB AB | addr | type | len | payload(len) | chk | 0D 0A
    Mapping d'après le dump: température = payload[0..1] (big-endian) en dixièmes de °C.
    Renvoie (t_C, t_raw) ou (None, None).
    """
    deadline = time.time() + WINDOW_SEC_AB
    ser = open_serial_8N1()
    buf = bytearray()
    try:
        while time.time() < deadline:
            # lire une trame complète valide
            buf += ser.read(256)
            i = buf.find(HDR_AB)
            if i < 0:
                if len(buf) > 4096:
                    del buf[:-1024]
                continue
            if i > 0:
                del buf[:i]
            j = buf.find(EOT, 3)
            if j < 0:
                continue
            frame = bytes(buf[: j + 2])
            del buf[: j + 2]

            # contrôles minimum + checksum
            if not frame.startswith(HDR_AB) or not frame.endswith(EOT) or len(frame) < 8:
                continue
            chk = frame[-3]
            if (sum(frame[:-3]) & 0xFF) != chk:
                continue

            # découpage payload
            ln = frame[4]
            start = 5
            end = start + ln
            if end > len(frame) - 2:  # -2 pour 0D 0A
                continue
            payload = frame[start:end]
            if len(payload) < 2:
                continue

            # >>> Température: 2 octets big-endian AU DÉBUT DU PAYLOAD
            t_raw = (payload[0] << 8) | payload[1]
            t = t_raw / 10.0

            # borne physique simple
            if -40.0 <= t <= 125.0:
                return round(t, 2), t_raw

        return None, None
    finally:
        ser.close()



# =======================
#   Thread d’acquisition
# =======================
class AcquisitionThread(threading.Thread):
    def __init__(self, out_queue: queue.Queue, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.q = out_queue
        self.stop = stop_event
        self.last_t = None
        self.last_rh = None
        self.last_t_raw = None
        self.last_rh_raw = None

    def run(self):
        while not self.stop.is_set():
            # Température (ABAB, 2 octets big-endian au début du payload)
            t, t_raw = read_one_abab_temperature()

            if t is not None:
                self.last_t, self.last_t_raw = t, t_raw
                self.push_sample()

            # 2) Humidité relative (8N1, trame ABAB)
            rh, rh_raw = read_one_abab_humidity()
            if rh is not None:
                self.last_rh, self.last_rh_raw = rh, rh_raw
                self.push_sample()

            time.sleep(0.05)

    def push_sample(self):
        ts = datetime.now()
        if self.last_t is not None and self.last_rh is not None:
            try:
                ah = absolute_humidity_gm3(self.last_t, self.last_rh)
            except Exception:
                ah = None
        else:
            ah = None

        self.q.put({
            "ts": ts,
            "temp_C": self.last_t,
            "RH_pct": self.last_rh,
            "AH_gm3": ah,
            "temp_raw": self.last_t_raw,
            "rh_raw": self.last_rh_raw
        })

# =======================
#   UI Tkinter + Plot
# =======================
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("TRH Monitor — Humidité absolue")
        self.geometry("1100x600")
        self.minsize(900, 500)

        # --- Queue & thread ---
        self.q = queue.Queue()
        self.stop_event = threading.Event()
        self.acq_thread = None

        # Données en mémoire (session)
        self.data = []  # dicts {"ts", "temp_C", "RH_pct", "AH_gm3", "temp_raw", "rh_raw"}

        # Tampon pour le tracé
        self.ts_buf = deque()
        self.ah_buf = deque()

        # --- Layout ---
        self.columnconfigure(0, weight=3)  # gauche = plot
        self.columnconfigure(1, weight=2)  # droite = display
        self.rowconfigure(0, weight=1)
        self.rowconfigure(1, weight=0)

        self._build_plot_frame()
        self._build_display_frame()
        self._build_controls()

        self.after(200, self._poll_queue)

    def _build_plot_frame(self):
        frame = ttk.Frame(self)
        frame.grid(row=0, column=0, sticky="nsew", padx=8, pady=8)

        fig = Figure(figsize=(5, 4), dpi=100)
        self.ax = fig.add_subplot(111)
        self.ax.set_title("Humidité absolue (g/m³) vs temps")
        self.ax.set_xlabel("Temps")
        self.ax.set_ylabel("AH (g/m³)")
        self.ax.grid(True, which="both")
        self.line, = self.ax.plot([], [], lw=2)

        # Ligne de référence rouge + annotation
        self.ref_ah = 12.45
        self.ref_line = self.ax.axhline(self.ref_ah, linestyle="--", linewidth=1.5, color="red")
        self.ref_text = self.ax.annotate(
            "Mosta's level",
            xy=(1.0, self.ref_ah),
            xycoords=("axes fraction", "data"),
            xytext=(-6, 4),
            textcoords="offset points",
            ha="right", va="bottom", color="red", fontsize=10
        )

        # Axe temps lisible
        self.ax.xaxis.set_major_locator(AutoDateLocator())
        self.ax.xaxis.set_major_formatter(DateFormatter("%H:%M:%S"))
        fig.autofmt_xdate()

        self.canvas = FigureCanvasTkAgg(fig, master=frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.fig = fig

    def _build_display_frame(self):
        frame = ttk.Frame(self)
        frame.grid(row=0, column=1, sticky="nsew", padx=8, pady=8)
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)

        self.display = tk.Label(
            frame, text="--.-- g/m³", font=("Segoe UI", 64, "bold"),
            fg="white", bg="#0d47a1", anchor="center"
        )
        self.display.grid(row=0, column=0, sticky="nsew")

        self.subtext = tk.Label(
            frame, text="T: --.- °C    RH: --.- %",
            font=("Segoe UI", 16), fg="white", bg="#0d47a1"
        )
        self.subtext.grid(row=1, column=0, sticky="ew")

    def _build_controls(self):
        bar = ttk.Frame(self)
        bar.grid(row=1, column=0, columnspan=2, sticky="ew", padx=8, pady=(0,8))
        for i in range(6):
            bar.columnconfigure(i, weight=1)

        self.btn_start = ttk.Button(bar, text="Démarrer", command=self.start_acquisition)
        self.btn_stop  = ttk.Button(bar, text="Arrêter",  command=self.stop_acquisition, state="disabled")
        self.btn_save  = ttk.Button(bar, text="Enregistrer CSV…", command=self.save_csv)
        self.lbl_status = ttk.Label(bar, text="Prêt.")

        self.btn_start.grid(row=0, column=0, padx=4, sticky="ew")
        self.btn_stop.grid(row=0, column=1, padx=4, sticky="ew")
        self.btn_save.grid(row=0, column=2, padx=4, sticky="ew")
        self.lbl_status.grid(row=0, column=3, columnspan=3, padx=4, sticky="w")

    # --- Acquisition control ---
    def start_acquisition(self):
        ctypes.windll.kernel32.SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED | ES_DISPLAY_REQUIRED)
        if self.acq_thread and self.acq_thread.is_alive():
            return
        self.stop_event.clear()
        self.acq_thread = AcquisitionThread(self.q, self.stop_event)
        self.acq_thread.start()
        self.btn_start.config(state="disabled")
        self.btn_stop.config(state="normal")
        self.lbl_status.config(text=f"Acquisition en cours sur {PORT}@{BAUD}…")

    def stop_acquisition(self):
        ctypes.windll.kernel32.SetThreadExecutionState(ES_CONTINUOUS)
        if self.acq_thread and self.acq_thread.is_alive():
            self.stop_event.set()
            self.acq_thread.join(timeout=2.0)
        self.btn_start.config(state="normal")
        self.btn_stop.config(state="disabled")
        self.lbl_status.config(text="Acquisition arrêtée.")

    # --- Queue polling / UI update ---
    def _poll_queue(self):
        updated = False
        try:
            while True:
                item = self.q.get_nowait()
                self.data.append(item)
                self._append_and_trim(item["ts"], item["AH_gm3"])
                self._update_display(item)
                updated = True
        except queue.Empty:
            pass

        if updated:
            self._redraw_plot()

        self.after(200, self._poll_queue)

    def _append_and_trim(self, ts, ah):
        self.ts_buf.append(ts)
        self.ah_buf.append(ah)
        if PLOT_WINDOW_SECONDS is not None:
            cutoff = datetime.now().timestamp() - PLOT_WINDOW_SECONDS
            while self.ts_buf and self.ts_buf[0].timestamp() < cutoff:
                self.ts_buf.popleft()
                self.ah_buf.popleft()

    def _update_display(self, item):
        T = item["temp_C"]
        RH = item["RH_pct"]
        AH = item["AH_gm3"]

        self.display.config(text=f"{AH:0.2f} g/m³" if AH is not None else "--.-- g/m³")
        t_txt = f"{T:0.1f} °C" if T is not None else "--.- °C"
        rh_txt = f"{RH:0.1f} %" if RH is not None else "--.- %"
        self.subtext.config(text=f"T: {t_txt}    RH: {rh_txt}")

    def _redraw_plot(self):
        if not self.ts_buf:
            return
        xs = list(self.ts_buf)
        ys = [float('nan') if y is None else y for y in self.ah_buf]

        self.line.set_xdata(xs)
        self.line.set_ydata(ys)

        self.ax.relim()
        self.ax.autoscale_view()

        # maintenir la ligne d'alerte et son texte
        self.ref_line.set_ydata([self.ref_ah, self.ref_ah])
        self.ref_text.set_position((1.0, self.ref_ah))

        self.fig.autofmt_xdate()
        self.canvas.draw_idle()

    # --- Save CSV ---
    def save_csv(self):
        if not self.data:
            messagebox.showinfo("Enregistrer CSV", "Aucune donnée à enregistrer.")
            return
        default_name = f"trh_session_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv")],
            initialfile=default_name,
            title="Enregistrer les données de la session"
        )
        if not path:
            return
        try:
            with open(path, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["timestamp", "temp_C", "RH_pct", "AH_gm3", "temp_raw", "rh_raw"])
                for d in self.data:
                    ts = d["ts"].strftime("%Y-%m-%d %H:%M:%S")
                    T  = f"{d['temp_C']:.2f}"  if d["temp_C"]  is not None else ""
                    RH = f"{d['RH_pct']:.2f}" if d["RH_pct"] is not None else ""
                    AH = f"{d['AH_gm3']:.2f}" if d["AH_gm3"] is not None else ""
                    w.writerow([ts, T, RH, AH, d["temp_raw"] or "", d["rh_raw"] or ""])
            self.lbl_status.config(text=f"Données enregistrées : {os.path.basename(path)}")
        except Exception as e:
            messagebox.showerror("Erreur", f"Échec de l'enregistrement : {e}")

    # --- Lifecycle ---
    def on_close(self):
        ctypes.windll.kernel32.SetThreadExecutionState(ES_CONTINUOUS)
        try:
            self.stop_acquisition()
        finally:
            self.destroy()

# =======================
#   Entrée
# =======================
if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()
