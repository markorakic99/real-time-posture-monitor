"""
DZ2 - Pametni monitor za drzanje kicme (Python deo)
Praktikum iz merno-akvizicionih sistema (13E052PMS)

Prijem i obrada podataka poslatih sa Arduino ploce, i graficki
interfejs (tkinter). Arduino strana samo ocitava senzore i izvrsava
komande; sva logika (kalibracija, racunanje ugla nagiba, brojanje
prekrsaja, tajmovanje, cuvanje sesije) je ovde.

Komunikacioni protokol (ASCII linije zavrsene karakterom '\n'):
  Arduino -> PC:  "A,<ax>,<ay>,<az>"   sirove vrednosti akcelerometra (0-1023)
                  "T"                  detektovan dodir senzora
  PC -> Arduino:  "START"              pokreni akviziciju (Timer1)
                  "STOP"               zaustavi akviziciju, ugasi LED/displej
                  "CNT,<n>"            prikazi broj n na displeju
                  "LED,1" / "LED,0"    upali / ugasi LED upozorenje

Pretpostavke (nisu eksplicitno definisane u tekstu zadatka):
  - Ugao nagiba se racuna preko kosinusa izmedju trenutnog vektora
    ubrzanja i kalibrisanog (referentnog, uspravnog) vektora. Sirova
    ADC ocitavanja NISU cista gravitacija - svaka osa ima veliki
    zajednicki "zero-g" ofset (isti napon kad na tu osu ne deluje
    ubrzanje) na koji se dodaje mala promena usled same gravitacije.
    Taj ofset se metodom najmanjih kvadrata procenjuje iz podataka
    prikupljenih tokom faze testa nagiba (razlicite orijentacije) i
    oduzima se pre racunanja ugla - bez toga ofset dominira vektorom i
    ugao ispadne mnogo manji od stvarnog fizickog nagiba. Apsolutna
    osetljivost (mV/g) i dalje nije potrebna, samo ofset.
  - Arduino se detektuje kao prvi dostupan serijski (USB) port; ako
    zelite fiksni COM port, postavite SERIAL_PORT_OVERRIDE ispod.
  - "Prekrsaj" se broji jednom po ulasku u zonu loseg drzanja (uzlazna
    ivica prelaska praga), a ne po svakom uzorku.
  - LED upozorenje na Arduinu se pali kada neprekidno trajanje loseg
    drzanja predje minimalno dozvoljeno vreme uneto u GUI, i gasi se
    cim korisnik ispravi drzanje.
  - Sesija (i cuvanje log fajla) se zavrsava pritiskom na Stop ili
    zatvaranjem prozora.
"""

import math
import time
import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports

SERIAL_PORT_OVERRIDE = None  # npr. "COM5"; ako je None, koristi se auto-detekcija
BAUD_RATE = 115200
ARDUINO_RESET_DELAY_MS = 2000  # otvaranje porta resetuje Arduino Uno
CALIBRATION_DURATION_S = 3.0
TEST_DURATION_S = 8.0
LOG_FILE_NAME = "DZ2_session_log.txt"
ANGLE_SMOOTHING_ALPHA = 0.4  # EWMA za ugao (0-1, manje = glace ali sporije)
MONITOR_BIAS_WINDOW = 100  # koliko poslednjih uzoraka iz nadzora ulazi u procenu ofseta

STATE_IDLE = "IDLE"
STATE_WAIT_TOUCH_CAL = "WAIT_TOUCH_CAL"
STATE_CALIBRATING = "CALIBRATING"
STATE_WAIT_TOUCH_TEST = "WAIT_TOUCH_TEST"
STATE_TESTING = "TESTING"
STATE_MONITORING = "MONITORING"


def find_arduino_port():
    """Vraca COM port Arduino ploce - fiksni override ako je zadat,
    inace se trazi port ciji opis/hwid ukazuje na Arduino (npr. VID
    2341 za original Arduino ili CH340 za klonove), a tek ako takav
    nije pronadjen vraca se prvi dostupan port. Ovo je bitno jer
    Windows cesto prijavljuje i "prazne" legacy portove (npr. COM1
    "Communications Port") koji se pojave pre stvarnog USB uredjaja."""
    if SERIAL_PORT_OVERRIDE is not None:
        return SERIAL_PORT_OVERRIDE
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        text = f"{p.description} {p.hwid}".upper()
        if "ARDUINO" in text or "CH340" in text or "VID:PID=2341" in text:
            return p.device
    return ports[0].device if ports else None


class PostureMonitorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Pametni monitor za drzanje kicme")

        # Serijska konekcija i pomocna stanja za nju
        self.serial_conn = None
        self.rx_buffer = ""
        self.state = STATE_IDLE
        self._poll_job = None
        self._begin_job = None

        # Kalibracija, procena ofseta akcelerometra i filtriranje ugla
        self.cal_samples = []
        self.test_samples = []
        self.ax_cal = self.ay_cal = self.az_cal = 0.0
        self.bias = 0.0
        self.calibrated = False
        self.phase_start_time = None
        self.raw_window = []
        self.monitor_samples = []
        self.smoothed_angle = None

        # Brojanje prekrsaja i pracenje vremena u losem drzanju
        self.violation_count = 0
        self.total_bad_posture_time = 0.0
        self.in_violation = False
        self.violation_start_time = None
        self.led_on = False

        self._build_gui()
        self._poll_serial()

    # GUI
    def _build_gui(self):
        """Pravi sve elemente iz specifikacije zadatka: taster
        start/stop, polja za prag i minimalno vreme, i labele za
        trenutne vrednosti, kalibraciju, broj prekrsaja i ukupno
        vreme provedeno u losem drzanju."""
        frm = ttk.Frame(self.root, padding=10)
        frm.grid()

        self.start_btn = ttk.Button(frm, text="Start", command=self.on_start)
        self.start_btn.grid(row=0, column=0, sticky="ew", padx=2, pady=2)
        self.stop_btn = ttk.Button(frm, text="Stop", command=self.on_stop, state="disabled")
        self.stop_btn.grid(row=0, column=1, sticky="ew", padx=2, pady=2)

        ttk.Label(frm, text="Prag nagiba [deg]:").grid(row=1, column=0, sticky="w")
        self.threshold_var = tk.StringVar(value="15")
        ttk.Entry(frm, textvariable=self.threshold_var, width=10).grid(row=1, column=1, sticky="w")

        ttk.Label(frm, text="Min. trajanje loseg drzanja [s]:").grid(row=2, column=0, sticky="w")
        self.min_duration_var = tk.StringVar(value="3")
        ttk.Entry(frm, textvariable=self.min_duration_var, width=10).grid(row=2, column=1, sticky="w")

        self.status_var = tk.StringVar(value="Spremno. Pritisnite Start.")
        ttk.Label(frm, textvariable=self.status_var, foreground="blue", wraplength=320).grid(
            row=3, column=0, columnspan=2, sticky="w", pady=(8, 2)
        )

        self.accel_var = tk.StringVar(value="ax=-- ay=-- az=--  |  ugao=-- deg")
        ttk.Label(frm, textvariable=self.accel_var).grid(row=4, column=0, columnspan=2, sticky="w")

        self.cal_var = tk.StringVar(value="Kalibracija: ax_cal=-- ay_cal=-- az_cal=--")
        ttk.Label(frm, textvariable=self.cal_var).grid(row=5, column=0, columnspan=2, sticky="w")

        self.count_var = tk.StringVar(value="Ukupan broj prekrsaja: 0")
        ttk.Label(frm, textvariable=self.count_var).grid(row=6, column=0, columnspan=2, sticky="w", pady=(8, 2))

        self.time_var = tk.StringVar(value="Ukupno vreme u losem drzanju: 0.0 s")
        ttk.Label(frm, textvariable=self.time_var).grid(row=7, column=0, columnspan=2, sticky="w")

        # Pomocno dugme za testiranje kompletnog toka aplikacije bez
        # fizickog dodirivanja senzora - poziva istu funkciju kao i
        # pravi dodir sa Arduino strane, pa se ponasa identicno.
        ttk.Button(frm, text="Simuliraj dodir (DEBUG)", command=self.on_simulate_touch).grid(
            row=8, column=0, columnspan=2, sticky="ew", pady=(8, 0)
        )

    # Start / Stop
    def on_start(self):
        port = find_arduino_port()
        if port is None:
            messagebox.showerror("Greska", "Arduino nije pronadjen ni na jednom serijskom portu.")
            return
        try:
            self.serial_conn = serial.Serial(port, BAUD_RATE, timeout=0, write_timeout=0.5)
        except serial.SerialException as exc:
            messagebox.showerror("Greska", f"Ne mogu da otvorim port {port}: {exc}")
            return

        self._reset_session_state()
        self.start_btn.config(state="disabled")
        self.status_var.set("Povezivanje sa Arduino plocom...")
        # Otvaranje serijskog porta izazove hardverski reset Arduino Uno
        # ploce, pa se sa slanjem START komande ceka da sketch ponovo
        # stigne do setup()-a, umesto da se ta komanda izgubi.
        self._begin_job = self.root.after(ARDUINO_RESET_DELAY_MS, self._begin_acquisition)

    def _begin_acquisition(self):
        self._send("START")
        self.state = STATE_WAIT_TOUCH_CAL
        self.status_var.set("Dodirnite senzor da zapocnete kalibraciju (mirujte u uspravnom polozaju).")
        self.stop_btn.config(state="normal")

    def on_stop(self):
        self._end_session()

    def on_simulate_touch(self):
        self._handle_touch()

    def _end_session(self):
        if self.serial_conn is not None:
            self._send("STOP")
            try:
                self._save_log()
            except OSError as exc:
                messagebox.showwarning(
                    "Upozorenje", f"Nisam mogao da sacuvam {LOG_FILE_NAME}:\n{exc}"
                )
            try:
                self.serial_conn.close()
            except (OSError, serial.SerialException):
                pass
            self.serial_conn = None
        self.state = STATE_IDLE
        self.status_var.set(f"Zaustavljeno. Sesija sacuvana u {LOG_FILE_NAME}")
        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")

    def on_close(self):
        if self._poll_job is not None:
            self.root.after_cancel(self._poll_job)
            self._poll_job = None
        if self._begin_job is not None:
            self.root.after_cancel(self._begin_job)
            self._begin_job = None
        self._end_session()
        self.root.destroy()

    def _reset_session_state(self):
        self.state = STATE_IDLE
        self.rx_buffer = ""
        self.cal_samples = []
        self.test_samples = []
        self.ax_cal = self.ay_cal = self.az_cal = 0.0
        self.bias = 0.0
        self.calibrated = False
        self.raw_window = []
        self.monitor_samples = []
        self.smoothed_angle = None
        self.violation_count = 0
        self.total_bad_posture_time = 0.0
        self.in_violation = False
        self.violation_start_time = None
        self.led_on = False
        self.count_var.set("Ukupan broj prekrsaja: 0")
        self.time_var.set("Ukupno vreme u losem drzanju: 0.0 s")
        self.cal_var.set("Kalibracija: ax_cal=-- ay_cal=-- az_cal=--")
        self.accel_var.set("ax=-- ay=-- az=--  |  ugao=-- deg")

    # Serijska komunikacija
    def _send(self, line):
        if self.serial_conn is not None:
            try:
                self.serial_conn.write((line + "\n").encode("ascii"))
            except (OSError, serial.SerialException) as exc:
                self._handle_serial_error(exc)

    def _poll_serial(self):
        if self.serial_conn is not None:
            try:
                waiting = self.serial_conn.in_waiting
                if waiting:
                    self.rx_buffer += self.serial_conn.read(waiting).decode("ascii", errors="ignore")
                    while "\n" in self.rx_buffer:
                        line, self.rx_buffer = self.rx_buffer.split("\n", 1)
                        self._handle_line(line.strip())
            except (OSError, serial.SerialException) as exc:
                self._handle_serial_error(exc)
        self._poll_job = self.root.after(20, self._poll_serial)

    def _handle_serial_error(self, exc):
        """Poziva se kad se veza sa Arduinom izgubi (npr. iskopcan USB
        kabl) usred rada. Bezbedno zatvara port, cuva dotadasnju
        sesiju, vraca aplikaciju u pocetno ("spremno za Start") stanje
        i obavestava korisnika - umesto da aplikacija ostane zamrznuta
        na neuspelom citanju/pisanju porta."""
        if self.serial_conn is None:
            return  # greska je vec obradjena (npr. i read i write su otkazali istovremeno)
        try:
            self._save_log()
        except OSError:
            pass
        try:
            self.serial_conn.close()
        except (OSError, serial.SerialException):
            pass
        self.serial_conn = None
        self.state = STATE_IDLE
        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        self.status_var.set("Veza sa Arduinom je izgubljena. Proveri kabl i klikni Start ponovo.")
        messagebox.showerror("Greska", f"Veza sa Arduino plocom je izgubljena:\n{exc}")

    def _handle_line(self, line):
        if not line:
            return
        if line == "T":
            self._handle_touch()
        elif line.startswith("A,"):
            parts = line.split(",")
            if len(parts) == 4:
                try:
                    ax, ay, az = float(parts[1]), float(parts[2]), float(parts[3])
                except ValueError:
                    return
                self._handle_sample(ax, ay, az)

    # Logika stanja
    def _handle_touch(self):
        now = time.time()
        if self.state == STATE_WAIT_TOUCH_CAL:
            self.cal_samples = []
            self.phase_start_time = now
            self.state = STATE_CALIBRATING
            self.status_var.set("Kalibracija u toku (3 s) - mirujte u uspravnom polozaju...")
        elif self.state == STATE_WAIT_TOUCH_TEST:
            self.phase_start_time = now
            self.state = STATE_TESTING
            self.status_var.set(
                "Test nagiba u toku (8 s) - nagnite se napred/nazad da simulirate lose drzanje. "
                "Nakon nekoliko sekundi podesite zeljeni prag u polju iznad."
            )

    def _handle_sample(self, ax, ay, az):
        now = time.time()
        ax, ay, az = self._median_filter(ax, ay, az)

        if self.state == STATE_CALIBRATING:
            self.accel_var.set(f"ax={ax:.0f} ay={ay:.0f} az={az:.0f}  |  ugao=-- deg")
            self.cal_samples.append((ax, ay, az))
            if now - self.phase_start_time >= CALIBRATION_DURATION_S:
                n = len(self.cal_samples)
                self.ax_cal = sum(s[0] for s in self.cal_samples) / n
                self.ay_cal = sum(s[1] for s in self.cal_samples) / n
                self.az_cal = sum(s[2] for s in self.cal_samples) / n
                self.calibrated = True
                self.cal_var.set(
                    f"Kalibracija: ax_cal={self.ax_cal:.1f} ay_cal={self.ay_cal:.1f} az_cal={self.az_cal:.1f}"
                )
                self.state = STATE_WAIT_TOUCH_TEST
                self.status_var.set("Kalibracija zavrsena. Dodirnite senzor da zapocnete test nagiba.")
            return

        if self.state == STATE_TESTING:
            # Korisnik namerno naginje plocu na razlicite strane tokom
            # ovih 8s - ta raznovrsnost orijentacija je bas ono sto je
            # potrebno da _fit_zero_g_bias moze da proceni ofset.
            self.test_samples.append((ax, ay, az))
            self.bias = self._fit_zero_g_bias(self.cal_samples + self.test_samples)
            angle = self._smooth_angle(self._compute_angle(ax, ay, az))
            self.accel_var.set(f"ax={ax:.0f} ay={ay:.0f} az={az:.0f}  |  ugao={angle:.1f} deg")
            if now - self.phase_start_time >= TEST_DURATION_S:
                self.state = STATE_MONITORING
                self.status_var.set("Nadzor drzanja aktivan.")
            return

        if self.state == STATE_MONITORING:
            # Procena ofseta se nastavlja i ovde (na klizecem prozoru
            # poslednjih MONITOR_BIAS_WINDOW odbiraka) da bi se dodatno
            # doradila kroz prirodno kretanje tokom nadzora, ne samo
            # kroz onih 8s testa nagiba.
            self.monitor_samples.append((ax, ay, az))
            if len(self.monitor_samples) > MONITOR_BIAS_WINDOW:
                self.monitor_samples.pop(0)
            self.bias = self._fit_zero_g_bias(self.cal_samples + self.test_samples + self.monitor_samples)
            angle = self._smooth_angle(self._compute_angle(ax, ay, az))
            self.accel_var.set(f"ax={ax:.0f} ay={ay:.0f} az={az:.0f}  |  ugao={angle:.1f} deg")
            self._evaluate_posture(angle, now)
            return

        # WAIT_TOUCH_CAL / WAIT_TOUCH_TEST - samo prikazi sirove vrednosti,
        # ugao se ne moze racunati pre kalibracije/procene ofseta.
        self.accel_var.set(f"ax={ax:.0f} ay={ay:.0f} az={az:.0f}  |  ugao=-- deg")

    def _median_filter(self, ax, ay, az):
        """Medijan poslednja 3 uzorka po osi - eliminise pojedinacne
        'skokove' (elektricni sum, mikro-prekid kontakta) jer usamljen
        outlier ne moze biti medijana od 3 vrednosti."""
        self.raw_window.append((ax, ay, az))
        if len(self.raw_window) > 3:
            self.raw_window.pop(0)
        if len(self.raw_window) < 3:
            return ax, ay, az
        fx = sorted(s[0] for s in self.raw_window)[1]
        fy = sorted(s[1] for s in self.raw_window)[1]
        fz = sorted(s[2] for s in self.raw_window)[1]
        return fx, fy, fz

    def _smooth_angle(self, angle):
        """Eksponencijalno uglacavanje (EWMA) da bi se smanjilo
        preostalo 'drhtanje' ugla usled ADC suma (acos je najosetljiviji
        na sum bas oko malih uglova)."""
        if self.smoothed_angle is None:
            self.smoothed_angle = angle
        else:
            self.smoothed_angle += ANGLE_SMOOTHING_ALPHA * (angle - self.smoothed_angle)
        return self.smoothed_angle

    def _fit_zero_g_bias(self, samples):
        """Metoda najmanjih kvadrata za zajednicki 'zero-g' ofset (isti
        na sve tri ose). Koristi se ideja da velicina vektora ubrzanja
        MINUS ofset mora biti priblizno konstantna (uvek ~1g), bez
        obzira na orijentaciju - pa se ofset bira tako da varijansa te
        velicine po uzorcima bude minimalna. Zahteva uzorke iz VISE
        razlicitih orijentacija (zato se poziva tokom faze testa
        nagiba); ako nema dovoljno raznovrsnosti, vraca prethodnu
        (ili pocetnu, 0.0) procenu ofseta umesto nestabilnog rezultata."""
        n = len(samples)
        if n < 2:
            return self.bias
        sum_T = sum_S = sum_T2 = sum_ST = 0.0
        for sx, sy, sz in samples:
            T = sx + sy + sz
            S = sx * sx + sy * sy + sz * sz
            sum_T += T
            sum_S += S
            sum_T2 += T * T
            sum_ST += S * T
        mean_T = sum_T / n
        mean_S = sum_S / n
        var_T = sum_T2 / n - mean_T * mean_T
        if var_T < 1.0:
            return self.bias
        cov_ST = sum_ST / n - mean_S * mean_T
        return cov_ST / (2 * var_T)

    def _compute_angle(self, ax, ay, az):
        """Ugao izmedju trenutnog i kalibrisanog (referentnog,
        uspravnog) vektora ubrzanja, preko skalarnog proizvoda. Prvo
        se oduzima zajednicki 'zero-g' ofset (self.bias, vidi
        _fit_zero_g_bias) - bez toga veliki ofset dominira vektorom i
        ugao izmedju sirovih vektora ispadne mnogo manji od stvarnog
        fizickog nagiba."""
        bx, by, bz = ax - self.bias, ay - self.bias, az - self.bias
        cx = self.ax_cal - self.bias
        cy = self.ay_cal - self.bias
        cz = self.az_cal - self.bias
        dot = bx * cx + by * cy + bz * cz
        mag_current = math.sqrt(bx ** 2 + by ** 2 + bz ** 2)
        mag_cal = math.sqrt(cx ** 2 + cy ** 2 + cz ** 2)
        if mag_current == 0 or mag_cal == 0:
            return 0.0
        cos_angle = max(-1.0, min(1.0, dot / (mag_current * mag_cal)))
        return math.degrees(math.acos(cos_angle))

    def _evaluate_posture(self, angle, now):
        threshold = self._safe_float(self.threshold_var.get(), 15.0)
        min_duration = self._safe_float(self.min_duration_var.get(), 3.0)
        bad_posture = angle > threshold

        # Ulazak u lose drzanje - broji se samo jednom po epizodi
        # (uzlazna ivica), ne na svakih 100ms dok se u njoj ostaje.
        if bad_posture and not self.in_violation:
            self.in_violation = True
            self.violation_start_time = now
            self.violation_count += 1
            self.count_var.set(f"Ukupan broj prekrsaja: {self.violation_count}")
            self._send(f"CNT,{self.violation_count}")

        # Povratak u pravilno drzanje - epizoda se zatvara, njeno
        # trajanje se dodaje na ukupno vreme, LED se gasi ako je bila upaljena.
        elif not bad_posture and self.in_violation:
            self.in_violation = False
            self.total_bad_posture_time += now - self.violation_start_time
            self.time_var.set(f"Ukupno vreme u losem drzanju: {self.total_bad_posture_time:.1f} s")
            if self.led_on:
                self._send("LED,0")
                self.led_on = False

        # Dok se epizoda i dalje traje, proveri da li je predugo trajala
        # (preslo minimalno dozvoljeno vreme) i ako jeste, upali LED.
        if self.in_violation:
            duration = now - self.violation_start_time
            if duration >= min_duration and not self.led_on:
                self._send("LED,1")
                self.led_on = True

    @staticmethod
    def _safe_float(text, default):
        try:
            value = float(text)
        except ValueError:
            return default
        if math.isnan(value) or math.isinf(value):
            return default
        return value

    # Cuvanje sesije
    def _save_log(self):
        if self.in_violation and self.violation_start_time is not None:
            self.total_bad_posture_time += time.time() - self.violation_start_time
        with open(LOG_FILE_NAME, "w", encoding="utf-8") as f:
            f.write(f"ax_cal={self.ax_cal:.2f}\n")
            f.write(f"ay_cal={self.ay_cal:.2f}\n")
            f.write(f"az_cal={self.az_cal:.2f}\n")
            f.write(f"broj_prekrsaja={self.violation_count}\n")
            f.write(f"ukupno_vreme_loseg_drzanja_s={self.total_bad_posture_time:.1f}\n")


def main():
    root = tk.Tk()
    app = PostureMonitorApp(root)
    # Vezuje klik na X (zatvaranje prozora) za nas on_close, umesto da
    # Tkinter samo nasilno unisti prozor bez uredne obrade resursa.
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
