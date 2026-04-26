import serial, time, threading, collections, json, csv
from datetime import datetime
from pathlib import Path

import numpy as np
from PySide6.QtWidgets import (
    QApplication, QWidget, QHBoxLayout, QVBoxLayout, QGroupBox,
    QFormLayout, QDoubleSpinBox, QPushButton, QLabel, QTextEdit,
    QComboBox, QMessageBox
)
from PySide6.QtCore import QTimer, Qt
import pyqtgraph as pg
from serial.tools import list_ports


BAUD = 115200

WIN_SEC = 10.0
HZ = 50
GUI_HZ = 20

CONFIG_PATH = Path("pid_config.json")
DATA_DIR = Path("coletas")
DATA_DIR.mkdir(exist_ok=True)

ser = None


def serial_is_open():
    return ser is not None and ser.is_open


def list_serial_ports():
    return [p.device for p in list_ports.comports()]


def open_serial(port):
    global ser
    close_serial()
    ser = serial.Serial(port, BAUD, timeout=0.4, write_timeout=0.8)


def close_serial():
    global ser
    try:
        if ser is not None and ser.is_open:
            ser.close()
    except Exception:
        pass
    ser = None


def drain():
    if not serial_is_open():
        return
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception:
        pass
    time.sleep(0.03)


def send_line(s: str):
    if not serial_is_open():
        raise RuntimeError("Porta serial não conectada.")
    ser.write((s + "\n").encode())
    ser.flush()


def read_until_prefix(prefix: str, tmo=2.0):
    if not serial_is_open():
        return None
    t0 = time.time()
    while time.time() - t0 < tmo:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue
        if line.startswith(prefix):
            return line
    return None


def load_config():
    if not CONFIG_PATH.exists():
        return None
    try:
        with CONFIG_PATH.open("r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return None


def save_config(vals):
    data = {
        "pendulo": {
            "kp": float(vals["kp"]),
            "ki": float(vals["ki"]),
            "kd": float(vals["kd"]),
        },
        "fuzzy_braco": {
            "inicio_acao_deg": float(vals["inicio_acao_deg"]),
            "acao_maxima_deg": float(vals["acao_maxima_deg"]),
            "offset_max_setpoint_deg": float(vals["offset_max_setpoint_deg"]),
            "limiar_vel_alta_deg_s": float(vals["limiar_vel_alta_deg_s"]),
        }
    }
    try:
        with CONFIG_PATH.open("w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
    except Exception:
        pass


def wrap180(x):
    return ((x + 180.0) % 360.0) - 180.0


def parse_kv(line: str):
    try:
        parts = line.strip().split(",")
        d, i = {}, 0
        while i < len(parts) - 1:
            k = parts[i].strip()
            v = parts[i + 1].strip()
            d[k] = v
            i += 2
        return d
    except Exception:
        return {}


def hud_text(lines):
    return "\n".join(lines)


def mf_tri(x, left, peak, right):
    x = np.asarray(x, dtype=float)
    y = np.zeros_like(x)

    if peak > left:
        m1 = (x > left) & (x < peak)
        y[m1] = (x[m1] - left) / (peak - left)

    if right > peak:
        m2 = (x >= peak) & (x < right)
        y[m2] = (right - x[m2]) / (right - peak)

    y[x == peak] = 1.0
    return np.clip(y, 0.0, 1.0)


def mf_trap(x, left0, left1, right1, right0):
    x = np.asarray(x, dtype=float)
    y = np.zeros_like(x)

    if left1 == left0:
        m_up = (x >= left0) & (x <= left1)
        y[m_up] = 1.0
    else:
        m_up = (x > left0) & (x < left1)
        y[m_up] = (x[m_up] - left0) / (left1 - left0)

    m_mid = (x >= left1) & (x <= right1)
    y[m_mid] = 1.0

    if right0 == right1:
        m_dn = (x >= right1) & (x <= right0)
        y[m_dn] = 1.0
    else:
        m_dn = (x > right1) & (x < right0)
        y[m_dn] = (right0 - x[m_dn]) / (right0 - right1)

    return np.clip(y, 0.0, 1.0)


class TunerWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PID Tuner (Minimal Telemetry)")

        self.inicio_acao_deg = 20.0
        self.acao_maxima_deg = 30.0
        self.offset_max_setpoint_deg = 0.5
        self.limiar_vel_alta_deg_s = 500.0

        self.buf_t = collections.deque(maxlen=int(WIN_SEC * HZ))
        self.buf = {
            k: collections.deque(maxlen=int(WIN_SEC * HZ))
            for k in ["TH", "SP", "THM", "E", "DUTY"]
        }

        self.log_buf = collections.deque(maxlen=300)
        self.stop_reader = False

        self.is_collecting = False
        self.collect_file = None
        self.collect_writer = None
        self.collect_start_t = None
        self.collect_meta = {}

        self._build_ui()
        self._load_initial_config()
        self.on_update_fuzzy_plot()
        self._refresh_serial_ports()
        self._start_reader_thread()
        self._start_timer()

    def _build_ui(self):
        main = QHBoxLayout(self)

        pg.setConfigOptions(antialias=False)

        self.plot_widget = pg.GraphicsLayoutWidget()
        main.addWidget(self.plot_widget, stretch=3)

        self.p1 = self.plot_widget.addPlot(row=0, col=0, title="Ângulo pêndulo (θ) & setpoint")
        self.p1.setLabel("left", "θ (deg)")
        self.p1.setLabel("bottom", "tempo", units="s")
        self.p1.showGrid(x=True, y=True, alpha=0.3)
        self.p1.setYRange(-180, 180)
        self.p1.disableAutoRange(axis="y")
        self.p1.addLegend()
        self.cur_th = self.p1.plot(pen=pg.mkPen(color=(0, 180, 255), width=2), name="θ")
        self.cur_sp = self.p1.plot(pen=pg.mkPen(color=(255, 80, 80), width=2, style=Qt.DashLine), name="SP")

        self.hud_p1 = pg.TextItem(
            text="",
            anchor=(0, 0),
            color=(255, 255, 255),
            fill=pg.mkBrush(0, 0, 0, 170),
            border=pg.mkPen(255, 255, 255, 120)
        )
        self.hud_p1.setZValue(10)
        self.p1.addItem(self.hud_p1)

        self.p2 = self.plot_widget.addPlot(row=1, col=0, title="Pertinência fuzzy do braço (|θ_motor|)")
        self.p2.setLabel("left", "μ")
        self.p2.setLabel("bottom", "|θ_motor|", units="deg")
        self.p2.showGrid(x=True, y=True, alpha=0.3)
        self.p2.setXRange(0, 180)
        self.p2.setYRange(0, 1.05)
        self.p2.disableAutoRange(axis="x")
        self.p2.disableAutoRange(axis="y")
        self.p2.addLegend()

        pen_seguro = pg.mkPen(color=(0, 200, 120), width=2)
        pen_perto = pg.mkPen(color=(255, 200, 0), width=2)
        pen_crit = pg.mkPen(color=(255, 80, 80), width=2)

        self.cur_seguro = self.p2.plot([], [], pen=pen_seguro, name="seguro")
        self.cur_perto = self.p2.plot([], [], pen=pen_perto, name="perto")
        self.cur_crit = self.p2.plot([], [], pen=pen_crit, name="crítico")

        pen_linha = pg.mkPen(color=(0, 180, 255), width=2)
        self.linha_braco = pg.InfiniteLine(pos=0.0, angle=90, movable=False, pen=pen_linha)
        self.p2.addItem(self.linha_braco)

        self.hud_p2 = pg.TextItem(
            text="",
            anchor=(0, 0),
            color=(255, 255, 255),
            fill=pg.mkBrush(0, 0, 0, 170),
            border=pg.mkPen(255, 255, 255, 120)
        )
        self.hud_p2.setZValue(10)
        self.p2.addItem(self.hud_p2)

        self.p3 = self.plot_widget.addPlot(row=2, col=0, title="Erro (SP - θ)")
        self.p3.setLabel("left", "erro (deg)")
        self.p3.setLabel("bottom", "tempo", units="s")
        self.p3.showGrid(x=True, y=True, alpha=0.3)
        self.p3.setYRange(-40, 40)
        self.p3.disableAutoRange(axis="y")
        self.p3.addLegend()
        self.cur_e = self.p3.plot(pen=pg.mkPen(color=(255, 170, 0), width=2), name="erro")

        self.hud_p3 = pg.TextItem(
            text="",
            anchor=(0, 0),
            color=(255, 255, 255),
            fill=pg.mkBrush(0, 0, 0, 170),
            border=pg.mkPen(255, 255, 255, 120)
        )
        self.hud_p3.setZValue(10)
        self.p3.addItem(self.hud_p3)

        self.p4 = self.plot_widget.addPlot(row=3, col=0, title="Duty")
        self.p4.setLabel("left", "duty")
        self.p4.setLabel("bottom", "tempo", units="s")
        self.p4.showGrid(x=True, y=True, alpha=0.3)
        self.p4.setYRange(-255, 255)
        self.p4.disableAutoRange(axis="y")
        self.p4.addLegend()
        self.cur_duty = self.p4.plot(pen=pg.mkPen(color=(200, 100, 255), width=2), name="duty")

        self.hud_p4 = pg.TextItem(
            text="",
            anchor=(0, 0),
            color=(255, 255, 255),
            fill=pg.mkBrush(0, 0, 0, 170),
            border=pg.mkPen(255, 255, 255, 120)
        )
        self.hud_p4.setZValue(10)
        self.p4.addItem(self.hud_p4)

        side = QVBoxLayout()
        main.addLayout(side, stretch=2)

        grp_serial = QGroupBox("Conexão serial")
        form_serial = QFormLayout(grp_serial)

        self.cmb_port = QComboBox()
        self.btn_refresh_ports = QPushButton("Atualizar")
        self.btn_connect = QPushButton("Conectar")
        self.btn_disconnect = QPushButton("Desconectar")
        self.lbl_serial_status = QLabel("Desconectado")

        row_ports = QHBoxLayout()
        row_ports.addWidget(self.cmb_port)
        row_ports.addWidget(self.btn_refresh_ports)

        box_ports = QWidget()
        box_ports.setLayout(row_ports)

        row_conn = QHBoxLayout()
        row_conn.addWidget(self.btn_connect)
        row_conn.addWidget(self.btn_disconnect)

        box_conn = QWidget()
        box_conn.setLayout(row_conn)

        form_serial.addRow("Porta", box_ports)
        form_serial.addRow("Ação", box_conn)
        form_serial.addRow("Status", self.lbl_serial_status)

        side.addWidget(grp_serial)

        grp_p = QGroupBox("PID pêndulo")
        form_p = QFormLayout(grp_p)

        self.spn_kp = QDoubleSpinBox()
        self.spn_kp.setRange(0.0, 100000.0)
        self.spn_kp.setDecimals(6)
        self.spn_kp.setSingleStep(1.0)

        self.spn_ki = QDoubleSpinBox()
        self.spn_ki.setRange(0.0, 100000.0)
        self.spn_ki.setDecimals(6)
        self.spn_ki.setSingleStep(0.1)

        self.spn_kd = QDoubleSpinBox()
        self.spn_kd.setRange(0.0, 100000.0)
        self.spn_kd.setDecimals(6)
        self.spn_kd.setSingleStep(1.0)

        form_p.addRow("Kp", self.spn_kp)
        form_p.addRow("Ki", self.spn_ki)
        form_p.addRow("Kd", self.spn_kd)

        side.addWidget(grp_p)

        grp_fuzzy = QGroupBox("Fuzzy do braço")
        form_fuzzy = QFormLayout(grp_fuzzy)

        self.spn_inicio_acao = QDoubleSpinBox()
        self.spn_inicio_acao.setRange(0.0, 180.0)
        self.spn_inicio_acao.setDecimals(3)
        self.spn_inicio_acao.setSingleStep(1.0)

        self.spn_acao_maxima = QDoubleSpinBox()
        self.spn_acao_maxima.setRange(0.0, 180.0)
        self.spn_acao_maxima.setDecimals(3)
        self.spn_acao_maxima.setSingleStep(1.0)

        self.spn_offset_max = QDoubleSpinBox()
        self.spn_offset_max.setRange(0.0, 20.0)
        self.spn_offset_max.setDecimals(4)
        self.spn_offset_max.setSingleStep(0.05)

        self.spn_vel_alta = QDoubleSpinBox()
        self.spn_vel_alta.setRange(0.0, 10000.0)
        self.spn_vel_alta.setDecimals(3)
        self.spn_vel_alta.setSingleStep(10.0)

        self.btn_aplicar_fuzzy_plot = QPushButton("Atualizar gráfico")
        self.btn_send_fuzzy = QPushButton("Enviar fuzzy p/ ESP32")

        form_fuzzy.addRow("Início ação (deg)", self.spn_inicio_acao)
        form_fuzzy.addRow("Ação máxima (deg)", self.spn_acao_maxima)
        form_fuzzy.addRow("Offset máx SP (deg)", self.spn_offset_max)
        form_fuzzy.addRow("Limiar vel alta (deg/s)", self.spn_vel_alta)
        form_fuzzy.addRow(self.btn_aplicar_fuzzy_plot)
        form_fuzzy.addRow(self.btn_send_fuzzy)

        side.addWidget(grp_fuzzy)

        btn_row1 = QHBoxLayout()
        self.btn_apply_gp = QPushButton("Aplicar GP")
        self.btn_save_json = QPushButton("Salvar JSON")
        btn_row1.addWidget(self.btn_apply_gp)
        btn_row1.addWidget(self.btn_save_json)
        side.addLayout(btn_row1)

        btn_row2 = QHBoxLayout()
        self.btn_start = QPushButton("INICIAR")
        self.btn_stop = QPushButton("STOP")
        btn_row2.addWidget(self.btn_start)
        btn_row2.addWidget(self.btn_stop)
        side.addLayout(btn_row2)

        btn_row3 = QHBoxLayout()
        self.btn_start_collect = QPushButton("Iniciar coleta")
        self.btn_stop_collect = QPushButton("Parar coleta")
        self.btn_stop_collect.setEnabled(False)
        btn_row3.addWidget(self.btn_start_collect)
        btn_row3.addWidget(self.btn_stop_collect)
        side.addLayout(btn_row3)

        side.addWidget(QLabel("Log da serial:"))
        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        side.addWidget(self.txt_log, stretch=1)

        self.btn_disconnect.setEnabled(False)

        self.btn_refresh_ports.clicked.connect(self._refresh_serial_ports)
        self.btn_connect.clicked.connect(self.on_connect_serial)
        self.btn_disconnect.clicked.connect(self.on_disconnect_serial)
        self.btn_apply_gp.clicked.connect(self.on_apply_gp)
        self.btn_save_json.clicked.connect(self.on_save_json)
        self.btn_start.clicked.connect(self.on_start)
        self.btn_stop.clicked.connect(self.on_stop)
        self.btn_aplicar_fuzzy_plot.clicked.connect(self.on_update_fuzzy_plot)
        self.btn_send_fuzzy.clicked.connect(self.on_send_fuzzy)
        self.btn_start_collect.clicked.connect(self.on_start_collect)
        self.btn_stop_collect.clicked.connect(self.on_stop_collect)

        self.spn_inicio_acao.valueChanged.connect(self.on_update_fuzzy_plot)
        self.spn_acao_maxima.valueChanged.connect(self.on_update_fuzzy_plot)
        self.spn_offset_max.valueChanged.connect(self.on_update_fuzzy_plot)
        self.spn_vel_alta.valueChanged.connect(self.on_update_fuzzy_plot)

    def _refresh_serial_ports(self):
        current = self.cmb_port.currentText()
        ports = list_serial_ports()

        self.cmb_port.clear()
        self.cmb_port.addItems(ports)

        if current and current in ports:
            self.cmb_port.setCurrentText(current)

        if not ports:
            self.lbl_serial_status.setText("Nenhuma porta encontrada")
        elif not serial_is_open():
            self.lbl_serial_status.setText("Desconectado")

    def _load_initial_config(self):
        cfg = load_config()
        if cfg is None:
            self.spn_kp.setValue(290.0)
            self.spn_ki.setValue(0.0)
            self.spn_kd.setValue(0.0)

            self.spn_inicio_acao.setValue(self.inicio_acao_deg)
            self.spn_acao_maxima.setValue(self.acao_maxima_deg)
            self.spn_offset_max.setValue(self.offset_max_setpoint_deg)
            self.spn_vel_alta.setValue(self.limiar_vel_alta_deg_s)
            return

        p = cfg.get("pendulo", {})
        self.spn_kp.setValue(float(p.get("kp", 290.0)))
        self.spn_ki.setValue(float(p.get("ki", 0.0)))
        self.spn_kd.setValue(float(p.get("kd", 0.0)))

        fz = cfg.get("fuzzy_braco", {})
        self.spn_inicio_acao.setValue(float(fz.get("inicio_acao_deg", self.inicio_acao_deg)))
        self.spn_acao_maxima.setValue(float(fz.get("acao_maxima_deg", self.acao_maxima_deg)))
        self.spn_offset_max.setValue(float(fz.get("offset_max_setpoint_deg", self.offset_max_setpoint_deg)))
        self.spn_vel_alta.setValue(float(fz.get("limiar_vel_alta_deg_s", self.limiar_vel_alta_deg_s)))

    def _start_reader_thread(self):
        def reader():
            while not self.stop_reader:
                if not serial_is_open():
                    time.sleep(0.1)
                    continue

                try:
                    line = ser.readline().decode(errors="ignore").strip()
                    if not line:
                        continue

                    if line.startswith("T,"):
                        d = parse_kv(line)

                        tms = int(float(d.get("T", "0")))
                        t = tms / 1000.0
                        self.buf_t.append(t)

                        parsed_vals = {}

                        for k in self.buf.keys():
                            raw = d.get(k, "nan")
                            try:
                                val = float(raw)
                            except Exception:
                                val = float("nan")

                            if k in {"TH", "THM", "SP", "E"} and not (val != val):
                                val /= 100.0

                            if k in {"TH", "SP", "THM"} and not (val != val):
                                val = wrap180(val)

                            self.buf[k].append(val)
                            parsed_vals[k] = val

                        if self.is_collecting and self.collect_writer is not None:
                            if self.collect_start_t is None:
                                self.collect_start_t = t

                            self.collect_writer.writerow([
                                t - self.collect_start_t,
                                parsed_vals.get("TH", float("nan")),
                                parsed_vals.get("SP", float("nan")),
                                parsed_vals.get("THM", float("nan")),
                                parsed_vals.get("E", float("nan")),
                                parsed_vals.get("DUTY", float("nan")),
                                self.collect_meta.get("kp", float("nan")),
                                self.collect_meta.get("ki", float("nan")),
                                self.collect_meta.get("kd", float("nan")),
                                self.collect_meta.get("inicio_acao_deg", float("nan")),
                                self.collect_meta.get("acao_maxima_deg", float("nan")),
                                self.collect_meta.get("offset_max_setpoint_deg", float("nan")),
                                self.collect_meta.get("limiar_vel_alta_deg_s", float("nan"))
                            ])
                            self.collect_file.flush()
                    else:
                        if len(self.log_buf) < 300:
                            self.log_buf.append(line)
                except Exception:
                    time.sleep(0.01)

        threading.Thread(target=reader, daemon=True).start()

    def _start_timer(self):
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.on_timer)
        self.timer.start(int(1000 / GUI_HZ))

    def _current_vals(self):
        return {
            "kp": self.spn_kp.value(),
            "ki": self.spn_ki.value(),
            "kd": self.spn_kd.value(),
            "inicio_acao_deg": self.spn_inicio_acao.value(),
            "acao_maxima_deg": self.spn_acao_maxima.value(),
            "offset_max_setpoint_deg": self.spn_offset_max.value(),
            "limiar_vel_alta_deg_s": self.spn_vel_alta.value(),
        }

    def on_connect_serial(self):
        port = self.cmb_port.currentText().strip()
        if not port:
            QMessageBox.warning(self, "Serial", "Selecione uma porta serial.")
            return

        try:
            open_serial(port)
            self.lbl_serial_status.setText(f"Conectado em {port}")
            self.log_buf.append(f"[INFO] Conectado em {port}")
            self.btn_connect.setEnabled(False)
            self.btn_disconnect.setEnabled(True)
        except Exception as e:
            self.lbl_serial_status.setText("Erro ao conectar")
            QMessageBox.critical(self, "Serial", f"Não foi possível abrir a porta {port}.\n\n{e}")

    def on_disconnect_serial(self):
        try:
            if self.is_collecting:
                self.on_stop_collect()
            port = self.cmb_port.currentText().strip()
            close_serial()
            self.lbl_serial_status.setText("Desconectado")
            self.log_buf.append(f"[INFO] Desconectado de {port}" if port else "[INFO] Serial desconectada")
            self.btn_connect.setEnabled(True)
            self.btn_disconnect.setEnabled(False)
        except Exception as e:
            QMessageBox.critical(self, "Serial", f"Erro ao desconectar.\n\n{e}")

    def on_apply_gp(self):
        if not serial_is_open():
            QMessageBox.warning(self, "Serial", "Conecte a porta serial primeiro.")
            return

        try:
            vals = self._current_vals()
            drain()
            send_line(f"GP {vals['kp']:.6f} {vals['ki']:.6f} {vals['kd']:.6f}")
            _ = read_until_prefix("GP")
            save_config(vals)
        except Exception as e:
            QMessageBox.critical(self, "Erro", str(e))

    def on_send_fuzzy(self):
        if not serial_is_open():
            QMessageBox.warning(self, "Serial", "Conecte a porta serial primeiro.")
            return

        try:
            vals = self._current_vals()
            drain()
            send_line(
                f"FZ {vals['inicio_acao_deg']:.6f} "
                f"{vals['acao_maxima_deg']:.6f} "
                f"{vals['offset_max_setpoint_deg']:.6f} "
                f"{vals['limiar_vel_alta_deg_s']:.6f}"
            )
            _ = read_until_prefix("FZ")
            save_config(vals)
        except Exception as e:
            QMessageBox.critical(self, "Erro", str(e))

    def on_save_json(self):
        try:
            vals = self._current_vals()
            save_config(vals)
        except Exception:
            pass

    def on_start(self):
        if not serial_is_open():
            QMessageBox.warning(self, "Serial", "Conecte a porta serial primeiro.")
            return

        try:
            vals = self._current_vals()
            drain()
            send_line(f"GP {vals['kp']:.6f} {vals['ki']:.6f} {vals['kd']:.6f}")
            _ = read_until_prefix("GP")
            drain()
            send_line(
                f"FZ {vals['inicio_acao_deg']:.6f} "
                f"{vals['acao_maxima_deg']:.6f} "
                f"{vals['offset_max_setpoint_deg']:.6f} "
                f"{vals['limiar_vel_alta_deg_s']:.6f}"
            )
            _ = read_until_prefix("FZ")
            drain()
            send_line("SP 0.000")
            save_config(vals)
            drain()
            send_line("START")
        except Exception as e:
            QMessageBox.critical(self, "Erro", str(e))

    def on_stop(self):
        if not serial_is_open():
            QMessageBox.warning(self, "Serial", "Conecte a porta serial primeiro.")
            return

        try:
            drain()
            send_line("STOP")
        except Exception as e:
            QMessageBox.critical(self, "Erro", str(e))

    def on_start_collect(self):
        if self.is_collecting:
            return

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = DATA_DIR / f"coleta_{ts}.csv"

        try:
            self.collect_meta = self._current_vals()
            self.collect_file = filepath.open("w", newline="", encoding="utf-8")
            self.collect_writer = csv.writer(self.collect_file)
            self.collect_writer.writerow([
                "t_s",
                "TH_deg",
                "SP_deg",
                "THM_deg",
                "E_deg",
                "DUTY",
                "kp",
                "ki",
                "kd",
                "inicio_acao_deg",
                "acao_maxima_deg",
                "offset_max_setpoint_deg",
                "limiar_vel_alta_deg_s"
            ])
            self.collect_start_t = None
            self.is_collecting = True
            self.btn_start_collect.setEnabled(False)
            self.btn_stop_collect.setEnabled(True)
            self.log_buf.append(f"[COLETA] Iniciada: {filepath.name}")
        except Exception as e:
            self.is_collecting = False
            self.collect_writer = None
            self.collect_file = None
            self.collect_start_t = None
            self.collect_meta = {}
            QMessageBox.critical(self, "Erro", f"Não foi possível iniciar a coleta.\n\n{e}")

    def on_stop_collect(self):
        if not self.is_collecting:
            return

        try:
            self.is_collecting = False
            self.collect_writer = None
            if self.collect_file is not None:
                self.collect_file.close()
            self.collect_file = None
            self.collect_start_t = None
            self.collect_meta = {}
            self.btn_start_collect.setEnabled(True)
            self.btn_stop_collect.setEnabled(False)
            self.log_buf.append("[COLETA] Finalizada")
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao finalizar coleta.\n\n{e}")

    def on_update_fuzzy_plot(self):
        self.inicio_acao_deg = self.spn_inicio_acao.value()
        self.acao_maxima_deg = self.spn_acao_maxima.value()
        self.offset_max_setpoint_deg = self.spn_offset_max.value()
        self.limiar_vel_alta_deg_s = self.spn_vel_alta.value()

        if self.acao_maxima_deg <= self.inicio_acao_deg:
            self.acao_maxima_deg = self.inicio_acao_deg + 0.1
            self.spn_acao_maxima.blockSignals(True)
            self.spn_acao_maxima.setValue(self.acao_maxima_deg)
            self.spn_acao_maxima.blockSignals(False)

        x_deg = np.linspace(0.0, 180.0, 721)

        y_seguro = mf_trap(
            x_deg,
            0.0,
            0.0,
            max(0.0, self.inicio_acao_deg - 10.0),
            self.inicio_acao_deg
        )

        y_perto = mf_tri(
            x_deg,
            self.inicio_acao_deg,
            0.5 * (self.inicio_acao_deg + self.acao_maxima_deg),
            self.acao_maxima_deg
        )

        y_crit = mf_trap(
            x_deg,
            self.acao_maxima_deg - 8.0,
            self.acao_maxima_deg - 2.0,
            120.0,
            180.0
        )

        self.cur_seguro.setData(x_deg, y_seguro)
        self.cur_perto.setData(x_deg, y_perto)
        self.cur_crit.setData(x_deg, y_crit)

    def _hud_pos(self, plot):
        (xmin, xmax), (ymin, ymax) = plot.viewRange()
        xr = (xmax - xmin) if (xmax > xmin) else 1.0
        yr = (ymax - ymin) if (ymax > ymin) else 1.0
        x = xmin + 0.02 * xr
        y = ymax - 0.05 * yr
        return x, y

    def on_timer(self):
        if self.buf_t:
            t1 = self.buf_t[-1]
            t0 = max(self.buf_t[0], t1 - WIN_SEC)

            self.p1.setXRange(t0, t1, padding=0.0)
            self.p3.setXRange(t0, t1, padding=0.0)
            self.p4.setXRange(t0, t1, padding=0.0)

            t_list = list(self.buf_t)

            th = list(self.buf["TH"])
            sp = list(self.buf["SP"])
            thm = list(self.buf["THM"])
            e = list(self.buf["E"])
            duty = list(self.buf["DUTY"])

            self.cur_th.setData(t_list, th)
            self.cur_sp.setData(t_list, sp)
            self.cur_e.setData(t_list, e)
            self.cur_duty.setData(t_list, duty)

            th_last = th[-1] if th else float("nan")
            sp_last = sp[-1] if sp else float("nan")
            thm_last = thm[-1] if thm else float("nan")
            e_last = e[-1] if e else float("nan")
            duty_last = duty[-1] if duty else float("nan")

            if not (thm_last != thm_last):
                self.linha_braco.setPos(abs(thm_last))

            x1, y1 = self._hud_pos(self.p1)
            self.hud_p1.setPos(x1, y1)
            self.hud_p1.setText(hud_text([
                f"θ   = {th_last: .2f}°",
                f"SP  = {sp_last: .2f}°",
            ]))

            (xmin, xmax), (ymin, ymax) = self.p2.viewRange()
            xr = (xmax - xmin) if (xmax > xmin) else 1.0
            yr = (ymax - ymin) if (ymax > ymin) else 1.0

            x2 = xmax - 0.18 * xr
            y2 = ymax - 0.05 * yr

            self.hud_p2.setPos(x2, y2)
            self.hud_p2.setText(hud_text([
                f"|θm| = {abs(thm_last): .2f}°",
                f"início = {self.inicio_acao_deg: .2f}°",
                f"máx = {self.acao_maxima_deg: .2f}°",
            ]))

            x3, y3 = self._hud_pos(self.p3)
            self.hud_p3.setPos(x3, y3)
            self.hud_p3.setText(hud_text([
                f"e   = {e_last: .2f}°",
            ]))

            x4, y4 = self._hud_pos(self.p4)
            self.hud_p4.setPos(x4, y4)
            self.hud_p4.setText(hud_text([
                f"duty = {duty_last: .0f}",
            ]))

        flush = 0
        while self.log_buf and flush < 5:
            self.txt_log.append(self.log_buf.popleft())
            flush += 1

    def closeEvent(self, event):
        self.stop_reader = True
        if self.is_collecting:
            try:
                self.on_stop_collect()
            except Exception:
                pass
        close_serial()
        event.accept()


if __name__ == "__main__":
    app = QApplication([])
    w = TunerWindow()
    w.resize(1280, 860)
    w.show()
    app.exec()