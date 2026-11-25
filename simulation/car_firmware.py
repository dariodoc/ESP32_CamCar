"""Replica exacta del firmware ESP32 (motor_control, peripherals, wifi_server) y puente 127.0.0.1."""

from __future__ import annotations

import errno
import math
import queue
import random
import socket
import struct
import threading
import time
from typing import Callable

import jpeg_mini
import numpy as np

AUTO_STATE_NAMES = {
    0: 'Avanzando',
    1: 'Frenando',
    2: 'Reversa',
    3: 'Girando',
    4: 'Strafing Mecanum',
}


def clamp_int(val: int, lo: int, hi: int) -> int:
  return lo if val < lo else (hi if val > hi else val)


def map_motor_value(raw_value: int) -> int:
  """Port exacto de mapMotorValue() en src/motor_control.cpp."""
  abs_raw = abs(int(raw_value))
  if abs_raw <= 210:
    return 0
  min_pwm = 400
  max_pwm = 4095
  sign = 1 if raw_value > 0 else -1
  abs_val = clamp_int(abs_raw, 210, 1500)
  mapped = ((abs_val - 210) * (max_pwm - min_pwm)) // (1500 - 210) + min_pwm
  return int(mapped * sign)


def compute_mecanum_pwms(
    angle: float, speed: float, rotation: float, rotation_speed: float
) -> tuple[int, int, int, int]:
  """Port exacto de driveMecanum() en src/motor_control.cpp -> (fl, bl, fr, br)."""
  angle_rad = math.radians(angle)
  x_trans = -speed * math.sin(angle_rad)
  y_trans = speed * math.cos(angle_rad)

  rot_rad = math.radians(rotation)
  rot = -rotation_speed * math.sin(rot_rad)

  fl = y_trans + x_trans + rot
  fr = y_trans - x_trans - rot
  bl = y_trans - x_trans + rot
  br = y_trans + x_trans - rot

  max_val = max(abs(fl), abs(fr), abs(bl), abs(br))
  if max_val > 1500.0:
    scale = 1500.0 / max_val
    fl *= scale
    fr *= scale
    bl *= scale
    br *= scale

  return (
      map_motor_value(int(fl)),
      map_motor_value(int(bl)),
      map_motor_value(int(fr)),
      map_motor_value(int(br)),
  )


def compute_joystick_mecanum_pwms(
    ly: float, lx: float, rx: float
) -> tuple[int, int, int, int]:
  """Port exacto de Main.py poll_xbox() -> driveSafe(fl, bl, fr, br)."""
  fl = ly + lx + rx
  bl = ly - lx + rx
  fr = ly - lx - rx
  br = ly + lx - rx

  max_val = max(abs(fl), abs(bl), abs(fr), abs(br))
  if max_val > 1500.0:
    scale = 1500.0 / max_val
    fl *= scale
    bl *= scale
    fr *= scale
    br *= scale

  return (
      map_motor_value(int(fl)),
      map_motor_value(int(bl)),
      map_motor_value(int(fr)),
      map_motor_value(int(br)),
  )


def _apply_ramp_channel(current: int, target: int) -> int:
  """Port exacto de applyRamp() en src/motor_control.cpp (50 Hz)."""
  slew_step = 150
  min_pwm = 400

  if 0 < target < min_pwm:
    target = min_pwm
  elif -min_pwm < target < 0:
    target = -min_pwm

  if current < target:
    if current == 0:
      current = min_pwm
    else:
      current = min(current + slew_step, target)
  elif current > target:
    if current == 0:
      current = -min_pwm
    else:
      current = max(current - slew_step, target)

  if abs(current) < min_pwm and (
      target == 0
      or (current > 0 and target < 0)
      or (current < 0 and target > 0)
  ):
    current = 0
  return current


class Esp32CarFirmware:
  """Maquina de estado completa del ESP32_CamCar corriendo deterministamente en sim_time."""

  def __init__(self, rng_seed: int = 42) -> None:
    self._rng = random.Random(rng_seed)
    self.reset(sim_time=0.0)

  def reset(self, sim_time: float = 0.0) -> None:
    self.mode = 'manual'  # 'manual', 'auto_us_ir', 'auto_ir_only'

    # Estado de motores (motor_control.cpp).
    self.target_fl = 0
    self.target_bl = 0
    self.target_fr = 0
    self.target_br = 0
    self.current_fl = 0
    self.current_bl = 0
    self.current_fr = 0
    self.current_br = 0
    self.immediate_brake = True
    self.immediate_stop = False
    self.braking = True
    self.coasting = False
    self.last_manual_drive_time = sim_time

    # Estado de torreta Pan/Tilt (peripherals.cpp).
    # Nota: el firmware almacena current_tilt = 180 - safe_ui_angle.
    self.current_pan = 90
    self.current_tilt = 90
    self.target_pan = 90
    self.target_tilt = 90

    # Perifericos y banderas.
    self.laser_on = False
    self.melody_on = False
    self.video_flag = False
    self.sonic_stream_enabled = False
    self.latest_sonic_cm = -1.0

    # Estado de evasion de obstaculos (peripherals.cpp).
    self.us_history = [100.0, 100.0, 100.0]
    self.obstacle_counter = 0
    self.obstacle_found = False
    self.obstacle_alert = ''
    self.auto_state = 0
    self.state_timer = 0
    self.turn_direction = 90
    self.strafe_direction = 90
    self.next_maneuver_strafe = False
    self.stuck_counter = 0
    self.forward_counter = 0

    # Relojes internos de tareas FreeRTOS (50 Hz motores/servos, 60 ms autopilot).
    self._next_50hz_time = sim_time
    self._next_auto_time = sim_time
    self._next_manual_sonic_time = sim_time
    self._next_sonic_tx_time = sim_time
    self._melody_end_time = 0.0

    # Cola de lineas de respuesta TCP (CMD_POWER / CMD_SONIC).
    self._tx_lines: list[str] = []

  def set_mode(self, mode: str, sim_time: float) -> None:
    if mode not in ('manual', 'auto_us_ir', 'auto_ir_only'):
      return
    if mode == self.mode:
      return
    self.mode = mode
    if mode == 'manual':
      self.brake_all_motors()
      self.obstacle_found = False
      self.obstacle_alert = ''
      self._reset_auto_state()
    else:
      self._reset_auto_state()
      self._next_auto_time = sim_time

  def _reset_auto_state(self) -> None:
    self.auto_state = 0
    self.state_timer = 0
    self.next_maneuver_strafe = False
    self.stuck_counter = 0
    self.forward_counter = 0
    self.obstacle_counter = 0
    self.us_history = [100.0, 100.0, 100.0]

  def brake_all_motors(self) -> None:
    self.target_fl = self.target_bl = self.target_fr = self.target_br = 0
    self.immediate_brake = True
    self.immediate_stop = False

  def stop_all_motors(self) -> None:
    self.mode = 'manual'
    self.melody_on = False
    self.target_fl = self.target_bl = self.target_fr = self.target_br = 0
    self.immediate_stop = True
    self.immediate_brake = False
    self.obstacle_found = False
    self.obstacle_alert = ''
    self._reset_auto_state()

  def toggle_melody(self, sim_time: float) -> None:
    """Alterna la melodia Game of Thrones (104.325 s en Melodies.cpp)."""
    self.melody_on = not self.melody_on
    if self.melody_on:
      self._melody_end_time = sim_time + 104.325

  def drive_direct_raw(
      self, fl: int, bl: int, fr: int, br: int, sim_time: float
  ) -> None:
    if fl == 0 and bl == 0 and fr == 0 and br == 0:
      self.target_fl = self.target_bl = self.target_fr = self.target_br = 0
      self.immediate_brake = True
      self.immediate_stop = False
    else:
      self.target_fl = int(fl)
      self.target_bl = int(bl)
      self.target_fr = int(fr)
      self.target_br = int(br)
      self.immediate_brake = False
      self.immediate_stop = False
      self.last_manual_drive_time = sim_time

  def drive_safe(
      self, p1: int, p2: int, p3: int, p4: int, sim_time: float
  ) -> None:
    self.drive_direct_raw(
        map_motor_value(p1),
        map_motor_value(p2),
        map_motor_value(p3),
        map_motor_value(p4),
        sim_time,
    )

  def drive_mecanum(
      self,
      angle: float,
      speed: float,
      rotation: float,
      rotation_speed: float,
      sim_time: float,
  ) -> None:
    fl, bl, fr, br = compute_mecanum_pwms(
        angle, speed, rotation, rotation_speed
    )
    self.drive_direct_raw(fl, bl, fr, br, sim_time)

  def set_pan_angle(self, angle: float) -> None:
    self.target_pan = clamp_int(int(round(angle)), 30, 150)

  def set_tilt_angle(self, ui_angle: float) -> None:
    safe_angle = clamp_int(int(round(ui_angle)), 50, 130)
    self.target_tilt = 180 - safe_angle

  def center_servos(self) -> None:
    self.set_pan_angle(90)
    self.set_tilt_angle(90)

  @property
  def pan_ui_deg(self) -> float:
    return float(180 - self.target_pan)

  @property
  def tilt_ui_deg(self) -> float:
    return float(180 - self.target_tilt)

  @property
  def act_tilt_ui_deg(self) -> float:
    return float(180 - self.current_tilt)

  def pop_tx_lines(self) -> list[str]:
    if not self._tx_lines:
      return []
    lines = self._tx_lines
    self._tx_lines = []
    return lines

  def handle_tcp_line(self, line: str, sim_time: float) -> None:
    """Procesa un comando del protocolo wifi_server.cpp / Main.py."""
    clean = line.strip()
    if not clean:
      return
    parts = clean.split('#')
    cmd = parts[0]

    def _p(idx: int, default: int = 0) -> int:
      if idx >= len(parts):
        return default
      try:
        return int(float(parts[idx]))
      except ValueError:
        return default

    if cmd == 'CMD_SERVO' and len(parts) >= 3:
      sid = _p(1)
      val = _p(2, 90)
      if sid == 0:
        self.set_pan_angle(180 - val)
      elif sid == 1:
        self.set_tilt_angle(val)
    elif cmd == 'CMD_CAMERA' and len(parts) >= 3:
      p1, p2 = _p(1, 90), _p(2, 90)
      if p1 == 90 and p2 == 90:
        self.center_servos()
      else:
        self.set_pan_angle(p1)
        self.set_tilt_angle(p2)
    elif cmd == 'CMD_VIDEO' and len(parts) >= 2:
      self.video_flag = _p(1) == 1
    elif cmd == 'CMD_BUZZER' and len(parts) >= 2:
      if _p(1) == 1:
        self.toggle_melody(sim_time)
    elif cmd == 'CMD_LIGHT' and len(parts) >= 2:
      if _p(1) == 1:
        self.laser_on = not self.laser_on
      else:
        self.laser_on = False
    elif cmd == 'CMD_LED_MOD' and len(parts) >= 2:
      mod = _p(1)
      if mod == 2:
        self.set_mode('auto_us_ir', sim_time)
      elif mod == 3:
        self.set_mode('auto_ir_only', sim_time)
      else:
        self.set_mode('manual', sim_time)
    elif cmd == 'CMD_MODE' and len(parts) >= 2:
      if parts[1] == 'three':
        self.set_mode('auto_us_ir', sim_time)
      else:
        self.set_mode('manual', sim_time)
    elif cmd == 'CMD_MOTOR' and len(parts) >= 5:
      if self.mode == 'manual':
        self.drive_safe(_p(1), _p(2), _p(3), _p(4), sim_time)
    elif cmd in ('CMD_M_MOTOR', 'CMD_CAR_ROTATE') and len(parts) >= 5:
      if self.mode == 'manual':
        self.drive_mecanum(_p(1), _p(2), _p(3), _p(4), sim_time)
    elif cmd == 'CMD_POWER':
      self._tx_lines.append('CMD_POWER#8.25\n')
    elif cmd == 'CMD_SONIC':
      self.sonic_stream_enabled = _p(1, 1) == 1

  def _tick_motors_50hz(self, sim_time: float) -> None:
    if self.immediate_stop:
      self.current_fl = self.current_bl = 0
      self.current_fr = self.current_br = 0
      self.coasting = True
      self.braking = False
      self.immediate_stop = False
      self.immediate_brake = False
      return

    if self.immediate_brake:
      self.current_fl = self.current_bl = 0
      self.current_fr = self.current_br = 0
      self.coasting = False
      self.braking = True
      self.immediate_brake = False
      return

    # Watchdog de 800 ms exacto de motorSlewTask().
    if self.mode == 'manual' and (
        self.target_fl != 0
        or self.target_bl != 0
        or self.target_fr != 0
        or self.target_br != 0
        or self.current_fl != 0
        or self.current_bl != 0
        or self.current_fr != 0
        or self.current_br != 0
    ):
      if (sim_time - self.last_manual_drive_time) > 0.800:
        self.target_fl = self.target_bl = 0
        self.target_fr = self.target_br = 0
        self.current_fl = self.current_bl = 0
        self.current_fr = self.current_br = 0
        self.coasting = False
        self.braking = True
        return

    # Si sigue en Coast (STBY=LOW) sin nuevo comando de conduccion/freno, mantener rueda libre.
    if (
        self.coasting
        and self.target_fl == 0
        and self.target_bl == 0
        and self.target_fr == 0
        and self.target_br == 0
    ):
      self.current_fl = self.current_bl = 0
      self.current_fr = self.current_br = 0
      self.braking = False
      return

    self.coasting = False
    self.current_fl = _apply_ramp_channel(self.current_fl, self.target_fl)
    self.current_bl = _apply_ramp_channel(self.current_bl, self.target_bl)
    self.current_fr = _apply_ramp_channel(self.current_fr, self.target_fr)
    self.current_br = _apply_ramp_channel(self.current_br, self.target_br)
    self.braking = (
        self.current_fl == 0
        and self.current_bl == 0
        and self.current_fr == 0
        and self.current_br == 0
    )

  def _tick_servos_50hz(self) -> None:
    if self.current_pan < self.target_pan:
      self.current_pan += 1
    elif self.current_pan > self.target_pan:
      self.current_pan -= 1

    if self.current_tilt < self.target_tilt:
      self.current_tilt += 1
    elif self.current_tilt > self.target_tilt:
      self.current_tilt -= 1

  def _tick_autopilot_60ms(
      self,
      raw_us_cm: float,
      ir1: bool,
      ir2: bool,
      ir3: bool,
      ir4: bool,
      sim_time: float,
  ) -> None:
    if self.mode == 'manual':
      return

    ir_only = self.mode == 'auto_ir_only'
    distance = -1.0
    us_obstacle = False

    if not ir_only:
      sample = 100.0 if raw_us_cm < 0.0 else min(raw_us_cm, 120.0)
      self.us_history = [self.us_history[1], self.us_history[2], sample]
      distance = sorted(self.us_history)[1]
      if distance >= 100.0:
        distance = -1.0
      self.latest_sonic_cm = distance
      us_obstacle = 0.0 < distance <= 22.0
    else:
      self.latest_sonic_cm = -1.0

    raw_obstacle = ir1 or ir2 or ir3 or ir4 or us_obstacle
    if raw_obstacle:
      self.obstacle_counter = 5
      self.obstacle_found = True
    else:
      if self.obstacle_counter > 0:
        self.obstacle_counter -= 1
      else:
        self.obstacle_found = False

    if self.obstacle_found:
      if us_obstacle:
        self.obstacle_alert = f'OBSTACULO: US ({distance:.0f}cm)'
      elif ir2:
        self.obstacle_alert = 'OBSTACULO: IR2 (Der)'
      elif ir3:
        self.obstacle_alert = 'OBSTACULO: IR3 (Izq)'
      elif ir1:
        self.obstacle_alert = 'OBSTACULO: IR1 (Atras)'
      elif ir4:
        self.obstacle_alert = 'OBSTACULO: IR4 (Atras)'
      else:
        self.obstacle_alert = 'OBSTACULO DETECTADO'
    else:
      self.obstacle_alert = ''

    speed = 1200
    if not ir_only and 18.0 < distance < 40.0:
      speed = 700

    if self.auto_state == 0:  # AVANZANDO
      if us_obstacle or ir2 or ir3:
        self.stuck_counter += 1
        self.forward_counter = 0
        self.auto_state = 1
        self.state_timer = 2
        self.brake_all_motors()

        if self.stuck_counter >= 4:
          self.next_maneuver_strafe = False
          self.turn_direction = self._rng.choice((90, -90))
        elif self.stuck_counter >= 2 and not (ir2 and ir3):
          self.next_maneuver_strafe = True
          if ir3:
            self.strafe_direction = -90
          elif ir2:
            self.strafe_direction = 90
          else:
            self.strafe_direction = self._rng.choice((90, -90))
        else:
          self.next_maneuver_strafe = False
          if ir3 and not ir2:
            self.turn_direction = -90
          elif ir2 and not ir3:
            self.turn_direction = 90
          else:
            self.turn_direction = self._rng.choice((90, -90))
      else:
        self.drive_mecanum(0, speed, 0, 0, sim_time)
        self.forward_counter += 1
        if self.forward_counter > 25:
          self.stuck_counter = 0
          self.forward_counter = min(self.forward_counter, 100)
    elif self.auto_state == 1:  # FRENANDO
      self.brake_all_motors()
      if self.state_timer > 0:
        self.state_timer -= 1
      else:
        if self.next_maneuver_strafe:
          self.auto_state = 4
          self.state_timer = 15
        else:
          self.auto_state = 2
          self.state_timer = 14 if self.stuck_counter >= 4 else 7
    elif self.auto_state == 2:  # REVERSA
      if ir1 or ir4:
        self.brake_all_motors()
        if ir1 and not ir4:
          self.turn_direction = 90
        elif ir4 and not ir1:
          self.turn_direction = -90
        self.auto_state = 3
        self.state_timer = 18 if self.stuck_counter >= 4 else 9
      else:
        self.drive_mecanum(180, speed, 0, 0, sim_time)
        if self.state_timer > 0:
          self.state_timer -= 1
        else:
          self.auto_state = 3
          self.state_timer = 18 if self.stuck_counter >= 4 else 9
    elif self.auto_state == 3:  # GIRANDO
      self.drive_mecanum(0, 0, self.turn_direction, speed, sim_time)
      if self.state_timer > 0:
        self.state_timer -= 1
      else:
        self.auto_state = 0
    elif self.auto_state == 4:  # STRAFING LATERAL (MECANUM)
      side_blocked = (ir3 or ir4) if self.strafe_direction > 0 else (ir2 or ir1)
      if side_blocked:
        self.brake_all_motors()
        self.auto_state = 2
        self.state_timer = 7
      else:
        self.drive_mecanum(self.strafe_direction, speed, 0, 0, sim_time)
        if self.state_timer > 0:
          self.state_timer -= 1
        else:
          self.auto_state = 0

  def step(
      self,
      sim_time: float,
      raw_us_cm: float,
      ir1: bool,
      ir2: bool,
      ir3: bool,
      ir4: bool,
  ) -> None:
    """Avanza las tareas periodicas del firmware hasta sim_time."""
    if sim_time < self._next_50hz_time - 0.5:
      self._next_50hz_time = sim_time
      self._next_auto_time = sim_time
      self._next_manual_sonic_time = sim_time
      self._next_sonic_tx_time = sim_time

    if self.melody_on and sim_time >= self._melody_end_time:
      self.melody_on = False

    while self._next_auto_time <= sim_time + 1e-9:
      self._tick_autopilot_60ms(
          raw_us_cm, ir1, ir2, ir3, ir4, self._next_auto_time
      )
      self._next_auto_time += 0.060

    if self.mode == 'manual':
      if self.sonic_stream_enabled:
        if sim_time >= self._next_manual_sonic_time - 1e-9:
          self.latest_sonic_cm = raw_us_cm
          self._next_manual_sonic_time = sim_time + 0.120
      else:
        self.latest_sonic_cm = -1.0

    while self._next_50hz_time <= sim_time + 1e-9:
      self._tick_motors_50hz(self._next_50hz_time)
      self._tick_servos_50hz()
      self._next_50hz_time += 0.020

    if self.sonic_stream_enabled and sim_time >= self._next_sonic_tx_time - 1e-9:
      dist = self.latest_sonic_cm
      dist_str = f'{dist:.1f}' if dist > 0.0 else '200.0'
      self._tx_lines.append(f'CMD_SONIC#{dist_str}\n')
      self._next_sonic_tx_time = sim_time + 0.150

  @property
  def auto_state_name(self) -> str:
    if self.mode == 'manual':
      return 'Manual'
    return AUTO_STATE_NAMES.get(self.auto_state, 'Desconocido')

  def wheel_target_rad_s(self, max_wheel_rad_s: float) -> tuple[float, float, float, float]:
    scale = max_wheel_rad_s / 4095.0
    return (
        self.current_fl * scale,
        self.current_bl * scale,
        self.current_fr * scale,
        self.current_br * scale,
    )


class Esp32NetworkBridge:
  """Servidor TCP:5000 + Streamer UDP:8000 en 127.0.0.1 compatible con Main.py."""

  def __init__(
      self, host: str = '127.0.0.1', tcp_port: int = 5000, udp_port: int = 8000
  ) -> None:
    self.host = host
    self.tcp_port = tcp_port
    self.udp_port = udp_port

    self._enabled = False
    self._status = 'Desactivado'
    self._tcp_server: socket.socket | None = None
    self._tcp_client: socket.socket | None = None
    self._tcp_buf = b''
    self._udp_sock: socket.socket | None = None
    self._udp_client_addr: tuple[str, int] | None = None
    self._udp_last_heartbeat = 0.0
    self._frame_id = 0
    self._fps_count = 0
    self._fps_t0 = time.monotonic()
    self._udp_fps = 0.0

    self._frame_q: queue.Queue[np.ndarray] = queue.Queue(maxsize=1)
    self._stop_thread = threading.Event()
    self._tx_thread: threading.Thread | None = None
    self._lock = threading.Lock()

  @property
  def enabled(self) -> bool:
    return self._enabled

  @property
  def status(self) -> str:
    return self._status

  @property
  def tcp_connected(self) -> bool:
    return self._tcp_client is not None

  @property
  def udp_streaming(self) -> bool:
    return (
        self._udp_client_addr is not None
        and (time.monotonic() - self._udp_last_heartbeat) <= 8.0
    )

  @property
  def udp_fps(self) -> float:
    if not self.udp_streaming:
      return 0.0
    return self._udp_fps

  def set_enabled(self, enable: bool) -> None:
    if enable == self._enabled:
      return
    if enable:
      self._start()
    else:
      self.close()

  def _start(self) -> None:
    try:
      tcp_srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      tcp_srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
      tcp_srv.setblocking(False)
      tcp_srv.bind((self.host, self.tcp_port))
      tcp_srv.listen(1)

      udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
      udp_sock.setblocking(False)
      udp_sock.bind((self.host, self.udp_port))
    except OSError as exc:
      for s in (locals().get('tcp_srv'), locals().get('udp_sock')):
        if s is not None:
          try:
            s.close()
          except OSError:
            pass
      self._enabled = False
      self._status = f'Puerto ocupado ({exc.errno})'
      return

    self._tcp_server = tcp_srv
    self._udp_sock = udp_sock
    self._tcp_client = None
    self._tcp_buf = b''
    self._udp_client_addr = None
    self._frame_id = 0
    self._enabled = True
    self._status = f'Escuchando {self.host}:{self.tcp_port}/{self.udp_port}'

    self._stop_thread.clear()
    self._tx_thread = threading.Thread(
        target=self._udp_tx_worker, name='Esp32SimUdpTx', daemon=True
    )
    self._tx_thread.start()

  def close(self) -> None:
    self._enabled = False
    self._stop_thread.set()
    if self._tx_thread is not None and self._tx_thread.is_alive():
      try:
        self._frame_q.put_nowait(np.empty((0, 0, 3), dtype=np.uint8))
      except queue.Full:
        pass
      self._tx_thread.join(timeout=0.5)
    self._tx_thread = None

    with self._lock:
      for sock_attr in ('_tcp_client', '_tcp_server', '_udp_sock'):
        sock = getattr(self, sock_attr)
        if sock is not None:
          try:
            sock.close()
          except OSError:
            pass
          setattr(self, sock_attr, None)
      self._udp_client_addr = None
    self._tcp_buf = b''
    self._udp_fps = 0.0
    self._status = 'Desactivado'

  def poll(
      self,
      firmware: Esp32CarFirmware,
      sim_time: float,
      on_disconnect: Callable[[], None] | None = None,
  ) -> None:
    """Acepta conexiones y lee paquetes TCP/UDP de manera no bloqueante."""
    if not self._enabled or self._tcp_server is None or self._udp_sock is None:
      return

    # 1. Aceptar nuevo cliente TCP (reemplaza anterior si se reconecta Main.py).
    while True:
      try:
        conn, _ = self._tcp_server.accept()
      except BlockingIOError:
        break
      except OSError:
        break
      conn.setblocking(False)
      try:
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
      except OSError:
        pass
      if self._tcp_client is not None:
        try:
          self._tcp_client.close()
        except OSError:
          pass
        firmware.stop_all_motors()
        firmware.sonic_stream_enabled = False
      self._tcp_client = conn
      self._tcp_buf = b''
      self._status = f'Main.py conectado ({self.host}:{self.tcp_port})'

    # 2. Leer comandos desde cliente TCP conectado.
    if self._tcp_client is not None:
      while True:
        try:
          chunk = self._tcp_client.recv(1024)
        except BlockingIOError:
          break
        except OSError as exc:
          if exc.errno in (errno.EWOULDBLOCK, errno.EAGAIN):
            break
          self._close_tcp_client(firmware, on_disconnect)
          break

        if not chunk:
          self._close_tcp_client(firmware, on_disconnect)
          break

        self._tcp_buf += chunk
        while b'\n' in self._tcp_buf:
          raw_line, self._tcp_buf = self._tcp_buf.split(b'\n', 1)
          firmware.handle_tcp_line(
              raw_line.decode('utf-8', errors='ignore'), sim_time
          )

      # Enviar respuestas TCP pendientes (CMD_POWER, CMD_SONIC).
      if self._tcp_client is not None:
        for tx_line in firmware.pop_tx_lines():
          try:
            self._tcp_client.sendall(tx_line.encode('utf-8'))
          except OSError:
            self._close_tcp_client(firmware, on_disconnect)
            break

    # 3. Leer handshake UDP ("START" / "STOP") en puerto 8000.
    while True:
      try:
        data, addr = self._udp_sock.recvfrom(64)
      except BlockingIOError:
        break
      except OSError:
        break
      msg = data.decode('ascii', errors='ignore').strip()
      if msg.startswith('START'):
        with self._lock:
          self._udp_client_addr = addr
          self._udp_last_heartbeat = time.monotonic()
        firmware.video_flag = True
      elif msg.startswith('STOP'):
        with self._lock:
          self._udp_client_addr = None
        firmware.video_flag = False

    if (
        self._udp_client_addr is not None
        and (time.monotonic() - self._udp_last_heartbeat) > 8.0
    ):
      with self._lock:
        self._udp_client_addr = None

  def _close_tcp_client(
      self,
      firmware: Esp32CarFirmware,
      on_disconnect: Callable[[], None] | None,
  ) -> None:
    if self._tcp_client is not None:
      try:
        self._tcp_client.close()
      except OSError:
        pass
      self._tcp_client = None
    self._tcp_buf = b''
    firmware.stop_all_motors()
    firmware.sonic_stream_enabled = False
    if on_disconnect is not None:
      on_disconnect()
    self._status = f'Escuchando {self.host}:{self.tcp_port}/{self.udp_port}'

  def should_capture_frame(self, firmware: Esp32CarFirmware) -> bool:
    return self._enabled and self.udp_streaming and firmware.video_flag

  def submit_frame(self, rgb: np.ndarray) -> None:
    if not self._enabled or not self.udp_streaming:
      return
    try:
      if self._frame_q.full():
        self._frame_q.get_nowait()
    except queue.Empty:
      pass
    try:
      self._frame_q.put_nowait(rgb.copy())
    except queue.Full:
      pass

  def _udp_tx_worker(self) -> None:
    chunk_size = 1400
    magic = 0x5649444F  # "VIDO"
    while not self._stop_thread.is_set():
      try:
        rgb = self._frame_q.get(timeout=0.2)
      except queue.Empty:
        continue
      if rgb.size == 0 or self._stop_thread.is_set():
        continue

      with self._lock:
        sock = self._udp_sock
        addr = self._udp_client_addr
      if sock is None or addr is None:
        continue

      jpg = jpeg_mini.encode_jpeg_rgb(rgb, quality=68)
      total_len = len(jpg)
      total_chunks = (total_len + chunk_size - 1) // chunk_size
      fid = self._frame_id
      self._frame_id += 1

      for cid in range(total_chunks):
        start = cid * chunk_size
        chunk = jpg[start : start + chunk_size]
        hdr = struct.pack('<I I H H H', magic, fid, cid, total_chunks, len(chunk))
        try:
          sock.sendto(hdr + chunk, addr)
        except OSError:
          break

      self._fps_count += 1
      now = time.monotonic()
      if now - self._fps_t0 >= 1.0:
        self._udp_fps = self._fps_count / (now - self._fps_t0)
        self._fps_count = 0
        self._fps_t0 = now
