"""Contratos de mensajes compartidos entre sim_plugin.py y viewer_plugin.py.

Define los Snapshot y Event inmutables para el gemelo digital en MuJoCo Studio
del Freenove ESP32 4WD Mecanum CamCar.
"""

import dataclasses
from mujoco.studio import messages


@dataclasses.dataclass(frozen=True)
class CarCommand(messages.Snapshot):
  """UI -> sim: comandos continuos de conduccion, torreta, laser y puente."""

  mode: str = 'manual'  # 'manual', 'auto_us_ir', 'auto_ir_only'

  # Mandos continuos Mecanum (rango ESP32 / Xbox: -1500 a 1500).
  ly: float = 0.0  # + adelante (+X), - atras (-X)
  lx: float = 0.0  # + strafe derecha (-Y), - strafe izquierda (+Y)
  rx: float = 0.0  # + giro derecha CW, - giro izquierda CCW

  # Convencion UI (Main.py): Pan [30..150], Tilt [50..130].
  pan_ui_deg: float = 90.0
  tilt_ui_deg: float = 90.0
  laser_on: bool = False
  bridge_enabled: bool = True

  # Secuencia monotona para refrescar watchdog al interactuar desde la UI.
  command_seq: int = 0


@dataclasses.dataclass(frozen=True)
class CarDrivePulseEvent(messages.Event):
  """UI -> sim: pulsos momentaneos del pad, botones de accion o reset."""

  cmd_type: str = 'stop'  # 'mecanum', 'stop', 'brake', 'center_turret', 'reset_pose', 'toggle_laser', 'toggle_melody'
  angle: int = 0
  speed: int = 0
  rot: int = 0
  rot_speed: int = 0


@dataclasses.dataclass(frozen=True)
class SceneParams(messages.Snapshot):
  """UI -> sim: parametros fisicos de arena y calibracion (reconstruye MjSpec)."""

  arena_half_size_m: float = 1.65
  spawn_obstacles: bool = True
  max_wheel_rad_s: float = 18.0
  ir_threshold_cm: float = 13.0


@dataclasses.dataclass(frozen=True)
class BakeEvent(messages.Event):
  """UI -> sim: exporta el MjSpec procedural actual a simulation/scene.xml."""


@dataclasses.dataclass(frozen=True)
class CarTelemetry(messages.Snapshot):
  """sim -> UI: estado del firmware, sensores, ruedas y puente TCP/UDP."""

  sim_time: float = 0.0
  mode: str = 'manual'
  auto_state_name: str = 'Manual'
  stuck_counter: int = 0
  obstacle_found: bool = False
  obstacle_alert: str = ''

  # PWM actual tras ApplyRamp (-4095 .. +4095) y freno activo.
  pwm_fl: int = 0
  pwm_bl: int = 0
  pwm_fr: int = 0
  pwm_br: int = 0
  braking: bool = True
  coasting: bool = False

  # Estado de torreta Pan/Tilt (convencion UI Main.py y servo interno).
  pan_ui_deg: float = 90.0
  tilt_ui_deg: float = 90.0
  act_pan_deg: float = 90.0
  act_tilt_deg: float = 90.0
  laser_on: bool = False
  melody_on: bool = False

  # Sensores simulados por raycast (mj_ray).
  us_distance_cm: float = -1.0
  ir1_br: bool = False  # Trasero Derecho
  ir2_fr: bool = False  # Delantero Derecho
  ir3_fl: bool = False  # Delantero Izquierdo
  ir4_bl: bool = False  # Trasero Izquierdo

  # Pose y velocidad real del chasis en MuJoCo.
  pos_x_m: float = 0.0
  pos_y_m: float = 0.0
  yaw_deg: float = 0.0
  vel_forward_m_s: float = 0.0
  vel_left_m_s: float = 0.0
  yaw_rate_deg_s: float = 0.0

  # Estado del servidor virtual 127.0.0.1 (TCP 5000 / UDP 8000).
  bridge_enabled: bool = True
  bridge_status: str = 'Iniciando...'
  tcp_connected: bool = False
  udp_streaming: bool = False
  udp_fps: float = 0.0
