"""Plugin de interfaz Dear ImGui para el Freenove ESP32 4WD Mecanum CamCar."""

from __future__ import annotations

import messages as user_messages
from mujoco.studio import messages
from mujoco.studio import plugin
from mujoco.studio.app import plugins as app_plugins

imgui = app_plugins.imgui


def _table_slider_row(
    label: str,
    widget_id: str,
    value: float,
    vmin: float,
    vmax: float,
    fmt: str = '%.0f',
) -> tuple[bool, float]:
  """Dibuja una fila alineada de etiqueta fija + slider ancho completo."""
  imgui.TableNextRow()
  imgui.TableNextColumn()
  imgui.AlignTextToFramePadding()
  imgui.TextUnformatted(label)
  imgui.TableNextColumn()
  imgui.SetNextItemWidth(-1)
  return imgui.SliderFloat(widget_id, value, vmin, vmax, fmt)


class ViewerPlugin(plugin.ViewerPlugin):
  """Panel interactivo de control y telemetria para el CamCar en MuJoCo Studio."""

  window_open: bool = True

  mode: str = 'manual'
  ly: float = 0.0
  lx: float = 0.0
  rx: float = 0.0
  pan_ui_deg: float = 90.0
  tilt_ui_deg: float = 90.0
  laser_on: bool = False
  bridge_enabled: bool = True
  cmd_seq: int = 0

  arena_half_size_m: float = 1.65
  spawn_obstacles: bool = True
  max_wheel_rad_s: float = 18.0
  ir_threshold_cm: float = 13.0

  _pad_active: bool = False
  telemetry: user_messages.CarTelemetry = user_messages.CarTelemetry()

  @messages.handler
  def on_telemetry(self, snapshot: user_messages.CarTelemetry) -> None:
    self.telemetry = snapshot

  def _send_command(self, bump_seq: bool = False) -> None:
    if bump_seq:
      self.cmd_seq += 1
    self.viewer.send_to_sim(
        user_messages.CarCommand(
            mode=self.mode,
            ly=self.ly,
            lx=self.lx,
            rx=self.rx,
            pan_ui_deg=self.pan_ui_deg,
            tilt_ui_deg=self.tilt_ui_deg,
            laser_on=self.laser_on,
            bridge_enabled=self.bridge_enabled,
            command_seq=self.cmd_seq,
        )
    )

  def _send_pulse(
      self,
      cmd_type: str,
      angle: int = 0,
      speed: int = 0,
      rot: int = 0,
      rot_speed: int = 0,
  ) -> None:
    self.viewer.send_to_sim(
        user_messages.CarDrivePulseEvent(
            cmd_type=cmd_type,
            angle=angle,
            speed=speed,
            rot=rot,
            rot_speed=rot_speed,
        )
    )

  def _send_scene_params(self) -> None:
    self.viewer.send_to_sim(
        user_messages.SceneParams(
            arena_half_size_m=self.arena_half_size_m,
            spawn_obstacles=self.spawn_obstacles,
            max_wheel_rad_s=self.max_wheel_rad_s,
            ir_threshold_cm=self.ir_threshold_cm,
        )
    )

  def build_mode_gui(self) -> None:
    t = self.telemetry
    cur_mode = t.mode if t.bridge_enabled and t.tcp_connected else self.mode
    if imgui.RadioButton('Manual###ModeManual', cur_mode == 'manual'):
      self.mode = 'manual'
      self._send_command(bump_seq=True)
    imgui.SameLine()
    if imgui.RadioButton(
        'Auto US+IR (M2)###ModeAutoUS', cur_mode == 'auto_us_ir'
    ):
      self.mode = 'auto_us_ir'
      self._send_command(bump_seq=True)
    imgui.SameLine()
    if imgui.RadioButton(
        'Auto Solo IR (M3)###ModeAutoIR', cur_mode == 'auto_ir_only'
    ):
      self.mode = 'auto_ir_only'
      self._send_command(bump_seq=True)

  def build_mecanum_pad_gui(self) -> None:
    any_pad_held = False
    flags = int(imgui.TableFlags.SizingStretchSame) | int(
        imgui.TableFlags.NoPadOuterX
    )
    pad_rows = (
        (
            ('NW###PadNW', 45, 1500, 0, 0, 'Diagonal adelante-izquierda'),
            ('Adelante###PadFwd', 0, 1500, 0, 0, 'Avanzar recto (+X)'),
            ('NE###PadNE', -45, 1500, 0, 0, 'Diagonal adelante-derecha'),
        ),
        (
            ('Strafe Izq###PadL', 90, 1500, 0, 0, 'Desplazamiento lateral +Y'),
            ('FRENAR###PadStop', 0, 0, 0, 0, 'Freno activo instantaneo'),
            ('Strafe Der###PadR', -90, 1500, 0, 0, 'Desplazamiento lateral -Y'),
        ),
        (
            ('SW###PadSW', 135, 1500, 0, 0, 'Diagonal atras-izquierda'),
            ('Atras###PadBwd', 180, 1500, 0, 0, 'Retroceso recto (-X)'),
            ('SE###PadSE', -135, 1500, 0, 0, 'Diagonal atras-derecha'),
        ),
        (
            ('Giro Izq (CCW)###PadRotL', 0, 0, 90, 1500, 'Rotar sobre su eje CCW'),
            ('Reset Pose###PadReset', -1, 0, 0, 0, 'Recolocar coche en el origen'),
            ('Giro Der (CW)###PadRotR', 0, 0, -90, 1500, 'Rotar sobre su eje CW'),
        ),
    )

    if imgui.BeginTable('##mecanum_pad_table', 3, flags):
      try:
        for row in pad_rows:
          imgui.TableNextRow()
          for label, ang, spd, rot, rspd, tip in row:
            imgui.TableNextColumn()
            clicked = imgui.Button(label, imgui.Vec2(-1.0, 0.0))
            imgui.SetItemTooltip(tip)
            if ang == -1:
              if clicked:
                self.ly = self.lx = self.rx = 0.0
                self.mode = 'manual'
                self._send_pulse('reset_pose')
            elif label.startswith('FRENAR'):
              if clicked:
                self.ly = self.lx = self.rx = 0.0
                self.mode = 'manual'
                self._send_pulse('brake')
            elif imgui.IsItemActive():
              any_pad_held = True
              self.mode = 'manual'
              self._send_pulse('mecanum', ang, spd, rot, rspd)
      finally:
        imgui.EndTable()

    if any_pad_held:
      self._pad_active = True
    elif self._pad_active:
      self._pad_active = False
      self._send_pulse('brake')

    # Sliders continuos tipo crucero / Xbox.
    tflags = int(imgui.TableFlags.SizingStretchProp) | int(
        imgui.TableFlags.NoPadOuterX
    )
    changed = False
    if imgui.BeginTable('##cruise_sliders', 2, tflags):
      try:
        imgui.TableSetupColumn('Label', int(imgui.TableColumnFlags.WidthFixed))
        imgui.TableSetupColumn(
            'Slider', int(imgui.TableColumnFlags.WidthStretch)
        )
        c, self.ly = _table_slider_row(
            'Avance LY:', '##ly', self.ly, -1500.0, 1500.0
        )
        changed |= c
        c, self.lx = _table_slider_row(
            'Strafe LX:', '##lx', self.lx, -1500.0, 1500.0
        )
        changed |= c
        c, self.rx = _table_slider_row(
            'Giro RX:', '##rx', self.rx, -1500.0, 1500.0
        )
        changed |= c
      finally:
        imgui.EndTable()

    if changed:
      self.mode = 'manual'
      self._send_command(bump_seq=True)

    if imgui.Button('Poner sliders a 0 / Detener###StopCruiseBtn', imgui.Vec2(-1.0, 0.0)):
      self.ly = self.lx = self.rx = 0.0
      self._send_pulse('stop')

  def build_turret_gui(self) -> None:
    t = self.telemetry
    tflags = int(imgui.TableFlags.SizingStretchProp) | int(
        imgui.TableFlags.NoPadOuterX
    )
    changed = False
    if imgui.BeginTable('##turret_table', 2, tflags):
      try:
        imgui.TableSetupColumn('Label', int(imgui.TableColumnFlags.WidthFixed))
        imgui.TableSetupColumn(
            'Slider', int(imgui.TableColumnFlags.WidthStretch)
        )
        c, self.pan_ui_deg = _table_slider_row(
            'Pan Camara:', '##pan_deg', self.pan_ui_deg, 30.0, 150.0, '%.0f deg'
        )
        changed |= c
        c, self.tilt_ui_deg = _table_slider_row(
            'Tilt Camara:', '##tilt_deg', self.tilt_ui_deg, 50.0, 130.0, '%.0f deg'
        )
        changed |= c
      finally:
        imgui.EndTable()

    if changed:
      self._send_command()

    if imgui.Button('Centrar Torreta###CenterTurretBtn'):
      self.pan_ui_deg = 90.0
      self.tilt_ui_deg = 90.0
      self._send_pulse('center_turret')
    imgui.SameLine()
    laser_label = (
        'Laser: ENCENDIDO###LaserBtn'
        if t.laser_on
        else 'Laser: APAGADO###LaserBtn'
    )
    if imgui.Button(laser_label):
      self.laser_on = not t.laser_on
      self._send_pulse('toggle_laser')
    imgui.SetItemTooltip(
        'Activa el diodo laser sobre la torreta.\nUsa la camara esp32_cam en Render -> Camera.'
    )
    imgui.TextUnformatted(
        f'Angulo Real Torreta: Pan {t.act_pan_deg:5.1f} deg  |  Tilt {t.act_tilt_deg:5.1f} deg'
    )

  def build_bridge_gui(self) -> None:
    t = self.telemetry
    changed, self.bridge_enabled = imgui.Checkbox(
        'Servidor Virtual 127.0.0.1 (Main.py / Xbox)', self.bridge_enabled
    )
    imgui.SetItemTooltip(
        'Abre TCP 5000 (comandos) y UDP 8000 (video JPEG)\npara conectar Main.py escribiendo IP 127.0.0.1'
    )
    if changed:
      self._send_command()

    tcp_txt = 'CONECTADO' if t.tcp_connected else 'Esperando'
    udp_txt = f'Activo ({t.udp_fps:.1f} FPS)' if t.udp_streaming else 'Inactivo'
    imgui.TextUnformatted(f'Estado: {t.bridge_status}')
    imgui.TextUnformatted(f'TCP 5000: {tcp_txt}  |  UDP 8000: {udp_txt}')

  def build_telemetry_gui(self) -> None:
    t = self.telemetry
    us_txt = f'{t.us_distance_cm:5.1f} cm' if t.us_distance_cm > 0 else 'Libre (>400 cm)'
    driver_txt = (
        'COAST (STBY=LOW)'
        if t.coasting
        else ('FRENO ACTIVO' if t.braking else 'TRACCION')
    )
    melody_txt = '  |  Melodia: ON' if t.melody_on else ''
    imgui.TextUnformatted(
        f'Ultrasonico: {us_txt}  |  Estado: {t.auto_state_name}'
    )
    imgui.TextUnformatted(f'Driver TB6612: {driver_txt}{melody_txt}')
    imgui.TextUnformatted(
        f'IR [FL:{int(t.ir3_fl)} FR:{int(t.ir2_fr)} BL:{int(t.ir4_bl)} BR:{int(t.ir1_br)}]'
        f'  Atasco: {t.stuck_counter}'
    )
    if t.obstacle_alert:
      imgui.TextUnformatted(f'TFT Alerta: {t.obstacle_alert}')
    imgui.TextUnformatted(
        f'PWM [FL:{t.pwm_fl:5d} BL:{t.pwm_bl:5d} FR:{t.pwm_fr:5d} BR:{t.pwm_br:5d}]'
    )
    imgui.TextUnformatted(
        f'Pos: ({t.pos_x_m:+.2f}, {t.pos_y_m:+.2f}) m  Yaw: {t.yaw_deg:+.1f} deg'
    )
    imgui.TextUnformatted(
        f'Vel: Fwd {t.vel_forward_m_s:+.2f}  Lat {t.vel_left_m_s:+.2f} m/s  |  t={t.sim_time:.1f}s'
    )

  def build_arena_gui(self) -> None:
    if not imgui.CollapsingHeader('Arena y Calibracion Fisica'):
      return
    tflags = int(imgui.TableFlags.SizingStretchProp) | int(
        imgui.TableFlags.NoPadOuterX
    )
    changed = False
    if imgui.BeginTable('##arena_table', 2, tflags):
      try:
        imgui.TableSetupColumn('Label', int(imgui.TableColumnFlags.WidthFixed))
        imgui.TableSetupColumn(
            'Slider', int(imgui.TableColumnFlags.WidthStretch)
        )
        c, self.arena_half_size_m = _table_slider_row(
            'Tamano Arena:', '##arena_half', self.arena_half_size_m, 1.0, 3.0, '%.2f m'
        )
        changed |= c
        c, self.max_wheel_rad_s = _table_slider_row(
            'Vel. Max Rueda:', '##max_w', self.max_wheel_rad_s, 6.0, 26.0, '%.1f rad/s'
        )
        changed |= c
        c, self.ir_threshold_cm = _table_slider_row(
            'Alcance IR:', '##ir_rng', self.ir_threshold_cm, 6.0, 25.0, '%.0f cm'
        )
        changed |= c
      finally:
        imgui.EndTable()

    obs_changed, self.spawn_obstacles = imgui.Checkbox(
        'Obstaculos en arena', self.spawn_obstacles
    )
    if changed or obs_changed:
      self._send_scene_params()

    if imgui.Button('Exportar a exported_scene.xml###BakeXmlBtn'):
      self.viewer.send_to_sim(user_messages.BakeEvent())
    imgui.SetItemTooltip(
        'Guarda la escena procedural actual en simulation/exported_scene.xml\npara abrirla en otros visores MuJoCo sin reemplazar la escena dinamica.'
    )

  @messages.handler
  def on_build_gui(self, event: messages.BuildGuiEvent) -> None:
    del event
    if not self.window_open:
      return
    expanded, self.window_open = imgui.Begin(
        'ESP32 4WD Mecanum CamCar###Esp32CamCarPanel', self.window_open
    )
    try:
      if not expanded:
        return
      self.build_mode_gui()
      imgui.Separator()
      self.build_mecanum_pad_gui()
      imgui.Separator()
      self.build_turret_gui()
      imgui.Separator()
      self.build_bridge_gui()
      imgui.Separator()
      self.build_telemetry_gui()
      imgui.Separator()
      self.build_arena_gui()
    finally:
      imgui.End()
