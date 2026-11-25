"""Plugin de simulacion MuJoCo Studio para el Freenove ESP32 4WD Mecanum CamCar.

Construye el gemelo digital completo mediante mujoco.MjSpec:
- Chasis 4WD con 4 ruedas Mecanum fisicas de 12 rodillos pasivos en patron X
- Torreta Pan/Tilt 2-DOF con camara FPV (esp32_cam) y diodo/haz laser dinamico
- Sensor ultrasonico HC-SR04 + 4 sensores IR de esquinas mediante mj_ray
- LEDs traseros de freno dinamicos y pantalla trasera ST7735
- Replica 1:1 del firmware ESP32 + puente opcional 127.0.0.1 (TCP:5000 / UDP:8000)
"""

from __future__ import annotations

import math
import os

import car_firmware as fw
import messages as user_messages
import mujoco
from mujoco.studio import messages
from mujoco.studio import plugin
import numpy as np

EXPORT_XML = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'exported_scene.xml'
)

WHEEL_DEFS = (
    # (prefix, x_m, y_m, roller_sign_s)
    ('fl', 0.068, 0.088, -1.0),
    ('bl', -0.068, 0.088, 1.0),
    ('fr', 0.068, -0.088, 1.0),
    ('br', -0.068, -0.088, -1.0),
)

IR_DEFS = (
    # (name_idx, site_name, led_geom_name, pos_local, dir_local)
    (1, 'ir1_site', 'ir1_led_vis', [-0.116, -0.044, 0.008], [-0.94, -0.34, 0.0]),
    (2, 'ir2_site', 'ir2_led_vis', [0.116, -0.044, 0.008], [0.94, -0.34, 0.0]),
    (3, 'ir3_site', 'ir3_led_vis', [0.116, 0.044, 0.008], [0.94, 0.34, 0.0]),
    (4, 'ir4_site', 'ir4_led_vis', [-0.116, 0.044, 0.008], [-0.94, 0.34, 0.0]),
)

# Mascara para mj_ray: solo entorno/arena (group 0 y group 3); excluye coche (group 4).
_ENV_RAY_GROUP = np.array([1, 0, 0, 1, 0, 0], dtype=np.uint8)


def _quat_from_z_to_vec(vx: float, vy: float, vz: float) -> list[float]:
  """Cuaternion [w, x, y, z] que alinea el eje +Z local con un vector dado."""
  vec = np.array([vx, vy, vz], dtype=np.float64)
  nrm = float(np.linalg.norm(vec))
  if nrm < 1e-9:
    return [1.0, 0.0, 0.0, 0.0]
  vec /= nrm
  z_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
  dot = float(np.dot(z_axis, vec))
  if dot > 0.999999:
    return [1.0, 0.0, 0.0, 0.0]
  if dot < -0.999999:
    return [0.0, 1.0, 0.0, 0.0]
  cross = np.cross(z_axis, vec)
  q = np.array([1.0 + dot, cross[0], cross[1], cross[2]], dtype=np.float64)
  q /= float(np.linalg.norm(q))
  return q.tolist()


def build_esp32_camcar_spec(
    arena_half_size_m: float = 1.65,
    spawn_obstacles: bool = True,
    wheel_kp: float = 0.06,
) -> mujoco.MjSpec:
  """Construye el MjSpec procedural del Freenove 4WD Mecanum Car y arena."""
  spec = mujoco.MjSpec()
  spec.modelname = 'esp32_mecanum_camcar'

  # Fisica precisa a 500 Hz y renderizado offscreen para esp32_cam.
  spec.option.timestep = 0.002
  spec.option.gravity = [0.0, 0.0, -9.81]
  spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
  spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
  spec.option.iterations = 50

  spec.visual.global_.offwidth = 320
  spec.visual.global_.offheight = 240
  spec.visual.global_.fovy = 48.0
  spec.visual.headlight.ambient = [0.35, 0.35, 0.35]
  spec.visual.headlight.diffuse = [0.65, 0.65, 0.65]

  # Clases por defecto: visuales (sin colision/masa) vs colisiones por grupo.
  vis_cls = spec.add_default('visual', None)
  vis_cls.geom.group = 2
  vis_cls.geom.contype = 0
  vis_cls.geom.conaffinity = 0
  vis_cls.geom.density = 0.0
  vis_cls.geom.mass = 0.0

  env_col_cls = spec.add_default('env_collision', None)
  env_col_cls.geom.group = 3
  env_col_cls.geom.contype = 1
  env_col_cls.geom.conaffinity = 3
  env_col_cls.geom.solref = [0.005, 1.0]
  env_col_cls.geom.rgba = [0.4, 0.6, 0.8, 0.35]

  car_col_cls = spec.add_default('car_collision', None)
  car_col_cls.geom.group = 4
  car_col_cls.geom.contype = 1
  car_col_cls.geom.conaffinity = 1
  car_col_cls.geom.solref = [0.004, 1.0]
  car_col_cls.geom.rgba = [0.2, 0.85, 0.4, 0.45]

  roller_col_cls = spec.add_default('roller_collision', None)
  roller_col_cls.geom.group = 4
  roller_col_cls.geom.contype = 2
  roller_col_cls.geom.conaffinity = 1
  roller_col_cls.geom.condim = 4
  roller_col_cls.geom.friction = [1.25, 0.015, 0.004]
  roller_col_cls.geom.solref = [0.004, 1.0]
  roller_col_cls.geom.solimp = [0.92, 0.96, 0.001, 0.5, 2.0]
  roller_col_cls.geom.rgba = [0.1, 0.7, 0.9, 0.45]

  # Texturas y materiales de arena y coche.
  floor_tex = spec.add_texture(
      name='arena_grid_tex',
      type=mujoco.mjtTexture.mjTEXTURE_2D,
      builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
      rgb1=[0.22, 0.25, 0.30],
      rgb2=[0.16, 0.19, 0.24],
      width=512,
      height=512,
      mark=mujoco.mjtMark.mjMARK_EDGE,
      markrgb=[0.32, 0.36, 0.42],
  )
  floor_mat = spec.add_material(name='arena_floor_mat')
  floor_mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = floor_tex.name
  floor_mat.texrepeat = [10.0, 10.0]
  floor_mat.texuniform = True
  floor_mat.reflectance = 0.08

  acrylic_mat = spec.add_material(
      name='dark_acrylic',
      rgba=[0.11, 0.13, 0.16, 0.92],
      specular=0.7,
      shininess=0.8,
  )
  pcb_mat = spec.add_material(
      name='pcb_black',
      rgba=[0.07, 0.08, 0.10, 1.0],
      specular=0.4,
      shininess=0.5,
  )
  pcb_blue = spec.add_material(
      name='pcb_blue',
      rgba=[0.06, 0.25, 0.58, 1.0],
      specular=0.4,
      shininess=0.5,
  )
  brass_mat = spec.add_material(
      name='brass',
      rgba=[0.82, 0.65, 0.25, 1.0],
      metallic=0.85,
      roughness=0.25,
  )
  alu_mat = spec.add_material(
      name='aluminum',
      rgba=[0.78, 0.80, 0.84, 1.0],
      metallic=0.85,
      roughness=0.28,
  )
  motor_yellow = spec.add_material(
      name='tt_yellow',
      rgba=[0.94, 0.72, 0.08, 1.0],
      roughness=0.4,
  )
  rubber_mat = spec.add_material(
      name='roller_rubber',
      rgba=[0.18, 0.20, 0.22, 1.0],
      roughness=0.85,
  )
  wheel_rim_mat = spec.add_material(
      name='mecanum_hub',
      rgba=[0.12, 0.14, 0.17, 1.0],
      metallic=0.4,
      roughness=0.45,
  )

  world = spec.worldbody
  world.add_light(
      name='sun_light',
      pos=[0.0, 0.0, 3.2],
      dir=[0.15, -0.15, -1.0],
      diffuse=[0.85, 0.85, 0.85],
      ambient=[0.28, 0.28, 0.28],
      type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
  )
  world.add_light(
      name='fill_light',
      pos=[-1.5, 1.5, 2.0],
      dir=[0.4, -0.4, -0.8],
      diffuse=[0.35, 0.38, 0.45],
  )

  # Piso de la arena.
  world.add_geom(
      env_col_cls,
      name='arena_floor',
      type=mujoco.mjtGeom.mjGEOM_PLANE,
      size=[arena_half_size_m + 0.5, arena_half_size_m + 0.5, 0.05],
      material=floor_mat.name,
      condim=4,
      friction=[1.1, 0.02, 0.005],
      group=0,  # Visible siempre
  )

  # Camaras fijas panoramicas.
  world.add_camera(
      name='overview_3q',
      pos=[0.0, -2.15, 1.55],
      xyaxes=[1.0, 0.0, 0.0, 0.0, 0.58, 0.81],
      fovy=46.0,
  )
  world.add_camera(
      name='top_down',
      pos=[0.0, 0.0, 3.2],
      xyaxes=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
      fovy=52.0,
  )

  # Muros perimetrales bajos (altura suficiente para HC-SR04 y rayos IR).
  half = arena_half_size_m
  wall_t = 0.04
  wall_h = 0.14
  walls = (
      ('wall_north', [half + wall_t, 0.0, wall_h], [wall_t, half + wall_t * 2, wall_h]),
      ('wall_south', [-half - wall_t, 0.0, wall_h], [wall_t, half + wall_t * 2, wall_h]),
      ('wall_east', [0.0, -half - wall_t, wall_h], [half, wall_t, wall_h]),
      ('wall_west', [0.0, half + wall_t, wall_h], [half, wall_t, wall_h]),
  )
  for wname, wpos, wsize in walls:
    world.add_geom(
        vis_cls,
        name=f'{wname}_vis',
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=wpos,
        size=wsize,
        rgba=[0.28, 0.34, 0.42, 1.0],
    )
    world.add_geom(
        env_col_cls,
        name=f'{wname}_col',
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=wpos,
        size=wsize,
    )

  # Obstaculos variados dentro de la arena para probar piloto automatico US/IR.
  if spawn_obstacles:
    obs_defs = (
        ('obs_red_box', mujoco.mjtGeom.mjGEOM_BOX, [0.72, 0.12, 0.11], [0.10, 0.16, 0.11], [0.85, 0.22, 0.20, 1.0]),
        ('obs_blue_cyl', mujoco.mjtGeom.mjGEOM_CYLINDER, [0.58, -0.48, 0.12], [0.09, 0.12], [0.18, 0.52, 0.88, 1.0]),
        ('obs_orange_box', mujoco.mjtGeom.mjGEOM_BOX, [-0.62, 0.55, 0.11], [0.13, 0.11, 0.11], [0.92, 0.52, 0.12, 1.0]),
        ('obs_green_cyl', mujoco.mjtGeom.mjGEOM_CYLINDER, [-0.55, -0.62, 0.12], [0.10, 0.12], [0.20, 0.74, 0.42, 1.0]),
        ('obs_yellow_barrier', mujoco.mjtGeom.mjGEOM_BOX, [1.05, -0.78, 0.10], [0.06, 0.28, 0.10], [0.92, 0.76, 0.15, 1.0]),
        ('obs_purple_pillar', mujoco.mjtGeom.mjGEOM_CYLINDER, [0.08, 0.92, 0.12], [0.085, 0.12], [0.58, 0.34, 0.82, 1.0]),
    )
    for oname, otype, opos, osize, orgba in obs_defs:
      world.add_geom(
          vis_cls,
          name=f'{oname}_vis',
          type=otype,
          pos=opos,
          size=osize,
          rgba=orgba,
      )
      world.add_geom(
          env_col_cls,
          name=f'{oname}_col',
          type=otype,
          pos=opos,
          size=osize,
      )

  # =====================================================================
  # COCHE MECANUM ESP32_CamCar (+X adelante, +Y izquierda, +Z arriba)
  # =====================================================================
  chassis_z = 0.048  # Eje ruedas queda en 0.048 - 0.0155 = 0.0325 m (radio efectivo)
  car = world.add_body(name='car_chassis', pos=[0.0, 0.0, chassis_z])
  car.add_freejoint(name='car_free')

  # Camara de seguimiento suave centrada en el coche.
  car.add_camera(
      name='follow_cam',
      mode=mujoco.mjtCamLight.mjCAMLIGHT_TRACKCOM,
      pos=[-0.52, -0.34, 0.34],
      xyaxes=[0.55, -0.83, 0.0, 0.35, 0.23, 0.91],
      fovy=48.0,
  )

  # Caja de colision principal del chasis y parachoques (despejada entre ruedas).
  car.add_geom(
      car_col_cls,
      name='chassis_col',
      type=mujoco.mjtGeom.mjGEOM_BOX,
      pos=[0.0, 0.0, 0.012],
      size=[0.118, 0.056, 0.025],
      mass=0.82,
  )

  # Placa inferior acrilica.
  car.add_geom(
      vis_cls,
      name='plate_bottom',
      type=mujoco.mjtGeom.mjGEOM_BOX,
      pos=[0.0, 0.0, -0.010],
      size=[0.118, 0.068, 0.002],
      material=acrylic_mat.name,
  )
  # Placa superior acrilica.
  car.add_geom(
      vis_cls,
      name='plate_top',
      type=mujoco.mjtGeom.mjGEOM_BOX,
      pos=[-0.004, 0.0, 0.024],
      size=[0.112, 0.064, 0.002],
      material=acrylic_mat.name,
  )
  # Postes separadores de laton.
  for sx in (-0.095, -0.02, 0.055, 0.098):
    for sy in (-0.052, 0.052):
      car.add_geom(
          vis_cls,
          name=f'standoff_{sx:.3f}_{sy:.3f}',
          type=mujoco.mjtGeom.mjGEOM_CYLINDER,
          pos=[sx, sy, 0.007],
          size=[0.0032, 0.016],
          material=brass_mat.name,
      )

  # Portapilas 18650 y electronica entre placas.
  car.add_geom(
      vis_cls,
      name='battery_pack',
      type=mujoco.mjtGeom.mjGEOM_BOX,
      pos=[-0.032, 0.0, 0.008],
      size=[0.042, 0.038, 0.011],
      rgba=[0.10, 0.10, 0.12, 1.0],
  )
  car.add_geom(
      vis_cls,
      name='pcb_main',
      type=mujoco.mjtGeom.mjGEOM_BOX,
      pos=[-0.008, 0.0, 0.028],
      size=[0.052, 0.042, 0.002],
      material=pcb_mat.name,
  )

  # 4 motores TT amarillos y ejes.
  for wname, wx, wy, _ in WHEEL_DEFS:
    inner_y = wy * 0.62
    car.add_geom(
        vis_cls,
        name=f'motor_tt_{wname}',
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=[wx, inner_y, -0.0155],
        size=[0.028, 0.011, 0.011],
        material=motor_yellow.name,
    )

  # Pantalla ST7735 trasera orientada hacia atras-arriba.
  car.add_geom(
      vis_cls,
      name='tft_bezel',
      type=mujoco.mjtGeom.mjGEOM_BOX,
      pos=[-0.094, 0.0, 0.040],
      quat=[0.9239, 0.0, -0.3827, 0.0],
      size=[0.003, 0.024, 0.018],
      material=pcb_blue.name,
  )
  car.add_geom(
      vis_cls,
      name='tft_screen',
      type=mujoco.mjtGeom.mjGEOM_BOX,
      pos=[-0.096, 0.0, 0.041],
      quat=[0.9239, 0.0, -0.3827, 0.0],
      size=[0.0015, 0.019, 0.014],
      rgba=[0.08, 0.65, 0.85, 1.0],
  )

  # LEDs traseros rojos de freno (se animan en PreStepEvent).
  car.add_geom(
      vis_cls,
      name='rear_led_left_vis',
      type=mujoco.mjtGeom.mjGEOM_BOX,
      pos=[-0.120, 0.036, 0.012],
      size=[0.003, 0.009, 0.005],
      rgba=[1.0, 0.12, 0.12, 1.0],
  )
  car.add_geom(
      vis_cls,
      name='rear_led_right_vis',
      type=mujoco.mjtGeom.mjGEOM_BOX,
      pos=[-0.120, -0.036, 0.012],
      size=[0.003, 0.009, 0.005],
      rgba=[1.0, 0.12, 0.12, 1.0],
  )

  # Modulo Ultrasonico frontal HC-SR04.
  car.add_geom(
      vis_cls,
      name='hcsr04_pcb',
      type=mujoco.mjtGeom.mjGEOM_BOX,
      pos=[0.118, 0.0, 0.012],
      size=[0.003, 0.023, 0.011],
      material=pcb_blue.name,
  )
  for side, sy in (('l', 0.013), ('r', -0.013)):
    car.add_geom(
        vis_cls,
        name=f'hcsr04_cyl_{side}',
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        pos=[0.125, sy, 0.012],
        quat=[0.7071, 0.0, 0.7071, 0.0],
        size=[0.008, 0.006],
        material=alu_mat.name,
    )
  car.add_site(
      name='us_ray_site',
      pos=[0.129, 0.0, 0.012],
      quat=[0.7071, 0.0, 0.7071, 0.0],
      size=[0.004, 0.004, 0.004],
      rgba=[0.1, 0.9, 1.0, 0.8],
      group=2,
  )

  # 4 sensores IR de esquinas + testigos visuales.
  for _, sname, led_name, spos, sdir in IR_DEFS:
    car.add_geom(
        vis_cls,
        name=f'{sname}_pcb',
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=spos,
        size=[0.007, 0.007, 0.003],
        material=pcb_blue.name,
    )
    car.add_geom(
        vis_cls,
        name=led_name,
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        pos=[spos[0], spos[1], spos[2] + 0.005],
        size=[0.0035],
        rgba=[0.15, 0.30, 0.18, 1.0],
    )
    car.add_site(
        name=sname,
        pos=spos,
        quat=_quat_from_z_to_vec(*sdir),
        size=[0.003, 0.003, 0.003],
        rgba=[1.0, 0.4, 0.1, 0.7],
        group=2,
    )

  # =====================================================================
  # TORRETA PAN / TILT 2-DOF + CAMARA ESP32-CAM + LASER
  # =====================================================================
  pan_base = car.add_body(name='turret_pan_body', pos=[0.072, 0.0, 0.032])
  pan_jnt = pan_base.add_joint(
      name='pan_joint',
      type=mujoco.mjtJoint.mjJNT_HINGE,
      axis=[0.0, 0.0, 1.0],
      range=[-65.0, 65.0],
      damping=0.04,
      armature=0.002,
  )
  pan_base.add_geom(
      vis_cls,
      name='pan_servo_housing',
      type=mujoco.mjtGeom.mjGEOM_BOX,
      pos=[0.0, 0.0, 0.012],
      size=[0.015, 0.015, 0.014],
      rgba=[0.10, 0.35, 0.85, 1.0],
      mass=0.035,
  )

  tilt_head = pan_base.add_body(name='turret_tilt_head', pos=[0.0, 0.0, 0.036])
  tilt_jnt = tilt_head.add_joint(
      name='tilt_joint',
      type=mujoco.mjtJoint.mjJNT_HINGE,
      axis=[0.0, 1.0, 0.0],
      range=[-45.0, 45.0],
      damping=0.04,
      armature=0.002,
  )
  tilt_head.add_geom(
      vis_cls,
      name='cam_pcb',
      type=mujoco.mjtGeom.mjGEOM_BOX,
      pos=[0.008, 0.0, 0.008],
      size=[0.004, 0.020, 0.014],
      material=pcb_mat.name,
      mass=0.030,
  )
  tilt_head.add_geom(
      vis_cls,
      name='cam_lens_barrel',
      type=mujoco.mjtGeom.mjGEOM_CYLINDER,
      pos=[0.016, 0.0, 0.008],
      quat=[0.7071, 0.0, 0.7071, 0.0],
      size=[0.0065, 0.006],
      rgba=[0.05, 0.05, 0.06, 1.0],
  )
  tilt_head.add_geom(
      vis_cls,
      name='cam_lens_glass',
      type=mujoco.mjtGeom.mjGEOM_CYLINDER,
      pos=[0.0225, 0.0, 0.008],
      quat=[0.7071, 0.0, 0.7071, 0.0],
      size=[0.0042, 0.001],
      rgba=[0.12, 0.35, 0.72, 1.0],
  )
  tilt_head.add_geom(
      vis_cls,
      name='laser_diode_housing',
      type=mujoco.mjtGeom.mjGEOM_CYLINDER,
      pos=[0.015, 0.0, 0.027],
      quat=[0.7071, 0.0, 0.7071, 0.0],
      size=[0.0038, 0.007],
      material=brass_mat.name,
  )
  tilt_head.add_site(
      name='laser_site',
      pos=[0.023, 0.0, 0.027],
      quat=[0.7071, 0.0, 0.7071, 0.0],
      size=[0.0025, 0.0025, 0.0025],
      rgba=[1.0, 0.1, 0.1, 0.9],
      group=2,
  )
  # Haz laser visual dinamico (escalado / ocultado por alfa segun mj_ray).
  tilt_head.add_geom(
      vis_cls,
      name='laser_beam_vis',
      type=mujoco.mjtGeom.mjGEOM_CYLINDER,
      pos=[0.52, 0.0, 0.027],
      quat=[0.7071, 0.0, 0.7071, 0.0],
      size=[0.0014, 0.50],
      rgba=[1.0, 0.08, 0.08, 0.0],
  )
  # Camara virtual ESP32-CAM mirando hacia +X del cabezal (OV2640 HQVGA 240x176).
  tilt_head.add_camera(
      name='esp32_cam',
      pos=[0.026, 0.0, 0.008],
      xyaxes=[0.0, -1.0, 0.0, 0.0, 0.0, 1.0],
      fovy=44.0,
  )

  # Actuadores posicionales para Pan y Tilt.
  pan_act = spec.add_actuator(
      name='pan_servo',
      trntype=mujoco.mjtTrn.mjTRN_JOINT,
      target=pan_jnt.name,
  )
  pan_act.dyntype = mujoco.mjtDyn.mjDYN_NONE
  pan_act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
  pan_act.biastype = mujoco.mjtBias.mjBIAS_AFFINE
  pan_act.gainprm[0] = 3.5
  pan_act.biasprm = [0.0, -3.5, -0.15] + [0.0] * 7
  pan_act.ctrlrange = [-1.15, 1.15]

  tilt_act = spec.add_actuator(
      name='tilt_servo',
      trntype=mujoco.mjtTrn.mjTRN_JOINT,
      target=tilt_jnt.name,
  )
  tilt_act.dyntype = mujoco.mjtDyn.mjDYN_NONE
  tilt_act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
  tilt_act.biastype = mujoco.mjtBias.mjBIAS_AFFINE
  tilt_act.gainprm[0] = 3.5
  tilt_act.biasprm = [0.0, -3.5, -0.15] + [0.0] * 7
  tilt_act.ctrlrange = [-0.85, 0.85]

  # =====================================================================
  # 4 RUEDAS MECANUM FISICAS CON 12 RODILLOS PASIVOS CADA UNA
  # =====================================================================
  n_rollers = 12
  r_ring = 0.0255
  r_roller = 0.0070
  half_roller_len = 0.0075
  z_wheel_local = -(chassis_z - (r_ring + r_roller))

  for wname, wx, wy, roller_s in WHEEL_DEFS:
    wbody = car.add_body(
        name=f'{wname}_wheel', pos=[wx, wy, z_wheel_local]
    )
    wjnt = wbody.add_joint(
        name=f'{wname}_wheel_joint',
        type=mujoco.mjtJoint.mjJNT_HINGE,
        axis=[0.0, 1.0, 0.0],
        damping=0.002,
        armature=0.0008,
    )
    # Masa y geometria visual del rin de la rueda.
    wbody.add_geom(
        vis_cls,
        name=f'{wname}_hub_vis',
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        quat=[0.7071, 0.7071, 0.0, 0.0],
        size=[0.022, 0.013],
        material=wheel_rim_mat.name,
        mass=0.075,
    )
    for k in range(n_rollers):
      theta = 2.0 * math.pi * k / n_rollers
      sin_t = math.sin(theta)
      cos_t = math.cos(theta)
      # theta=0 esta abajo (0, 0, -r_ring). Eje del rodillo u_k:
      # u_k = (cos(theta), roller_s, -sin(theta)) / sqrt(2)
      ux = cos_t / math.sqrt(2.0)
      uy = roller_s / math.sqrt(2.0)
      uz = -sin_t / math.sqrt(2.0)
      r_pos = [-r_ring * sin_t, 0.0, -r_ring * cos_t]

      rbody = wbody.add_body(name=f'{wname}_roller_{k}', pos=r_pos)
      rbody.add_joint(
          name=f'{wname}_roller_jnt_{k}',
          type=mujoco.mjtJoint.mjJNT_HINGE,
          axis=[ux, uy, uz],
          damping=1e-5,
          armature=5e-6,
      )
      fromto = [
          -half_roller_len * ux,
          -half_roller_len * uy,
          -half_roller_len * uz,
          half_roller_len * ux,
          half_roller_len * uy,
          half_roller_len * uz,
      ]
      rbody.add_geom(
          vis_cls,
          name=f'{wname}_roller_vis_{k}',
          type=mujoco.mjtGeom.mjGEOM_CAPSULE,
          fromto=fromto,
          size=[r_roller],
          material=rubber_mat.name,
      )
      rbody.add_geom(
          roller_col_cls,
          name=f'{wname}_roller_col_{k}',
          type=mujoco.mjtGeom.mjGEOM_CAPSULE,
          fromto=fromto,
          size=[r_roller],
          mass=0.004,
      )

    # Actuador de velocidad (servomotor angular / freno cuando ctrl = 0).
    wact = spec.add_actuator(
        name=f'{wname}_motor',
        trntype=mujoco.mjtTrn.mjTRN_JOINT,
        target=wjnt.name,
    )
    wact.dyntype = mujoco.mjtDyn.mjDYN_NONE
    wact.gaintype = mujoco.mjtGain.mjGAIN_FIXED
    wact.biastype = mujoco.mjtBias.mjBIAS_AFFINE
    wact.gainprm[0] = wheel_kp
    wact.biasprm = [0.0, 0.0, -wheel_kp] + [0.0] * 7
    wact.ctrlrange = [-28.0, 28.0]
    wact.forcerange = [-0.45, 0.45]

  return spec


class SimPlugin(plugin.SimPlugin):
  """Plugin de simulacion del Freenove ESP32 4WD Mecanum CamCar."""

  arena_half_size_m: float = 1.65
  spawn_obstacles: bool = True
  max_wheel_rad_s: float = 18.0
  ir_threshold_cm: float = 13.0
  wheel_kp: float = 0.06

  # Ultima intencion continua proveniente del panel Viewer.
  ui_mode: str = 'manual'
  ui_ly: float = 0.0
  ui_lx: float = 0.0
  ui_rx: float = 0.0
  ui_pan_deg: float = 90.0
  ui_tilt_deg: float = 90.0
  ui_laser_on: bool = False
  ui_bridge_enabled: bool = True
  ui_cmd_seq: int = -1

  def __init__(self, *args, **kwargs) -> None:
    super().__init__(*args, **kwargs)
    self.firmware = fw.Esp32CarFirmware()
    self.bridge = fw.Esp32NetworkBridge()
    self.spec: mujoco.MjSpec | None = None
    self._renderer: mujoco.Renderer | None = None
    self._telemetry_step = 0
    self._video_step = 0
    self._cached_ids: dict[str, int] = {}
    self._last_us_cm = -1.0
    self._last_ir = [False, False, False, False]
    self._reset_pose_requested = True

  def _close_renderer(self) -> None:
    if self._renderer is not None:
      try:
        self._renderer.close()
      except Exception:  # pylint: disable=broad-except
        pass
      self._renderer = None

  def _cache_model_ids(self, model: mujoco.MjModel) -> None:
    names = {
        'chassis_body': (mujoco.mjtObj.mjOBJ_BODY, 'car_chassis'),
        'us_site': (mujoco.mjtObj.mjOBJ_SITE, 'us_ray_site'),
        'laser_site': (mujoco.mjtObj.mjOBJ_SITE, 'laser_site'),
        'laser_beam_geom': (mujoco.mjtObj.mjOBJ_GEOM, 'laser_beam_vis'),
        'rear_led_l': (mujoco.mjtObj.mjOBJ_GEOM, 'rear_led_left_vis'),
        'rear_led_r': (mujoco.mjtObj.mjOBJ_GEOM, 'rear_led_right_vis'),
        'pan_act': (mujoco.mjtObj.mjOBJ_ACTUATOR, 'pan_servo'),
        'tilt_act': (mujoco.mjtObj.mjOBJ_ACTUATOR, 'tilt_servo'),
        'pan_jnt': (mujoco.mjtObj.mjOBJ_JOINT, 'pan_joint'),
        'tilt_jnt': (mujoco.mjtObj.mjOBJ_JOINT, 'tilt_joint'),
        'fl_act': (mujoco.mjtObj.mjOBJ_ACTUATOR, 'fl_motor'),
        'bl_act': (mujoco.mjtObj.mjOBJ_ACTUATOR, 'bl_motor'),
        'fr_act': (mujoco.mjtObj.mjOBJ_ACTUATOR, 'fr_motor'),
        'br_act': (mujoco.mjtObj.mjOBJ_ACTUATOR, 'br_motor'),
    }
    for idx, sname, led_name, _, _ in IR_DEFS:
      names[f'ir{idx}_site'] = (mujoco.mjtObj.mjOBJ_SITE, sname)
      names[f'ir{idx}_led'] = (mujoco.mjtObj.mjOBJ_GEOM, led_name)
    self._cached_ids = {
        k: mujoco.mj_name2id(model, otype, oname)
        for k, (otype, oname) in names.items()
    }

  def _cast_site_ray(
      self,
      model: mujoco.MjModel,
      data: mujoco.MjData,
      site_id: int,
      max_range_m: float,
  ) -> float:
    if site_id < 0:
      return -1.0
    pnt = data.site_xpos[site_id]
    # El eje +Z del site en mat (fila/col 2 en MuJoCo row-major 3x3: indices 2, 5, 8)
    smat = data.site_xmat[site_id]
    vec = np.array([smat[2], smat[5], smat[8]], dtype=np.float64)
    nrm = float(np.linalg.norm(vec))
    if nrm < 1e-9:
      return -1.0
    vec /= nrm
    geomid = np.array([-1], dtype=np.int32)
    dist = float(
        mujoco.mj_ray(
            model,
            data,
            pnt,
            vec,
            _ENV_RAY_GROUP,
            1,
            self._cached_ids.get('chassis_body', -1),
            geomid,
        )
    )
    if dist < 0.0 or dist > max_range_m:
      return -1.0
    return dist

  def _cast_site_fan(
      self,
      model: mujoco.MjModel,
      data: mujoco.MjData,
      site_id: int,
      yaw_angles_deg: tuple[float, ...],
      min_range_m: float,
      max_range_m: float,
  ) -> float:
    """Lanza un abanico horizontal de rayos alrededor del eje +Z del site."""
    if site_id < 0:
      return -1.0
    pnt = data.site_xpos[site_id]
    smat = data.site_xmat[site_id]
    fwd = np.array([smat[2], smat[5], smat[8]], dtype=np.float64)
    fwd_nrm = float(np.linalg.norm(fwd))
    if fwd_nrm < 1e-9:
      return -1.0
    fwd /= fwd_nrm

    cid = self._cached_ids.get('chassis_body', -1)
    if cid >= 0:
      cmat = data.xmat[cid]
      up = np.array([cmat[2], cmat[5], cmat[8]], dtype=np.float64)
      up_nrm = float(np.linalg.norm(up))
      up = up / up_nrm if up_nrm > 1e-9 else np.array([0.0, 0.0, 1.0])
    else:
      up = np.array([0.0, 0.0, 1.0], dtype=np.float64)

    side = np.cross(up, fwd)
    side_nrm = float(np.linalg.norm(side))
    if side_nrm < 1e-6:
      side = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    else:
      side /= side_nrm

    geomid = np.array([-1], dtype=np.int32)
    best = -1.0
    for deg in yaw_angles_deg:
      rad = math.radians(deg)
      vec = math.cos(rad) * fwd + math.sin(rad) * side
      vnrm = float(np.linalg.norm(vec))
      if vnrm < 1e-9:
        continue
      vec /= vnrm
      dist = float(
          mujoco.mj_ray(
              model,
              data,
              pnt,
              vec,
              _ENV_RAY_GROUP,
              1,
              cid,
              geomid,
          )
      )
      if min_range_m <= dist <= max_range_m:
        if best < 0.0 or dist < best:
          best = dist
    return best

  def _read_sensors(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
    # Cono acustico HC-SR04 (~24 deg total: +-12 deg) y ventana 1.8 cm .. 400 cm.
    us_m = self._cast_site_fan(
        model,
        data,
        self._cached_ids.get('us_site', -1),
        (-12.0, -6.0, 0.0, 6.0, 12.0),
        0.018,
        4.00,
    )
    self._last_us_cm = us_m * 100.0 if us_m >= 0.0 else -1.0

    # Cono optico de sensores IR de esquinas (+-8 deg).
    ir_thresh_m = self.ir_threshold_cm / 100.0
    ir_vals: list[bool] = []
    for idx in (1, 2, 3, 4):
      d_m = self._cast_site_fan(
          model,
          data,
          self._cached_ids.get(f'ir{idx}_site', -1),
          (-8.0, 0.0, 8.0),
          0.0,
          ir_thresh_m,
      )
      ir_vals.append(d_m >= 0.0)
    self._last_ir = ir_vals

  def _update_visuals_and_actuators(
      self, model: mujoco.MjModel, data: mujoco.MjData
  ) -> None:
    # 1. Ruedas Mecanum (Short Brake STBY=HIGH vs High-Z Coast STBY=LOW).
    w_fl, w_bl, w_fr, w_br = self.firmware.wheel_target_rad_s(
        self.max_wheel_rad_s
    )
    is_coasting = self.firmware.coasting
    kp = self.wheel_kp
    for key, val in (
        ('fl_act', w_fl),
        ('bl_act', w_bl),
        ('fr_act', w_fr),
        ('br_act', w_br),
    ):
      aid = self._cached_ids.get(key, -1)
      if aid >= 0:
        if is_coasting:
          model.actuator_gainprm[aid, 0] = 0.0
          model.actuator_biasprm[aid, 2] = 0.0
          data.ctrl[aid] = 0.0
        else:
          model.actuator_gainprm[aid, 0] = kp
          model.actuator_biasprm[aid, 2] = -kp
          data.ctrl[aid] = val

    # 2. Torreta Pan/Tilt.
    pan_rad = math.radians(self.firmware.current_pan - 90.0)
    tilt_rad = math.radians(self.firmware.current_tilt - 90.0)
    pan_aid = self._cached_ids.get('pan_act', -1)
    tilt_aid = self._cached_ids.get('tilt_act', -1)
    if pan_aid >= 0:
      data.ctrl[pan_aid] = pan_rad
    if tilt_aid >= 0:
      data.ctrl[tilt_aid] = tilt_rad

    # 3. LEDs traseros de freno (encendidos en Short Brake y en Coast).
    led_rgba = (
        [1.0, 0.10, 0.10, 1.0]
        if (self.firmware.braking or self.firmware.coasting)
        else [0.28, 0.04, 0.04, 1.0]
    )
    for key in ('rear_led_l', 'rear_led_r'):
      gid = self._cached_ids.get(key, -1)
      if gid >= 0:
        model.geom_rgba[gid] = led_rgba

    # 4. LEDs indicadores de sensores IR.
    for idx in (1, 2, 3, 4):
      gid = self._cached_ids.get(f'ir{idx}_led', -1)
      if gid >= 0:
        model.geom_rgba[gid] = (
            [0.2, 1.0, 0.35, 1.0]
            if self._last_ir[idx - 1]
            else [0.12, 0.25, 0.15, 1.0]
        )

    # 5. Haz laser dinamico con longitud exacta hasta el primer obstaculo.
    beam_gid = self._cached_ids.get('laser_beam_geom', -1)
    if beam_gid >= 0:
      if self.firmware.laser_on:
        d_laser = self._cast_site_ray(
            model, data, self._cached_ids.get('laser_site', -1), 3.0
        )
        beam_len = d_laser if d_laser > 0.02 else 2.2
        half_len = beam_len * 0.5
        model.geom_size[beam_gid] = [0.0014, half_len, 0.0]
        model.geom_pos[beam_gid] = [0.023 + half_len, 0.0, 0.027]
        model.geom_rgba[beam_gid] = [1.0, 0.05, 0.05, 0.85]
      else:
        model.geom_rgba[beam_gid] = [1.0, 0.05, 0.05, 0.0]

  @messages.handler
  def on_load_model(self, event: messages.LoadModelEvent) -> bool:
    if event.path:
      return False
    self.spec = build_esp32_camcar_spec(
        arena_half_size_m=self.arena_half_size_m,
        spawn_obstacles=self.spawn_obstacles,
    )
    self.handle.set_model(self.spec)
    return True

  @messages.handler
  def on_post_model(self, event: messages.PostModelEvent) -> None:
    self._close_renderer()
    self._cache_model_ids(event.model)
    if self._reset_pose_requested:
      self._reset_pose_requested = False
      mujoco.mj_resetData(event.model, event.data)
      self.firmware.reset(float(event.data.time))
    mujoco.mj_forward(event.model, event.data)
    self.bridge.set_enabled(self.ui_bridge_enabled)

  @messages.handler
  def on_hot_reload(self, event: messages.HotReloadEvent) -> None:
    if self in event.plugins:
      self._close_renderer()
      self.bridge.close()
      self.firmware = fw.Esp32CarFirmware()
      self.bridge = fw.Esp32NetworkBridge()
      self.handle.reload()

  @messages.handler
  def on_exit(self, event: messages.ExitEvent) -> None:
    del event
    self._close_renderer()
    self.bridge.close()

  @messages.handler
  def on_bake(self, event: user_messages.BakeEvent) -> None:
    del event
    if self.spec is None:
      return
    with open(EXPORT_XML, 'w', encoding='utf-8') as f:
      f.write(self.spec.to_xml())

  @messages.handler
  def on_scene_params(self, params: user_messages.SceneParams) -> None:
    rebuild = (
        abs(params.arena_half_size_m - self.arena_half_size_m) > 1e-4
        or params.spawn_obstacles != self.spawn_obstacles
    )
    self.arena_half_size_m = params.arena_half_size_m
    self.spawn_obstacles = params.spawn_obstacles
    self.max_wheel_rad_s = params.max_wheel_rad_s
    self.ir_threshold_cm = params.ir_threshold_cm
    if rebuild:
      self._reset_pose_requested = True
      self.handle.reload()

  @messages.handler
  def on_car_command(self, cmd: user_messages.CarCommand) -> None:
    sim_time = 0.0
    if self.firmware is not None:
      sim_time = self.firmware._next_50hz_time  # pylint: disable=protected-access

    if cmd.bridge_enabled != self.ui_bridge_enabled:
      self.ui_bridge_enabled = cmd.bridge_enabled
      self.bridge.set_enabled(self.ui_bridge_enabled)

    if cmd.mode != self.ui_mode:
      self.ui_mode = cmd.mode
      self.firmware.set_mode(cmd.mode, sim_time)

    if cmd.pan_ui_deg != self.ui_pan_deg:
      self.ui_pan_deg = cmd.pan_ui_deg
      self.firmware.set_pan_angle(180.0 - cmd.pan_ui_deg)

    if cmd.tilt_ui_deg != self.ui_tilt_deg:
      self.ui_tilt_deg = cmd.tilt_ui_deg
      self.firmware.set_tilt_angle(cmd.tilt_ui_deg)

    if cmd.laser_on != self.ui_laser_on:
      self.ui_laser_on = cmd.laser_on
      self.firmware.laser_on = cmd.laser_on

    if cmd.command_seq != self.ui_cmd_seq:
      self.ui_cmd_seq = cmd.command_seq
      self.ui_ly = cmd.ly
      self.ui_lx = cmd.lx
      self.ui_rx = cmd.rx
      if self.firmware.mode == 'manual':
        if abs(cmd.ly) > 1.0 or abs(cmd.lx) > 1.0 or abs(cmd.rx) > 1.0:
          fl, bl, fr, br = fw.compute_joystick_mecanum_pwms(
              cmd.ly, cmd.lx, cmd.rx
          )
          self.firmware.drive_direct_raw(fl, bl, fr, br, sim_time)
        elif not self.bridge.tcp_connected:
          self.firmware.brake_all_motors()

  @messages.handler
  def on_drive_pulse(self, ev: user_messages.CarDrivePulseEvent) -> None:
    sim_time = self.firmware._next_50hz_time  # pylint: disable=protected-access
    if ev.cmd_type == 'mecanum':
      self.firmware.set_mode('manual', sim_time)
      self.ui_mode = 'manual'
      self.firmware.drive_mecanum(
          ev.angle, ev.speed, ev.rot, ev.rot_speed, sim_time
      )
    elif ev.cmd_type == 'stop':
      self.ui_ly = self.ui_lx = self.ui_rx = 0.0
      self.firmware.stop_all_motors()
      self.ui_mode = 'manual'
    elif ev.cmd_type == 'brake':
      self.ui_ly = self.ui_lx = self.ui_rx = 0.0
      self.firmware.brake_all_motors()
    elif ev.cmd_type == 'center_turret':
      self.ui_pan_deg = 90.0
      self.ui_tilt_deg = 90.0
      self.firmware.center_servos()
    elif ev.cmd_type == 'toggle_laser':
      self.firmware.laser_on = not self.firmware.laser_on
      self.ui_laser_on = self.firmware.laser_on
    elif ev.cmd_type == 'toggle_melody':
      self.firmware.toggle_melody(sim_time)
    elif ev.cmd_type == 'reset_pose':
      self.ui_ly = self.ui_lx = self.ui_rx = 0.0
      self.ui_mode = 'manual'
      self._reset_pose_requested = True
      self.handle.dispatch(messages.ResetEvent())

  @messages.handler
  def on_reset(self, event: messages.ResetEvent) -> None:
    del event
    self._reset_pose_requested = False
    self.firmware.reset(0.0)
    self.ui_pan_deg = 90.0
    self.ui_tilt_deg = 90.0
    self.ui_laser_on = False

  @messages.handler
  def on_pre_step(self, event: messages.PreStepEvent) -> None:
    model = event.model
    data = event.data
    sim_time = float(data.time)

    # Modo crucero si el usuario mantiene sliders continuos en ImGui.
    if (
        self.firmware.mode == 'manual'
        and (abs(self.ui_ly) > 1.0 or abs(self.ui_lx) > 1.0 or abs(self.ui_rx) > 1.0)
    ):
      self.firmware.last_manual_drive_time = sim_time

    # Atender comandos TCP / handshakes UDP de Main.py sin bloquear.
    self.bridge.poll(self.firmware, sim_time)

    # Medir rayos HC-SR04 e IR1..IR4.
    self._read_sensors(model, data)

    # Avanzar firmware (50 Hz rampa/servos + 60 ms piloto automatico).
    self.firmware.step(
        sim_time,
        self._last_us_cm,
        self._last_ir[0],
        self._last_ir[1],
        self._last_ir[2],
        self._last_ir[3],
    )

    # Aplicar actuadores y visuales dinamicos.
    self._update_visuals_and_actuators(model, data)

  @messages.handler
  def on_post_step(self, event: messages.PostStepEvent) -> None:
    model = event.model
    data = event.data

    # Renderizado FPV offscreen solo si Main.py solicito video UDP ("START").
    self._video_step += 1
    if (
        self._video_step % 25 == 0
        and self.bridge.should_capture_frame(self.firmware)
    ):
      if self._renderer is None:
        try:
          self._renderer = mujoco.Renderer(model, height=176, width=240)
        except Exception:  # pylint: disable=broad-except
          self._renderer = None
      if self._renderer is not None:
        try:
          self._renderer.update_scene(data, camera='esp32_cam')
          rgb = self._renderer.render()
          self.bridge.submit_frame(rgb)
        except Exception:  # pylint: disable=broad-except
          pass

    # Enviar telemetria al ViewerPlugin a 100 Hz (cada 5 pasos de 2 ms).
    self._telemetry_step += 1
    if self._telemetry_step % 5 != 0:
      return

    cid = self._cached_ids.get('chassis_body', -1)
    pos_x = pos_y = yaw_deg = v_fwd = v_left = yaw_rate_deg = 0.0
    if cid >= 0:
      pos = data.xpos[cid]
      pos_x, pos_y = float(pos[0]), float(pos[1])
      xmat = data.xmat[cid]
      # Vector +X local del chasis en XY global: (xmat[0], xmat[3]).
      yaw_rad = math.atan2(float(xmat[3]), float(xmat[0]))
      yaw_deg = math.degrees(yaw_rad)
      cvel = data.cvel[cid]  # [wx, wy, wz, vx, vy, vz] en marco mundo alineado al CoM
      cos_y = math.cos(yaw_rad)
      sin_y = math.sin(yaw_rad)
      vx_w, vy_w = float(cvel[3]), float(cvel[4])
      v_fwd = vx_w * cos_y + vy_w * sin_y
      v_left = -vx_w * sin_y + vy_w * cos_y
      yaw_rate_deg = math.degrees(float(cvel[2]))

    pan_jid = self._cached_ids.get('pan_jnt', -1)
    tilt_jid = self._cached_ids.get('tilt_jnt', -1)
    act_pan_ui = (
        90.0 - math.degrees(float(data.qpos[model.jnt_qposadr[pan_jid]]))
        if pan_jid >= 0
        else float(180 - self.firmware.current_pan)
    )
    act_tilt_ui = (
        90.0 - math.degrees(float(data.qpos[model.jnt_qposadr[tilt_jid]]))
        if tilt_jid >= 0
        else self.firmware.act_tilt_ui_deg
    )

    self.handle.send_to_viewer(
        user_messages.CarTelemetry(
            sim_time=float(data.time),
            mode=self.firmware.mode,
            auto_state_name=self.firmware.auto_state_name,
            stuck_counter=self.firmware.stuck_counter,
            obstacle_found=self.firmware.obstacle_found,
            obstacle_alert=self.firmware.obstacle_alert,
            pwm_fl=self.firmware.current_fl,
            pwm_bl=self.firmware.current_bl,
            pwm_fr=self.firmware.current_fr,
            pwm_br=self.firmware.current_br,
            braking=self.firmware.braking,
            coasting=self.firmware.coasting,
            pan_ui_deg=self.firmware.pan_ui_deg,
            tilt_ui_deg=self.firmware.tilt_ui_deg,
            act_pan_deg=act_pan_ui,
            act_tilt_deg=act_tilt_ui,
            laser_on=self.firmware.laser_on,
            melody_on=self.firmware.melody_on,
            us_distance_cm=self._last_us_cm,
            ir1_br=self._last_ir[0],
            ir2_fr=self._last_ir[1],
            ir3_fl=self._last_ir[2],
            ir4_bl=self._last_ir[3],
            pos_x_m=pos_x,
            pos_y_m=pos_y,
            yaw_deg=yaw_deg,
            vel_forward_m_s=v_fwd,
            vel_left_m_s=v_left,
            yaw_rate_deg_s=yaw_rate_deg,
            bridge_enabled=self.bridge.enabled,
            bridge_status=self.bridge.status,
            tcp_connected=self.bridge.tcp_connected,
            udp_streaming=self.bridge.udp_streaming,
            udp_fps=self.bridge.udp_fps,
        )
    )
