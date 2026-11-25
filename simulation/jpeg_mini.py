"""Codificador JPEG JFIF Baseline compacto usando NumPy puro (o PIL si existe).

Genera cuadros JPEG estandar (FFD8 ... FFD9) compatibles con Video.py y
cv2.imdecode de Main.py sin requerir OpenCV/Pillow dentro del venv de MuJoCo Studio.
"""

from __future__ import annotations

import io
import math
import struct
import numpy as np

try:
  from PIL import Image as _PIL_Image  # type: ignore
except Exception:  # pylint: disable=broad-except
  _PIL_Image = None

# Matrices y tablas estandar JPEG Annex K.
_STD_LUMA_Q = np.array([
    [16, 11, 10, 16, 24, 40, 51, 61],
    [12, 12, 14, 19, 26, 58, 60, 55],
    [14, 13, 16, 24, 40, 57, 69, 56],
    [14, 17, 22, 29, 51, 87, 80, 62],
    [18, 22, 37, 56, 68, 109, 103, 77],
    [24, 35, 55, 64, 81, 104, 113, 92],
    [49, 64, 78, 87, 103, 121, 120, 101],
    [72, 92, 95, 98, 112, 100, 103, 99],
], dtype=np.float32)

_STD_CHROMA_Q = np.array([
    [17, 18, 24, 47, 99, 99, 99, 99],
    [18, 21, 26, 66, 99, 99, 99, 99],
    [24, 26, 56, 99, 99, 99, 99, 99],
    [47, 66, 99, 99, 99, 99, 99, 99],
    [99, 99, 99, 99, 99, 99, 99, 99],
    [99, 99, 99, 99, 99, 99, 99, 99],
    [99, 99, 99, 99, 99, 99, 99, 99],
    [99, 99, 99, 99, 99, 99, 99, 99],
], dtype=np.float32)

_ZIGZAG = np.array([
    0, 1, 8, 16, 9, 2, 3, 10,
    17, 24, 32, 25, 18, 11, 4, 5,
    12, 19, 26, 33, 40, 48, 41, 34,
    27, 20, 13, 6, 7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36,
    29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46,
    53, 60, 61, 54, 47, 55, 62, 63,
], dtype=np.int32)

# Tablas Huffman estandar JPEG (Annex K.3).
_DC_LUMA_BITS = bytes([0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0])
_DC_LUMA_VALS = bytes([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11])

_DC_CHROMA_BITS = bytes([0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0])
_DC_CHROMA_VALS = bytes([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11])

_AC_LUMA_BITS = bytes([0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7D])
_AC_LUMA_VALS = bytes([
    0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12,
    0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
    0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xA1, 0x08,
    0x23, 0x42, 0xB1, 0xC1, 0x15, 0x52, 0xD1, 0xF0,
    0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0A, 0x16,
    0x17, 0x18, 0x19, 0x1A, 0x25, 0x26, 0x27, 0x28,
    0x29, 0x2A, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
    0x3A, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
    0x4A, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
    0x5A, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
    0x6A, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
    0x7A, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
    0x8A, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
    0x99, 0x9A, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7,
    0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6,
    0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3, 0xC4, 0xC5,
    0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xD2, 0xD3, 0xD4,
    0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xE1, 0xE2,
    0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA,
    0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8,
    0xF9, 0xFA,
])

_AC_CHROMA_BITS = bytes([0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77])
_AC_CHROMA_VALS = bytes([
    0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21,
    0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
    0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
    0xA1, 0xB1, 0xC1, 0x09, 0x23, 0x33, 0x52, 0xF0,
    0x15, 0x62, 0x72, 0xD1, 0x0A, 0x16, 0x24, 0x34,
    0xE1, 0x25, 0xF1, 0x17, 0x18, 0x19, 0x1A, 0x26,
    0x27, 0x28, 0x29, 0x2A, 0x35, 0x36, 0x37, 0x38,
    0x39, 0x3A, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
    0x49, 0x4A, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
    0x59, 0x5A, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
    0x69, 0x6A, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
    0x79, 0x7A, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
    0x88, 0x89, 0x8A, 0x92, 0x93, 0x94, 0x95, 0x96,
    0x97, 0x98, 0x99, 0x9A, 0xA2, 0xA3, 0xA4, 0xA5,
    0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4,
    0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3,
    0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xD2,
    0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA,
    0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9,
    0xEA, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8,
    0xF9, 0xFA,
])


def _build_huff_lut(bits: bytes, vals: bytes) -> dict[int, tuple[int, int]]:
  lut: dict[int, tuple[int, int]] = {}
  code = 0
  idx = 0
  for bit_len in range(1, 17):
    count = bits[bit_len - 1]
    for _ in range(count):
      sym = vals[idx]
      lut[sym] = (code, bit_len)
      code += 1
      idx += 1
    code <<= 1
  return lut


_DC_LUMA_LUT = _build_huff_lut(_DC_LUMA_BITS, _DC_LUMA_VALS)
_DC_CHROMA_LUT = _build_huff_lut(_DC_CHROMA_BITS, _DC_CHROMA_VALS)
_AC_LUMA_LUT = _build_huff_lut(_AC_LUMA_BITS, _AC_LUMA_VALS)
_AC_CHROMA_LUT = _build_huff_lut(_AC_CHROMA_BITS, _AC_CHROMA_VALS)


def _build_dct_matrix() -> np.ndarray:
  c = np.zeros((8, 8), dtype=np.float32)
  for k in range(8):
    alpha = math.sqrt(1.0 / 8.0) if k == 0 else math.sqrt(2.0 / 8.0)
    for n in range(8):
      c[k, n] = alpha * math.cos((2 * n + 1) * k * math.pi / 16.0)
  return c


_DCT_M = _build_dct_matrix()
_DCT_MT = _DCT_M.T.copy()


def _scale_quant_table(base: np.ndarray, quality: int) -> np.ndarray:
  q = max(1, min(100, int(quality)))
  scale = 5000 // q if q < 50 else 200 - 2 * q
  qt = np.floor((base * scale + 50.0) / 100.0)
  return np.clip(qt, 1, 255).astype(np.int32)


def _vli_bits(val: int) -> tuple[int, int]:
  if val == 0:
    return 0, 0
  abs_v = -val if val < 0 else val
  nbits = abs_v.bit_length()
  code = val if val > 0 else ((1 << nbits) - 1 + val)
  return code, nbits


def _build_dht_segment(tc_th: int, bits: bytes, vals: bytes) -> bytes:
  payload = bytes([tc_th]) + bits + vals
  return b'\xff\xc4' + struct.pack('>H', len(payload) + 2) + payload


def encode_jpeg_rgb(rgb: np.ndarray, quality: int = 68) -> bytes:
  """Codifica un frame uint8 RGB (H, W, 3) a bytes JFIF JPEG."""
  if _PIL_Image is not None:
    buf = io.BytesIO()
    _PIL_Image.fromarray(rgb, mode='RGB').save(
        buf, format='JPEG', quality=int(quality)
    )
    return buf.getvalue()

  h, w, _ = rgb.shape
  pad_h = (8 - (h % 8)) % 8
  pad_w = (8 - (w % 8)) % 8
  if pad_h or pad_w:
    rgb_p = np.pad(rgb, ((0, pad_h), (0, pad_w), (0, 0)), mode='edge')
  else:
    rgb_p = rgb

  hp, wp, _ = rgb_p.shape
  rf = rgb_p[..., 0].astype(np.float32)
  gf = rgb_p[..., 1].astype(np.float32)
  bf = rgb_p[..., 2].astype(np.float32)

  # YCbCr centrado en -128..127 (level shift ya incluido).
  y_plane = 0.299 * rf + 0.587 * gf + 0.114 * bf - 128.0
  cb_plane = -0.168736 * rf - 0.331264 * gf + 0.5 * bf
  cr_plane = 0.5 * rf - 0.418688 * gf - 0.081312 * bf

  q_y = _scale_quant_table(_STD_LUMA_Q, quality)
  q_c = _scale_quant_table(_STD_CHROMA_Q, quality)

  def _quant_blocks(plane: np.ndarray, qmat: np.ndarray) -> np.ndarray:
    blocks = (
        plane.reshape(hp // 8, 8, wp // 8, 8)
        .transpose(0, 2, 1, 3)
        .reshape(-1, 8, 8)
    )
    dct = _DCT_M @ blocks @ _DCT_MT
    q = np.rint(dct / qmat).astype(np.int32).reshape(-1, 64)
    return q[:, _ZIGZAG]

  qy_blocks = _quant_blocks(y_plane, q_y)
  qcb_blocks = _quant_blocks(cb_plane, q_c)
  qcr_blocks = _quant_blocks(cr_plane, q_c)

  # Entropia JPEG entrelazada MCU 4:4:4 (1 bloque Y, 1 Cb, 1 Cr por MCU 8x8).
  out = bytearray()
  bit_buf = 0
  bit_cnt = 0

  def _put_bits(code: int, nbits: int) -> None:
    nonlocal bit_buf, bit_cnt
    if nbits == 0:
      return
    bit_buf = (bit_buf << nbits) | (code & ((1 << nbits) - 1))
    bit_cnt += nbits
    while bit_cnt >= 8:
      bit_cnt -= 8
      b = (bit_buf >> bit_cnt) & 0xFF
      out.append(b)
      if b == 0xFF:
        out.append(0x00)
    bit_buf &= (1 << bit_cnt) - 1

  dc_luma = _DC_LUMA_LUT
  dc_chroma = _DC_CHROMA_LUT
  ac_luma = _AC_LUMA_LUT
  ac_chroma = _AC_CHROMA_LUT

  prev_dc = [0, 0, 0]
  n_mcu = qy_blocks.shape[0]
  qy_list = qy_blocks.tolist()
  qcb_list = qcb_blocks.tolist()
  qcr_list = qcr_blocks.tolist()

  for idx in range(n_mcu):
    for comp_i, blk, dc_lut, ac_lut in (
        (0, qy_list[idx], dc_luma, ac_luma),
        (1, qcb_list[idx], dc_chroma, ac_chroma),
        (2, qcr_list[idx], dc_chroma, ac_chroma),
    ):
      dc_val = blk[0]
      diff = dc_val - prev_dc[comp_i]
      prev_dc[comp_i] = dc_val
      if diff == 0:
        c, n = dc_lut[0]
        _put_bits(c, n)
      else:
        vc, vn = _vli_bits(diff)
        c, n = dc_lut[vn]
        _put_bits(c, n)
        _put_bits(vc, vn)

      # AC: encontrar ultimo coeficiente no cero.
      last_nz = 0
      for k in range(63, 0, -1):
        if blk[k] != 0:
          last_nz = k
          break

      zero_run = 0
      for k in range(1, last_nz + 1):
        val = blk[k]
        if val == 0:
          zero_run += 1
          if zero_run == 16:
            c, n = ac_lut[0xF0]
            _put_bits(c, n)
            zero_run = 0
        else:
          vc, vn = _vli_bits(val)
          c, n = ac_lut[(zero_run << 4) | vn]
          _put_bits(c, n)
          _put_bits(vc, vn)
          zero_run = 0

      if last_nz < 63:
        c, n = ac_lut[0x00]  # EOB
        _put_bits(c, n)

  if bit_cnt > 0:
    _put_bits((1 << (8 - bit_cnt)) - 1, 8 - bit_cnt)

  # Construir cabeceras JFIF completas.
  qy_zz = bytes(int(v) for v in q_y.reshape(64)[_ZIGZAG])
  qc_zz = bytes(int(v) for v in q_c.reshape(64)[_ZIGZAG])

  header = bytearray()
  header += b'\xff\xd8'  # SOI
  # APP0 JFIF
  header += b'\xff\xe0' + struct.pack('>H', 16) + b'JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00'
  # DQT 0 (Y) y DQT 1 (C)
  header += b'\xff\xdb' + struct.pack('>H', 67) + b'\x00' + qy_zz
  header += b'\xff\xdb' + struct.pack('>H', 67) + b'\x01' + qc_zz
  # SOF0 Baseline 4:4:4
  sof_body = struct.pack('>BHHB', 8, h, w, 3) + bytes([
      1, 0x11, 0,
      2, 0x11, 1,
      3, 0x11, 1,
  ])
  header += b'\xff\xc0' + struct.pack('>H', len(sof_body) + 2) + sof_body
  # DHT
  header += _build_dht_segment(0x00, _DC_LUMA_BITS, _DC_LUMA_VALS)
  header += _build_dht_segment(0x10, _AC_LUMA_BITS, _AC_LUMA_VALS)
  header += _build_dht_segment(0x01, _DC_CHROMA_BITS, _DC_CHROMA_VALS)
  header += _build_dht_segment(0x11, _AC_CHROMA_BITS, _AC_CHROMA_VALS)
  # SOS
  sos_body = bytes([3, 1, 0x00, 2, 0x11, 3, 0x11, 0x00, 0x3F, 0x00])
  header += b'\xff\xda' + struct.pack('>H', len(sos_body) + 2) + sos_body
  header += out
  header += b'\xff\xd9'  # EOI
  return bytes(header)
