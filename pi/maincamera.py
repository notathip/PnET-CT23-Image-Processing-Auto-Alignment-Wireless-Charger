# -*- coding: utf-8 -*-
import pigpio
import time
import cv2
import numpy as np
import multiprocessing
import math
import serial
import csv
import os
import json
from picamera2 import Picamera2
from flask import Flask, render_template_string, jsonify, Response, request, redirect

# ==========================================
# ⚙️ ตั้งค่าระบบฮาร์ดแวร์หลัก
# ==========================================
X_STEP, X_DIR, X_EN = 22, 27, 17
Y_STEP, Y_DIR, Y_EN = 21, 20, 16
LIMIT_X, LIMIT_Y = 23, 26
LIMIT_DEBOUNCE_MS = 10

STEPS_PER_MM = 80.0
TRAVEL_X_MM, TRAVEL_Y_MM = 262.0, 255.5
COIL_HOME_X_MM, COIL_HOME_Y_MM = 100.0, 72.0
CENTER_X_MM = 125
CENTER_Y_MM = 133
SPEED_US = 150
HOMING_DELAY = 0.0001

REACH_MARGIN_MM = 3.0
REACH_SNAP_MM = 6.0

# ขนาดภาพที่ประมวลผล — จุดกลางจอคำนวณจากค่านี้ ห้าม hardcode 320/180 ที่อื่น
CAM_W, CAM_H = 640, 360
CAM_CX, CAM_CY = CAM_W // 2, CAM_H // 2

SPIRAL_START_MM = 20
SPIRAL_INCREMENT_MM = 30
MAX_LOOPS = 5

# ID ที่ติดจริงบนรถ (มุมละ 1 ตัว) — marker ที่ไม่อยู่ในลิสต์นี้ถือว่าตรวจจับผิด
VALID_MARKER_IDS = (1, 2, 3, 4)

# เมื่อ center_on_target หลุด (LOST) marker ต้องอยู่แถวนั้น วนหาแคบๆ ก่อนยอมแพ้
LOCAL_SEARCH_START_MM = 10
LOCAL_SEARCH_INCREMENT_MM = 20
LOCAL_SEARCH_LOOPS = 3
LOST_MAX_RETRY = 2

# ==========================================
# 🔌 Arduino (inverter + CT sensor) — เฟส B
# ==========================================
SERIAL_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
SERIAL_BAUD = 115200

ARDUINO_BOOT_WAIT = 2.5

PING_INTERVAL = 0.2

# ถ้าสั่ง ON แล้ว Arduino ไม่ตอบว่าติดภายในเวลานี้ ให้ยกเลิก (กันค้างรอเก้อ)
INVERTER_START_TIMEOUT = 3.0

ZERO_TIMEOUT = 2.0

LOG_DIR = "/home/notathip/ev_charger/logs"

DEV_SETTINGS_FILE = "/home/notathip/ev_charger/RaspberryPI/dev_settings.json"
AIM_ERR_MAX_MM = 20.0

# ==========================================
# 🕹️ โหมด jog ระหว่าง state 5 — ใช้เก็บกราฟกระแส-vs-ตำแหน่ง (กราฟตัว V)
# ==========================================
JOG_STEP_MM = 2.0        # ระยะต่อการกด 1 ครั้ง (2mm = 160 สเต็ป ใช้เวลา ~48ms)
JOG_MAX_MM = 20.0
JOG_SETTLE_S = 0.6       # รอกระแสนิ่งหลังขยับเสร็จ ก่อนเริ่มนับตัวอย่าง
JOG_AVG_N = 10

# ==========================================
# 🎯 Grid Search — หาจุด coupling ดีที่สุดอัตโนมัติ
# ==========================================
GRID_SPAN_MM = 20.0
GRID_POINTS = 11
GRID_VERIFY_MARGIN_A = 0.08
GRID_REACH_MM = 60.0
GRID_PAST_MIN = 2

AUTO_GRID_SETTLE_S = 1.5

# ==========================================
# 🦘 กระโดดตรงไปหาขดลวด — ข้ามขั้น center_on_target()
# ==========================================
JUMP_PX_PER_MM = 8.40
JUMP_MAX_R_PX = 240.0      # ไกลกว่านี้ = เข้าเขตที่ความคลาดพุ่ง -> ขยับเข้ามาก่อนค่อยกระโดด
JUMP_ZONE_R_PX = 150.0     # ขยับเข้ามาให้เหลือรัศมีประมาณเท่านี้
JUMP_SAMPLE_S = 0.6        # เก็บตัวอย่างตำแหน่ง marker นานเท่าไหร่ก่อนตัดสินใจ
JUMP_MIN_SAMPLES = 5       # ต้องได้อย่างน้อยกี่ตัวอย่างถึงจะเชื่อผล
JUMP_NUDGE_MAX = 0

# ==========================================
# 🌟 Shared Memory
# ==========================================
program_running = multiprocessing.Value('i', 1) 
is_marker_visible = multiprocessing.Value('i', 0)
enable_vision = multiprocessing.Value('i', 0)
marker_x = multiprocessing.Value('i', 0)
marker_y = multiprocessing.Value('i', 0)
marker_id = multiprocessing.Value('i', 0) 
marker_angle = multiprocessing.Value('d', 0.0) 

web_command = multiprocessing.Value('i', 0)
system_state = multiprocessing.Value('i', 0)
auto_run = multiprocessing.Value('i', 0)
frame_queue = multiprocessing.Queue(maxsize=1)

# --- ค่าจาก Arduino (motor process เป็นคนเขียน เว็บเป็นคนอ่าน) ---
serial_ok = multiprocessing.Value('i', 0)    # เปิดพอร์ต Arduino ได้ไหม
inv_on = multiprocessing.Value('i', 0)       # inverter ติดอยู่ไหม (ตามที่ Arduino รายงาน)
ct_idc = multiprocessing.Value('d', 0.0)
ct_ipp = multiprocessing.Value('d', 0.0)     # ระลอกคลื่นเป็นหน่วยกระแส (A)
ct_vdiff = multiprocessing.Value('d', 0.0)   # Vout - Vref (V) ค่าดิบ
ct_vref = multiprocessing.Value('d', 0.0)    # แรงดันขา A1 (V) ควรนิ่งราว 2.5
ct_vout = multiprocessing.Value('d', 0.0)    # แรงดันขา A0 (V)
inv_event = multiprocessing.Value('i', 0)
oc_req = multiprocessing.Value('d', 0.0)
oc_lim = multiprocessing.Value('d', 0.0)
oc_err = multiprocessing.Value('i', 0)

jog_x = multiprocessing.Value('d', 0.0)
jog_y = multiprocessing.Value('d', 0.0)
# 0=ว่าง 1=กำลังขยับ/รอค่านิ่ง (เว็บเอาไปหรี่ปุ่มกันกดรัว)
jog_busy = multiprocessing.Value('i', 0)
# 0=ไม่ได้กวาดอัตโนมัติ · >0 = กำลังกวาด (ค่า = จุดที่วัดไปแล้ว ไว้โชว์ความคืบหน้า)
jog_sweep = multiprocessing.Value('i', 0)
step_i = multiprocessing.Value('i', 0)      # จุดที่ (1..n) · 0 = ไม่ได้อยู่ในโหมดนี้
step_n = multiprocessing.Value('i', 0)
step_wait = multiprocessing.Value('i', 0)   # 1 = ถึงจุดแล้ว วัดแล้ว กำลังรอคนกด "ถัดไป"
aim_ex = multiprocessing.Value('d', 0.0)
aim_ey = multiprocessing.Value('d', 0.0)

ARDUINO_BUNDLE = (serial_ok, inv_on, ct_idc, ct_ipp, ct_vdiff, ct_vref, ct_vout, inv_event,
                  jog_x, jog_y, jog_busy, jog_sweep, oc_req, oc_lim, oc_err,
                  step_i, step_n, step_wait, aim_ex, aim_ey)

# ==========================================
# 📍 Position tracking — พิกัดสัมบูรณ์ของแท่น (มม. จาก home)
# ==========================================
pos_x = multiprocessing.Value('d', 0.0)      # มม. จาก home (0 = limit switch ถูกกด)
pos_y = multiprocessing.Value('d', 0.0)
pos_ok = multiprocessing.Value('i', 0)       # 0 = ยังไม่รู้พิกัด (ต้อง home ก่อน) · 1 = รู้
pos_msg = multiprocessing.Array('c', 256)

aim_t = multiprocessing.Value('d', time.time())

auto_tm = multiprocessing.Array('d', 12)

POS_BUNDLE = (pos_x, pos_y, pos_ok, pos_msg, aim_t, auto_tm)

# ==========================================
# 🌐 ส่วนของหน้าเว็บ UI (Dashboard) - อัปเดตสคริปต์ Auto-Reconnect
# ==========================================
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="th">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>EV Smart Charger</title>
    <style>
        :root {
            --bg:        #080b12;
            --surface:   #111827;
            --surface-2: #171f31;
            --line:      #232f47;
            --line-soft: #1b2437;
            --text:      #eef2fa;
            --dim:       #a3b0c6;
            --muted:     #6b7896;
            --accent:    #22d3ee;
            --blue:      #3b82f6;
            --ok:        #34d399;
            --warn:      #fbbf24;
            --danger:    #f87171;
            --r:  14px;
            --r-s: 10px;
        }
        * { box-sizing: border-box; }
        html { -webkit-text-size-adjust: 100%; }
        body {
            margin: 0;
            padding: 20px 16px 40px;
            min-height: 100vh;
            font-family: system-ui, -apple-system, 'Segoe UI', 'Noto Sans Thai', sans-serif;
            background: var(--bg);
            background-image:
                radial-gradient(1100px 600px at 12% -15%, rgba(34,211,238,0.10), transparent 62%),
                radial-gradient(800px 500px at 105% -5%, rgba(59,130,246,0.09), transparent 58%);
            background-attachment: fixed;
            color: var(--text);
            font-size: 15px;
            line-height: 1.5;
        }
        .shell { max-width: 1120px; margin: 0 auto; }

        /* ================= Header ================= */
        header {
            display: flex; align-items: center; justify-content: space-between;
            gap: 14px; flex-wrap: wrap; margin-bottom: 18px;
        }
        .brand { display: flex; align-items: center; gap: 13px; }
        .brand-mark {
            cursor: default; user-select: none; -webkit-user-select: none; -webkit-tap-highlight-color: transparent;
            width: 42px; height: 42px; border-radius: 12px;
            display: grid; place-items: center; font-size: 21px;
            background: linear-gradient(150deg, #0891b2, #2563eb);
            box-shadow: 0 8px 22px rgba(37,99,235,0.35);
        }
        .brand h1 { margin: 0; font-size: 1.2rem; font-weight: 700; letter-spacing: 0.2px; }
        .brand p  { margin: 1px 0 0; font-size: 0.7rem; color: var(--muted);
                    letter-spacing: 1.6px; text-transform: uppercase; font-weight: 600; }

        .chips { display: flex; align-items: center; gap: 9px; flex-wrap: wrap; }
        .chip {
            display: inline-flex; align-items: center; gap: 9px;
            padding: 9px 16px 9px 13px; border-radius: 999px;
            background: var(--surface); border: 1px solid var(--line);
            font-size: 0.87rem; font-weight: 600; white-space: nowrap;
            transition: border-color 0.3s ease, color 0.3s ease;
        }
        .chip.sm { font-size: 0.78rem; padding: 8px 13px 8px 11px; color: var(--dim); }
        .dot { width: 9px; height: 9px; border-radius: 50%; background: currentColor; flex: 0 0 auto; }
        .dot.live { animation: pulse 1.7s infinite; }
        @keyframes pulse {
            0%   { box-shadow: 0 0 0 0 currentColor; }
            70%  { box-shadow: 0 0 0 8px transparent; }
            100% { box-shadow: 0 0 0 0 transparent; }
        }

        /* ================= Stepper ================= */
        .flow {
            display: grid; grid-template-columns: repeat(5, 1fr);
            gap: 6px; margin-bottom: 18px;
            background: var(--surface); border: 1px solid var(--line);
            border-radius: var(--r); padding: 14px 12px;
        }
        .fstep { position: relative; text-align: center; padding-top: 4px; }
        .fstep::before {
            content: ""; position: absolute; top: 17px; left: -50%;
            width: 100%; height: 2px; background: var(--line); z-index: 0;
            transition: background 0.35s ease;
        }
        .fstep:first-child::before { display: none; }
        .fstep .node {
            position: relative; z-index: 1; margin: 0 auto 7px;
            width: 28px; height: 28px; border-radius: 50%;
            display: grid; place-items: center;
            font-size: 0.76rem; font-weight: 700;
            background: var(--surface-2); color: var(--muted);
            border: 1.5px solid var(--line);
            transition: all 0.3s ease;
        }
        .fstep .lab {
            font-size: 0.72rem; font-weight: 600; color: var(--muted);
            line-height: 1.3; transition: color 0.3s ease;
        }
        .fstep.done .node { background: rgba(52,211,153,0.15); border-color: var(--ok); color: var(--ok); }
        .fstep.done .lab  { color: var(--dim); }
        .fstep.done::before { background: rgba(52,211,153,0.5); }
        .fstep.on .node {
            background: rgba(34,211,238,0.18); border-color: var(--accent); color: var(--accent);
            animation: pulse 1.7s infinite;
        }
        .fstep.on .lab { color: var(--text); }
        @media (max-width: 620px) {
            .flow { padding: 12px 6px; }
            .fstep .lab { font-size: 0.62rem; }
        }

        /* ================= Alerts ================= */
        .alert {
            display: none; align-items: flex-start; gap: 10px;
            margin-bottom: 16px; padding: 13px 16px;
            border-radius: var(--r-s); font-size: 0.87rem; font-weight: 600; line-height: 1.5;
        }
        .alert.show { display: flex; }
        .alert.danger { background: rgba(248,113,113,0.11); border: 1px solid rgba(248,113,113,0.38); color: #fca5a5; }
        .alert.warn   { background: rgba(251,191,36,0.10);  border: 1px solid rgba(251,191,36,0.35);  color: #fcd34d; }
        .alert .ai { font-size: 1.05rem; line-height: 1.35; }

        /* ================= Layout ================= */
        .grid {
            display: grid; grid-template-columns: minmax(0, 1.5fr) minmax(300px, 1fr);
            gap: 16px; align-items: start;
        }
        @media (max-width: 900px) { .grid { grid-template-columns: 1fr; } }

        .card {
            background: var(--surface); border: 1px solid var(--line);
            border-radius: var(--r); overflow: hidden;
            box-shadow: 0 14px 38px rgba(0,0,0,0.4);
        }
        .card-head {
            padding: 13px 17px; border-bottom: 1px solid var(--line-soft);
            display: flex; align-items: center; justify-content: space-between; gap: 10px;
            font-size: 0.72rem; letter-spacing: 1.5px; text-transform: uppercase;
            color: var(--muted); font-weight: 700;
        }

        /* ================= Camera ================= */
        .video-wrap {
            position: relative; aspect-ratio: 16 / 9; background: #04060b;
            display: grid; place-items: center; overflow: hidden;
        }
        .video-wrap img { width: 100%; height: 100%; object-fit: contain; display: block; }
        .cam-off {
            display: none; position: absolute; inset: 0;
            flex-direction: column; align-items: center; justify-content: center; gap: 8px;
            color: var(--muted); text-align: center; padding: 20px;
        }
        .cam-off .icon { font-size: 2.2rem; opacity: 0.5; }
        .cam-off .sub  { font-size: 0.82rem; max-width: 320px; line-height: 1.55; }
        .cam-toggle {
            position: absolute; right: 13px; bottom: 13px; z-index: 5;
            width: 44px; height: 44px; border-radius: 50%;
            border: 1px solid rgba(255,255,255,0.16);
            background: rgba(8,11,18,0.75); backdrop-filter: blur(7px);
            color: #fff; font-size: 1.2rem; cursor: pointer;
            display: grid; place-items: center;
            transition: transform 0.15s ease, background 0.2s ease, opacity 0.2s ease;
        }
        .cam-toggle:hover:not(:disabled) { background: var(--blue); transform: scale(1.08); }

        
        .video-wrap.as-grid { aspect-ratio: auto; display: block; background: var(--surface); }
        .grid-view { display: none; flex-direction: column; gap: 10px; padding: 12px 14px 14px; width: 100%; box-sizing: border-box; }
        .grid-view.show { display: flex; }
        .gv-row { display: grid; grid-template-columns: minmax(0, 1fr) minmax(0, 1fr); gap: 10px; }
        @media (max-width: 600px) { .gv-row { grid-template-columns: minmax(0, 1fr); } }
        .gv-panel {
            min-width: 0; border: 1px solid var(--line); border-radius: 10px; padding: 8px 8px 6px;
            opacity: 0.7; transition: opacity 0.2s ease, border-color 0.2s ease;
        }
        .gv-panel.active { opacity: 1; border-color: var(--accent); }
        .gv-panel.done { opacity: 1; }
        .gv-title { font-size: 0.78rem; font-weight: 700; color: var(--dim); padding: 0 4px 4px; }
        .gv-panel.active .gv-title { color: var(--accent); }
        .gv-svg svg { width: 100%; height: auto; display: block; }
        .gv-svg text { fill: var(--muted); font-size: 9px; font-family: inherit; }
        .gv-hit { cursor: pointer; }
        .gv-tip { pointer-events: none; }
        .gv-cap { font-size: 0.78rem; color: var(--dim); padding: 4px 4px 0; line-height: 1.5; font-variant-numeric: tabular-nums; }
        .gv-sum {
            font-size: 0.9rem; color: var(--text); line-height: 1.6; padding-right: 54px;
            font-variant-numeric: tabular-nums;
        }
        .gv-sum b { color: var(--ok); }
        .gv-key { font-size: 0.72rem; color: var(--muted); line-height: 1.5; padding-right: 54px; }
        .video-wrap.gv-nobtn .cam-toggle { display: none; }
        .video-wrap.gv-nobtn .gv-sum, .video-wrap.gv-nobtn .gv-key { padding-right: 0; }
        .gv-key .k1 { color: var(--accent); } .gv-key .k2 { color: var(--warn); } .gv-key .k3 { color: var(--ok); }
        .cam-toggle:active:not(:disabled) { transform: scale(0.94); }
        .cam-toggle:disabled { opacity: 0.3; cursor: not-allowed; }
        .live-badge {
            position: absolute; left: 13px; top: 13px; z-index: 5;
            display: flex; align-items: center; gap: 6px;
            padding: 5px 11px; border-radius: 999px;
            background: rgba(8,11,18,0.75); backdrop-filter: blur(7px);
            border: 1px solid rgba(255,255,255,0.12);
            font-size: 0.66rem; font-weight: 800; letter-spacing: 1.3px; color: #fca5a5;
        }
        .live-badge .dot { width: 6px; height: 6px; background: var(--danger); }
        .note {
            padding: 11px 17px; font-size: 0.76rem; color: var(--muted);
            border-top: 1px solid var(--line-soft); line-height: 1.55;
        }
        .note b { color: var(--dim); font-weight: 700; }

        /* ================= Actions ================= */
        .acts { padding: 15px; display: flex; flex-direction: column; gap: 9px; }
        .btn {
            position: relative; width: 100%;
            display: flex; align-items: center; gap: 12px;
            padding: 13px 15px; border: 1px solid var(--line);
            border-radius: 12px; background: var(--surface-2);
            font-family: inherit; font-size: 0.95rem; font-weight: 700;
            color: var(--text); cursor: pointer; text-align: left;
            transition: transform 0.12s ease, border-color 0.2s ease, background 0.2s ease;
        }
        .btn:hover:not(:disabled) { background: #1e2841; border-color: #33415c; }
        .btn:active:not(:disabled) { transform: scale(0.985); }
        .btn .ico {
            flex: 0 0 auto; width: 34px; height: 34px; border-radius: 9px;
            display: grid; place-items: center; font-size: 1.05rem;
            background: rgba(255,255,255,0.05);
        }
        .btn .txt { display: flex; flex-direction: column; gap: 1px; line-height: 1.3; min-width: 0; }
        .btn .txt small { font-weight: 500; font-size: 0.71rem; color: var(--muted); }
        .btn:disabled { opacity: 0.34; cursor: not-allowed; }

        .btn.next {
            border-color: var(--accent);
            background: linear-gradient(150deg, rgba(34,211,238,0.16), rgba(59,130,246,0.10));
            box-shadow: 0 0 0 1px rgba(34,211,238,0.22), 0 8px 24px rgba(34,211,238,0.14);
        }
        .btn.next .ico { background: rgba(34,211,238,0.2); }
        .btn.next .txt small { color: #a5f3fc; }

        .btn-power:not(:disabled) .ico { background: rgba(251,191,36,0.2); }
        .btn-aim:not(:disabled)   .ico { background: rgba(52,211,153,0.18); }
        .btn-grid:not(:disabled)  .ico { background: rgba(167,139,250,0.2); }

        .sep { height: 1px; background: var(--line-soft); margin: 4px 0 2px; }

        
        .acts > div { display: flex; flex-direction: column; gap: 9px; }
        
        .dev { display: none; margin-top: 16px; border-color: rgba(251,191,36,0.35); }
        .dev.show { display: block; }
        .dev .card-head { color: var(--warn); }
        .dev-body { padding: 16px 17px; display: flex; flex-direction: column; gap: 12px; }
        .dev-row { display: flex; gap: 10px; align-items: stretch; flex-wrap: wrap; }
        .dev-row input {
            flex: 1 1 120px; min-width: 0; padding: 11px 13px; border-radius: var(--r-s);
            border: 1px solid var(--line); background: var(--surface-2); color: var(--text);
            font-size: 1rem; font-variant-numeric: tabular-nums;
        }
        .dev-row input:focus { outline: 2px solid var(--warn); outline-offset: 1px; }
        .dev-row button {
            padding: 11px 18px; border-radius: var(--r-s); border: 1px solid rgba(251,191,36,0.5);
            background: rgba(251,191,36,0.12); color: var(--warn); font-weight: 700; font-size: 0.9rem; cursor: pointer;
        }
        .dev-row button:disabled { opacity: 0.4; cursor: not-allowed; }
        .dev-msg { font-size: 0.8rem; color: var(--dim); line-height: 1.55; min-height: 1.2em; }
        .dev-msg.bad { color: var(--danger); }
        .dev-msg.good { color: var(--ok); }
        
        .atm-total { font-size: 2rem; font-weight: 800; font-variant-numeric: tabular-nums; }
        .atm-hist { font-size: 0.8rem; color: var(--dim); line-height: 1.6; font-variant-numeric: tabular-nums; }
        .atm-item { display: flex; gap: 8px; align-items: flex-start; padding: 4px 0; border-bottom: 1px solid var(--line); }
        .atm-item > div { flex: 1; }
        .atm-del { flex: none; padding: 2px 10px; font-size: 0.8rem; }
        .dev-sep { border-top: 1px solid var(--line); padding-top: 12px; font-weight: 700; color: var(--warn); font-size: 0.9rem; }
        .step-stat { font-size: 0.95rem; line-height: 1.6; font-variant-numeric: tabular-nums; }
        .step-stat b { font-size: 1.25rem; }
        .step-stat.wait { color: var(--ok); }
        .dev-row button.step-next { flex: 1 1 140px; font-size: 1rem; }
        .dev-exit {
            align-self: flex-start; background: none; border: none; padding: 0;
            color: var(--muted); font-size: 0.78rem; text-decoration: underline; cursor: pointer;
        }

        .modebar {
            display: flex; gap: 6px; padding: 6px; margin-bottom: 14px;
            background: var(--surface); border: 1px solid var(--line); border-radius: 12px;
        }
        .modebar button {
            flex: 1; padding: 11px 10px; border: 1px solid transparent; border-radius: 9px;
            background: transparent; cursor: pointer; font-family: inherit;
            font-size: 0.9rem; font-weight: 700; color: var(--dim);
            transition: background 0.18s ease, color 0.18s ease, border-color 0.18s ease;
        }
        .modebar button:hover:not(:disabled):not(.on) { color: var(--text); background: var(--surface-2); }
        .modebar button.on {
            background: linear-gradient(150deg, rgba(34,211,238,0.18), rgba(59,130,246,0.10));
            border-color: rgba(34,211,238,0.45); color: #a5f3fc;
        }
        .modebar button:disabled { opacity: 0.38; cursor: not-allowed; }

        .btn-auto { padding: 20px 15px; font-size: 1.05rem; }
        .btn-auto .ico { width: 40px; height: 40px; font-size: 1.2rem; background: rgba(34,211,238,0.22); }
        .autostat {
            padding: 12px 14px; border-radius: 10px; font-size: 0.82rem; line-height: 1.6;
            background: var(--surface-2); border: 1px solid var(--line-soft); color: var(--dim);
        }
        .autostat.run { border-color: rgba(251,191,36,0.4); background: rgba(251,191,36,0.07); color: #fde68a; }

        .btn-estop {
            justify-content: center; gap: 10px; text-align: center;
            background: rgba(248,113,113,0.10); border-color: rgba(248,113,113,0.45);
            color: #fca5a5; font-size: 0.9rem; letter-spacing: 0.6px;
        }
        .btn-estop:hover:not(:disabled) { background: rgba(248,113,113,0.2); border-color: var(--danger); }
        .btn-estop .ico { background: rgba(248,113,113,0.16); }

        /* ================= Telemetry ================= */
        .telem { margin-top: 16px; }
        .telem-body {
            display: grid; grid-template-columns: minmax(0,1.1fr) minmax(0,1fr);
            gap: 16px; padding: 18px;
        }
        @media (max-width: 700px) { .telem-body { grid-template-columns: 1fr; } }

        .amp {
            border-radius: 12px; padding: 16px 18px;
            background: var(--surface-2); border: 1px solid var(--line);
            transition: border-color 0.3s ease, box-shadow 0.3s ease;
        }
        .amp.hot { border-color: rgba(251,191,36,0.55); box-shadow: 0 0 28px rgba(251,191,36,0.16); }
        .amp-cap { font-size: 0.7rem; letter-spacing: 1.3px; text-transform: uppercase;
                   color: var(--muted); font-weight: 700; margin-bottom: 4px; }
        .amp-val { display: flex; align-items: baseline; gap: 6px; }
        .amp-num { font-size: 2.9rem; font-weight: 800; letter-spacing: -1.5px;
                   color: var(--dim); font-variant-numeric: tabular-nums; line-height: 1.05;
                   transition: color 0.3s ease; }
        .amp.hot .amp-num { color: var(--warn); }
        .amp-unit { font-size: 1.05rem; color: var(--muted); font-weight: 700; }
        .amp-sub { font-size: 0.75rem; color: var(--muted); margin-top: 4px; }
        .amp-live { font-size: 0.75rem; color: var(--dim); margin-top: 2px; font-variant-numeric: tabular-nums; }

        .facts { display: flex; flex-direction: column; gap: 10px; justify-content: center; }
        .fact {
            display: flex; align-items: baseline; justify-content: space-between; gap: 12px;
            padding-bottom: 9px; border-bottom: 1px dashed var(--line-soft);
            font-size: 0.82rem; color: var(--muted);
        }
        .fact:last-of-type { border-bottom: none; padding-bottom: 0; }
        .fact b { color: var(--text); font-weight: 700; font-variant-numeric: tabular-nums;
                  text-align: right; }

        details.raw { margin-top: 2px; }
        details.raw summary {
            cursor: pointer; list-style: none; user-select: none;
            font-size: 0.75rem; color: var(--muted); font-weight: 600;
            padding: 7px 0; display: flex; align-items: center; gap: 6px;
        }
        details.raw summary::-webkit-details-marker { display: none; }
        details.raw summary::before { content: "\\25B8"; transition: transform 0.2s ease; display: inline-block; }
        details.raw[open] summary::before { transform: rotate(90deg); }
        details.raw summary:hover { color: var(--dim); }
        .raw-grid {
            display: grid; grid-template-columns: repeat(2, minmax(0,1fr));
            gap: 7px 16px; padding: 10px 12px; margin-top: 4px;
            border-radius: var(--r-s); background: rgba(0,0,0,0.25);
            border: 1px solid var(--line-soft);
            font-size: 0.75rem; color: var(--muted);
        }
        .raw-grid div { display: flex; justify-content: space-between; gap: 8px; }
        .raw-grid span { color: var(--dim); font-variant-numeric: tabular-nums; font-weight: 600; }
        .raw-grid span.bad { color: var(--danger); }
        .raw-tip { grid-column: 1 / -1; color: var(--muted); font-size: 0.7rem;
                   border-top: 1px dashed var(--line-soft); padding-top: 7px; line-height: 1.5; }

        footer { margin-top: 18px; text-align: center; font-size: 0.72rem; color: var(--muted); }
    </style>
</head>
<body>
<div class="shell">

    <header>
        <div class="brand">
            <div class="brand-mark" id="brand-mark" onclick="devTap()">&#9889;</div>
            <div>
                <h1>EV Smart Charger</h1>
                <p>Wireless Alignment System</p>
            </div>
        </div>
        <div class="chips">
            <span class="chip sm" id="link-chip"><span class="dot" id="link-dot"></span><span id="link-text">Arduino</span></span>
            <span class="chip" id="pill" style="color:#a3b0c6"><span class="dot" id="pill-dot"></span><span id="status-text">สแตนด์บาย</span></span>
        </div>
    </header>

    <div class="modebar">
        <button id="mode-auto"  onclick="setMode('auto')">&#9889; อัตโนมัติ</button>
        <button id="mode-stage" onclick="setMode('stage')">&#9881; ทีละขั้น</button>
    </div>

    <div class="flow">
        <div class="fstep" id="fs-1"><div class="node">1</div><div class="lab">เตรียมเครื่อง</div></div>
        <div class="fstep" id="fs-2"><div class="node">2</div><div class="lab">พร้อมชาร์จ</div></div>
        <div class="fstep" id="fs-3"><div class="node">3</div><div class="lab">ค้นหาและเล็ง</div></div>
        <div class="fstep" id="fs-4"><div class="node">4</div><div class="lab">ล็อกเป้าแล้ว</div></div>
        <div class="fstep" id="fs-5"><div class="node">5</div><div class="lab">กำลังจ่ายไฟ</div></div>
    </div>

    <div class="alert danger" id="alert-evt"><span class="ai">&#128721;</span><span id="alert-evt-text"></span></div>
    <div class="alert warn"   id="alert-aim"><span class="ai">&#129514;</span><span id="alert-aim-text"></span></div>
    <div class="alert warn"   id="alert-pos"><span class="ai">&#128663;</span><span id="alert-pos-text"></span></div>

    <div class="grid">

        <div class="card">
            <div class="card-head">
                <span id="vh-title">กล้องตรวจจับ ArUco</span>
                <span id="vh-sub">{{ cam_w }} &times; {{ cam_h }}</span>
            </div>
            <div class="video-wrap" id="video-wrap">
                <div class="live-badge" id="live-badge"><span class="dot"></span>LIVE</div>
                <img id="cam-stream" alt=""
                     onerror="this.style.display='none'; document.getElementById('offline-msg').style.display='flex'; document.getElementById('live-badge').style.display='none';">
                <div class="cam-off" id="offline-msg">
                    <div class="icon">&#128247;</div>
                    <div><b>กล้องปิดอยู่</b></div>
                    <div class="sub" id="cam-off-sub">กล้องจะเปิดเองตอนค้นหา · กดปุ่มมุมขวาล่างถ้าอยากเปิดดูเอง</div>
                </div>
                
                <div class="grid-view" id="grid-view">
                    <div class="gv-row">
                        <div class="gv-panel" id="gv-x"><div class="gv-title">แกน X</div><div class="gv-svg"></div><div class="gv-cap">&nbsp;</div></div>
                        <div class="gv-panel" id="gv-y"><div class="gv-title">แกน Y</div><div class="gv-svg"></div><div class="gv-cap">&nbsp;</div></div>
                    </div>
                    <div class="gv-sum" id="gv-sum">&nbsp;</div>
                    <div class="gv-key">ชี้/แตะจุดเพื่อดูค่า · แกนตั้ง = กระแส (A) · แกนนอน = ระยะจากจุดที่กล้องเล็ง (mm) · <span class="k1">●</span> จุดที่วัด · <span class="k2">●</span> เดินต่อจากขอบ · <span class="k3">┆</span> จุดที่เลือก</div>
                </div>
                <button class="cam-toggle" id="cam-btn" onclick="sendCommand(3)" title="เปิด / ปิด กล้อง">&#128247;</button>
            </div>
            <div class="note">
                &#128161; <b>ใช้งานในห้องที่แสงสว่างพอ</b> (ระดับห้องทำงานกลางวัน)
                แสงน้อยเกินไปกล้องจะตรวจจับ ArUco ได้ไม่ต่อเนื่อง
            </div>
        </div>

        <div class="card">
            <div class="card-head"><span id="panel-title">แผงควบคุม</span></div>
            <div class="acts">

              <!-- ===== โหมดอัตโนมัติ: ปุ่มเดียวตั้งแต่ home ยันปรับตำแหน่ง (คำสั่ง 20) ===== -->
              <div id="panel-auto">
                <div class="autostat" id="auto-stat">&nbsp;</div>

                <button class="btn btn-auto" id="b20" onclick="startAuto()" disabled>
                    <span class="ico">&#9654;</span>
                    <span class="txt">เริ่มชาร์จอัตโนมัติ<small>เตรียมเครื่อง &rarr; ค้นหารถ &rarr; เล็ง &rarr; จ่ายไฟ &rarr; ปรับตำแหน่ง</small></span>
                </button>

                <button class="btn" id="b5a" onclick="sendCommand(5)" disabled>
                    <span class="ico">&#9209;</span>
                    <span class="txt">หยุดจ่ายไฟ<small>ดับ inverter กลับสู่สถานะพร้อม</small></span>
                </button>

                <div class="sep"></div>

                <button class="btn btn-estop" onclick="sendCommand(99)">
                    <span class="ico">&#128721;</span>
                    <span class="txt">หยุดฉุกเฉิน</span>
                </button>
              </div>

              
              <div id="panel-stage">
                <button class="btn" id="b1" onclick="sendCommand(1)">
                    <span class="ico">&#9881;</span>
                    <span class="txt">เตรียมความพร้อม<small>หาจุด home แล้วไปจุดกลาง</small></span>
                </button>

                <button class="btn btn-aim" id="b2" onclick="sendCommand(2)">
                    <span class="ico">&#128269;</span>
                    <span class="txt">ค้นหาและเล็งจุดชาร์จ<small>วิ่งก้นหอยหารถ แล้วเลื่อนขดลวดเข้าที่</small></span>
                </button>

                <button class="btn btn-power" id="b4" onclick="sendCommand(4)" disabled>
                    <span class="ico">&#9889;</span>
                    <span class="txt">เริ่มจ่ายไฟ<small>เปิด inverter (ต้องล็อกเป้าสำเร็จก่อน)</small></span>
                </button>

                <button class="btn btn-grid" id="b16" onclick="startGrid()" disabled>
                    <span class="ico">&#127919;</span>
                    <span class="txt">ปรับตำแหน่งให้ดีที่สุด<small id="b16-sub">วัดกระแสหลายจุดแล้วหาจุด coupling ดีสุด</small></span>
                </button>

                <button class="btn" id="b5" onclick="sendCommand(5)" disabled>
                    <span class="ico">&#9209;</span>
                    <span class="txt">หยุดจ่ายไฟ<small>ดับ inverter กลับสู่สถานะพร้อม</small></span>
                </button>

                <div class="sep"></div>

                <button class="btn btn-estop" onclick="sendCommand(99)">
                    <span class="ico">&#128721;</span>
                    <span class="txt">หยุดฉุกเฉิน</span>
                </button>
              </div>

            </div>
        </div>
    </div>

    <div class="card telem">
        <div class="card-head">
            <span>ค่าวัดจากระบบ</span>
            <span id="telem-mode">พร้อมใช้งาน</span>
        </div>
        <div class="telem-body">
            <div class="amp" id="amp-box">
                <div class="amp-cap">กระแสที่ inverter ดึงไปใช้</div>
                <div class="amp-val">
                    <span class="amp-num" id="amp-value">0.000</span>
                    <span class="amp-unit">A</span>
                </div>
                <div class="amp-live" id="amp-live">&nbsp;</div>
                <div class="amp-sub">ขดลวดยิ่งตรงกัน ค่ายิ่งต่ำ</div>
            </div>

            <div class="facts">
                <div class="fact"><span>กึ่งกลางขดลวด (จากมุมกล่อง)</span><b id="f-pos">ยังไม่ทราบ</b></div>
                <div class="fact"><span>ระยะที่ปรับจากจุดที่เล็งไว้</span><b id="f-jog">&mdash;</b></div>
                <div class="fact"><span>ลิมิตตัดกระแสเกิน</span><b id="f-oc">&mdash;</b></div>

                <details class="raw">
                    <summary>รายละเอียดทางเทคนิค</summary>
                    <div class="raw-grid">
                        <div>Vout &minus; Vref <span id="r-vdiff">&mdash;</span></div>
                        <div>Ripple <span id="r-ipp">&mdash;</span></div>
                        <div>Vout (A0) <span id="r-vout">&mdash;</span></div>
                        <div>Vref (A1) <span id="r-vref">&mdash;</span></div>
                        <div class="raw-tip" id="r-tip">Vref ปกติต้องอยู่ราว 2.49 V</div>
                    </div>
                </details>
            </div>
        </div>
    </div>

    
    <div class="card dev" id="dev-card">
        <div class="card-head">
            <span>&#128295; โหมดผู้พัฒนา</span>
            <span>ลิมิตตัดกระแสเกิน</span>
        </div>
        <div class="dev-body">
            <div class="facts">
                <div class="fact"><span>ค่าที่ Arduino ใช้ตัดอยู่ตอนนี้</span><b id="dev-oc-now">&mdash;</b></div>
            </div>
            <div class="dev-row">
                <input type="number" id="dev-oc-in" step="0.1" inputmode="decimal" placeholder="เช่น 4.5">
                <button id="dev-oc-btn" onclick="devSetOc()">ตั้งค่า</button>
            </div>
            <div class="dev-msg" id="dev-msg">&nbsp;</div>

            
            <div class="dev-sep">จับเวลาโหมดอัตโนมัติ</div>
            <div class="atm-total" id="atm-total">&mdash;</div>
            <div class="facts" id="atm-rows"></div>
            <div class="dev-msg">เริ่มนับตอนกดปุ่มอัตโนมัติ · หยุดตอน grid search จบ · บันทึกลง CSV แถว AUTO_TIME ด้วย</div>
            <div class="atm-hist" id="atm-hist"></div>
            <div class="dev-row"><button onclick="atmExport()">&#11015; บันทึกเป็นไฟล์</button><button onclick="atmClear()">ล้างประวัติทั้งหมด</button></div>

            
            <div class="dev-sep">จำลองกล้องเล็งพลาด (mm)</div>
            <div class="facts">
                <div class="fact"><span>ใช้อยู่ตอนนี้</span><b id="dev-aim-now">&mdash;</b></div>
            </div>
            <div class="dev-row">
                <input type="number" id="dev-aim-x" step="1" inputmode="decimal" placeholder="X เช่น 10">
                <input type="number" id="dev-aim-y" step="1" inputmode="decimal" placeholder="Y เช่น -5">
                <button onclick="devSetAim()">ตั้งค่า</button>
                <button onclick="devSetAim(true)">ล้าง</button>
            </div>
            <div class="dev-msg" id="dev-aim-msg">มีผลตอนเล็งครั้งถัดไป · + = ออกห่างจากมุม home (ทิศเดียวกับพิกัดขดลวด) · ไม่จำค่า รีสตาร์ตแล้วกลับเป็น 0</div>

            
            <div class="dev-sep">เก็บผลทีละจุด (X 11 จุด แล้ว Y 11 จุด)</div>
            <div class="step-stat" id="step-stat">&nbsp;</div>
            <div class="dev-row">
                <button id="step-start" onclick="stepStart()">&#9654; เริ่ม</button>
                <button id="step-next" class="step-next" onclick="sendCommand(18)">ถัดไป &#9654;</button>
                <button id="step-prev" onclick="sendCommand(19)">&#8617; ย้อน</button>
                <button id="step-quit" onclick="stepQuit()">&#9209; เลิก</button>
            </div>
            <div class="dev-msg">ใช้ได้ตอนกำลังจ่ายไฟ · เครื่องหยุดรอทุกจุดจนกว่าจะกดถัดไป · ครบแล้วไปจอดจุดกระแสต่ำสุด · เลิก = กลับจุดเริ่ม (ยังจ่ายไฟต่อ)</div>

            
            <div class="dev-sep">เครือข่าย</div>
            <div class="facts">
                <div class="fact"><span>ตอนนี้ต่ออยู่กับ</span><b id="dev-net-now">&mdash;</b></div>
            </div>
            <div class="dev-row">
                <button id="dev-net-btn" onclick="devNetSwitch('home')">&#128246; สลับไป hotspot</button>
                <button id="dev-net-off" onclick="devNetSwitch('ap')">&#10006; ตัดการเชื่อมต่อ hotspot</button>
            </div>
            <div class="dev-msg" id="dev-net-msg">เป็น AP: เปิด hotspot ที่มือถือก่อนแล้วกด "สลับไป hotspot" · ต่อ hotspot อยู่: กด "ตัดการเชื่อมต่อ" = Pi เปิด AP evchargerPnET ทันที (เครื่องนี้จะหลุด ให้ไปต่อ AP แล้วเปิด evcharger.pnet)</div>
            <button class="dev-exit" onclick="devToggle(false)">ออกจากโหมดผู้พัฒนา</button>
        </div>
    </div>

    <footer>ห้ามเปิดกล้องขณะจ่ายไฟ &middot; ห้ามจ่ายไฟตอนไม่มีรถจอดทับขดลวด</footer>
</div>

<script>
    

    function sendCommand(cmd) { fetch('/command/' + cmd); }

    
    function startGrid() { sendCommand(16); }

    
    var uiMode = 'auto';
    try { uiMode = localStorage.getItem('evMode') || 'auto'; } catch (e) { uiMode = 'auto'; }
    if (uiMode !== 'stage') { uiMode = 'auto'; }

    function applyMode() {
        document.getElementById('panel-auto').style.display  = (uiMode === 'auto')  ? 'flex' : 'none';
        document.getElementById('panel-stage').style.display = (uiMode === 'stage') ? 'flex' : 'none';
        document.getElementById('mode-auto').className  = (uiMode === 'auto')  ? 'on' : '';
        document.getElementById('mode-stage').className = (uiMode === 'stage') ? 'on' : '';
        document.getElementById('panel-title').textContent = (uiMode === 'auto')
            ? 'แผงควบคุม · โหมดอัตโนมัติ' : 'แผงควบคุม · โหมดทีละขั้น';
    }

    function setMode(m) {
        uiMode = (m === 'stage') ? 'stage' : 'auto';
        try { localStorage.setItem('evMode', uiMode); } catch (e) {}
        applyMode();
    }

    
    var devOn = false, devTaps = [], ocNow = 0, devWant = 0;
    try { devOn = localStorage.getItem('evDev') === '1'; } catch (e) { devOn = false; }

    
    var devBoot = null;
    function devBootCheck(boot) {
        if (!boot) return;
        devBoot = boot;
        var saved = null;
        try { saved = localStorage.getItem('evDevBoot'); } catch (e) {}
        if (devOn && saved !== boot) {
            if (saved) { devToggle(false); }
            else { try { localStorage.setItem('evDevBoot', boot); } catch (e) {} }
        }
    }
    function devToggle(on) {
        devOn = on;
        try { localStorage.setItem('evDev', on ? '1' : '0'); } catch (e) {}
        if (on && devBoot) { try { localStorage.setItem('evDevBoot', devBoot); } catch (e) {} }
        document.getElementById('dev-card').className = 'card dev' + (on ? ' show' : '');
    }

    function devTap() {
        var t = Date.now();
        devTaps = devTaps.filter(function (x) { return t - x < 3000; });
        devTaps.push(t);
        if (devTaps.length >= 5) {
            devTaps = []; devToggle(!devOn);
            if (devOn) { document.getElementById('dev-card').scrollIntoView({behavior: 'smooth', block: 'start'}); }
        }
    }

    var devNoBoard = false;
    function devMsg(txt, cls) {
        var m = document.getElementById('dev-msg');
        m.textContent = txt;
        m.className = 'dev-msg' + (cls ? ' ' + cls : '');
    }

    function devSetOc() {
        var v = parseFloat(document.getElementById('dev-oc-in').value);
        if (isNaN(v) || v <= 0) {
            devMsg('ต้องเป็นตัวเลขมากกว่า 0', 'bad');
            return;
        }
        if (v < ocNow && !confirm('ลดลิมิตเป็น ' + v.toFixed(2) + ' A? '
                + 'ถ้ากำลังจ่ายไฟอยู่และกระแสเกินค่านี้ inverter จะถูกตัดทันที')) { return; }
        devWant = Math.round(v * 100) / 100;
        devMsg('กำลังส่งไปที่ Arduino...', '');
        fetch('/set_oc', {method: 'POST', headers: {'Content-Type': 'application/json'},
                          body: JSON.stringify({a: v})})
            .then(function (r) { return r.json(); })
            .then(function (j) { if (!j.ok) { devWant = 0; devMsg(j.msg || 'ตั้งค่าไม่สำเร็จ', 'bad'); } })
            .catch(function () { devWant = 0; devMsg('ติดต่อ Raspberry Pi ไม่ได้', 'bad'); });
    }

    
    var netLast = 0, netBusy = false, netSent = 0;
    var NET_MODE = {home: 'hotspot Sumlee', ap: 'AP evchargerPnET (Pi ปล่อย WiFi เอง)', none: 'กำลังเปลี่ยน / ไม่ได้ต่อ', unknown: 'ไม่ทราบ (ตัวสลับ WiFi ไม่ได้รัน?)'};
    function devNetTick() {
        var now = Date.now();
        if (!devOn || netBusy || now - netLast < 2000) return;
        netBusy = true; netLast = now;
        fetch('/wifi', {cache: 'no-store'})
            .then(function (r) { return r.json(); })
            .then(function (w) {
                netBusy = false;
                document.getElementById('dev-net-now').textContent = NET_MODE[w.mode] || w.mode;
                var btn = document.getElementById('dev-net-btn'), m = document.getElementById('dev-net-msg');
                var working = w.pending === 1 || w.state === 'scanning' || w.state === 'switching';
                btn.disabled = (w.mode !== 'ap') || working;
                document.getElementById('dev-net-off').disabled = (w.mode !== 'home') || working;
                var fresh = (w.age !== null && w.age < 300) || (netSent && now - netSent < 60000);
                if (w.pending === 1) { m.textContent = 'ส่งคำขอแล้ว รอตัวสลับ WiFi รับ...'; m.className = 'dev-msg'; }
                else if (fresh && w.detail) {
                    m.textContent = w.detail;
                    m.className = 'dev-msg' + ((w.state === 'ok' || w.state === 'switching') ? ' good'
                                 : (w.state === 'scanning' || w.state === 'already') ? '' : ' bad');
                }
            })
            .catch(function () { netBusy = false; });
    }
    function devNetSwitch(to) {
        var m = document.getElementById('dev-net-msg');
        if (to === 'ap' && !confirm('ตัดการเชื่อมต่อ hotspot แล้วเปิด AP evchargerPnET ?' + String.fromCharCode(10) + 'เครื่องนี้จะหลุด ต้องไปต่อ WiFi evchargerPnET แล้วเปิด evcharger.pnet')) return;
        fetch('/wifi_switch', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({to: to})})
            .then(function (r) { return r.json(); })
            .then(function (j) {
                if (j.ok) { netSent = Date.now(); netLast = 0; m.textContent = 'ส่งคำขอแล้ว รอตัวสลับ WiFi รับ...'; m.className = 'dev-msg'; }
                else { m.textContent = j.msg || 'ส่งคำขอไม่ได้'; m.className = 'dev-msg bad'; }
            })
            .catch(function () { m.textContent = 'ติดต่อ Raspberry Pi ไม่ได้'; m.className = 'dev-msg bad'; });
    }

    
    var aimMax = 20;
    function devSetAim(clear) {
        var ix = document.getElementById('dev-aim-x'), iy = document.getElementById('dev-aim-y');
        var m = document.getElementById('dev-aim-msg');
        var x = clear ? 0 : (ix.value === '' ? 0 : parseFloat(ix.value));
        var y = clear ? 0 : (iy.value === '' ? 0 : parseFloat(iy.value));
        if (isNaN(x) || isNaN(y) || Math.abs(x) > aimMax || Math.abs(y) > aimMax) {
            m.textContent = 'ต้องเป็นตัวเลขในช่วง ±' + aimMax + ' mm'; m.className = 'dev-msg bad'; return;
        }
        fetch('/set_aim_err', {method: 'POST', headers: {'Content-Type': 'application/json'},
                               body: JSON.stringify({x: x, y: y})})
            .then(function (r) { return r.json(); })
            .then(function (j) {
                if (j.ok) {
                    if (clear) { ix.value = ''; iy.value = ''; }
                    m.textContent = clear ? 'ล้างแล้ว — เล็งตามกล้องปกติ' : 'ตั้งแล้ว — มีผลตอนเล็งครั้งถัดไป';
                    m.className = 'dev-msg good';
                } else { m.textContent = j.msg || 'ตั้งค่าไม่สำเร็จ'; m.className = 'dev-msg bad'; }
            })
            .catch(function () { m.textContent = 'ติดต่อ Raspberry Pi ไม่ได้'; m.className = 'dev-msg bad'; });
    }

    
    var stepKey = '';
    function stepStart() {
        if (confirm('เริ่มเก็บผลทีละจุด? แท่นจะเลื่อนไป 22 จุด (X แล้ว Y ข้างละ 20 mm) '
                  + 'และหยุดรอทุกจุดจนกว่าจะกดถัดไป · ลิมิตตัดกระแสตอนนี้ '
                  + (ocNow > 0 ? ocNow.toFixed(2) + ' A' : 'ยังไม่ทราบ'))) {
            sendCommand(17);
        }
    }
    function stepQuit() {
        if (confirm('เลิกเก็บผลทีละจุด แล้วกลับจุดเริ่ม? (ยังจ่ายไฟต่อ)')) { sendCommand(21); }
    }

    function startAuto() { sendCommand(20); }

    var STATES = {
        0:  { text: 'สแตนด์บาย',                    color: '#a3b0c6', live: false, next: 1 },
        1:  { text: 'กำลังเตรียมเครื่อง...',         color: '#22d3ee', live: true,  next: 0 },
        2:  { text: 'เครื่องพร้อมชาร์จ',             color: '#34d399', live: false, next: 2 },
        3:  { text: 'กำลังค้นหาและเล็งเป้า...',      color: '#fbbf24', live: true,  next: 0 },
        4:  { text: 'ล็อกเป้าสำเร็จ',                color: '#34d399', live: false, next: 4 },
        5:  { text: 'กำลังจ่ายไฟ',                   color: '#fbbf24', live: true,  next: 16 },
        99: { text: 'หยุดฉุกเฉิน',                   color: '#f87171', live: true,  next: 1 }
    };

    var EVENTS = {
        1: 'Arduino ตัดไฟเอง: กระแสเกินพิกัดที่ตั้งไว้',
        2: 'Arduino ตัดไฟเอง: ขาดการติดต่อกับ Raspberry Pi',
        3: 'มีคนกดปุ่ม OFF ที่ตัวเครื่อง'
    };

    var camShown = 1;
    var ampBuf = [];   
    var ampT0 = 0;     /* เวลาที่เริ่มเก็บ (inverter ติด หรือเพิ่งเปิดหน้าเว็บ) · 0 = ยังไม่เริ่ม */

    
    var camLoop = true, camBusy = false, camUrl = null;
    function camNext() {
        if (!camLoop || camBusy) return;
        camBusy = true;
        fetch('/snapshot?' + Date.now(), {cache: 'no-store'})
            .then(function (r) { return r.status === 200 ? r.blob() : null; })
            .then(function (b) {
                if (b && camLoop) {
                    var u = URL.createObjectURL(b);
                    document.getElementById('cam-stream').src = u;
                    if (camUrl) URL.revokeObjectURL(camUrl);
                    camUrl = u;
                }
                camBusy = false;
                setTimeout(camNext, b ? 0 : 300);
            })
            .catch(function () { camBusy = false; setTimeout(camNext, 1000); });
    }

    
    var gvData = null, gvKey = '', gvLast = 0, gvBusy = false;
    function gvFetch(now, period) {
        if (gvBusy || now - gvLast < period) return;
        gvBusy = true; gvLast = now;
        fetch('/grid_data', {cache: 'no-store'})
            .then(function (r) { return r.json(); })
            .then(function (j) { gvData = j; gvBusy = false; })
            .catch(function () { gvBusy = false; });
    }
    function gvNum(v, n) { return (v >= 0 ? '+' : '') + v.toFixed(n); }
    function gvPlot(ax, g) {
        var W = 300, H = 160, L = 36, R = 8, T = 10, B = 20;
        var pts = g.pts[ax], fit = g.fit[ax], best = g.best[ax];
        var base = ax === 'x' ? g.base[0] : g.base[1];
        if (!pts.length) {
            return '<svg viewBox="0 0 ' + W + ' ' + H + '"><text x="' + (W / 2) + '" y="' + (H / 2)
                 + '" text-anchor="middle">รอวัด</text></svg>';
        }
        var x0 = base - 20, x1 = base + 20, y0 = 1e9, y1 = -1e9, i;
        for (i = 0; i < pts.length; i++) {
            x0 = Math.min(x0, pts[i][0]); x1 = Math.max(x1, pts[i][0]);
            y0 = Math.min(y0, pts[i][1]); y1 = Math.max(y1, pts[i][1]);
        }
        var pad = Math.max(0.05, (y1 - y0) * 0.12);
        y0 -= pad; y1 += pad;
        function sx(v) { return L + (v - x0) / (x1 - x0) * (W - L - R); }
        function sy(v) { return T + (y1 - v) / (y1 - y0) * (H - T - B); }
        var o = '<svg viewBox="0 0 ' + W + ' ' + H + '">';
        o += '<defs><clipPath id="gvc-' + ax + '"><rect x="' + L + '" y="' + T + '" width="' + (W - L - R)
           + '" height="' + (H - T - B) + '"/></clipPath></defs>';
        for (i = 0; i <= 3; i++) {
            var yv = y0 + (y1 - y0) * i / 3, yy = sy(yv);
            o += '<line x1="' + L + '" x2="' + (W - R) + '" y1="' + yy + '" y2="' + yy
               + '" stroke="var(--line)" stroke-width="1"/>';
            o += '<text x="' + (L - 4) + '" y="' + (yy + 3) + '" text-anchor="end">' + yv.toFixed(2) + '</text>';
        }
        var t0 = Math.ceil(x0 / 10) * 10;
        for (var xt = t0; xt <= x1 + 1e-6; xt += 10) {
            var anc = sx(xt) > W - R - 10 ? 'end' : 'middle';
            o += '<text x="' + sx(xt) + '" y="' + (H - 6) + '" text-anchor="' + anc + '">' + (xt > 0 ? '+' : '') + xt + '</text>';
        }
        if (fit && !fit.nofit && fit.a !== undefined && fit.b === undefined && best) {
            fit.b = -2 * fit.a * fit.vertex;   /* CSV ก่อน 14 ต.ค. log แค่ a + vertex */
            fit.c = best[1] - fit.a * fit.vertex * fit.vertex - fit.b * fit.vertex;
        }
        if (fit && !fit.nofit && fit.a !== undefined && fit.b !== undefined) {
            var px0 = x0, px1 = x1, d = '', lm = g.local[ax];
            if (lm !== undefined && lm !== null) {   /* ฟิตเฉพาะที่: วาดเส้นแค่ช่วงจุดต่ำสุด ±2 ก้าว */
                px0 = Math.max(x0, lm - 2.5 * g.step); px1 = Math.min(x1, lm + 2.5 * g.step);
            }
            for (i = 0; i <= 60; i++) {
                var xv = px0 + (px1 - px0) * i / 60;
                var yc = fit.a * xv * xv + fit.b * xv + fit.c;
                d += (i ? 'L' : 'M') + sx(xv).toFixed(1) + ' ' + sy(yc).toFixed(1);
            }
            o += '<path d="' + d + '" fill="none" stroke="var(--ok)" stroke-width="2" opacity="0.85" clip-path="url(#gvc-' + ax + ')"/>';
        }
        if (fit && fit.vertex !== undefined && fit.vertex !== null) {
            var vx = sx(fit.vertex);
            o += '<line x1="' + vx + '" x2="' + vx + '" y1="' + T + '" y2="' + (H - B)
               + '" stroke="var(--ok)" stroke-width="1.5" stroke-dasharray="4 3"/>';
        }
        for (i = 0; i < pts.length; i++) {
            var last = (i === pts.length - 1 && !fit);
            o += '<circle cx="' + sx(pts[i][0]) + '" cy="' + sy(pts[i][1]) + '" r="' + (last ? 4.5 : 3.5)
               + '" fill="' + (pts[i][2] ? 'var(--warn)' : 'var(--accent)') + '"/>';
        }
        
        for (i = 0; i < pts.length; i++) {
            o += '<circle class="gv-hit" cx="' + sx(pts[i][0]) + '" cy="' + sy(pts[i][1]) + '" r="9" fill="transparent"'
               + ' data-v="' + pts[i][0] + '|' + pts[i][1] + '"/>';
        }
        if (best) {
            o += '<circle cx="' + sx(best[0]) + '" cy="' + sy(best[1]) + '" r="6" fill="none" stroke="var(--ok)" stroke-width="2"/>';
        }
        return o + '</svg>';
    }
    
    function gvTipShow(t) {
        var svg = t.ownerSVGElement, p = t.getAttribute('data-v').split('|');
        gvTipHide(svg);
        var cx = +t.getAttribute('cx'), cy = +t.getAttribute('cy');
        var txt = gvNum(+p[0], 1) + ' mm · ' + (+p[1]).toFixed(3) + ' A';
        var w = txt.length * 5.4 + 12, x = Math.min(Math.max(cx - w / 2, 2), 298 - w), y = cy - 26;
        if (y < 2) y = cy + 10;
        var g = document.createElementNS('http://www.w3.org/2000/svg', 'g');
        g.setAttribute('class', 'gv-tip');
        g.innerHTML = '<circle cx="' + cx + '" cy="' + cy + '" r="6" fill="none" stroke="var(--text)" stroke-width="1.5"/>'
            + '<rect x="' + x + '" y="' + y + '" width="' + w + '" height="17" rx="4" fill="var(--surface-2)" stroke="var(--accent)"/>'
            + '<text x="' + (x + w / 2) + '" y="' + (y + 12) + '" text-anchor="middle" style="fill:var(--text);font-size:10px">' + txt + '</text>';
        svg.appendChild(g);
    }
    function gvTipHide(svg) {
        var old = svg && svg.querySelector('.gv-tip');
        if (old) old.parentNode.removeChild(old);
    }
    ['mouseover', 'click'].forEach(function (ev) {
        document.addEventListener(ev, function (e) {
            var t = e.target;
            if (t && t.classList && t.classList.contains('gv-hit')) gvTipShow(t);
        });
    });
    document.addEventListener('mouseout', function (e) {
        var t = e.target;
        if (t && t.classList && t.classList.contains('gv-hit')) gvTipHide(t.ownerSVGElement);
    });

    function gvRender(j, d) {
        var g = j.grid, running = (d.jsweep === 1 && !g.end);
        var cur = g.pts.y.length ? 'y' : 'x';
        ['x', 'y'].forEach(function (ax) {
            var box = document.getElementById('gv-' + ax);
            var fit = g.fit[ax], best = g.best[ax], pts = g.pts[ax], cap;
            var active = running && ax === cur && !fit;
            box.className = 'gv-panel' + (active ? ' active' : '') + (fit ? ' done' : '');
            box.querySelector('.gv-title').textContent = 'แกน ' + ax.toUpperCase()
                + (active ? ' · กำลังวัด' : (fit ? ' · เสร็จ' : ''));
            box.querySelector('.gv-svg').innerHTML = gvPlot(ax, g);
            if (fit && best) {
                cap = 'จอดที่ ' + gvNum(best[0], 1) + ' mm · ' + best[1].toFixed(2) + ' A';
                if (fit.nofit) cap += ' (ฟิตไม่ได้ ใช้จุดที่วัดต่ำสุด)';
            } else if (fit) {
                cap = 'เลือก ' + gvNum(fit.vertex, 1) + ' mm';
            } else if (pts.length) {
                cap = 'วัดแล้ว ' + pts.length + ' จุด' + (g.ext[ax] ? ' · เดินต่อจากขอบ' : '');
            } else {
                cap = '\u00a0';
            }
            box.querySelector('.gv-cap').textContent = cap;
        });
        var before = null, i;
        for (i = 0; i < g.pts.x.length; i++) {
            if (Math.abs(g.pts.x[i][0] - g.base[0]) < 0.01) { before = g.pts.x[i][1]; break; }
        }
        var sum = document.getElementById('gv-sum'), html = '';
        if (g.end === 'done' && before !== null && g.final !== null) {
            var pct = (before - g.final) / before * 100;
            html = 'กระแส <b>' + before.toFixed(2) + ' → ' + g.final.toFixed(2) + ' A</b> ('
                 + (pct >= 0 ? 'ลด ' : 'เพิ่ม ') + Math.abs(pct).toFixed(0) + '%) · จอดที่ ('
                 + gvNum(g.pos[0], 1) + ', ' + gvNum(g.pos[1], 1) + ') mm จากจุดที่กล้องเล็ง';
        } else if (g.end === 'done') {
            html = 'เสร็จแล้ว · จอดที่ (' + gvNum(g.pos[0], 1) + ', ' + gvNum(g.pos[1], 1) + ') mm';
        } else if (g.end === 'cancel' || !running) {
            html = 'grid search หยุดก่อนจบ' + (before !== null ? ' · กระแสตอนเริ่ม ' + before.toFixed(2) + ' A' : '');
        } else {
            html = 'กำลังหาจุดที่ขดลวดตรงกันที่สุด... วัดแล้ว ' + (g.pts.x.length + g.pts.y.length) + ' จุด'
                 + (before !== null ? ' · กระแสตอนเริ่ม ' + before.toFixed(2) + ' A' : '');
        }
        if (g.fallback) html += ' · ถอยกลับจุดที่วัดได้ดีที่สุด';
        if (j.aim) html += '<br><span style="color:var(--warn)">จำลองกล้องเล็งพลาด (' + gvNum(j.aim[0], 0)
                         + ', ' + gvNum(j.aim[1], 0) + ') mm</span>';
        sum.innerHTML = html;
    }
    function gvTick(d) {
        var now = Date.now();
        if (d.cam === 0) gvFetch(now, d.state === 5 ? 1000 : 4000);
        var show = (d.cam === 0 && gvData && gvData.grid);
        var wrap = document.getElementById('video-wrap'), gv = document.getElementById('grid-view');
        wrap.classList.toggle('as-grid', !!show);
        
        wrap.classList.toggle('gv-nobtn', !!show && d.state === 5);
        gv.classList.toggle('show', !!show);
        if (d.cam === 0) document.getElementById('offline-msg').style.display = show ? 'none' : 'flex';
        document.getElementById('vh-title').textContent = show ? 'Grid search' : 'กล้องตรวจจับ ArUco';
        document.getElementById('vh-sub').textContent = show ? 'หาจุดที่กระแสต่ำสุด' : vhSub;
        if (show) {
            var key = JSON.stringify(gvData) + '|' + d.jsweep;
            if (key !== gvKey) { gvKey = key; gvRender(gvData, d); }
        }
    }
    var vhSub = '';

    function setAlert(id, txt) {
        var box = document.getElementById(id);
        if (txt) {
            document.getElementById(id + '-text').textContent = txt;
            box.classList.add('show');
        } else {
            box.classList.remove('show');
        }
    }

    
    var ATM_STEPS = ['เตรียมพร้อม (home + ไปจุดกลาง)', 'ค้นหา + เล็ง', 'เปิด inverter', 'รอกระแสนิ่ง', 'grid search'];
    var atmHist = [], atmSkip = 0, atmCur = 0;
    try { atmHist = JSON.parse(localStorage.getItem('evAtm') || '[]') || []; } catch (e) { atmHist = []; }
    try { atmSkip = parseFloat(localStorage.getItem('evAtmSkip') || '0') || 0; } catch (e) { atmSkip = 0; }
    /* [22 ต.ค.] ทิ้งรอบที่บันทึกผิดจากบั๊กแรก (เล็งพลาดแล้ว grid ขึ้นเป็นพันล้านวิ) */
    atmHist = atmHist.filter(function (e) { return e.parts.every(function (v) { return v < 36000; }); });
    function atmFmt(s) {
        var m = Math.floor(s / 60), r = s - m * 60;
        return m > 0 ? m + ' นาที ' + r.toFixed(1) + ' วิ' : s.toFixed(1) + ' วิ';
    }
    function atmXY(p) { return '(' + p[0].toFixed(1) + ', ' + p[1].toFixed(1) + ')'; }
    function atmSave() {
        try { localStorage.setItem('evAtm', JSON.stringify(atmHist)); localStorage.setItem('evAtmSkip', String(atmSkip)); } catch (e) {}
    }
    function atmDrawHist() {
        var h = '';
        for (var i = 0; i < atmHist.length; i++) {
            var e = atmHist[i];
            var pos = '';
            if (e.pb && e.pa) {
                pos = '<br>ขดลวด ' + atmXY(e.pb) + ' &rarr; ' + atmXY(e.pa) + ' mm'
                    + ' (เลื่อน ' + (e.pa[0] - e.pb[0] >= 0 ? '+' : '') + (e.pa[0] - e.pb[0]).toFixed(1)
                    + ', ' + (e.pa[1] - e.pb[1] >= 0 ? '+' : '') + (e.pa[1] - e.pb[1]).toFixed(1) + ')';
            }
            h += '<div class="atm-item"><div>#' + (i + 1) + ' ' + e.when + ' &middot; <b>' + atmFmt(e.total) + '</b>'
               + (e.ok ? '' : ' (หยุดกลางคัน)') + ' &middot; ' + e.parts.map(function (v) { return v.toFixed(1); }).join(' / ')
               + pos + '</div><button class="atm-del" onclick="atmDel(' + i + ')">&#10005; ลบ</button></div>';
        }
        document.getElementById('atm-hist').innerHTML = h
            ? 'ประวัติ (วิ: เตรียม / เล็ง / เปิดไฟ / รอนิ่ง / grid · ขดลวดวัดจากมุมกล่อง ก่อน &rarr; หลัง grid):' + h : '';
    }
    function atmDel(i) {
        var e = atmHist[i];
        if (!e) return;
        if (e.t0 === atmCur) { atmSkip = atmCur; }   /* รอบล่าสุดของ Pi — กันถูกเพิ่มกลับ */
        atmHist.splice(i, 1);
        atmSave(); atmDrawHist();
    }
    
    function atmExport() {
        if (!atmHist.length) { alert('ยังไม่มีประวัติให้บันทึก'); return; }
        var NL = String.fromCharCode(13, 10), TB = String.fromCharCode(9);
        var f = function (v) { return (typeof v === 'number') ? v.toFixed(1) : (v || ''); };
        var head = ['รอบ', 'เวลาเริ่ม', 'รวม (วิ)', 'เตรียมพร้อม', 'ค้นหา+เล็ง', 'เปิด inverter', 'รอนิ่ง', 'grid search', 'สถานะ',
                    'ขดลวดก่อน X', 'ขดลวดก่อน Y', 'ขดลวดหลัง X', 'ขดลวดหลัง Y', 'เลื่อน X', 'เลื่อน Y'];
        var lines = ['จับเวลาโหมดอัตโนมัติ (วินาที · พิกัดขดลวด mm วัดจากมุมกล่อง)', head.join(TB)];
        for (var i = 0; i < atmHist.length; i++) {
            var e = atmHist[i], pb = e.pb || [], pa = e.pa || [];
            var row = [String(i + 1), e.when, e.total].concat(e.parts.map(function (v) { return v || ''; })).concat([e.ok ? 'จบครบ' : 'หยุดกลางคัน',
                       pb[0], pb[1], pa[0], pa[1],
                       (e.pb && e.pa) ? pa[0] - pb[0] : '', (e.pb && e.pa) ? pa[1] - pb[1] : '']);
            lines.push(row.map(f).join(TB));
        }
        var blob = new Blob([String.fromCharCode(65279) + lines.join(NL) + NL], { type: 'text/plain;charset=utf-8' });
        var d = new Date(), z = function (n) { return ('0' + n).slice(-2); };
        var a = document.createElement('a');
        a.href = URL.createObjectURL(blob);
        a.download = 'auto_time_' + d.getFullYear() + z(d.getMonth() + 1) + z(d.getDate()) + '_' + z(d.getHours()) + z(d.getMinutes()) + '.txt';
        document.body.appendChild(a); a.click(); a.remove();
        setTimeout(function () { URL.revokeObjectURL(a.href); }, 2000);
    }
    function atmClear() {
        atmHist = [];
        if (atmCur) { atmSkip = atmCur; }
        atmSave(); atmDrawHist();
    }
    function atmShow(d) {
        var t = d.atm, st = d.ast;
        if (!t || !st || !t[0]) { document.getElementById('atm-total').textContent = 'ยังไม่ได้กดอัตโนมัติ'; return; }
        atmCur = t[0];
        var end = (st === 1) ? d.now : t[6];
        var total = Math.max(0, end - t[0]);
        var tag = st === 1 ? ' (กำลังนับ)' : (st === 2 ? ' ✓ จบ' : ' (หยุด/ไม่สำเร็จ)');
        document.getElementById('atm-total').textContent = atmFmt(total) + tag;
        var rows = '', prev = t[0], parts = [];
        for (var i = 1; i <= 5; i++) {
            var v = '', dur = 0;
            if (t[i] && prev) { dur = t[i] - prev; v = dur.toFixed(1) + ' วิ'; prev = t[i]; }
            else if (st === 1 && prev) { dur = d.now - prev; v = dur.toFixed(1) + ' วิ …'; prev = 0; }
            else { v = '&mdash;'; prev = 0; }
            parts.push(dur);
            rows += '<div class="fact"><span>' + ATM_STEPS[i - 1] + '</span><b>' + v + '</b></div>';
        }
        /* พิกัดขดลวดก่อน/หลัง grid (วัดจากมุมกล่อง แบบเดียวกับหน้าหลัก) */
        var pb = null, pa = null;
        if (t[4] && d.apos) { pb = [d.apos[0] + d.cox, d.apos[1] + d.coy]; }
        if (t[5] && t[4] && d.apos) { pa = [d.apos[2] + d.cox, d.apos[3] + d.coy]; }
        rows += '<div class="fact"><span>ขดลวดก่อน &rarr; หลัง grid (mm)</span><b>'
              + (pb ? atmXY(pb) + ' &rarr; ' + (pa ? atmXY(pa) : '…') : '&mdash;') + '</b></div>';
        document.getElementById('atm-rows').innerHTML = rows;
        
        if (st !== 1 && t[0] !== atmSkip && !atmHist.some(function (e) { return e.t0 === t[0]; })) {
            var dt = new Date(t[0] * 1000);
            var when = ('0' + dt.getHours()).slice(-2) + ':' + ('0' + dt.getMinutes()).slice(-2);
            atmHist.push({ t0: t[0], when: when, total: total, ok: st === 2, parts: parts, pb: pb, pa: pa });
            if (atmHist.length > 20) atmHist.shift();
            atmSave(); atmDrawHist();
        }
    }
    atmDrawHist();

    function updateStatus() {
        fetch('/status').then(function (r) { return r.json(); }).then(function (d) {
            var s = STATES[d.state] || STATES[0];
            var busy     = (d.state === 1 || d.state === 3);
            var charging = (d.state === 5);
            var autoRun  = (d.auto === 1);

            
            var lockMode = busy || charging || autoRun;
            document.getElementById('mode-auto').disabled  = lockMode;
            document.getElementById('mode-stage').disabled = lockMode;

            /* ---- แถบสถานะ ---- */
            var pill = document.getElementById('pill');
            pill.style.color = s.color;
            pill.style.borderColor = s.color + '55';
            document.getElementById('status-text').textContent = s.text;
            document.getElementById('pill-dot').className = s.live ? 'dot live' : 'dot';

            var lchip = document.getElementById('link-chip');
            if (d.serial === 1) {
                document.getElementById('link-text').textContent = charging ? 'INVERTER ทำงาน' : 'Arduino เชื่อมต่อแล้ว';
                lchip.style.color = charging ? '#fbbf24' : '#34d399';
            } else {
                document.getElementById('link-text').textContent = 'ไม่พบบอร์ด Arduino';
                lchip.style.color = '#f87171';
            }
            lchip.style.borderColor = lchip.style.color + '55';

            /* ---- ลำดับการทำงาน ---- */
            for (var i = 1; i <= 5; i++) {
                var el = document.getElementById('fs-' + i);
                el.className = 'fstep';
                if (d.state === 99) continue;
                if (d.state > i)   el.classList.add('done');
                if (d.state === i) el.classList.add('on');
            }

            /* ---- ปุ่ม ---- */
            var gridOK = charging && d.inv === 1 && d.jbusy === 0;
            var able = {
                b1:  !busy && !charging,
                b2:  !busy && !charging,
                b4:  (d.state === 4 && d.serial === 1),
                b16: gridOK,
                b5:  charging,
                b20: !busy && !charging && !autoRun && d.serial === 1,
                b5a: charging
            };
            Object.keys(able).forEach(function (id) {
                var b = document.getElementById(id);
                b.disabled = !able[id];
                b.classList.remove('next');
            });
            var nb = document.getElementById('b' + s.next);
            if (nb && !nb.disabled) nb.classList.add('next');
            if (uiMode === 'auto') {
                var ab = document.getElementById('b20');
                if (!ab.disabled) ab.classList.add('next');
            }

            /* ---- ป้ายบอกความคืบหน้าของโหมดอัตโนมัติ ---- */
            var asBox = document.getElementById('auto-stat');
            var asTxt, asRun = true;
            if (autoRun && d.state === 1)                  { asTxt = 'ขั้นที่ 1/4 — กำลังเตรียมเครื่อง (หาจุด home แล้วไปจุดกลาง)'; }
            else if (autoRun && d.state === 3)             { asTxt = 'ขั้นที่ 2/4 — กำลังค้นหารถและเล็งขดลวด'; }
            else if (autoRun && d.state === 4)             { asTxt = 'ขั้นที่ 3/4 — เล็งสำเร็จแล้ว กำลังเปิด inverter'; }
            else if (charging && d.sti > 0)                { asTxt = 'โหมดผู้พัฒนา: เก็บผลทีละจุด ' + d.sti + '/' + d.stn + (d.stw === 1 ? ' — รอคุณจดแล้วกดถัดไป' : ' — กำลังเลื่อน'); }
            else if (autoRun && charging && d.jbusy === 1) { asTxt = 'ขั้นที่ 4/4 — กำลังปรับตำแหน่งให้ได้ coupling ดีที่สุด'; }
            else if (autoRun && charging)                  { asTxt = 'กำลังจ่ายไฟ — กดหยุดจ่ายไฟเมื่อต้องการเลิก'; }
            else if (charging)        { asTxt = 'กำลังจ่ายไฟอยู่ (สั่งไว้จากโหมดทีละขั้น)'; }
            else if (busy)            { asTxt = 'เครื่องกำลังทำงานอยู่ — รอให้จบก่อนสั่งอัตโนมัติ'; }
            else if (d.serial !== 1)  { asTxt = 'ยังไม่พบบอร์ด Arduino — เสียบสาย USB แล้วรอสักครู่ ระบบจะต่อเอง'; asRun = false; }
            else                      { asTxt = ''; asRun = false; }   /* [23 ก.ย.] ตอนว่างไม่ต้องมีข้อความ — ซ่อนกล่องไปเลย */
            asBox.textContent = asTxt;
            asBox.className = 'autostat' + (asRun ? ' run' : '');
            asBox.style.display = asTxt ? 'block' : 'none';

            document.getElementById('b16-sub').textContent = (charging && d.jbusy === 1)
                ? 'กำลังทำงาน... ปล่อยให้มันวัดจนจบ'
                : 'วัดกระแสหลายจุดแล้วหาจุด coupling ดีสุด';

            document.getElementById('cam-btn').disabled = charging;
            document.getElementById('cam-off-sub').textContent = charging
                ? 'ปิดกล้องอัตโนมัติระหว่างจ่ายไฟ เพื่อความปลอดภัย'
                : 'กล้องจะเปิดเองตอนค้นหา · กดปุ่มมุมขวาล่างถ้าอยากเปิดดูเอง';

            /* ---- แจ้งเตือน ---- */
            setAlert('alert-evt', (d.evt && EVENTS[d.evt]) ? EVENTS[d.evt] : '');
            setAlert('alert-pos', d.pmsg ? d.pmsg : '');
            
            var aimTxt = '(' + (d.aex >= 0 ? '+' : '') + d.aex.toFixed(1) + ', '
                       + (d.aey >= 0 ? '+' : '') + d.aey.toFixed(1) + ') mm';
            var aimOn = (d.aex !== 0 || d.aey !== 0);
            setAlert('alert-aim', aimOn ? 'โหมดผู้พัฒนา: กำลังจำลองกล้องเล็งพลาด ' + aimTxt
                                         + ' — ล้างค่าในโหมดผู้พัฒนาเมื่อทดลองเสร็จ' : '');
            document.getElementById('dev-aim-now').textContent = aimOn ? aimTxt : 'ปิด (0, 0)';
            atmShow(d);
            if (d.aemax) { aimMax = d.aemax; }

            /* ---- ค่าวัด ---- */
            
            var tnow = Date.now();
            
            var sk = d.sti + ':' + d.stw;
            if (d.sti > 0 && (d.stw !== 1 || sk !== stepKey)) { ampBuf = []; ampT0 = 0; }
            stepKey = sk;
            if (d.inv === 1 && !(d.sti > 0 && d.stw !== 1)) {
                if (ampT0 === 0) { ampT0 = tnow; }
                ampBuf.push([tnow, d.idc]);
                ampBuf = ampBuf.filter(function (x) { return tnow - x[0] <= 5000; });
            } else {
                ampBuf = []; ampT0 = 0;
            }
            var ampLive = document.getElementById('amp-live');
            if (d.inv === 1 && tnow - ampT0 >= 5000) {
                /* ครบ 5 วิแล้ว = ค่าเฉลี่ยเต็มหน้าต่าง พร้อมจด */
                var sum = 0;
                ampBuf.forEach(function (x) { sum += x[1]; });
                document.getElementById('amp-value').textContent = (sum / ampBuf.length).toFixed(2);
                ampLive.textContent = '\u2713 พร้อมจด · เฉลี่ย 5 วิ · ค่าสด ' + d.idc.toFixed(3) + ' A';
                ampLive.style.color = 'var(--ok)';
            } else if (d.inv === 1) {
                /* ยังไม่ครบ 5 วิ (เพิ่งเปิด inverter หรือเพิ่งเปิดหน้าเว็บ) — โชว์ค่าสด + นับถอยหลัง */
                var left = Math.max(1, Math.ceil((5000 - (tnow - ampT0)) / 1000));
                document.getElementById('amp-value').textContent = d.idc.toFixed(3);
                ampLive.textContent = 'กำลังเก็บค่าเฉลี่ย... อีก ' + left + ' วิ (ยังไม่ต้องจด)';
                ampLive.style.color = 'var(--warn)';
            } else {
                document.getElementById('amp-value').textContent = d.idc.toFixed(3);
                ampLive.textContent = '\u00a0';
                ampLive.style.color = '';
            }
            document.getElementById('amp-box').className = 'amp' + (d.inv === 1 ? ' hot' : '');
            document.getElementById('telem-mode').textContent = charging ? 'กำลังวัดสด' : 'พร้อมใช้งาน';

            document.getElementById('f-pos').textContent = (d.pok === 1)
                ? ('X ' + (d.px + d.cox).toFixed(1) + ' , Y ' + (d.py + d.coy).toFixed(1) + ' mm')
                : 'ยังไม่ทราบ (ต้องเตรียมเครื่องก่อน)';
            document.getElementById('f-jog').textContent = charging
                ? ('X ' + d.jx.toFixed(1) + ' , Y ' + d.jy.toFixed(1) + ' mm')
                : '\\u2014';

            
            ocNow = d.oc;
            var ocTxt = (d.oc > 0) ? (d.oc.toFixed(2) + ' A') : (d.serial === 1 ? 'กำลังอ่าน...' : '\u2014');
            document.getElementById('f-oc').textContent = ocTxt;
            document.getElementById('dev-oc-now').textContent = ocTxt;
            document.getElementById('dev-oc-btn').disabled = (d.serial !== 1);
            if (d.ocerr === 1) {
                devWant = 0;
                devMsg('Arduino ปฏิเสธค่านี้ (ต้องมากกว่า 0)', 'bad');
            } else if (d.ocpend === 1) {
                devMsg('รอส่ง — เครื่องกำลังทำงานอื่นอยู่ จะส่งให้เองเมื่อว่าง', '');
            } else if (d.serial !== 1) {
                devMsg('ยังไม่พบบอร์ด Arduino', 'bad');
                devNoBoard = true;
            } else {
                var dm = document.getElementById('dev-msg');
                
                if (devNoBoard) { devNoBoard = false; dm.className = 'dev-msg'; }
                if (devWant > 0 && Math.abs(d.oc - devWant) < 0.005) {
                    devWant = 0;
                    devMsg('Arduino ยืนยันแล้ว: ' + ocTxt, 'good');
                } else if (devWant > 0) {
                    devMsg('กำลังส่งไปที่ Arduino...', '');
                } else if (!dm.className.match(/bad|good/)) {
                    devMsg('ใส่ค่าใหม่ (A) แล้วกดตั้งค่า · ค่าจะถูกจำไว้แม้รีสตาร์ต', '');
                }
            }

            
            var stBox = document.getElementById('step-stat');
            if (d.sti > 0) {
                var ax = (d.sti <= d.stn / 2) ? 'X' : 'Y';
                var where = 'จุด <b>' + d.sti + '/' + d.stn + '</b> · แกน ' + ax
                          + ' · X ' + d.jx.toFixed(1) + ' , Y ' + d.jy.toFixed(1) + ' mm';
                if (d.stw === 1) {
                    stBox.innerHTML = where + '<br>' + document.getElementById('amp-value').textContent + ' A · '
                                    + document.getElementById('amp-live').textContent;
                    stBox.className = 'step-stat wait';
                } else {
                    stBox.innerHTML = where + '<br>กำลังเลื่อน/วัด... รอสักครู่';
                    stBox.className = 'step-stat';
                }
            } else {
                stBox.textContent = charging ? 'พร้อม — กดเริ่มได้' : 'ต้องกำลังจ่ายไฟอยู่ก่อน';
                stBox.className = 'step-stat';
            }
            document.getElementById('step-start').disabled = !(charging && d.inv === 1 && d.jbusy === 0 && d.sti === 0);
            document.getElementById('step-next').disabled  = !(d.sti > 0 && d.stw === 1);
            document.getElementById('step-prev').disabled  = !(d.sti > 1 && d.stw === 1);
            document.getElementById('step-quit').disabled  = !(d.sti > 0 && d.stw === 1);

            document.getElementById('r-vdiff').textContent = d.vdiff.toFixed(4) + ' V';
            document.getElementById('r-ipp').textContent   = d.ipp.toFixed(3) + ' A';
            document.getElementById('r-vout').textContent  = d.vout.toFixed(3) + ' V';
            var vr = document.getElementById('r-vref');
            vr.textContent = d.vref.toFixed(3) + ' V';
            var vrBad = (d.serial === 1 && (d.vref < 2.40 || d.vref > 2.60));
            vr.className = vrBad ? 'bad' : '';
            document.getElementById('r-tip').textContent = vrBad
                ? 'Vref ผิดปกติ (ควรอยู่ราว 2.49 V) มักแปลว่าแจ็ค DC ของ Arduino ไม่ได้จ่ายไฟ ทำให้กระแสที่อ่านได้เพี้ยนทั้งหมด'
                : 'Vref ปกติต้องอยู่ราว 2.49 V';

            /* ---- ภาพจากกล้อง ---- */
            var img   = document.getElementById('cam-stream');
            var off   = document.getElementById('offline-msg');
            var badge = document.getElementById('live-badge');
            if (d.cam === 1 && camShown === 0) {
                off.style.display = 'none';
                img.style.display = 'block';
                badge.style.display = 'flex';
                camLoop = true; camNext();
                camShown = 1;
            } else if (d.cam === 0 && camShown === 1) {
                img.style.display = 'none';
                badge.style.display = 'none';
                off.style.display = 'flex';
                camLoop = false;
                camShown = 0;
            }
            gvTick(d);   
            devBootCheck(d.boot);
            devNetTick();
        });
    }
    vhSub = document.getElementById('vh-sub').textContent;
    applyMode();
    devToggle(devOn);
    setInterval(updateStatus, 300);
    updateStatus();
    camNext();
</script>
</body>
</html>
"""

# ==========================================
# 📸 Process กล้อง (Core 1)
# ==========================================
def camera_task(is_vis, en_vis, m_x, m_y, m_id, m_angle, is_run, q_frame):
    print(f"🎥 [Core 1] ระบบกล้องพร้อมรันแบบมุมกว้าง Full-FOV ({CAM_W}x{CAM_H})...")
    picam2 = Picamera2()

    config = picam2.create_preview_configuration(
        main={"size": (CAM_W, CAM_H), "format": "BGR888"},
        raw={"size": (2304, 1296)},
    )
    picam2.configure(config)

    picam2.set_controls({
        "AfMode": 0,
        "LensPosition": 15.0,
        "AeEnable": False,
        "ExposureTime": 10000,   # 10ms -> motion blur ~3px ตอน spiral (41.7 mm/s)
        "AnalogueGain": 2.0,
    })
    
    is_cam_started = False
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    parameters.adaptiveThreshConstant = 3  
    parameters.polygonalApproxAccuracyRate = 0.05  
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 23
    parameters.adaptiveThreshWinSizeStep = 10

    while is_run.value == 1:
        if en_vis.value == 1:
            if not is_cam_started:
                picam2.start(); is_cam_started = True
            
            image = picam2.capture_array()
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR) 
            image = cv2.flip(image, -1) 
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            idx = None
            if ids is not None:
                flat = [int(v) for v in ids.flatten()]
                idx = next((i for i, v in enumerate(flat) if v in VALID_MARKER_IDS), None)

            if idx is not None:
                is_vis.value = 1
                m_id.value = flat[idx]
                c = corners[idx][0]
                m_x.value = int((c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4.0)
                m_y.value = int((c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4.0)

                dx_edge = c[1][0] - c[0][0]
                dy_edge = c[1][1] - c[0][1]
                m_angle.value = math.atan2(dy_edge, dx_edge)
            else:
                is_vis.value = 0

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(image, corners, ids)
            cv2.circle(image, (CAM_CX, CAM_CY), 5, (0, 0, 255), -1)

            ret, buffer = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 70])
            
            if ret:
                if q_frame.full():
                    try: q_frame.get_nowait()
                    except: pass
                q_frame.put(buffer.tobytes())
        else:
            if is_cam_started:
                picam2.stop(); is_cam_started = False
                is_vis.value = 0
            time.sleep(0.1)

    if is_cam_started: picam2.stop()

# ==========================================
# 🎯 Process มอเตอร์ (Core 2)
# ==========================================
def motor_task(cmd_var, state_var, is_vis, en_vis, m_x, m_y, m_id, m_angle, is_run, ard, pos, s_auto):
    (s_ok, s_inv, s_idc, s_ipp, s_vdiff, s_vref, s_vout, s_evt,
     s_jx, s_jy, s_jbusy, s_jsweep, s_ocreq, s_oclim, s_ocerr,
     s_sti, s_stn, s_stw, s_aex, s_aey) = ard
    aim_err_used = {"xy": None}
    (s_px, s_py, s_pok, s_pmsg, s_aimt, s_atm) = pos

    # ==========================================
    # ==========================================
    ser = None
    rx_buf = bytearray()
    ard_h = {"last_try": 0.0, "announced": False}
    ARD_RECONNECT_S = 2.0

    def ard_open(first=False):
        nonlocal ser
        ard_h["last_try"] = time.time()
        try:
            ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0)  # timeout=0 = อ่านแบบไม่บล็อก
            time.sleep(ARDUINO_BOOT_WAIT)   # เปิดพอร์ต -> DTR รีเซ็ตบอร์ด -> ต้องรอ bootloader
            ser.reset_input_buffer()
            rx_buf.clear()
            s_ok.value = 1
            oc_h["push"] = True
            ard_h["announced"] = False
            print("🔌 เชื่อมต่อ Arduino สำเร็จ" if first else "🔌 เชื่อมต่อ Arduino ใหม่สำเร็จ (บอร์ดถูกรีเซ็ต inverter ดับอยู่)")
            return True
        except Exception as e:
            ser = None
            s_ok.value = 0
            if first:
                print(f"⚠️  เปิดพอร์ต Arduino ไม่ได้ ({e}) — ระบบเล็งเป้ายังใช้ได้ แต่สั่ง inverter ไม่ได้ · จะลองใหม่เองเมื่อเห็นพอร์ต")
            return False

    def ard_drop(why):
        nonlocal ser
        s_ok.value = 0
        if ser is not None:
            try: ser.close()
            except Exception: pass
        ser = None
        if not ard_h["announced"]:
            ard_h["announced"] = True
            print(f"⚠️  Arduino หลุด ({why}) — จะเชื่อมต่อใหม่เองเมื่อเสียบสายกลับ (ตอนไม่ได้ชาร์จ)")

    def ard_reconnect_tick():
        if ser is not None: return
        if time.time() - ard_h["last_try"] < ARD_RECONNECT_S: return
        if not os.path.exists(SERIAL_PORT):
            ard_h["last_try"] = time.time(); return
        ard_open()

    oc_h = {"want": None, "push": False}
    try:
        with open(DEV_SETTINGS_FILE) as f:
            v = float(json.load(f).get("oc_limit_a"))
        if v > 0:
            oc_h["want"] = v
    except Exception:
        pass

    def oc_push():
        s_oclim.value = 0.0   # ยังไม่รู้ จนกว่าจะได้ ACK,OC
        ard_send(f"OC,{oc_h['want']:.2f}" if oc_h["want"] is not None else "OC?")

    def oc_save(v):
        try:
            with open(DEV_SETTINGS_FILE, "w") as f:
                json.dump({"oc_limit_a": v}, f)
        except Exception as e:
            print(f"⚠️  บันทึกลิมิตกระแสลงไฟล์ไม่ได้ ({e}) — ใช้ได้จนกว่าบอร์ดจะรีเซ็ต")

    ard_open(first=True)

    log_h = {"f": None, "w": None, "t0": 0.0}
    # ผลของคำสั่ง ZERO ล่าสุด: 0=ยังไม่ตอบ 1=ACK 2=ERR (inverter ติดอยู่)
    zero_h = {"ack": 0}
    telem_h = {"n": 0}

    def log_open():
        try:
            os.makedirs(LOG_DIR, exist_ok=True)
            path = os.path.join(LOG_DIR, time.strftime("charge_%Y%m%d_%H%M%S.csv"))
            f = open(path, "w", newline="")
            w = csv.writer(f)
            w.writerow(["time", "elapsed_s", "event",
                        "inv", "idc_A", "ipp_A", "vdiff_V", "vpp_V", "vref_V", "vout_V",
                        "jog_x_mm", "jog_y_mm"])
            log_h["f"], log_h["w"], log_h["t0"] = f, w, time.time()
            print(f"📝 เริ่มบันทึกค่ากระแส: {path}")
        except Exception as e:
            log_h["f"] = log_h["w"] = None
            print(f"⚠️  เปิดไฟล์ log ไม่ได้ ({e}) — ชาร์จต่อได้แต่ข้อมูลจะไม่ถูกบันทึก")

    def log_row(event, telem=None):
        """telem = list 8 ช่องจากบรรทัด D (หรือ None สำหรับแถวเหตุการณ์)"""
        if log_h["w"] is None: return
        try:
            now = time.time()
            row = [f"{now:.3f}", f"{now - log_h['t0']:.3f}", event]
            row += telem[1:] if telem else [s_inv.value, s_idc.value, "", "", "", "", ""]
            row += [f"{s_jx.value:.2f}", f"{s_jy.value:.2f}"]
            log_h["w"].writerow(row)
            log_h["f"].flush()   # flush ทุกแถว — โปรแกรมตาย/ไฟดับข้อมูลก็ไม่หาย (10 แถว/วิ เขียนไหวสบาย)
        except Exception as e:
            print(f"⚠️  เขียน log ไม่ได้: {e}")
            log_h["f"] = log_h["w"] = None

    def log_close():
        if log_h["f"] is None: return
        try:
            log_h["f"].close()
            print("📝 ปิดไฟล์ log แล้ว")
        except Exception:
            pass
        log_h["f"] = log_h["w"] = None

    def ard_send(cmd):
        if ser is None: return
        try:
            ser.write((cmd + "\n").encode())
        except Exception as e:
            ard_drop(f"ส่งไม่ได้: {e}")

    def ard_poll():
        if ser is None: return
        if oc_h["push"]:
            oc_h["push"] = False
            oc_push()
        req = s_ocreq.value
        if req > 0:
            s_ocreq.value = 0.0
            ard_send(f"OC,{req:.2f}")
        try:
            data = ser.read(4096)
        except Exception as e:
            ard_drop(f"อ่านไม่ได้: {e}")
            return
        if not data: return
        rx_buf.extend(data)
        while b"\n" in rx_buf:
            raw, _, rest = bytes(rx_buf).partition(b"\n")
            rx_buf.clear(); rx_buf.extend(rest)
            line = raw.decode("utf-8", "ignore").strip()
            if not line: continue

            # D,<inv>,<idc>,<ipp>,<vdiff>,<vpp>,<vref>,<vout>
            if line.startswith("D,"):
                p = line.split(",")
                if len(p) == 8:
                    try:
                        s_inv.value = int(p[1])
                        s_idc.value = float(p[2])
                        s_ipp.value = float(p[3])
                        s_vdiff.value = float(p[4])
                        s_vref.value = float(p[6])
                        s_vout.value = float(p[7])
                        telem_h["n"] += 1
                        log_row("", p)   # แถวข้อมูล (เขียนเฉพาะตอน log เปิดอยู่)
                    except ValueError:
                        pass
            elif line.startswith("ACK,ZERO"):
                zero_h["ack"] = 1
                print(f"⚖️  auto-zero สำเร็จ ({line})")   # ท้ายบรรทัดมีค่า offset เป็น A
                log_row(line)
            elif line == "ERR,ZERO_WHILE_ON":
                zero_h["ack"] = 2
                print("⚠️  Arduino ปฏิเสธ ZERO: inverter ทำงานอยู่")
                log_row(line)
            elif line.startswith("ACK,OC,"):
                try:
                    v = float(line.split(",")[2])
                    s_oclim.value = v; s_ocerr.value = 0
                    if oc_h["want"] != v:
                        oc_h["want"] = v; oc_save(v)
                        print(f"⚙️  ตั้งลิมิตกระแสเกินเป็น {v:.2f} A แล้ว (Arduino ยืนยัน)")
                except (ValueError, IndexError):
                    pass
                log_row(line)
            elif line == "ERR,OC_RANGE":
                s_ocerr.value = 1
                print("⚠️  Arduino ปฏิเสธลิมิตกระแส (ต้องมากกว่า 0)")
                log_row(line)
            elif line.startswith("BOOT,"):
                print(f"🔄 Arduino รีบูต ({line}) — ส่งลิมิตกระแสซ้ำ")
                oc_push()
                log_row(line)
            elif line == "EVT,OC":
                s_evt.value = 1; print("🛑 Arduino ตัดไฟเอง: กระแสเกินพิกัด")
                log_row(line)
            elif line == "EVT,WDT":
                s_evt.value = 2; print("🛑 Arduino ตัดไฟเอง: ไม่ได้ยิน PING จาก Pi")
                log_row(line)
            elif line == "EVT,BTN_OFF":
                s_evt.value = 3; print("⏹️  มีคนกดปุ่ม OFF ที่ตัวเครื่อง")
                log_row(line)
            elif line.startswith(("ACK,", "EVT,", "BOOT,", "ERR,")):
                if line.startswith("ERR,"):
                    print(f"⚠️  Arduino ปฏิเสธคำสั่ง: {line}")
                log_row(line)

    pi = pigpio.pi()
    pi.set_mode(X_STEP, pigpio.OUTPUT); pi.set_mode(X_DIR, pigpio.OUTPUT); pi.set_mode(X_EN, pigpio.OUTPUT)
    pi.set_mode(Y_STEP, pigpio.OUTPUT); pi.set_mode(Y_DIR, pigpio.OUTPUT); pi.set_mode(Y_EN, pigpio.OUTPUT)
    pi.write(X_EN, 1); pi.write(Y_EN, 1)
    pi.set_mode(LIMIT_X, pigpio.INPUT); pi.set_pull_up_down(LIMIT_X, pigpio.PUD_UP)
    pi.set_mode(LIMIT_Y, pigpio.INPUT); pi.set_pull_up_down(LIMIT_Y, pigpio.PUD_UP)

    def limit_hit(pin):
        if pi.read(pin) == 0: return False
        n = LIMIT_DEBOUNCE_MS
        hi = 1
        for _ in range(n - 1):
            time.sleep(0.001)
            hi += pi.read(pin)
        return hi >= int(n * 0.8)

    pi.wave_clear() 
    pi.wave_add_generic([pigpio.pulse(1<<X_STEP, 0, SPEED_US), pigpio.pulse(0, 1<<X_STEP, SPEED_US)])
    WAVE_X = pi.wave_create()
    pi.wave_add_generic([pigpio.pulse(1<<Y_STEP, 0, SPEED_US), pigpio.pulse(0, 1<<Y_STEP, SPEED_US)])
    WAVE_Y = pi.wave_create()
    pi.wave_add_generic([pigpio.pulse((1<<X_STEP)|(1<<Y_STEP), 0, SPEED_US), pigpio.pulse(0, (1<<X_STEP)|(1<<Y_STEP), SPEED_US)])
    WAVE_XY = pi.wave_create()

    DIR_INVERT_X = 1
    DIR_INVERT_Y = 1
    
    COIL_OFFSET_X_MM = 0.0
    COIL_OFFSET_Y_MM = -70.0
    CAR_OFFSET_MM = 48.0

    limit_tripped = [False]  # กันก้นหอยวิ่งชนขอบราง (ไม่มี limit switch ฝั่งขอบไกล แต่กันฝั่งกลับบ้านได้)

    # ==========================================
    # 📍 Position tracking — ตัวนับพิกัดสัมบูรณ์
    # ==========================================
    STEP_RATE = 1.0 / (2 * SPEED_US * 1e-6)   # สเต็ป/วินาที ของ wave

    def pos_set_msg(text=""):
        try:
            s_pmsg.value = text.encode("utf-8")[:255]
        except Exception:
            pass

    def pos_home():
        """แตะลิมิตครบทั้งสองแกน = จุดอ้างอิงของระบบพิกัดทั้งหมด"""
        s_px.value = 0.0
        s_py.value = 0.0
        s_pok.value = 1

    def pos_add(axis, direction, steps):
        if steps <= 0:
            return
        mm = steps / STEPS_PER_MM
        if direction == 1:            # DIR 1 = เข้าหา home
            mm = -mm
        if axis == 'x':
            s_px.value += mm
        else:
            s_py.value += mm

    def pos_after_wave(axis, direction, planned, t0, completed):
        done = planned if completed else min(planned, (time.time() - t0) * STEP_RATE)
        pos_add(axis, direction, done)

    def pos_snap_limits():
        if limit_hit(LIMIT_X):
            if s_pok.value == 1 and abs(s_px.value) > 0.05:
                print(f"📍 แตะลิมิต X: ตัวนับอ่านได้ {s_px.value:+.2f} mm "
                      f"(ความจริงคือ 0) -> คลาด {abs(s_px.value):.2f} mm · รีเซ็ตให้แล้ว")
            s_px.value = 0.0
        if limit_hit(LIMIT_Y):
            if s_pok.value == 1 and abs(s_py.value) > 0.05:
                print(f"📍 แตะลิมิต Y: ตัวนับอ่านได้ {s_py.value:+.2f} mm "
                      f"(ความจริงคือ 0) -> คลาด {abs(s_py.value):.2f} mm · รีเซ็ตให้แล้ว")
            s_py.value = 0.0

    def move_axis_blind(d_pin, direction, wave_id, steps):
        steps = int(steps)
        if steps <= 0: return True
        axis = 'x' if d_pin == X_DIR else 'y'
        pi.write(d_pin, direction)
        t0 = time.time()
        pi.wave_chain([255, 0, wave_id, 255, 1, steps & 255, steps >> 8])
        while pi.wave_tx_busy():
            if cmd_var.value == 99:
                pi.wave_tx_stop()
                pos_after_wave(axis, direction, steps, t0, False)
                return False
            if limit_hit(LIMIT_X) or limit_hit(LIMIT_Y):
                pi.wave_tx_stop(); limit_tripped[0] = True
                pos_after_wave(axis, direction, steps, t0, False)
                pos_snap_limits()
                return False
            time.sleep(0.005)
        pos_after_wave(axis, direction, steps, t0, True)
        return True

    def move_xy_blind(steps_x, dir_x, steps_y, dir_y):
        steps_x, steps_y = int(steps_x), int(steps_y)
        if steps_x <= 0 and steps_y <= 0: return True

        pi.write(X_DIR, dir_x); pi.write(Y_DIR, dir_y)

        common = min(steps_x, steps_y)
        legs = []
        if common > 0:                 legs.append((WAVE_XY, common))
        if steps_x - common > 0:       legs.append((WAVE_X,  steps_x - common))
        if steps_y - common > 0:       legs.append((WAVE_Y,  steps_y - common))

        dirs = {'x': dir_x, 'y': dir_y}
        for wave_id, n in legs:
            axes = ('x', 'y') if wave_id == WAVE_XY else (('x',) if wave_id == WAVE_X else ('y',))
            t0 = time.time()
            pi.wave_chain([255, 0, wave_id, 255, 1, n & 255, n >> 8])
            while pi.wave_tx_busy():
                if cmd_var.value == 99:
                    pi.wave_tx_stop()
                    for a in axes: pos_after_wave(a, dirs[a], n, t0, False)
                    return False
                if limit_hit(LIMIT_X) or limit_hit(LIMIT_Y):
                    pi.wave_tx_stop(); limit_tripped[0] = True
                    for a in axes: pos_after_wave(a, dirs[a], n, t0, False)
                    pos_snap_limits()
                    return False
                time.sleep(0.005)
            for a in axes: pos_add(a, dirs[a], n)
        return True

    def move_axis_search_dma(d_pin, direction, wave_id, steps):
        steps = int(steps)
        axis = 'x' if d_pin == X_DIR else 'y'
        pi.write(X_EN, 0); pi.write(Y_EN, 0)
        pi.write(d_pin, direction)
        t0 = time.time()
        pi.wave_chain([255, 0, wave_id, 255, 1, steps & 255, steps >> 8])
        while pi.wave_tx_busy():
            if cmd_var.value == 99:
                pi.wave_tx_stop()
                pos_after_wave(axis, direction, steps, t0, False)
                return False
            if limit_hit(LIMIT_X) or limit_hit(LIMIT_Y):
                pi.wave_tx_stop()
                limit_tripped[0] = True
                pos_after_wave(axis, direction, steps, t0, False)
                pos_snap_limits()
                return False
            if is_vis.value == 1:
                pi.wave_tx_stop()
                pos_after_wave(axis, direction, steps, t0, False)
                return True
            time.sleep(0.005)
        pos_after_wave(axis, direction, steps, t0, True)
        return False

    def square_spiral_search(start_mm=SPIRAL_START_MM, inc_mm=SPIRAL_INCREMENT_MM, loops=MAX_LOOPS):
        limit_tripped[0] = False
        current_side = start_mm
        for i in range(loops - 1):
            if cmd_var.value == 99: return False
            if move_axis_search_dma(Y_DIR, 0, WAVE_Y, current_side * STEPS_PER_MM): return True
            if limit_tripped[0]: return False
            if move_axis_search_dma(X_DIR, 0, WAVE_X, current_side * STEPS_PER_MM): return True
            if limit_tripped[0]: return False
            current_side += inc_mm
            if move_axis_search_dma(Y_DIR, 1, WAVE_Y, current_side * STEPS_PER_MM): return True
            if limit_tripped[0]: return False
            if move_axis_search_dma(X_DIR, 1, WAVE_X, current_side * STEPS_PER_MM): return True
            if limit_tripped[0]: return False
            current_side += inc_mm
        return False

    def ultimate_homing():
        pos_pre = (s_px.value, s_py.value, s_pok.value)
        s_pok.value = 0
        pi.write(X_EN, 0); pi.write(Y_EN, 0)
        
        FAST_FREQ = 4000 
        pi.write(X_DIR, 1); pi.write(Y_DIR, 1) 
        x_done, y_done = False, False
        fast_hz = pi.set_PWM_frequency(X_STEP, FAST_FREQ)
        pi.set_PWM_frequency(Y_STEP, FAST_FREQ)
        HOME_TIMEOUT_S = 1.5 * (max(TRAVEL_X_MM, TRAVEL_Y_MM) + 20) / (fast_hz / STEPS_PER_MM)
        
        if not limit_hit(LIMIT_X): pi.set_PWM_dutycycle(X_STEP, 128)
        else: x_done = True
        if not limit_hit(LIMIT_Y): pi.set_PWM_dutycycle(Y_STEP, 128)
        else: y_done = True
        
        t_fast0 = time.time()
        t_hit = {"x": None, "y": None}
        if x_done: t_hit["x"] = t_fast0     # แตะสวิตช์อยู่แล้วตั้งแต่ต้น = วิ่ง 0mm
        if y_done: t_hit["y"] = t_fast0

        def stop_fast_pwm():
            pi.set_PWM_dutycycle(X_STEP, 0); pi.set_PWM_dutycycle(Y_STEP, 0)
            pi.set_mode(X_STEP, pigpio.OUTPUT); pi.set_mode(Y_STEP, pigpio.OUTPUT)

        while not (x_done and y_done):
            if cmd_var.value == 99: 
                stop_fast_pwm()
                return False
            if time.time() - t_fast0 > HOME_TIMEOUT_S:
                stop_fast_pwm()
                pi.write(X_EN, 1); pi.write(Y_EN, 1)
                miss = ("X" if not x_done else "") + ("Y" if not y_done else "")
                print(f"🛑 home ไม่สำเร็จ: ลิมิตแกน {miss} ไม่ทำงานภายใน {HOME_TIMEOUT_S:.1f} วิ — หยุดมอเตอร์แล้ว")
                pos_set_msg(f"หาจุด home ไม่สำเร็จ: สวิตช์ลิมิตแกน {miss} ไม่ทำงาน ตรวจสายและสวิตช์ก่อนสั่งใหม่")
                return False
            if not x_done and limit_hit(LIMIT_X):
                pi.set_PWM_dutycycle(X_STEP, 0); x_done = True 
                t_hit["x"] = time.time()
            if not y_done and limit_hit(LIMIT_Y):
                pi.set_PWM_dutycycle(Y_STEP, 0); y_done = True 
                t_hit["y"] = time.time()
            time.sleep(0.005)

        # --- ตรวจสอบข้ามวิธี: ระยะที่วิ่งจริงตอน home ควรเท่ากับพิกัดที่ตัวนับเชื่อไว้ ---
        if pos_pre[2] == 1:
            ran_x = (t_hit["x"] - t_fast0) * fast_hz / STEPS_PER_MM if t_hit["x"] else 0.0
            ran_y = (t_hit["y"] - t_fast0) * fast_hz / STEPS_PER_MM if t_hit["y"] else 0.0
            print(f"📍 ตรวจตัวนับตอน home: ตัวนับบอก ({pos_pre[0]:.1f}, {pos_pre[1]:.1f}) mm "
                  f"· วิ่งจริงจับเวลาได้ ({ran_x:.1f}, {ran_y:.1f}) mm "
                  f"· ต่าง ({ran_x - pos_pre[0]:+.1f}, {ran_y - pos_pre[1]:+.1f}) mm")
            
        stop_fast_pwm()
        time.sleep(0.2)
        
        # --- จังหวะที่ 2: ถอยหลังออกมา 3mm ---
        pi.write(X_DIR, 0); pi.write(Y_DIR, 0) 
        backoff_steps = int(3 * STEPS_PER_MM)
        if backoff_steps > 0:
            pi.wave_chain([255, 0, WAVE_XY, 255, 1, backoff_steps & 255, backoff_steps >> 8])
            while pi.wave_tx_busy():
                if cmd_var.value == 99: pi.wave_tx_stop(); return False
                time.sleep(0.01)
        time.sleep(0.2)
        
        # --- จังหวะที่ 3: เดินช้าๆ เข้าไปแตะให้แม่นที่สุด ---
        pi.write(X_DIR, 1); pi.write(Y_DIR, 1) 
        x_done, y_done = False, False
        while not (x_done and y_done):
            if cmd_var.value == 99: return False
            if not x_done:
                if limit_hit(LIMIT_X): x_done = True
                else: pi.write(X_STEP, 1)
            if not y_done:
                if limit_hit(LIMIT_Y): y_done = True
                else: pi.write(Y_STEP, 1)
            time.sleep(0.002); pi.write(X_STEP, 0); pi.write(Y_STEP, 0); time.sleep(0.002)

        pos_home()
        pos_set_msg("")
        return True

    def move_to_center_diagonal():
        pi.write(X_EN, 0); pi.write(Y_EN, 0)
        pi.write(X_DIR, 0); pi.write(Y_DIR, 0) 
        steps_x = int(CENTER_X_MM * STEPS_PER_MM)
        steps_y = int(CENTER_Y_MM * STEPS_PER_MM)
        common_steps = min(steps_x, steps_y)
        remaining_x = steps_x - common_steps
        remaining_y = steps_y - common_steps
        
        for wave_id, n, axes in ((WAVE_XY, common_steps, ('x', 'y')),
                                 (WAVE_X,  remaining_x, ('x',)),
                                 (WAVE_Y,  remaining_y, ('y',))):
            if n <= 0:
                continue
            t0 = time.time()
            pi.wave_chain([255, 0, wave_id, 255, 1, n & 255, n >> 8])
            while pi.wave_tx_busy():
                if cmd_var.value == 99:
                    pi.wave_tx_stop()
                    for a in axes: pos_after_wave(a, 0, n, t0, False)
                    return
                time.sleep(0.01)
            for a in axes: pos_add(a, 0, n)
        pi.write(X_EN, 1); pi.write(Y_EN, 1) 

    def center_on_target():
        s_pok.value = 0
        TRACK_SPEED = 800
        centered_start_time = None
        last_seen_time = time.time()
        
        last_err_x, last_err_y = 0, 0
        pi.write(X_EN, 0); pi.write(Y_EN, 0)

        def stop_and_release_pwm():
            pi.set_PWM_dutycycle(X_STEP, 0); pi.set_PWM_dutycycle(Y_STEP, 0)
            pi.set_mode(X_STEP, pigpio.OUTPUT); pi.set_mode(Y_STEP, pigpio.OUTPUT)

        while is_run.value == 1:
            if cmd_var.value == 99:
                stop_and_release_pwm()
                return "STOP"

            current_time = time.time()
            if is_vis.value == 1:
                last_seen_time = current_time
                last_err_x = m_x.value - CAM_CX
                last_err_y = m_y.value - CAM_CY
            else:
                time_blind = current_time - last_seen_time
                if time_blind > 1.5:
                    stop_and_release_pwm()
                    return "LOST"
                elif time_blind > 0.4:
                    pi.set_PWM_dutycycle(X_STEP, 0); pi.set_PWM_dutycycle(Y_STEP, 0)
                    time.sleep(0.01)
                    continue

            if abs(last_err_x) > 10:
                pi.write(X_DIR, 1 if (last_err_x * DIR_INVERT_X) > 0 else 0)
                pi.set_PWM_frequency(X_STEP, TRACK_SPEED); pi.set_PWM_dutycycle(X_STEP, 128)
                centered_start_time = None
            else:
                pi.set_PWM_dutycycle(X_STEP, 0)
                
            if abs(last_err_y) > 10:
                pi.write(Y_DIR, 1 if (last_err_y * DIR_INVERT_Y) > 0 else 0)
                pi.set_PWM_frequency(Y_STEP, TRACK_SPEED); pi.set_PWM_dutycycle(Y_STEP, 128)
                centered_start_time = None
            else:
                pi.set_PWM_dutycycle(Y_STEP, 0)
                
            if abs(last_err_x) <= 10 and abs(last_err_y) <= 10 and is_vis.value == 1:
                if centered_start_time is None: 
                    centered_start_time = time.time()
                elif current_time - centered_start_time >= 0.5:
                    stop_and_release_pwm()
                    return "SUCCESS"
            else:
                centered_start_time = None

            time.sleep(0.02)
        stop_and_release_pwm()
        return "STOP"

    # ==========================================
    # 🦘 กระโดดตรงไปหาขดลวด (แทน center_on_target)
    # ==========================================
    def arm_motors():
        pi.set_mode(X_STEP, pigpio.OUTPUT); pi.set_mode(Y_STEP, pigpio.OUTPUT)
        pi.write(X_EN, 0); pi.write(Y_EN, 0)

    def read_marker_stable(duration=JUMP_SAMPLE_S):
        xs, ys, angs, ids = [], [], [], []
        t0 = time.time()
        while time.time() - t0 < duration:
            if cmd_var.value == 99:
                return None
            if is_vis.value == 1:
                xs.append(m_x.value)
                ys.append(m_y.value)
                angs.append(m_angle.value)
                ids.append(m_id.value)
            time.sleep(0.03)
        if len(xs) < JUMP_MIN_SAMPLES:
            return None
        mid = max(set(ids), key=ids.count)                 # ID ที่เห็นบ่อยที่สุด
        keep = [i for i, v in enumerate(ids) if v == mid]
        if len(keep) < JUMP_MIN_SAMPLES:                   #   (เอามาเฉลี่ยรวมกัน = ได้จุดกลางอากาศ)
            return None

        def med(seq):
            q = sorted(seq)
            n = len(q)
            return q[n // 2] if n % 2 else 0.5 * (q[n // 2 - 1] + q[n // 2])

        ca = med([math.cos(angs[i]) for i in keep])
        sa = med([math.sin(angs[i]) for i in keep])
        return mid, med([xs[i] for i in keep]), med([ys[i] for i in keep]), math.atan2(sa, ca)

    def jump_to_coil():
        nudges = 0
        while True:
            pi.write(X_EN, 1); pi.write(Y_EN, 1)
            time.sleep(0.25)
            r = read_marker_stable()
            if cmd_var.value == 99:
                return "STOP"
            if r is None:
                return "LOST"
            mid, px, py, angle = r
            ex, ey = px - CAM_CX, py - CAM_CY
            rad = math.hypot(ex, ey)
            if rad <= JUMP_MAX_R_PX or nudges >= JUMP_NUDGE_MAX:
                if rad > JUMP_MAX_R_PX:
                    why = ("ปิดการขยับเข้ามาไว้" if JUMP_NUDGE_MAX == 0
                           else f"ขยับเข้ามาครบ {JUMP_NUDGE_MAX} ครั้งแล้ว")
                    print(f"   marker ห่างกลางจอ {rad:.0f}px (เกิน {JUMP_MAX_R_PX:.0f}) — {why} "
                          f"กระโดดจากตรงนี้เลย (คลาดอาจถึง ~5mm แต่ grid search ยังเก็บได้)")
                break
            # ไกลเกินโซนที่วัดไว้ว่าแม่น -> ขยับเข้ามาให้เหลือ ~JUMP_ZONE_R_PX ก่อน
            f = (rad - JUMP_ZONE_R_PX) / rad
            nx, ny = ex * f / JUMP_PX_PER_MM, ey * f / JUMP_PX_PER_MM
            print(f"   marker ห่างกลางจอ {rad:.0f}px (เกิน {JUMP_MAX_R_PX:.0f}) "
                  f"— ขยับเข้ามา ({nx:+.1f}, {ny:+.1f}) mm ก่อน")
            arm_motors()
            limit_tripped[0] = False
            if not move_xy_blind(abs(nx) * STEPS_PER_MM, 1 if nx > 0 else 0,
                                 abs(ny) * STEPS_PER_MM, 1 if ny > 0 else 0):
                return "STOP" if cmd_var.value == 99 else "LIMIT"
            nudges += 1

        local_x, local_y = {1: ( CAR_OFFSET_MM,  CAR_OFFSET_MM),
                            2: (-CAR_OFFSET_MM,  CAR_OFFSET_MM),
                            3: ( CAR_OFFSET_MM, -CAR_OFFSET_MM),
                            4: (-CAR_OFFSET_MM, -CAR_OFFSET_MM)}.get(mid, (0.0, 0.0))
        dx = (local_x * math.cos(angle)) - (local_y * math.sin(angle))
        dy = (local_x * math.sin(angle)) + (local_y * math.cos(angle))

        jump_x = ex / JUMP_PX_PER_MM
        jump_y = ey / JUMP_PX_PER_MM

        final_x = dx + COIL_OFFSET_X_MM + jump_x
        final_y = dy + COIL_OFFSET_Y_MM + jump_y
        print(f"🦘 กระโดด: ID {mid} ที่ ({px:.0f},{py:.0f})px ห่างกลางจอ {rad:.0f}px "
              f"มุม {math.degrees(angle):+.1f}°")
        print(f"   offset เดิม ({dx + COIL_OFFSET_X_MM:+.1f}, {dy + COIL_OFFSET_Y_MM:+.1f}) "
              f"+ เทอมใหม่ ({jump_x:+.1f}, {jump_y:+.1f}) = ({final_x:+.1f}, {final_y:+.1f}) mm")

        aex = max(-AIM_ERR_MAX_MM, min(AIM_ERR_MAX_MM, s_aex.value))
        aey = max(-AIM_ERR_MAX_MM, min(AIM_ERR_MAX_MM, s_aey.value))
        aim_err_used["xy"] = (aex, aey) if (aex or aey) else None
        if aex or aey:
            final_x -= aex
            final_y -= aey
            print(f"🧪 จำลองกล้องเล็งพลาด ({aex:+.1f}, {aey:+.1f}) mm "
                  f"-> ยิงจริง ({final_x:+.1f}, {final_y:+.1f}) mm")

        if s_pok.value == 1:
            tx = s_px.value - final_x
            ty = s_py.value - final_y
            lo_x, hi_x = REACH_MARGIN_MM, TRAVEL_X_MM - REACH_MARGIN_MM
            lo_y, hi_y = REACH_MARGIN_MM, TRAVEL_Y_MM - REACH_MARGIN_MM
            snap_x = min(hi_x, max(lo_x, tx))
            snap_y = min(hi_y, max(lo_y, ty))
            if (snap_x != tx or snap_y != ty) and abs(snap_x - tx) <= REACH_SNAP_MM \
                    and abs(snap_y - ty) <= REACH_SNAP_MM:
                print(f"📐 จุดที่คำนวณได้ ({tx:.1f}, {ty:.1f}) mm เลยขอบรางนิดเดียว "
                      f"-> ไปจอดที่ขอบราง ({snap_x:.1f}, {snap_y:.1f}) แทน (grid search จะหาต่อ)")
                tx, ty = snap_x, snap_y
                final_x = s_px.value - tx
                final_y = s_py.value - ty
            need = []
            if tx < REACH_MARGIN_MM:
                need.append(f"ถอยรถออกห่างจากมุม home ในแกน X อีก ~{REACH_MARGIN_MM - tx:.0f}mm")
            elif tx > TRAVEL_X_MM - REACH_MARGIN_MM:
                need.append(f"ขยับรถเข้าหามุม home ในแกน X อีก ~{tx - (TRAVEL_X_MM - REACH_MARGIN_MM):.0f}mm")
            if ty < REACH_MARGIN_MM:
                need.append(f"ถอยรถออกห่างจากมุม home ในแกน Y อีก ~{REACH_MARGIN_MM - ty:.0f}mm")
            elif ty > TRAVEL_Y_MM - REACH_MARGIN_MM:
                need.append(f"ขยับรถเข้าหามุม home ในแกน Y อีก ~{ty - (TRAVEL_Y_MM - REACH_MARGIN_MM):.0f}mm")
            if need:
                msg = "รถจอดนอกระยะที่ขดลวดเอื้อมถึง — " + " และ ".join(need)
                print(f"🚫 {msg}")
                print(f"   (แท่นอยู่ที่ ({s_px.value:.0f}, {s_py.value:.0f}) mm "
                      f"· ต้องไปที่ ({tx:.0f}, {ty:.0f}) mm · รางมีแค่ 0..{TRAVEL_X_MM:.0f} "
                      f"x 0..{TRAVEL_Y_MM:.0f})")
                pos_set_msg(msg)
                return "OUT_OF_REACH"

        arm_motors()
        limit_tripped[0] = False
        if not move_xy_blind(abs(final_x) * STEPS_PER_MM, 1 if final_x > 0 else 0,
                             abs(final_y) * STEPS_PER_MM, 1 if final_y > 0 else 0):
            return "STOP" if cmd_var.value == 99 else "LIMIT"
        return "SUCCESS"

    # ==========================================
    # 🕹️ โหมด jog ระหว่าง inverter ทำงาน (state 5)
    # ==========================================
    ping_h = {"t": 0.0}

    def ping_poll():
        ard_poll()
        now = time.time()
        if now - ping_h["t"] >= PING_INTERVAL:
            ard_send("PING")
            ping_h["t"] = now

    def jog_wave(d_pin, direction, wave_id, steps):
        steps = int(steps)
        if steps <= 0: return True
        axis = 'x' if d_pin == X_DIR else 'y'
        pi.write(d_pin, direction)
        t0 = time.time()
        pi.wave_chain([255, 0, wave_id, 255, 1, steps & 255, steps >> 8])
        while pi.wave_tx_busy():
            if cmd_var.value == 99:
                pi.wave_tx_stop()
                pos_after_wave(axis, direction, steps, t0, False)
                return False
            if limit_hit(LIMIT_X) or limit_hit(LIMIT_Y):
                pi.wave_tx_stop(); limit_tripped[0] = True
                pos_after_wave(axis, direction, steps, t0, False)
                pos_snap_limits()
                return False
            ping_poll()
            time.sleep(0.005)
        pos_after_wave(axis, direction, steps, t0, True)
        return True

    def jog_measure(tag):
        t_end = time.time() + JOG_SETTLE_S
        while time.time() < t_end:
            ping_poll()
            time.sleep(0.02)

        vals = []
        last_n = telem_h["n"]
        t_give_up = time.time() + JOG_AVG_N * 1.0 + 3.0   # กันค้างถ้า telemetry หยุดมา
        while len(vals) < JOG_AVG_N and time.time() < t_give_up:
            if cmd_var.value == 99 or s_evt.value != 0 or s_inv.value == 0:
                break                      # inverter ดับ/ถูกตัด — เลิกวัด ค่าที่ได้ใช้ไม่ได้
            ping_poll()
            if telem_h["n"] != last_n:
                last_n = telem_h["n"]
                vals.append(s_idc.value)
            time.sleep(0.02)

        if not vals:
            log_row(f"JOG_NO_DATA,{tag}")
            return None, 0
        mean = sum(vals) / len(vals)
        log_row(f"JOG,{tag},mean={mean:.4f},n={len(vals)}")
        print(f"🕹️  jog ({s_jx.value:+.1f}, {s_jy.value:+.1f}) mm  →  "
              f"{mean:.4f} A  (เฉลี่ย {len(vals)} ตัวอย่าง)")
        return mean, len(vals)

    def rail_ok(axis, delta_mm):
        if s_pok.value != 1:
            return True
        pos, travel = (s_px.value, TRAVEL_X_MM) if axis == 'x' else (s_py.value, TRAVEL_Y_MM)
        t = pos + delta_mm
        return REACH_MARGIN_MM - 1e-6 <= t <= travel - REACH_MARGIN_MM + 1e-6

    hold_h = {"on": False}

    def jog_goto(axis, target, tag=None, measure=True, cap=JOG_MAX_MM):
        if abs(target) > cap + 1e-6:
            print(f"⚠️  เป้าหมาย {target:+.1f}mm เกินเพดาน ±{cap:.0f}mm — ไม่ขยับ")
            log_row("JOG_MAX_REACHED")
            return False, None

        cur = s_jx.value if axis == 'x' else s_jy.value
        steps = int(round(abs(target - cur) * STEPS_PER_MM))

        if steps > 0 and not rail_ok(axis, target - cur):
            print(f"⚠️  เป้าหมาย jog {axis.upper()} {target:+.1f}mm จะเลยขอบราง — ไม่ขยับ")
            log_row("JOG_RAIL_LIMIT")
            return False, None

        if steps > 0:
            d_pin, wave_id = (X_DIR, WAVE_X) if axis == 'x' else (Y_DIR, WAVE_Y)
            direction = 0 if target > cur else 1
            limit_tripped[0] = False
            pi.write(X_EN, 0); pi.write(Y_EN, 0)      # จ่ายไฟคอยล์เฉพาะตอนขยับ
            ok = jog_wave(d_pin, direction, wave_id, steps)
            if not hold_h["on"]:                       # ปลดคอยล์ก่อนวัด (ดูเหตุผลใน jog_measure) · งานต่อเนื่องถือค้าง (hold_h)
                pi.write(X_EN, 1); pi.write(Y_EN, 1)
            if not ok:
                if limit_tripped[0]:
                    print("⚠️  jog ชนลิมิตสวิตช์ — หยุด (พิกัดที่บันทึกไว้อาจคลาดแล้ว)")
                    log_row("JOG_LIMIT_HIT")
                return False, None
            if axis == 'x': s_jx.value = target
            else:           s_jy.value = target

        if not measure:
            return True, None
        m, _ = jog_measure(tag if tag else f"{axis}={target:+.1f}")
        return True, m

    def do_jog(axis, sign):
        cur = s_jx.value if axis == 'x' else s_jy.value
        s_jbusy.value = 1
        try:
            ok, _ = jog_goto(axis, cur + JOG_STEP_MM * sign, tag=f"{axis}{sign:+d}")
            return ok
        finally:
            pi.write(X_EN, 1); pi.write(Y_EN, 1)
            # ระหว่างกวาดอัตโนมัติให้คา busy ไว้ตลอด ปุ่มบนเว็บจะได้ไม่กะพริบติด-ดับระหว่างก้าว
            s_jbusy.value = 1 if s_jsweep.value else 0

    def do_jog_home():
        s_jbusy.value = 1
        try:
            for axis in ('x', 'y'):
                ok, _ = jog_goto(axis, 0.0, measure=False)
                if not ok:
                    print("⚠️  กลับจุดเริ่มไม่สำเร็จ (ชนลิมิต/ถูกยกเลิก)")
                    log_row("JOG_HOME_FAILED")
                    return False
            jog_measure("home")
            return True
        finally:
            pi.write(X_EN, 1); pi.write(Y_EN, 1)
            # ระหว่างกวาดอัตโนมัติให้คา busy ไว้ตลอด ปุ่มบนเว็บจะได้ไม่กะพริบติด-ดับระหว่างก้าว
            s_jbusy.value = 1 if s_jsweep.value else 0

    # ==========================================
    # 🎯 Grid Search
    # ==========================================
    def fit_parabola(pts):
        if len(pts) < 3:
            return None
        S = [sum(x ** k for x, _ in pts) for k in range(5)]
        T = [sum((x ** k) * y for x, y in pts) for k in range(3)]
        M = [[S[4], S[3], S[2]],
             [S[3], S[2], S[1]],
             [S[2], S[1], S[0]]]
        rhs = [T[2], T[1], T[0]]

        def det3(A):
            return (A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
                  - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
                  + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]))

        d = det3(M)
        if abs(d) < 1e-12:
            return None
        out = []
        for col in range(3):
            A = [row[:] for row in M]
            for i in range(3):
                A[i][col] = rhs[i]
            out.append(det3(A) / d)
        return tuple(out)

    def do_grid_search():
        n = max(3, GRID_POINTS | 1)                    # บังคับเป็นเลขคี่
        step = 2.0 * GRID_SPAN_MM / (n - 1)
        offsets = [-GRID_SPAN_MM + i * step for i in range(n)]
        seen = []                                      # [(x, y, current)] ทุกจุดที่วัดจริง

        def alive():
            if cmd_var.value in (5, 99): return False
            if s_evt.value != 0: return False
            if s_inv.value == 0: return False
            return is_run.value == 1

        s_jsweep.value = 1
        s_jbusy.value = 1
        hold_h["on"] = True
        try:
            print(f"🎯 เริ่ม Grid Search — วัด {n} จุด/แกน ช่วง ±{GRID_SPAN_MM:.0f}mm "
                  f"(ก้าวละ {step:.0f}mm)")
            log_row(f"GRID_START,span={GRID_SPAN_MM},n={n}")

            def sweep_ok(axis, tgt):
                """จุดนี้ไปวัดได้ไหม: ไม่เกินเพดานรวม + ไม่เลยปลายราง"""
                if abs(tgt) > GRID_REACH_MM + 1e-6:
                    return False
                cur_j = s_jx.value if axis == 'x' else s_jy.value
                return rail_ok(axis, tgt - cur_j)

            def measure_at(axis, tgt, pts):
                ok, m = jog_goto(axis, tgt, tag=f"GRID_{axis}", cap=GRID_REACH_MM)
                if ok and m is not None:
                    pts.append((tgt, m))
                    seen.append((s_jx.value, s_jy.value, m))
                return ok

            def choose_vertex(axis, pts):
                pts = sorted(pts, key=lambda p: p[0])
                lo, hi = pts[0][0], pts[-1][0]
                i_min = min(range(len(pts)), key=lambda i: pts[i][1])
                x_min = pts[i_min][0]

                def local_fit():
                    sub = pts[max(0, i_min - 2): i_min + 3]
                    f = fit_parabola(sub) if len(sub) >= 3 else None
                    return f if (f is not None and f[0] > 0) else None

                fit = fit_parabola(pts)
                if fit is not None and fit[0] > 0 and abs(-fit[1] / (2.0 * fit[0]) - x_min) > step + 1e-6:
                    lf = local_fit()
                    if lf is not None:
                        log_row(f"GRID_LOCALFIT,{axis},all={-fit[1] / (2.0 * fit[0]):.2f},"
                                f"local={-lf[1] / (2.0 * lf[0]):.2f},min_at={x_min:.2f}")
                        fit = lf
                elif fit is None or fit[0] <= 0:
                    lf = local_fit()           # ทั้งชุดไม่เป็นแอ่ง แต่แถวก้นอาจเป็นแอ่งก็ได้
                    if lf is not None:
                        log_row(f"GRID_LOCALFIT,{axis},all=none,"
                                f"local={-lf[1] / (2.0 * lf[0]):.2f},min_at={x_min:.2f}")
                        fit = lf
                if fit is None or fit[0] <= 0:
                    print(f"⚠️  แกน {axis.upper()}: ฟิตพาราโบลาไม่ได้ (กราฟไม่เป็นรูปอ่าง) "
                          f"— ใช้จุดที่วัดต่ำสุด {x_min:+.1f}mm แทน")
                    log_row(f"GRID_NOFIT,{axis},use={x_min:.2f}")
                    return x_min, lo, hi
                a, b, _ = fit
                want = -b / (2.0 * a)
                vtx = max(lo, min(hi, want))
                if abs(vtx - want) > 1e-6:
                    log_row(f"GRID_CLAMP,{axis},want={want:.2f},use={vtx:.2f}")
                log_row(f"GRID_FIT,{axis},vertex={vtx:.2f},a={a:.6f},b={b:.6f},c={fit[2]:.6f}")
                return vtx, lo, hi

            stop_all = False
            for axis in ('x', 'y'):
                if stop_all or not alive(): break
                base = s_jx.value if axis == 'x' else s_jy.value
                pts = []
                # ไล่จากลบไปบวกตามลำดับ = เดินทางน้อยที่สุด และทิศเดียวตลอด (ไม่มี backlash ปน)
                for off in offsets:
                    if not alive(): break
                    tgt = base + off
                    if not sweep_ok(axis, tgt):
                        log_row(f"GRID_SKIP_RAIL,{axis},{tgt:.1f}")
                        continue
                    if not measure_at(axis, tgt, pts): break

                if len(pts) < 3 or not alive():
                    print(f"⚠️  แกน {axis.upper()}: จุดที่วัดได้ไม่พอ ({len(pts)}) — ข้ามการคำนวณ")
                    log_row(f"GRID_ABORT,{axis},pts={len(pts)}")
                    stop_all = True
                    break

                vtx, lo, hi = choose_vertex(axis, pts)

                x_meas = min(pts, key=lambda p: p[1])[0]
                m_hi, m_lo = abs(x_meas - hi) < 1e-6, abs(x_meas - lo) < 1e-6
                v_hi, v_lo = abs(vtx - hi) < 1e-6, abs(vtx - lo) < 1e-6
                if m_hi or m_lo or v_hi or v_lo:
                    d = (1.0 if m_hi else -1.0) if (m_hi or m_lo) else (1.0 if v_hi else -1.0)
                    why = "min" if (m_hi or m_lo) else "fit"
                    edge = hi if d > 0 else lo
                    print(f"➡️  แกน {axis.upper()}: จุดต่ำสุดอยู่ขอบ {edge:+.1f}mm ({why}) — เดินต่อทางนั้น")
                    log_row(f"GRID_EXTEND,{axis},from={edge:.2f},dir={d:+.0f},why={why}")
                    t = edge + d * step
                    while alive():
                        if not sweep_ok(axis, t):
                            log_row(f"GRID_EDGE_NOROOM,{axis},{t - d * step:.2f}")
                            print(f"   แกน {axis.upper()}: ไปต่อไม่ได้ (เพดาน ±{GRID_REACH_MM:.0f}mm หรือปลายราง)")
                            break
                        if not measure_at(axis, t, pts): break
                        # นับจุดที่วัดหลังจุดต่ำสุด (ตามทิศที่เดิน) — ครบ = เลยก้นแอ่งแล้ว หยุดได้
                        ordered = sorted(pts, key=lambda p: p[0] * d)
                        i_min = min(range(len(ordered)), key=lambda i: ordered[i][1])
                        if len(ordered) - 1 - i_min >= GRID_PAST_MIN:
                            break
                        t += d * step
                    if not alive(): break
                    vtx, lo, hi = choose_vertex(axis, pts)

                print(f"   ★ แกน {axis.upper()}: ก้นกราฟอยู่ที่ {vtx:+.2f} mm")
                if not alive(): break
                ok, m = jog_goto(axis, vtx, tag=f"GRID_{axis}_BEST", cap=GRID_REACH_MM)
                if ok and m is not None:
                    seen.append((s_jx.value, s_jy.value, m))

            if alive() and seen:
                ok, m = jog_goto('x', s_jx.value, tag="GRID_FINAL", cap=GRID_REACH_MM)   # ไม่ขยับ แค่วัดซ้ำ
                best = min(seen, key=lambda r: r[2])
                if m is not None and m > best[2] + GRID_VERIFY_MARGIN_A:
                    print(f"⚠️  จุดที่คำนวณได้ ({m:.4f} A) แย่กว่าจุดที่วัดได้ดีที่สุด "
                          f"({best[2]:.4f} A ที่ {best[0]:+.1f},{best[1]:+.1f}) — ถอยกลับ")
                    log_row(f"GRID_FALLBACK,to=({best[0]:.1f},{best[1]:.1f})")
                    jog_goto('x', best[0], measure=False, cap=GRID_REACH_MM)
                    jog_goto('y', best[1], tag="GRID_FALLBACK", cap=GRID_REACH_MM)
                else:
                    print(f"✅ Grid Search เสร็จ — จอดที่ ({s_jx.value:+.1f}, {s_jy.value:+.1f}) mm"
                          + (f" · {m:.4f} A" if m is not None else ""))
                    log_row(f"GRID_DONE,x={s_jx.value:.2f},y={s_jy.value:.2f}")
            elif not alive():
                print("⏹️  Grid Search ถูกยกเลิกกลางทาง (ข้อมูลเท่าที่เก็บได้ยังอยู่ใน CSV)")
                log_row("GRID_CANCELLED")
        finally:
            hold_h["on"] = False
            pi.write(X_EN, 1); pi.write(Y_EN, 1)
            s_jsweep.value = 0
            s_jbusy.value = 0

    def do_step_collect():
        n_ax = max(3, GRID_POINTS | 1)
        step = 2.0 * GRID_SPAN_MM / (n_ax - 1)
        offs = [-GRID_SPAN_MM + i * step for i in range(n_ax)]
        points = [('x', o, 0.0) for o in offs] + [('y', 0.0, o) for o in offs]
        arrive = {}                                    # index -> (x, y, mean) ค่าล่าสุดตอนมาถึง

        def alive():
            if cmd_var.value in (5, 99): return False
            if s_evt.value != 0: return False
            if s_inv.value == 0: return False
            return is_run.value == 1

        def goto_xy(x, y, tag):
            if abs(s_jx.value - x) > 1e-6:
                ok, _ = jog_goto('x', x, measure=False)
                if not ok: return False, None
            return jog_goto('y', y, tag=tag)           # ไม่ขยับ Y = วัดอย่างเดียว

        s_jsweep.value = 1; s_jbusy.value = 1
        s_stn.value = len(points); s_stw.value = 0
        hold_h["on"] = True                            # ถือคอยล์ค้างจนจบ รวมช่วงรอคนจด (ดู hold_h)
        try:
            print(f"🧑‍🔬 เริ่มเก็บผลทีละจุด — {len(points)} จุด (X {n_ax} · Y {n_ax}) รอคนกดถัดไปทุกจุด")
            log_row(f"STEP_START,n={len(points)},span={GRID_SPAN_MM}")
            i = 0
            quit_req = False
            while 0 <= i < len(points) and alive():
                axis, x, y = points[i]
                s_sti.value = i + 1; s_stw.value = 0
                cur_x, cur_y = s_jx.value, s_jy.value
                if not (rail_ok('x', x - cur_x) and rail_ok('y', y - cur_y)):
                    print(f"⚠️  จุด {i + 1} ({x:+.1f}, {y:+.1f}) เลยขอบราง — ข้าม")
                    log_row(f"STEP_SKIP_RAIL,i={i + 1},x={x:.1f},y={y:.1f}")
                    i += 1
                    continue
                ok, m = goto_xy(x, y, f"STEP{i + 1}_ARRIVE")
                if not ok or not alive():
                    break
                if m is not None:
                    arrive[i] = (x, y, m)
                print(f"   จุด {i + 1}/{len(points)} ({x:+.1f}, {y:+.1f}) mm — "
                      + (f"{m:.4f} A" if m is not None else "วัดไม่ได้") + " · รอคนจด กดถัดไปเมื่อเสร็จ")

                # --- รอคน: ถัดไป 18 / ย้อน 19 / เลิก 21 ---
                s_stw.value = 1
                go = 0
                while alive():
                    ping_poll()
                    c = cmd_var.value
                    if c in (18, 19, 21):
                        cmd_var.value = 0
                        go = c
                        break
                    if c not in (0, 5, 99):            # ปุ่มอื่น (กล้อง/jog/grid) ระหว่างนี้ = ไม่รับ
                        cmd_var.value = 0
                    time.sleep(0.02)
                s_stw.value = 0
                if not go:
                    break
                jog_measure(f"STEP{i + 1}_LEAVE")       # ไม่ขยับ แค่วัดก่อนออก (ดูกระแสลอยระหว่างรอ)
                if go == 18:   i += 1
                elif go == 19: i = max(0, i - 1)
                else:
                    quit_req = True
                    break

            if not alive():
                print("⏹️  เก็บผลทีละจุดถูกยกเลิก (ข้อมูลที่เก็บได้อยู่ใน CSV)")
                log_row("STEP_CANCELLED")
            elif quit_req:
                print("⏹️  เลิกเก็บผลทีละจุด — กลับจุดเริ่ม (0,0)")
                log_row(f"STEP_QUIT,at={s_sti.value}")
                goto_xy(0.0, 0.0, "STEP_QUIT_HOME")
            else:
                # --- ครบทุกจุด: จอดจุดกระแสต่ำสุด ---
                best = {}
                for axis in ('x', 'y'):
                    pts = sorted(((v[0] if axis == 'x' else v[1]), v[2])
                                 for k, v in arrive.items() if points[k][0] == axis)
                    if not pts:
                        best[axis] = 0.0
                        continue
                    i_min = min(range(len(pts)), key=lambda k: pts[k][1])
                    sub = pts[max(0, i_min - 2): i_min + 3]
                    f = fit_parabola(sub) if len(sub) >= 3 else None
                    if f is not None and f[0] > 0:
                        v = max(sub[0][0], min(sub[-1][0], -f[1] / (2.0 * f[0])))
                    else:
                        v = pts[i_min][0]
                    best[axis] = v
                    log_row(f"STEP_FIT,{axis},vertex={v:.2f},min_at={pts[i_min][0]:.2f}")
                bx, by = best['x'], best['y']
                print(f"✅ ครบ {len(points)} จุด — ไปจอดจุดกระแสต่ำสุด ({bx:+.2f}, {by:+.2f}) mm")
                ok, m = goto_xy(bx, by, "STEP_BEST")
                if ok and arrive and m is not None:
                    low = min(arrive.values(), key=lambda r: r[2])
                    if m > low[2] + GRID_VERIFY_MARGIN_A:
                        print(f"⚠️  จุดที่คำนวณ ({m:.4f} A) แย่กว่าจุดที่วัดได้ดีสุด ({low[2]:.4f} A) — ถอยกลับ")
                        log_row(f"STEP_FALLBACK,to=({low[0]:.1f},{low[1]:.1f})")
                        goto_xy(low[0], low[1], "STEP_FALLBACK")
                log_row(f"STEP_DONE,x={s_jx.value:.2f},y={s_jy.value:.2f}")
        finally:
            hold_h["on"] = False
            pi.write(X_EN, 1); pi.write(Y_EN, 1)
            s_sti.value = 0; s_stn.value = 0; s_stw.value = 0
            s_jsweep.value = 0; s_jbusy.value = 0

    def do_prepare():
        state_var.value = 1
        ok = ultimate_homing()
        if cmd_var.value == 99: return False
        if not ok:
            state_var.value = 0
            return False
        time.sleep(0.5)
        move_to_center_diagonal()
        if cmd_var.value == 99: return False
        state_var.value = 2
        return True

    def do_aim():
        state_var.value = 3; en_vis.value = 1
        s_aimt.value = time.time()
        pos_set_msg("")
        time.sleep(0.5)

        found = True if is_vis.value == 1 else square_spiral_search()

        jump_result = None   # ถ้า spiral แรกไม่เจอเลย ลูปข้างล่างจะไม่รัน ตัวแปรนี้ต้องมีค่า
        lost_retries = 0
        while found and cmd_var.value != 99:
            jump_result = jump_to_coil()
            if jump_result in ("SUCCESS", "STOP", "LIMIT", "OUT_OF_REACH"):
                break
            lost_retries += 1
            if lost_retries > LOST_MAX_RETRY:
                break
            pi.write(X_EN, 1); pi.write(Y_EN, 1)
            found = square_spiral_search(LOCAL_SEARCH_START_MM,
                                         LOCAL_SEARCH_INCREMENT_MM,
                                         LOCAL_SEARCH_LOOPS)

        if cmd_var.value == 99: return "STOP"

        pi.write(X_EN, 1); pi.write(Y_EN, 1)
        en_vis.value = 0
        if jump_result == "SUCCESS":
            state_var.value = 4
            pos_set_msg("")
            if s_pok.value == 1:
                print(f"📍 แท่นอยู่ที่ ({s_px.value:.1f}, {s_py.value:.1f}) mm จาก home")
        elif jump_result == "OUT_OF_REACH":
            state_var.value = 2
        elif jump_result != "STOP":
            print("⚠️  เล็งไม่สำเร็จ — re-home อัตโนมัติเพื่อกู้พิกัด")
            pi.wave_tx_stop()
            state_var.value = 1
            ok = ultimate_homing()
            if cmd_var.value == 99: return "STOP"
            if not ok:
                state_var.value = 0
                return jump_result
            time.sleep(0.5)
            move_to_center_diagonal()
            if cmd_var.value == 99: return "STOP"
            state_var.value = 2
        # jump_result == "STOP" -> คำสั่ง 99 จะถูกจัดการที่ต้นลูปรอบถัดไป
        return jump_result

    def run_auto():
        s_auto.value = 1
        for i in range(12): s_atm[i] = 0.0
        s_atm[0] = time.time(); s_atm[6] = 1
        try:
            if not do_prepare():
                return                       # โดน 99 กลางคัน — ลูปหลักจัดการต่อเอง
            s_atm[1] = time.time()

            result = do_aim()
            s_atm[2] = time.time()
            if result != "SUCCESS":
                # do_aim() ตั้ง state + ข้อความไว้ให้แล้ว (OUT_OF_REACH มีข้อความบอกทางของตัวเอง)
                if result != "OUT_OF_REACH" and cmd_var.value != 99:
                    pos_set_msg("โหมดอัตโนมัติ: เล็งเป้าไม่สำเร็จ — ลองขยับรถแล้วสั่งใหม่อีกครั้ง")
                print(f"⚠️  โหมดอัตโนมัติ: หยุดที่ขั้นเล็งเป้า (ผลลัพธ์ = {result})")
                return

            if cmd_var.value == 99:
                return
            if s_ok.value != 1:
                pos_set_msg("โหมดอัตโนมัติ: เล็งสำเร็จแล้ว แต่ไม่ได้เชื่อมต่อ Arduino จึงสั่งจ่ายไฟไม่ได้")
                print("⚠️  ไม่ได้เชื่อมต่อ Arduino — สั่ง inverter ไม่ได้")
                return
            if state_var.value != 4:
                # กันไว้อีกชั้น (do_aim คืน SUCCESS ต้องได้ state 4 อยู่แล้ว)
                print("⚠️  ยังไม่ได้ล็อกเป้า (state != 4) — ปฏิเสธคำสั่งเปิด inverter")
                return

            run_charging(auto_grid=True)
        finally:
            s_auto.value = 0
            if s_atm[6] == 1: s_atm[6] = 3
            if s_atm[6] == 3 and s_atm[7] == 0: s_atm[7] = time.time()

    def run_charging(auto_grid=False):
        # กล้องต้องดับสนิทก่อนจ่ายไฟ (state 4 ดับให้แล้ว แต่ย้ำอีกครั้งกันพลาด)
        en_vis.value = 0
        time.sleep(0.3)

        pi.write(X_EN, 1); pi.write(Y_EN, 1)

        s_jx.value = 0.0; s_jy.value = 0.0; s_jbusy.value = 0

        s_evt.value = 0
        try:
            ser.reset_input_buffer()
        except Exception:
            pass

        log_open()
        if aim_err_used["xy"] is not None:
            log_row(f"AIM_ERR,{aim_err_used['xy'][0]:.1f},{aim_err_used['xy'][1]:.1f}")

        zero_h["ack"] = 0
        ard_send("ZERO")
        t_zero = time.time()
        while time.time() - t_zero < ZERO_TIMEOUT:
            ard_poll()
            if zero_h["ack"] != 0: break
            time.sleep(0.02)
        if zero_h["ack"] != 1:
            print("⚠️  ไม่ได้รับ ACK,ZERO — ค่ากระแสรอบนี้ยังไม่ได้หัก offset (เฟิร์มแวร์เก่า?)")
            log_row("ZERO_TIMEOUT")

        ard_send("ON")
        state_var.value = 5
        t_start = time.time()
        ping_h["t"] = time.time()
        seen_on = False
        auto_h = {"done": False}

        while is_run.value == 1:
            ard_poll()

            if cmd_var.value == 99:
                break
            if cmd_var.value == 5:          # กดปุ่มหยุดชาร์จบนเว็บ
                cmd_var.value = 0
                break
            if cmd_var.value == 3:
                cmd_var.value = 0

            if s_evt.value != 0:            # Arduino ตัดไฟเอง / มีคนกดปุ่ม OFF ที่เครื่อง
                break
            if s_ok.value == 0:
                print("⚠️  Arduino หลุดระหว่างชาร์จ — ออกจากโหมดชาร์จ (WDT ฝั่ง Arduino ดับ inverter ให้แล้วใน 800ms)")
                break

            if s_inv.value == 1:
                seen_on = True
            elif seen_on:
                break
            elif time.time() - t_start > INVERTER_START_TIMEOUT:
                print("⚠️  สั่ง ON แล้ว Arduino ไม่รายงานว่า inverter ติด — ยกเลิก")
                break

            if auto_grid and seen_on and not auto_h["done"]:
                auto_h["done"] = True
                s_atm[3] = time.time()
                print("🤖 โหมดอัตโนมัติ: inverter ติดแล้ว — รอกระแสนิ่งแล้วเริ่มปรับตำแหน่ง")
                t_settle = time.time()
                while time.time() - t_settle < AUTO_GRID_SETTLE_S:
                    ping_poll()
                    if cmd_var.value in (5, 99) or s_evt.value != 0 or s_inv.value == 0:
                        break
                    time.sleep(0.02)
                if cmd_var.value not in (5, 99) and s_evt.value == 0 and s_inv.value == 1:
                    s_atm[4] = time.time(); s_atm[8] = s_px.value; s_atm[9] = s_py.value
                    do_grid_search()        # ยกเลิกกลางคันได้ — ไม่เคลียร์ cmd_var ให้ลูปนี้จัดการต่อ
                    s_atm[5] = time.time(); s_atm[10] = s_px.value; s_atm[11] = s_py.value
                    ok = cmd_var.value not in (5, 99) and s_evt.value == 0 and s_inv.value == 1
                    s_atm[7] = s_atm[5]
                    s_atm[6] = 2 if ok else 3
                    t = [s_atm[i] - s_atm[0] for i in range(1, 6)]
                    log_row("AUTO_TIME," + ",".join(f"{v:.1f}" for v in t) + f",{'OK' if ok else 'STOP'}"
                            + "," + ",".join(f"{s_atm[i]:.1f}" for i in range(8, 12)))   # + พิกัดก่อน/หลัง grid (มม. จาก home)
                    print(f"⏱️  โหมดอัตโนมัติ: เตรียม {t[0]:.1f} · เล็ง {t[1]-t[0]:.1f} · เปิดไฟ {t[2]-t[1]:.1f}"
                          f" · รอนิ่ง {t[3]-t[2]:.1f} · grid {t[4]-t[3]:.1f} · รวม {t[4]:.1f} วิ ({'ครบ' if ok else 'หยุดกลางคัน'})"
                          f" · แท่น ({s_atm[8]:.1f}, {s_atm[9]:.1f}) → ({s_atm[10]:.1f}, {s_atm[11]:.1f}) mm จาก home")
                continue

            jc = cmd_var.value
            if jc in (18, 19, 21):
                cmd_var.value = 0
            if seen_on and jc in (10, 11, 12, 13, 14, 15, 16, 17):
                cmd_var.value = 0
                if   jc == 10: do_jog('x', +1)
                elif jc == 11: do_jog('x', -1)
                elif jc == 12: do_jog('y', +1)
                elif jc == 13: do_jog('y', -1)
                elif jc == 14: do_jog_home()
                elif jc == 16: do_grid_search()
                elif jc == 17: do_step_collect()
                continue

            now = time.time()
            if now - ping_h["t"] >= PING_INTERVAL:
                ard_send("PING")            # ต่ออายุ watchdog ฝั่ง Arduino
                ping_h["t"] = now

            time.sleep(0.02)

        ard_send("OFF")
        time.sleep(0.15)
        ard_poll()                          # เก็บ ACK,OFF + ค่าสุดท้ายหลังดับ
        log_close()

        if cmd_var.value != 99:
            state_var.value = 4

    # 🔁 ลูปการทำงานหลัก
    try:
        while is_run.value == 1:
            ard_poll()   # อ่านค่ากระแสขึ้นเว็บตลอดเวลา แม้ inverter จะดับอยู่ (ดู noise floor ได้)
            ard_reconnect_tick()

            if cmd_var.value == 99:
                ard_send("OFF")
                pi.wave_tx_stop(); pi.write(X_EN, 1); pi.write(Y_EN, 1)
                en_vis.value = 0; state_var.value = 99
                time.sleep(0.1); continue
            
            elif cmd_var.value == 3:
                en_vis.value = 1 if en_vis.value == 0 else 0
                cmd_var.value = 0 

            elif cmd_var.value == 1:
                do_prepare()
                if cmd_var.value != 99: cmd_var.value = 0

            elif cmd_var.value == 2:
                do_aim()
                if cmd_var.value != 99: cmd_var.value = 0

            elif cmd_var.value == 20:
                cmd_var.value = 0
                run_auto()

            elif cmd_var.value == 4:
                cmd_var.value = 0
                if state_var.value != 4:
                    print("⚠️  ยังไม่ได้ล็อกเป้า (state != 4) — ปฏิเสธคำสั่งเปิด inverter")
                elif s_ok.value != 1:
                    print("⚠️  ไม่ได้เชื่อมต่อ Arduino — สั่ง inverter ไม่ได้")
                else:
                    run_charging()

            elif cmd_var.value == 5:
                cmd_var.value = 0   # ไม่ได้ชาร์จอยู่ ไม่มีอะไรต้องหยุด

            elif cmd_var.value in (10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 21):
                cmd_var.value = 0
                print("⚠️  jog ใช้ได้เฉพาะตอน inverter ทำงานอยู่ (state 5) — ไม่ทำอะไร")

            else:
                time.sleep(0.05)

    finally:
        ard_send("OFF")
        time.sleep(0.1)
        log_close()   # เผื่อตายกลาง run_charging — ปิดไฟล์ให้เรียบร้อย (ข้อมูล flush ไว้ทุกแถวแล้ว)
        # กันไฟแช่คอยล์มอเตอร์ค้าง แม้เกิด exception ระหว่างลูปหลัก
        pi.write(X_EN, 1); pi.write(Y_EN, 1); pi.stop()
        if ser is not None:
            try: ser.close()
            except Exception: pass

# ==========================================
# 🌐 Web Server
# ==========================================
app = Flask(__name__)
BOOT_ID = str(int(time.time() * 1000))

PORTAL_HOST = "evcharger.pnet"

@app.before_request
def captive_redirect():
    if request.host.startswith("["):              # IPv6 ตรงๆ
        return None
    host = request.host.split(":")[0].lower().rstrip(".")
    if (host == PORTAL_HOST or host.endswith(".local") or host in ("localhost", "")
            or host.replace(".", "").isdigit()):  # IPv4 ตรงๆ
        return None
    return redirect(f"http://{PORTAL_HOST}/", code=302)

@app.route('/')
def index(): return render_template_string(HTML_TEMPLATE, cam_w=CAM_W, cam_h=CAM_H)

@app.route('/video_feed')
def video_feed():
    def generate():
        while program_running.value == 1:
            if not frame_queue.empty():
                img_data = frame_queue.get()
                # คลีนท่อให้ส่งแต่ไฟล์ภาพล้วนๆ
                yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + img_data + b'\r\n')
            else: 
                time.sleep(0.01)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

_snap = {"jpg": None}

@app.route('/snapshot')
def snapshot():
    try:
        _snap["jpg"] = frame_queue.get(timeout=0.5)
    except Exception:
        if enable_vision.value != 1:
            _snap["jpg"] = None
    if _snap["jpg"] is None:
        return ('', 204)
    return Response(_snap["jpg"], mimetype='image/jpeg',
                    headers={'Cache-Control': 'no-store'})

@app.route('/command/<int:cmd>')
def command(cmd): web_command.value = cmd; return jsonify({"status": "ok"})

@app.route('/set_oc', methods=['POST'])
def set_oc():
    try:
        v = float((request.get_json(silent=True) or {}).get("a"))
    except (TypeError, ValueError):
        return jsonify({"ok": False, "msg": "ตัวเลขไม่ถูกต้อง"}), 400
    if not (0 < v < float("inf")):
        return jsonify({"ok": False, "msg": "ต้องเป็นตัวเลขมากกว่า 0"}), 400
    oc_err.value = 0
    oc_req.value = round(v, 2)
    return jsonify({"ok": True})

# ==========================================
# 📈 กราฟ grid search บนหน้าเว็บ — อ่านจาก CSV รอบล่าสุด (ไม่แตะ motor process)
# ==========================================
import threading
_gv = {"path": None, "pos": 0, "rest": "", "t0": None, "aim": None, "g": None}
_gv_lock = threading.Lock()

def _gv_kv(parts):
    out = {}
    for x in parts:
        if "=" in x:
            k, v = x.split("=", 1)
            try: out[k] = float(v)
            except ValueError: pass
    return out

def _gv_row(row):
    if len(row) < 12 or row[0] == "time":
        return
    if _gv["t0"] is None:
        try: _gv["t0"] = float(row[0])
        except ValueError: pass
    ev = row[2]
    if not ev:
        return
    f = ev.split(",")
    tag = f[0]
    try: jx, jy, idc = float(row[10]), float(row[11]), float(row[4] or 0)
    except ValueError: return
    if tag == "AIM_ERR" and len(f) >= 3:
        _gv["aim"] = [float(f[1]), float(f[2])]
        return
    if tag == "GRID_START":
        kv = _gv_kv(f[1:])
        step = 2.0 * kv.get("span", 20.0) / max(1.0, kv.get("n", 11.0) - 1)
        _gv["g"] = {"base": [jx, jy], "start_idc": idc, "step": step, "pts": {"x": [], "y": []},
                    "local": {},
                    "ext": {"x": 0, "y": 0}, "fit": {}, "best": {}, "final": None,
                    "fallback": 0, "end": None, "pos": None}
        return
    g = _gv["g"]
    if g is None:
        return
    if tag == "JOG" and len(f) >= 3:
        name, mean = f[1], _gv_kv(f[2:]).get("mean")
        if mean is None: return
        if name in ("GRID_x", "GRID_y"):
            ax = name[-1]
            g["pts"][ax].append([jx if ax == "x" else jy, mean, g["ext"][ax]])
        elif name in ("GRID_x_BEST", "GRID_y_BEST"):
            ax = name[5]
            g["best"][ax] = [jx if ax == "x" else jy, mean]
        elif name in ("GRID_FINAL", "GRID_FALLBACK"):
            g["final"] = mean
    elif tag == "GRID_LOCALFIT" and len(f) >= 3:
        g["local"][f[1]] = _gv_kv(f[2:]).get("min_at")   # ฟิตเฉพาะจุดต่ำสุด ±2 จุด -> เส้นวาดแค่ช่วงนั้น
    elif tag == "GRID_EXTEND" and len(f) >= 2:
        g["ext"][f[1]] = 1
    elif tag == "GRID_FIT" and len(f) >= 3:
        kv = _gv_kv(f[2:]); kv["nofit"] = 0
        g["fit"][f[1]] = kv
    elif tag == "GRID_NOFIT" and len(f) >= 3:
        g["fit"][f[1]] = {"vertex": _gv_kv(f[2:]).get("use"), "nofit": 1}
    elif tag == "GRID_FALLBACK":
        g["fallback"] = 1
    elif tag == "GRID_DONE":
        g["end"] = "done"; g["pos"] = [jx, jy]
    elif tag in ("GRID_CANCELLED", "GRID_ABORT"):
        g["end"] = "cancel"

def _gv_update():
    """หา CSV ล่าสุด แล้วอ่านเฉพาะส่วนที่เพิ่มขึ้นตั้งแต่ครั้งก่อน (ไฟล์ชาร์จยาวโตได้หลาย MB)"""
    try:
        names = sorted(n for n in os.listdir(LOG_DIR) if n.startswith("charge_") and n.endswith(".csv"))
    except OSError:
        return None
    if not names:
        return None
    path = os.path.join(LOG_DIR, names[-1])
    if path != _gv["path"]:
        _gv.update({"path": path, "pos": 0, "rest": "", "t0": None, "aim": None, "g": None})
    try:
        with open(path, newline="") as fh:
            fh.seek(_gv["pos"])
            chunk = fh.read()
            _gv["pos"] = fh.tell()
    except OSError:
        return None
    if chunk:
        text = _gv["rest"] + chunk
        lines = text.split("\n")
        _gv["rest"] = lines.pop()          # บรรทัดที่ยังเขียนไม่จบ เก็บไว้รอบหน้า
        for row in csv.reader(lines):
            _gv_row(row)
    return path

@app.route('/grid_data')
def grid_data():
    with _gv_lock:
        path = _gv_update()
        fresh = path is not None and _gv["t0"] is not None and _gv["t0"] >= aim_t.value - 1.0
        return jsonify({"file": os.path.basename(path) if path else None,
                        "aim": _gv["aim"] if fresh else None,
                        "grid": _gv["g"] if fresh else None})

# ==========================================
# ==========================================
WIFI_DIR = "/run/evcharger-wifi"

def _wifi_read(name):
    try:
        with open(os.path.join(WIFI_DIR, name)) as fh:
            return fh.read().strip()
    except OSError:
        return ""

@app.route('/wifi')
def wifi_status():
    st, detail, t = (_wifi_read("result").split("|") + ["", "", "0"])[:3]
    try: t = float(t)
    except ValueError: t = 0.0
    return jsonify({"mode": _wifi_read("mode") or "unknown",
                    "pending": 1 if os.path.exists(os.path.join(WIFI_DIR, "request")) else 0,
                    "state": st, "detail": detail, "age": round(time.time() - t, 1) if t else None})

@app.route('/wifi_switch', methods=['POST'])
def wifi_switch():
    to = (request.get_json(silent=True) or {}).get("to", "home")
    if to not in ("home", "ap"):
        return jsonify({"ok": False, "msg": "คำขอไม่ถูกต้อง"}), 400
    try:
        with open(os.path.join(WIFI_DIR, "request"), "w") as fh:
            fh.write(to)
    except OSError as e:
        return jsonify({"ok": False, "msg": f"ส่งคำขอไม่ได้ (ตัวสลับ WiFi ไม่ได้รันอยู่?) {e}"}), 500
    return jsonify({"ok": True})

@app.route('/set_aim_err', methods=['POST'])
def set_aim_err():
    j = request.get_json(silent=True) or {}
    try:
        x, y = float(j.get("x", 0)), float(j.get("y", 0))
    except (TypeError, ValueError):
        return jsonify({"ok": False, "msg": "ตัวเลขไม่ถูกต้อง"}), 400
    if not all(abs(v) <= AIM_ERR_MAX_MM for v in (x, y)):
        return jsonify({"ok": False, "msg": f"ต้องอยู่ในช่วง ±{AIM_ERR_MAX_MM:.0f} mm"}), 400
    aim_ex.value, aim_ey.value = round(x, 1), round(y, 1)
    return jsonify({"ok": True})

@app.route('/status')
def status():
    return jsonify({
        "state": system_state.value,
        "cam": enable_vision.value,
        "auto": auto_run.value,
        "serial": serial_ok.value,
        "inv": inv_on.value,
        "idc": round(ct_idc.value, 3),
        "ipp": round(ct_ipp.value, 3),
        "vdiff": round(ct_vdiff.value, 4),
        "vref": round(ct_vref.value, 3),
        "vout": round(ct_vout.value, 3),
        "evt": inv_event.value,
        "jx": round(jog_x.value, 1),
        "jy": round(jog_y.value, 1),
        "jbusy": jog_busy.value,
        "jsweep": jog_sweep.value,
        "sti": step_i.value,
        "stn": step_n.value,
        "stw": step_wait.value,
        "jmax": JOG_MAX_MM,
        "jstep": JOG_STEP_MM,
        "px": round(pos_x.value, 1),
        "py": round(pos_y.value, 1),
        "pok": pos_ok.value,
        "pmsg": pos_msg.value.decode("utf-8", "ignore"),
        "travx": TRAVEL_X_MM,
        "travy": TRAVEL_Y_MM,
        "cox": COIL_HOME_X_MM,
        "coy": COIL_HOME_Y_MM,
        "oc": round(oc_lim.value, 2),
        "ocpend": 1 if oc_req.value > 0 else 0,
        "ocerr": oc_err.value,
        "boot": BOOT_ID,
        "aex": aim_ex.value,
        "aey": aim_ey.value,
        "aemax": AIM_ERR_MAX_MM,
        "atm": [round(v, 2) for v in auto_tm[:6]] + [round(auto_tm[7], 2)],
        "ast": int(auto_tm[6]),
        "apos": [round(v, 1) for v in auto_tm[8:12]],   # พิกัดแท่นก่อน/หลัง grid (มม. จาก home)
        "now": round(time.time(), 2),
    })

if __name__ == "__main__":
    try:
        cam_p = multiprocessing.Process(target=camera_task, args=(is_marker_visible, enable_vision, marker_x, marker_y, marker_id, marker_angle, program_running, frame_queue))
        cam_p.start()
        motor_p = multiprocessing.Process(target=motor_task, args=(web_command, system_state, is_marker_visible, enable_vision, marker_x, marker_y, marker_id, marker_angle, program_running, ARDUINO_BUNDLE, POS_BUNDLE, auto_run))
        motor_p.start()
        app.run(host='0.0.0.0', port=int(os.environ.get("EV_PORT", "5000")), debug=False, use_reloader=False)
    except KeyboardInterrupt: pass
    finally:
        program_running.value = 0 
        if 'cam_p' in locals(): cam_p.join()
        if 'motor_p' in locals(): motor_p.join()
