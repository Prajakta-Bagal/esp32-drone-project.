#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// ==============================
// WIFI SETTINGS
// ==============================
const char* ssid     = "Redmi Note 14 5G";
const char* password = "suhani@2004";

// ==============================
// I2C TO ARDUINO UNO (N20 MOTORS VIA L293D)
// UNO is the I2C slave at address 0x08. Wiring: ESP32 GPIO21 (SDA) -> UNO A4,
// ESP32 GPIO22 (SCL) -> UNO A5, plus a shared GND between both boards.
// ==============================
#define I2C_SDA 21
#define I2C_SCL 22
#define UNO_I2C_ADDR 0x08

// ==============================
// PMS PARTICULATE SENSOR DATA FROM UNO OVER I2C
// UNO must return exactly 6 bytes on an I2C read:
// PM1.0 (uint16, big-endian), PM2.5 (uint16, big-endian), PM10 (uint16, big-endian).
// ESP32 is the I2C master; UNO remains slave at 0x08.
// ==============================
uint16_t pmsPM1 = 0;
uint16_t pmsPM25 = 0;
uint16_t pmsPM10 = 0;
bool pmsDataValid = false;
unsigned long lastPmsReadMillis = 0;
const unsigned long PMS_READ_INTERVAL_MS = 1000;

void readPmsDataFromUno() {
  Wire.requestFrom((uint8_t)UNO_I2C_ADDR, (uint8_t)6);

  if (Wire.available() < 6) {
    while (Wire.available()) Wire.read();
    pmsDataValid = false;
    return;
  }

  uint8_t data[6];
  for (int i = 0; i < 6; i++) {
    data[i] = (uint8_t)Wire.read();
  }

  pmsPM1  = ((uint16_t)data[0] << 8) | data[1];
  pmsPM25 = ((uint16_t)data[2] << 8) | data[3];
  pmsPM10 = ((uint16_t)data[4] << 8) | data[5];
  pmsDataValid = true;
}


// N20 motors always run at MAX speed, per requirement.
const int MOTOR_SPEED = 255;
bool systemStarted = false;

// ==============================
// VEHICLE-SIDE FAILSAFE (independent of the browser/UI)
// If the ESP32 loses WiFi entirely, or simply stops receiving commands for
// COMMAND_TIMEOUT_MS while armed, it stops the N20 motors AND the BLDC/ESCs
// on its own. This does NOT depend on the browser successfully sending a
// stop command over a broken link - the ESP32 notices the silence itself.
// ==============================
unsigned long lastCommandMillis = 0;
const unsigned long COMMAND_TIMEOUT_MS = 1000;

// ==============================
// SERVOS (ROBOT ARM) - DRIVEN DIRECTLY BY ESP32
//
// PIN NOTE: Servo3/Servo4 moved off GPIO21/22 since those are now dedicated
// to I2C (SDA/SCL) for the UNO link. Using GPIO4/GPIO23 instead of GPIO16/17,
// keeping 16/17 free for whatever you're planning to add there later.
// Rewire Servo3's signal wire to GPIO4 and Servo4's to GPIO23.
// ==============================
#define SERVO1_PIN 18
#define SERVO2_PIN 19
#define SERVO3_PIN 4  // moved from 21 (now I2C SDA); 16 kept free per request
#define SERVO4_PIN 23 // moved from 22 (now I2C SCL); 17 kept free per request

Servo servo1, servo2, servo3, servo4;

const int CLOSE_ANGLE = 0;
const int OPEN_ANGLE  = 109;
bool armOpen = false;

// ==============================
// BLDC / ESC CONTROL (4x, driven like servos with 1000-2000us pulses)
//
// IMPORTANT PIN NOTE: GPIO34 and GPIO35 on the ESP32 are INPUT-ONLY pins -
// they cannot physically generate a PWM/servo signal. An ESC wired to
// either of those pins will never receive a signal and will never spin,
// regardless of what code runs. ESC1/ESC2 use the requested D32/D33 (valid
// outputs). ESC3/ESC4 have been moved to D26/D27 (both valid outputs) in
// place of the requested D34/D35. Rewire ESC3 and ESC4's signal wires to
// D26 and D27, or tell me two different free GPIOs to use instead.
// ==============================
#define ESC1_PIN 32 // FRONT
#define ESC2_PIN 33 // RIGHT
#define ESC3_PIN 26 // REAR  (was D34 - input-only, moved)
#define ESC4_PIN 27 // LEFT  (was D35 - input-only, moved)

Servo esc1, esc2, esc3, esc4;

const int ESC_MIN_US = 1000; // full stop / idle
const int ESC_MAX_US = 2000; // full throttle

void stopAllEscs() {
  esc1.writeMicroseconds(ESC_MIN_US);
  esc2.writeMicroseconds(ESC_MIN_US);
  esc3.writeMicroseconds(ESC_MIN_US);
  esc4.writeMicroseconds(ESC_MIN_US);
}

// ==============================
// WEB SERVER
// ==============================
WebServer server(80);

// ==============================
// HELPERS
// ==============================
// Sends a 2-byte command to the UNO over I2C: direction char + speed (0-255).
// Replaces the old UART string protocol ("F255", "S", etc).
void sendMotorCommand(char direction, int speed) {
  Wire.beginTransmission(UNO_I2C_ADDR);
  Wire.write((uint8_t)direction);
  Wire.write((uint8_t)constrain(speed, 0, 255));
  byte result = Wire.endTransmission();

  Serial.print("-> UNO (I2C): ");
  Serial.print(direction);
  Serial.print(" speed=");
  Serial.print(speed);
  if (result != 0) {
    Serial.print("  [I2C ERROR code ");
    Serial.print(result);
    Serial.print(" - check wiring/UNO power]");
  }
  Serial.println();
}

void setServos(int angle) {
  servo1.write(angle);
  servo2.write(angle);
  servo3.write(angle);
  servo4.write(angle);
}

// ==============================
// DASHBOARD (transmitter-style UI)
// Ground Chassis (N20), Rotor Arms (servo), and Flight (BLDC) are all LIVE now.
// ==============================
const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Amphibian Drone Control Deck</title>
<style>
  :root{
    --accent:#00e6a8;
    --accent2:#3d7bff;
    --danger:#ff4d5e;
    --amber:#ffb648;
    --text:#eef1ff;
    --muted:#8b93b8;
    --plastic1:#22262f;
    --plastic2:#14161c;
    --plastic-edge:#33384a;
  }
  *{box-sizing:border-box;}
  html,body{height:100%;}
  body{
    margin:0;
    min-height:100vh;
    background:radial-gradient(circle at 50% -10%, #1c2330 0%, #05070c 60%);
    font-family:'Segoe UI', Roboto, Arial, sans-serif;
    color:var(--text);
    display:flex;
    flex-direction:column;
    align-items:center;
    padding:26px 14px 40px;
  }
  .mono{ font-family:'Consolas','Courier New',monospace; }

  h1{
    font-size:clamp(16px,2.6vw,20px);
    letter-spacing:1.5px;
    margin:0 0 2px;
    text-transform:uppercase;
    background:linear-gradient(90deg,#7fffd4,#3d7bff);
    -webkit-background-clip:text; background-clip:text; color:transparent;
    font-weight:800;
    text-align:center;
  }
  .subtitle{ color:var(--muted); font-size:11px; text-align:center; margin-bottom:18px; letter-spacing:0.5px; }

  /* ================= RADIO BODY ================= */
  .radio{
    position:relative;
    width:100%;
    max-width:440px;
    background:linear-gradient(165deg, var(--plastic1), var(--plastic2));
    border:1px solid var(--plastic-edge);
    border-radius:34px;
    padding:34px 20px 26px;
    box-shadow:0 30px 60px rgba(0,0,0,0.55), inset 0 1px 0 rgba(255,255,255,0.05);
  }
  .handle{
    position:absolute; top:-16px; left:50%; transform:translateX(-50%);
    width:96px; height:20px;
    border:6px solid var(--plastic-edge);
    border-bottom:none;
    border-radius:50px 50px 0 0;
    background:var(--plastic2);
  }
  .antenna{
    position:absolute; top:-46px; left:63%;
    width:5px; height:56px;
    background:linear-gradient(180deg,#3a3f4d,#181a22);
    border-radius:3px;
    transform:rotate(18deg);
    transform-origin:bottom center;
  }

  .brand-row{ display:flex; justify-content:center; margin-bottom:10px; }
  .brand-plate{
    background:linear-gradient(160deg,#eef1ff,#c3cbe0);
    color:#14161c; font-weight:900; letter-spacing:2px;
    text-align:center; padding:6px 22px; border-radius:8px; font-size:13px;
    box-shadow:0 3px 0 #8991a8, inset 0 -3px 5px rgba(0,0,0,0.15);
  }

  /* ---- LED status lamps ---- */
  .led-row{ display:flex; justify-content:center; gap:22px; margin-bottom:10px; }
  .led-item{ display:flex; flex-direction:column; align-items:center; gap:3px; }
  .led{
    width:9px; height:9px; border-radius:50%;
    background:#3a3f4d; box-shadow:inset 0 0 2px rgba(0,0,0,0.6);
  }
  .led.green{ background:var(--accent); box-shadow:0 0 8px var(--accent); }
  .led.amber{ background:var(--amber); box-shadow:0 0 8px var(--amber); }
  .led.red{ background:var(--danger); box-shadow:0 0 8px var(--danger); animation:blink 0.6s infinite; }
  @keyframes blink{ 0%,100%{opacity:1;} 50%{opacity:0.25;} }
  .led-label{ font-size:7.5px; letter-spacing:1px; color:var(--muted); }

  /* ---- top knobs/switches ---- */
  .top-controls{ display:flex; justify-content:space-between; align-items:flex-start; margin-bottom:6px; }
  .knob-cluster{ display:flex; flex-direction:column; align-items:center; gap:5px; width:64px; }
  .knob{
    width:34px; height:34px; border-radius:50%;
    background:radial-gradient(circle at 35% 30%, #c9cfe0, #7b8299 70%);
    box-shadow:0 3px 6px rgba(0,0,0,0.5), inset 0 -2px 3px rgba(0,0,0,0.25);
    position:relative;
  }
  .knob::after{
    content:''; position:absolute; top:4px; left:50%; width:2px; height:11px;
    background:#3a3f4d; transform:translateX(-50%);
  }
  .knob-label{ font-size:8px; letter-spacing:1px; color:var(--muted); }
  .toggle-lever{
    width:14px; height:26px; border-radius:6px;
    background:linear-gradient(180deg,#3a3f4d,#1c1f28);
    border:1px solid #454b5e;
    position:relative;
  }
  .toggle-lever::after{
    content:''; position:absolute; left:50%; top:3px; width:8px; height:8px;
    border-radius:50%; background:#8891a8; transform:translateX(-50%);
  }

  /* ---- stick wells (draggable joysticks) ---- */
  .stick-row{ display:flex; justify-content:space-between; align-items:center; margin:8px 0 4px; }
  .gimbal-col{ display:flex; flex-direction:column; align-items:center; gap:6px; }
  .gimbal{
    width:150px; height:150px; border-radius:50%;
    background:radial-gradient(circle at 35% 28%, #2b2f3c, #0f1116 72%);
    border:7px solid var(--plastic-edge);
    box-shadow:inset 0 0 22px rgba(0,0,0,0.7), 0 6px 16px rgba(0,0,0,0.4);
    position:relative;
    touch-action:none;
  }
  .gimbal::before, .gimbal::after{
    content:''; position:absolute; left:50%; width:3px; height:9px;
    background:var(--danger); border-radius:2px; transform:translateX(-50%);
  }
  .gimbal::before{ top:6px; } .gimbal::after{ bottom:6px; }
  .tick{
    position:absolute; font-size:9px; color:#454b5e; user-select:none; pointer-events:none;
  }
  .tick-n{ top:18px; left:50%; transform:translateX(-50%); }
  .tick-s{ bottom:18px; left:50%; transform:translateX(-50%); }
  .tick-w{ left:16px; top:50%; transform:translateY(-50%); }
  .tick-e{ right:16px; top:50%; transform:translateY(-50%); }

  .stick-knob{
    position:absolute; top:50%; left:50%;
    transform:translate(-50%, -50%);
    width:54px; height:54px; border-radius:50%;
    background:radial-gradient(circle at 35% 28%, #3a3f4d, #14161c 75%);
    border:3px solid #4a5068;
    box-shadow:0 6px 14px rgba(0,0,0,0.5), inset 0 2px 4px rgba(255,255,255,0.06);
    cursor:grab;
    display:flex; align-items:center; justify-content:center;
  }
  .stick-knob::after{
    content:''; width:14px; height:14px; border-radius:50%;
    background:radial-gradient(circle at 35% 30%, #ff8b95,#c8202f 75%);
    box-shadow:0 0 6px rgba(200,32,47,0.7);
  }
  .stick-knob.grabbing{ cursor:grabbing; }
  .gimbal-label{ font-size:9px; letter-spacing:1.2px; color:var(--muted); }
  .gimbal.locked{ opacity:0.35; }
  .gimbal.locked .stick-knob{ cursor:not-allowed; }

  /* ---- center column: throttle lever (System Armed) ---- */
  .center-col{ display:flex; flex-direction:column; align-items:center; gap:8px; width:64px; }
  .lever-track{
    width:18px; height:96px; border-radius:10px;
    background:linear-gradient(180deg,#0c0e13,#1c1f28);
    border:1px solid #33384a;
    position:relative;
    box-shadow:inset 0 0 8px rgba(0,0,0,0.6);
    cursor:pointer;
  }
  .lever-track.locked{ cursor:not-allowed; opacity:0.5; }
  .lever-knob{
    position:absolute; left:50%; transform:translateX(-50%);
    bottom:4px;
    width:30px; height:24px; border-radius:8px;
    background:linear-gradient(160deg,#dfe4f2,#9aa2ba);
    box-shadow:0 3px 6px rgba(0,0,0,0.5);
    transition:bottom .25s ease;
  }
  .lever-track.armed .lever-knob{ bottom:68px; background:linear-gradient(160deg,#7fffd4,#00b386); }
  .lever-caption{ font-size:8.5px; letter-spacing:1px; color:var(--muted); text-align:center; }
  .lever-state{ font-size:9px; font-weight:700; }
  .lever-state.on{ color:var(--accent); }
  .lever-state.off{ color:var(--danger); }

  /* ================= LCD SCREEN ================= */
  .lcd-frame{
    margin-top:14px;
    background:linear-gradient(160deg,#1a1d24,#0c0e13);
    border:3px solid #063d38;
    border-radius:16px;
    padding:10px;
    display:flex;
    gap:8px;
    box-shadow:inset 0 0 14px rgba(0,0,0,0.6);
  }
  .lcd-screen{
    flex:1;
    background:#050d0b;
    border-radius:10px;
    padding:10px 12px 8px;
    box-shadow:inset 0 0 18px rgba(0,230,168,0.08);
    min-width:0;
    position:relative;
    overflow:hidden;
  }
  .lcd-screen.failsafe{ box-shadow:inset 0 0 24px rgba(255,77,94,0.25); }
  .lcd-topline{
    display:flex; justify-content:space-between; align-items:center;
    font-size:9.5px; color:var(--accent); letter-spacing:0.5px; margin-bottom:8px;
    border-bottom:1px dashed #0e3a32; padding-bottom:6px;
  }
  .lcd-conn{ display:flex; align-items:center; gap:5px; }
  .lcd-dot{ width:6px; height:6px; border-radius:50%; background:var(--accent); box-shadow:0 0 6px var(--accent); animation:pulse 1.6s infinite; }
  .lcd-dot.lost{ background:var(--danger); box-shadow:0 0 6px var(--danger); animation:blink 0.5s infinite; }
  @keyframes pulse{0%{opacity:1;}50%{opacity:0.3;}100%{opacity:1;}}
  .lcd-batt{ color:var(--accent); }
  .lcd-batt.low{ color:var(--amber); }
  .lcd-batt.crit{ color:var(--danger); }

  .lcd-icons{ display:flex; justify-content:space-around; align-items:center; padding:2px 0 4px; transition:opacity .2s ease; }
  .lcd-icon-block{ display:flex; flex-direction:column; align-items:center; gap:2px; }
  .lcd-icon-block svg{ width:44px; height:37px; }
  .lcd-icon-label{ font-size:7.5px; color:var(--accent); letter-spacing:1px; }
  .lcd-icon-state{ font-size:8.5px; font-weight:700; color:var(--text); }

  .lcd-footline{
    font-size:8px; color:#0e6b5c; letter-spacing:0.5px; text-align:center;
    border-top:1px dashed #0e3a32; padding-top:5px; margin-top:2px;
  }

  /* ---- motor speeds, shown inline on the same screen as the status icons ---- */
  .lcd-motors-inline{
    display:flex; flex-direction:column;
    border-top:1px dashed #0e3a32; padding-top:5px; margin-top:2px;
  }
  .lcd-motor-row{ margin-bottom:4px; }
  .lcd-motor-label{
    display:flex; justify-content:space-between; font-size:8.5px; color:var(--accent);
    letter-spacing:0.5px; margin-bottom:2px;
  }
  .lcd-motor-bar{ width:100%; height:4px; border-radius:4px; background:#0e2a25; overflow:hidden; }
  .lcd-motor-bar-fill{
    height:100%; width:0%; border-radius:4px;
    background:linear-gradient(90deg,#00e6a8,#3d7bff);
    transition:width .2s ease;
  }

  .failsafe-banner{
    position:absolute; inset:0; display:none; flex-direction:column;
    align-items:center; justify-content:center; gap:4px;
    background:rgba(10,3,4,0.88);
  }
  .failsafe-banner.show{ display:flex; }
  .failsafe-title{
    color:var(--danger); font-weight:900; letter-spacing:2px; font-size:14px;
    animation:blink 0.6s infinite;
  }
  .failsafe-reason{ color:#ffb99e; font-size:9.5px; letter-spacing:0.5px; }

  .lcd-side-btns{ display:flex; flex-direction:column; gap:8px; justify-content:center; }
  .lcd-side-btns button{
    width:52px; padding:9px 0; font-size:9.5px; font-weight:800; letter-spacing:0.5px;
    border:none; border-radius:9px; cursor:pointer; color:#fff;
    transition:transform .08s ease, opacity .2s ease;
  }
  .lcd-side-btns button:active{ transform:scale(0.93); }
  .lcd-side-btns button:disabled{ opacity:0.35; cursor:not-allowed; }
  .btn-deploy{ background:linear-gradient(135deg,#3d7bff,#2456c9); }
  .btn-fold{ background:linear-gradient(135deg,#ffb648,#d98a12); color:#231400; }

  /* ---- wheel + rotor mini icons (monochrome LCD style) ---- */
  .chassis-group{ transition:transform .3s ease; transform-origin:100px 75px; }
  .chassis-group.lean-left{ transform:rotate(-6deg); }
  .chassis-group.lean-right{ transform:rotate(6deg); }
  .wheel-spokes{ transform-box:fill-box; transform-origin:center; }
  .wheel-spokes.rolling{ animation-name:roll; animation-timing-function:linear; animation-iteration-count:infinite; }
  .wheel-spokes.rolling.reverse{ animation-name:roll-rev; }
  @keyframes roll{ from{transform:rotate(0deg);} to{transform:rotate(360deg);} }
  @keyframes roll-rev{ from{transform:rotate(0deg);} to{transform:rotate(-360deg);} }

  .arm-fl, .arm-fr, .arm-bl, .arm-br{ transition:transform .45s cubic-bezier(.34,1.4,.64,1); }
  .arm-fl{ transform-origin:85px 80px; } .arm-fr{ transform-origin:115px 80px; }
  .arm-bl{ transform-origin:85px 120px; } .arm-br{ transform-origin:115px 120px; }
  .rotors.deployed .arm-fl{ transform:rotate(-45deg); }
  .rotors.deployed .arm-fr{ transform:rotate(45deg); }
  .rotors.deployed .arm-bl{ transform:rotate(45deg); }
  .rotors.deployed .arm-br{ transform:rotate(-45deg); }
  .prop{ transition:opacity .3s ease .15s; opacity:0.3; transform-box:fill-box; transform-origin:center; }
  .rotors.deployed .prop{ opacity:1; animation:spin 0.5s linear infinite; }
  @keyframes spin{ from{transform:rotate(0deg);} to{transform:rotate(360deg);} }

  /* ---- ALL OFF button, physical button on the radio body ---- */
  .bottom-row{ display:flex; justify-content:center; margin-top:14px; padding-top:12px; border-top:1px solid var(--plastic-edge); }
  .bind-col{ display:flex; flex-direction:column; align-items:center; gap:6px; }
  .bind-key{
    width:58px; height:58px; border-radius:50%;
    background:radial-gradient(circle at 35% 30%, #ff8b95, #c8202f 75%);
    border:4px solid #7a1119;
    box-shadow:0 6px 14px rgba(255,77,94,0.45), inset 0 -3px 6px rgba(0,0,0,0.3);
    cursor:pointer; color:#fff; font-size:17px;
    display:flex; align-items:center; justify-content:center;
    transition:transform .08s ease;
  }
  .bind-key:active{ transform:scale(0.92); }
  .bind-caption{ font-size:9px; letter-spacing:1.5px; color:var(--muted); }

  footer{ margin-top:16px; font-size:10px; color:var(--muted); text-align:center; }
  footer span{ color:var(--accent); font-weight:700; }

  /* ================= RESPONSIVE: MOBILE LANDSCAPE + PC ================= */
  @media (max-height:520px){
    body{ padding:8px 14px 10px; }
    h1{ font-size:15px; margin-bottom:0; }
    .subtitle{ font-size:9.5px; margin-bottom:8px; }
    .radio{ max-width:640px; padding:10px 18px 12px; }
    .handle, .antenna{ display:none; }
    .brand-row{ margin-bottom:6px; }
    .brand-plate{ padding:4px 16px; font-size:11px; }
    .led-row{ margin-bottom:6px; gap:18px; }
    .top-controls{ margin-bottom:2px; }
    .knob{ width:26px; height:26px; }
    .toggle-lever{ height:20px; }
    .stick-row{ margin:4px 0 2px; }
    .gimbal{ width:110px; height:110px; border-width:5px; }
    .stick-knob{ width:42px; height:42px; }
    .lever-track{ height:70px; }
    .lever-knob{ width:26px; height:20px; }
    .lever-track.armed .lever-knob{ bottom:46px; }
    .lcd-frame{ margin-top:8px; padding:8px; }
    .lcd-icon-block svg{ width:36px; height:30px; }
    .lcd-motor-row{ margin-bottom:2px; }
    .bottom-row{ margin-top:8px; padding-top:8px; }
    .bind-key{ width:46px; height:46px; font-size:14px; }
  }

  @media (min-width:900px) and (min-height:640px){
    .radio{ max-width:500px; }
  }
</style>
</head>
<body>

  <h1>Amphibian Drone Control Deck</h1>
  <div class="subtitle">ESP32 &bull; N20 Chassis + 4x BLDC + Servo Arm &mdash; FS-i6 Style</div>

  <div class="radio">
    <div class="handle"></div>
    <div class="antenna"></div>

    <div class="brand-row"><div class="brand-plate">AMPHIBIAN</div></div>

    <div class="led-row">
      <div class="led-item"><div class="led" id="ledArm"></div><div class="led-label">ARM</div></div>
      <div class="led-item"><div class="led amber" id="ledLink"></div><div class="led-label">LINK</div></div>
      <div class="led-item"><div class="led green" id="ledBatt"></div><div class="led-label">BATT</div></div>
    </div>

    <div class="top-controls">
      <div class="knob-cluster">
        <div class="knob"></div>
        <div class="knob-label">VRA</div>
        <div class="toggle-lever"></div>
      </div>
      <div style="width:64px;"></div>
      <div class="knob-cluster">
        <div class="knob"></div>
        <div class="knob-label">VRB</div>
        <div class="toggle-lever"></div>
      </div>
    </div>

    <div class="stick-row">
      <!-- LEFT: Ground chassis (N20) joystick -->
      <div class="gimbal-col">
        <div class="gimbal left-gimbal" id="leftGimbal">
          <div class="tick tick-n">&#9650;</div>
          <div class="tick tick-s">&#9660;</div>
          <div class="tick tick-w">&#9664;</div>
          <div class="tick tick-e">&#9654;</div>
          <div class="stick-knob" id="leftKnob"></div>
        </div>
        <div class="gimbal-label">GROUND CHASSIS</div>
      </div>

      <!-- CENTER: System armed lever (arms BOTH ground + flight) -->
      <div class="center-col">
        <div class="lever-caption">SYSTEM<br>ARMED</div>
        <div class="lever-track" id="armLever" onclick="toggleSystem()">
          <div class="lever-knob"></div>
        </div>
        <div class="lever-state off" id="leverState">OFF</div>
      </div>

      <!-- RIGHT: Flight (BLDC) joystick - now LIVE -->
      <div class="gimbal-col">
        <div class="gimbal right-gimbal" id="rightGimbal">
          <div class="tick tick-n">&#9650;</div>
          <div class="tick tick-s">&#9660;</div>
          <div class="tick tick-w">&#9664;</div>
          <div class="tick tick-e">&#9654;</div>
          <div class="stick-knob" id="rightKnob"></div>
        </div>
        <div class="gimbal-label">FLIGHT (BLDC)</div>
      </div>
    </div>

    <!-- LCD SCREEN: shows wheels + rotor arm telemetry + failsafe banner -->
    <div class="lcd-frame">
      <div class="lcd-screen" id="lcdScreen">
        <div class="lcd-topline">
          <span>AMPHIBIAN01</span>
          <span class="lcd-conn"><span class="lcd-dot" id="lcdDot"></span><span id="lcdConnTxt">...</span></span>
          <span class="lcd-batt mono" id="lcdBatt">100%</span>
        </div>

        <div class="lcd-icons" id="lcdIcons">
          <div class="lcd-icon-block">
            <svg viewBox="0 0 200 150">
              <g class="chassis-group" id="chassisGroup">
                <rect x="62" y="60" width="76" height="30" rx="15" fill="none" stroke="#00e6a8" stroke-width="3"/>
                <g id="wheelLeft">
                  <circle cx="42" cy="75" r="40" fill="none" stroke="#00e6a8" stroke-width="4"/>
                  <g class="wheel-spokes" id="leftSpokes">
                    <line x1="42" y1="38" x2="42" y2="112" stroke="#00e6a8" stroke-width="3"/>
                    <line x1="10" y1="75" x2="74" y2="75" stroke="#00e6a8" stroke-width="3"/>
                    <line x1="18" y1="49" x2="66" y2="101" stroke="#00e6a8" stroke-width="2"/>
                    <line x1="18" y1="101" x2="66" y2="49" stroke="#00e6a8" stroke-width="2"/>
                  </g>
                </g>
                <g id="wheelRight">
                  <circle cx="158" cy="75" r="40" fill="none" stroke="#00e6a8" stroke-width="4"/>
                  <g class="wheel-spokes" id="rightSpokes">
                    <line x1="158" y1="38" x2="158" y2="112" stroke="#00e6a8" stroke-width="3"/>
                    <line x1="126" y1="75" x2="190" y2="75" stroke="#00e6a8" stroke-width="3"/>
                    <line x1="134" y1="49" x2="182" y2="101" stroke="#00e6a8" stroke-width="2"/>
                    <line x1="134" y1="101" x2="182" y2="49" stroke="#00e6a8" stroke-width="2"/>
                  </g>
                </g>
              </g>
            </svg>
            <div class="lcd-icon-label">GROUND</div>
            <div class="lcd-icon-state" id="groundStateTxt">STOPPED</div>
          </div>

          <div class="lcd-icon-block">
            <svg viewBox="0 0 200 200" class="rotors" id="rotorSvg">
              <rect x="85" y="78" width="30" height="44" rx="8" fill="none" stroke="#00e6a8" stroke-width="3"/>
              <circle cx="100" cy="100" r="5" fill="#00e6a8"/>
              <g class="arm-fl">
                <line x1="85" y1="80" x2="85" y2="25" stroke="#00e6a8" stroke-width="5" stroke-linecap="round"/>
                <circle cx="85" cy="25" r="7" fill="#00e6a8"/>
                <g class="prop">
                  <line x1="70" y1="25" x2="100" y2="25" stroke="#00e6a8" stroke-width="2.5" stroke-linecap="round"/>
                  <line x1="85" y1="10" x2="85" y2="40" stroke="#00e6a8" stroke-width="2.5" stroke-linecap="round"/>
                </g>
              </g>
              <g class="arm-fr">
                <line x1="115" y1="80" x2="115" y2="25" stroke="#00e6a8" stroke-width="5" stroke-linecap="round"/>
                <circle cx="115" cy="25" r="7" fill="#00e6a8"/>
                <g class="prop">
                  <line x1="100" y1="25" x2="130" y2="25" stroke="#00e6a8" stroke-width="2.5" stroke-linecap="round"/>
                  <line x1="115" y1="10" x2="115" y2="40" stroke="#00e6a8" stroke-width="2.5" stroke-linecap="round"/>
                </g>
              </g>
              <g class="arm-bl">
                <line x1="85" y1="120" x2="85" y2="175" stroke="#00e6a8" stroke-width="5" stroke-linecap="round"/>
                <circle cx="85" cy="175" r="7" fill="#00e6a8"/>
                <g class="prop">
                  <line x1="70" y1="175" x2="100" y2="175" stroke="#00e6a8" stroke-width="2.5" stroke-linecap="round"/>
                  <line x1="85" y1="160" x2="85" y2="190" stroke="#00e6a8" stroke-width="2.5" stroke-linecap="round"/>
                </g>
              </g>
              <g class="arm-br">
                <line x1="115" y1="120" x2="115" y2="175" stroke="#00e6a8" stroke-width="5" stroke-linecap="round"/>
                <circle cx="115" cy="175" r="7" fill="#00e6a8"/>
                <g class="prop">
                  <line x1="100" y1="175" x2="130" y2="175" stroke="#00e6a8" stroke-width="2.5" stroke-linecap="round"/>
                  <line x1="115" y1="160" x2="115" y2="190" stroke="#00e6a8" stroke-width="2.5" stroke-linecap="round"/>
                </g>
              </g>
            </svg>
            <div class="lcd-icon-label">ARMS</div>
            <div class="lcd-icon-state" id="armStateTxt">FOLDED</div>
          </div>
        </div>

        <div class="lcd-motors-inline">
          <div class="lcd-motor-row">
            <div class="lcd-motor-label"><span>M1 FRONT</span><span class="mono" id="m1val">0%</span></div>
            <div class="lcd-motor-bar"><div class="lcd-motor-bar-fill" id="m1bar"></div></div>
          </div>
          <div class="lcd-motor-row">
            <div class="lcd-motor-label"><span>M2 RIGHT</span><span class="mono" id="m2val">0%</span></div>
            <div class="lcd-motor-bar"><div class="lcd-motor-bar-fill" id="m2bar"></div></div>
          </div>
          <div class="lcd-motor-row">
            <div class="lcd-motor-label"><span>M3 REAR</span><span class="mono" id="m3val">0%</span></div>
            <div class="lcd-motor-bar"><div class="lcd-motor-bar-fill" id="m3bar"></div></div>
          </div>
          <div class="lcd-motor-row">
            <div class="lcd-motor-label"><span>M4 LEFT</span><span class="mono" id="m4val">0%</span></div>
            <div class="lcd-motor-bar"><div class="lcd-motor-bar-fill" id="m4bar"></div></div>
          </div>
        </div>

        <div class="lcd-footline">MODE: <span id="modeFoot" class="mono">GROUND STANDBY</span></div>

        <div class="failsafe-banner" id="failsafeBanner">
          <div class="failsafe-title">! FAILSAFE !</div>
          <div class="failsafe-reason" id="failsafeReason">RF LINK LOST</div>
        </div>
      </div>

      <div class="lcd-side-btns">
        <button class="btn-deploy" id="deployBtn" onclick="sendServo('O')">DEPLOY</button>
        <button class="btn-fold" onclick="sendServo('C')">FOLD</button>
      </div>
    </div>

    <div class="bottom-row">
      <div class="bind-col">
        <button class="bind-key" onclick="allOff()">&#9762;</button>
        <div class="bind-caption">ALL OFF</div>
      </div>
    </div>
  </div>

  <footer>Last action: <span class="mono" id="lastResponse">-</span></footer>

<script>
/* ===================== STATE ===================== */
let systemOn = false;
let linkLost = false;
let failsafeActive = false;
let failsafeReasons = [];
let battery = 100;

/* ===================== FAILSAFE CORE ===================== */
function recomputeFailsafe(){
  failsafeReasons = [];
  if(linkLost) failsafeReasons.push('RF LINK LOST');
  if(battery <= 15) failsafeReasons.push('BATTERY CRITICAL');

  const shouldBeActive = failsafeReasons.length > 0;

  if(shouldBeActive && !failsafeActive){
    failsafeActive = true;
    if(systemOn) forceDisarm();
    wheelsStop();
    if(document.getElementById('rotorSvg').classList.contains('deployed')){
      sendServo('C');
    }
    setMotorsUI(0,0,0,0);
    sendBldcThrottles(0,0,0,0);
    document.getElementById('lastResponse').textContent = 'FAILSAFE: ' + failsafeReasons.join(', ');
  } else if(!shouldBeActive && failsafeActive){
    failsafeActive = false;
    document.getElementById('lastResponse').textContent = 'FAILSAFE CLEARED';
  }

  updateFailsafeUI();
  updateLockUI();
}

function updateFailsafeUI(){
  const banner = document.getElementById('failsafeBanner');
  const screen = document.getElementById('lcdScreen');
  const icons = document.getElementById('lcdIcons');
  const reasonEl = document.getElementById('failsafeReason');
  if(failsafeActive){
    banner.classList.add('show');
    screen.classList.add('failsafe');
    icons.style.opacity = '0.15';
    reasonEl.textContent = failsafeReasons.join(' + ');
  } else {
    banner.classList.remove('show');
    screen.classList.remove('failsafe');
    icons.style.opacity = '1';
  }
}

function updateLockUI(){
  const locked = failsafeActive;
  document.getElementById('leftGimbal').classList.toggle('locked', locked);
  document.getElementById('rightGimbal').classList.toggle('locked', locked);
  document.getElementById('armLever').classList.toggle('locked', locked);
  document.getElementById('deployBtn').disabled = locked;

  document.getElementById('ledLink').className = 'led ' + (linkStatus === 'lost' ? 'red' : linkStatus === 'ok' ? 'green' : 'amber');
  document.getElementById('lcdDot').className = 'lcd-dot' + (linkStatus === 'lost' ? ' lost' : '');
  document.getElementById('lcdConnTxt').textContent = linkStatus === 'lost' ? 'LOST' : linkStatus === 'ok' ? 'TX' : '...';

  const battLed = document.getElementById('ledBatt');
  const lcdBatt = document.getElementById('lcdBatt');
  if(battery <= 15){ battLed.className='led red'; lcdBatt.className='lcd-batt mono crit'; }
  else if(battery <= 30){ battLed.className='led amber'; lcdBatt.className='lcd-batt mono low'; }
  else { battLed.className='led green'; lcdBatt.className='lcd-batt mono'; }
  lcdBatt.textContent = battery + '%';
}

/* NOTE: linkLost is driven by the real /ping heartbeat below (checkConn()).
   battery still has no real sensor on this hardware yet - it stays at 100%
   until an ADC battery reading is added; call recomputeFailsafe() whenever
   that value changes too, and the lockout/banner/LED behavior applies. */

/* ===================== SINGLE-FLIGHT COMMAND QUEUES =====================
   The ESP32's WebServer handles one request at a time. Firing overlapping
   fetch() calls (which is what rapid stick movement naturally does) can let
   the server process them out of order - e.g. a "move" and a "stop" sent
   almost simultaneously arriving in the wrong order, making the motor look
   like it starts and immediately stops. These wrappers guarantee at most
   one request in flight per channel, and always send the MOST RECENT desired
   state once the previous request finishes - nothing gets reordered. */
let groundPending = false, groundLatest = null;
function sendGroundCmd(cmd){
  groundLatest = cmd;
  if(groundPending) return;
  groundPending = true;
  const toSend = groundLatest;
  fetch('/command?cmd=' + toSend)
    .then(r => r.text())
    .then(data => {
      document.getElementById('lastResponse').textContent = data;
      if(data === 'NOT STARTED') setChassisUI(false);
    })
    .catch(() => {})
    .finally(() => {
      groundPending = false;
      if(groundLatest !== toSend) sendGroundCmd(groundLatest);
    });
}

let bldcPending = false, bldcLatest = null;
function sendBldcThrottles(m1,m2,m3,m4){
  bldcLatest = [m1,m2,m3,m4];
  if(bldcPending) return;
  bldcPending = true;
  const [a,b,c,d] = bldcLatest;
  fetch(`/bldc?m1=${a}&m2=${b}&m3=${c}&m4=${d}`)
    .then(r => r.text())
    .then(data => {
      if(data === 'NOT STARTED') setChassisUI(false);
    })
    .catch(() => {})
    .finally(() => {
      bldcPending = false;
      const stillLatest = bldcLatest;
      if(stillLatest[0]!==a || stillLatest[1]!==b || stillLatest[2]!==c || stillLatest[3]!==d){
        sendBldcThrottles(...stillLatest);
      }
    });
}

/* ===================== SYSTEM ARM (lever) - arms BOTH ground + flight ===================== */
function setChassisUI(active){
  systemOn = active;
  document.getElementById('armLever').classList.toggle('armed', active);
  const st = document.getElementById('leverState');
  st.textContent = active ? 'ON' : 'OFF';
  st.className = 'lever-state ' + (active ? 'on' : 'off');
  document.getElementById('ledArm').className = 'led' + (active ? ' green' : '');
  document.getElementById('groundStateTxt').textContent = active ? 'ACTIVE' : 'STOPPED';
  document.getElementById('modeFoot').textContent = active ? 'GROUND ACTIVE' : 'GROUND STANDBY';
  if(active){
    startKeepAlive();
    // Safety interlock: if either stick is already deflected the moment we
    // arm (e.g. held forward while flipping the switch), don't let it drive
    // anything until it's been brought back to neutral at least once first.
    groundNeutralOk = false;
    bldcNeutralOk = false;
  }
  else { stopKeepAlive(); wheelsStop(); setMotorsUI(0,0,0,0); }
}

/* Keeps the ESP32's command-timeout watchdog fed while armed, independent of
   whether the stick is actively moving. If this traffic stops reaching the
   ESP32 (WiFi down, out of range), the ESP32 auto-stops itself within ~1s
   with no dependency on this code still being reachable. */
let keepAliveTimer = null;
function startKeepAlive(){
  if(keepAliveTimer) return;
  keepAliveTimer = setInterval(() => {
    if(!systemOn) return;
    sendGroundCmd(currentGroundDir || 'HALT');
  }, 400);
}
function stopKeepAlive(){
  if(keepAliveTimer){ clearInterval(keepAliveTimer); keepAliveTimer = null; }
}

function forceDisarm(){
  fetch('/command?cmd=STOP')
    .then(r => r.text())
    .then(data => { document.getElementById('lastResponse').textContent = data; })
    .catch(() => {});
  setChassisUI(false);
}

function toggleSystem(){
  if(failsafeActive){
    document.getElementById('lastResponse').textContent = 'ARM BLOCKED: FAILSAFE ACTIVE';
    return;
  }
  const cmd = systemOn ? 'STOP' : 'START';
  fetch('/command?cmd=' + cmd)
    .then(r => r.text())
    .then(data => {
      document.getElementById('lastResponse').textContent = data;
      setChassisUI(data === 'STARTED');
    });
}

/* ===================== WHEEL VISUAL ===================== */
function wheelsSpin(dir, speed){
  const left = document.getElementById('leftSpokes');
  const right = document.getElementById('rightSpokes');
  const chassis = document.getElementById('chassisGroup');
  const dur = (0.9 - speed*0.65).toFixed(2) + 's';
  [left, right].forEach(w => { w.style.animationDuration = dur; w.classList.remove('reverse'); });
  left.classList.add('rolling'); right.classList.add('rolling');
  chassis.classList.remove('lean-left','lean-right');
  if(dir === 'B'){ left.classList.add('reverse'); right.classList.add('reverse'); }
  if(dir === 'L'){ chassis.classList.add('lean-left'); }
  if(dir === 'R'){ chassis.classList.add('lean-right'); }
}
function wheelsStop(){
  const left = document.getElementById('leftSpokes');
  const right = document.getElementById('rightSpokes');
  const chassis = document.getElementById('chassisGroup');
  left.classList.remove('rolling','reverse');
  right.classList.remove('rolling','reverse');
  chassis.classList.remove('lean-left','lean-right');
}

/* ===================== SERVO ARM (real hardware call) ===================== */
function sendServo(cmd){
  if(failsafeActive && cmd === 'O'){
    document.getElementById('lastResponse').textContent = 'DEPLOY BLOCKED: FAILSAFE ACTIVE';
    return;
  }
  fetch('/servo?cmd=' + cmd)
    .then(r => r.text())
    .then(data => {
      document.getElementById('lastResponse').textContent = data;
      const rotors = document.getElementById('rotorSvg');
      const txt = document.getElementById('armStateTxt');
      if(data === 'OPEN'){
        rotors.classList.add('deployed');
        txt.textContent = 'DEPLOYED';
      } else if(data === 'CLOSED'){
        rotors.classList.remove('deployed');
        txt.textContent = 'FOLDED';
      }
    });
}

/* ===================== JOYSTICKS (pointer drag) ===================== */
function setupJoystick(gimbalId, knobId, onMove, onRelease){
  const gimbal = document.getElementById(gimbalId);
  const knob = document.getElementById(knobId);
  let dragging = false;
  let centerX = 0, centerY = 0, maxDist = 0;

  function start(e){
    if(gimbal.classList.contains('locked')) return;
    dragging = true;
    knob.classList.add('grabbing');
    knob.style.transition = 'none';
    const rect = gimbal.getBoundingClientRect();
    centerX = rect.left + rect.width/2;
    centerY = rect.top + rect.height/2;
    // Scale to the KNOB's actual rendered size (fixes the mobile-landscape
    // breakpoint, where the knob shrinks but this used to assume desktop size,
    // making the usable drag range too small and prone to jitter).
    const knobRadius = knob.offsetWidth / 2;
    maxDist = rect.width/2 - knobRadius - 4;
    knob.setPointerCapture(e.pointerId);
    move(e);
  }
  function move(e){
    if(!dragging) return;
    let dx = e.clientX - centerX;
    let dy = e.clientY - centerY;
    const dist = Math.sqrt(dx*dx + dy*dy);
    if(dist > maxDist){ const s = maxDist/dist; dx*=s; dy*=s; }
    knob.style.transform = 'translate(calc(-50% + ' + dx + 'px), calc(-50% + ' + dy + 'px))';
    const nx = dx/maxDist, ny = dy/maxDist;
    const mag = Math.min(1, Math.sqrt(nx*nx+ny*ny));
    onMove(nx, ny, mag);
  }
  function end(e){
    if(!dragging) return;
    dragging = false;
    knob.classList.remove('grabbing');
    knob.style.transition = 'transform .25s cubic-bezier(.34,1.4,.64,1)';
    knob.style.transform = 'translate(-50%, -50%)';
    onRelease();
  }
  knob.addEventListener('pointerdown', start);
  knob.addEventListener('pointermove', move);
  knob.addEventListener('pointerup', end);
  knob.addEventListener('pointercancel', end);
}

/* Hysteresis: a single deadzone threshold lets tiny finger tremor near that
   line flip direction/stop rapidly back and forth. Using a HIGHER threshold
   to start moving and a LOWER one to register "stopped" creates a dead band
   between them, so small jitter can no longer cause repeated start/stop. */
const STICK_ENTER = 0.30;
const STICK_EXIT  = 0.15;

let currentGroundDir = null;
let groundMoving = false;
let groundNeutralOk = true;
setupJoystick('leftGimbal','leftKnob', (nx,ny,mag) => {
  if(!systemOn){
    document.getElementById('lastResponse').textContent = 'ARM SYSTEM FIRST';
    return;
  }
  if(!groundNeutralOk){
    if(mag < STICK_EXIT){ groundNeutralOk = true; }
    else {
      document.getElementById('lastResponse').textContent = 'CENTER STICK TO ARM';
      return;
    }
  }
  const shouldStop = groundMoving ? (mag < STICK_EXIT) : (mag < STICK_ENTER);
  if(shouldStop){
    groundMoving = false;
    if(currentGroundDir !== null){
      currentGroundDir = null;
      wheelsStop();
      sendGroundCmd('HALT');
    }
    return;
  }
  groundMoving = true;
  let dir;
  if(Math.abs(ny) > Math.abs(nx)) dir = ny < 0 ? 'F' : 'B';
  else dir = nx < 0 ? 'L' : 'R';
  wheelsSpin(dir, mag);
  if(dir !== currentGroundDir){
    currentGroundDir = dir;
    sendGroundCmd(dir);
  }
}, () => {
  const hadDir = currentGroundDir !== null;
  currentGroundDir = null;
  groundMoving = false;
  wheelsStop();
  if(hadDir) sendGroundCmd('HALT');
});

/* ===================== FLIGHT (BLDC) - now LIVE ===================== */
function setMotorsUI(m1,m2,m3,m4){
  const vals = [m1,m2,m3,m4].map(v => Math.max(0, Math.min(100, Math.round(v))));
  ['m1','m2','m3','m4'].forEach((id,i) => {
    document.getElementById(id+'val').textContent = vals[i] + '%';
    document.getElementById(id+'bar').style.width = vals[i] + '%';
  });
  return vals;
}

let bldcMoving = false;
let bldcNeutralOk = true;
setupJoystick('rightGimbal','rightKnob', (nx,ny,mag) => {
  if(!systemOn){
    document.getElementById('lastResponse').textContent = 'ARM SYSTEM FIRST';
    return;
  }
  if(!bldcNeutralOk){
    if(mag < STICK_EXIT){ bldcNeutralOk = true; }
    else {
      document.getElementById('lastResponse').textContent = 'CENTER STICK TO ARM';
      setMotorsUI(0,0,0,0); // motors stay at zero the whole time this gate is closed
      return;
    }
  }
  const shouldStop = bldcMoving ? (mag < STICK_EXIT) : (mag < STICK_ENTER);
  if(shouldStop){
    bldcMoving = false;
    const vals = setMotorsUI(0,0,0,0);
    sendBldcThrottles(...vals);
    return;
  }
  bldcMoving = true;
  const base = Math.round(mag * 70);
  const bias = Math.round(mag * 30);
  let m1 = base, m2 = base, m3 = base, m4 = base; // FRONT, RIGHT, REAR, LEFT
  if(Math.abs(ny) > Math.abs(nx)){
    if(ny < 0){ m1 += bias; m3 -= bias; } else { m1 -= bias; m3 += bias; }
  } else {
    if(nx < 0){ m4 += bias; m2 -= bias; } else { m2 += bias; m4 -= bias; }
  }
  const vals = setMotorsUI(m1, m2, m3, m4);
  sendBldcThrottles(...vals);
}, () => {
  bldcMoving = false;
  const vals = setMotorsUI(0,0,0,0);
  sendBldcThrottles(...vals);
});

/* ===================== CONNECTION HEARTBEAT (feeds the failsafe engine) ===================== */
let linkStatus = 'connecting'; // 'connecting' | 'ok' | 'lost'
let missedPings = 0;
const MISS_THRESHOLD = 2;

function checkConn(){
  fetch('/ping').then(r => {
    if(!r.ok) throw new Error();
    missedPings = 0;
    if(linkStatus !== 'ok'){
      linkStatus = 'ok';
      if(linkLost){ linkLost = false; recomputeFailsafe(); }
      else { updateLockUI(); }
    }
  }).catch(() => {
    missedPings++;
    if(missedPings >= MISS_THRESHOLD && linkStatus !== 'lost'){
      linkStatus = 'lost';
      linkLost = true;
      recomputeFailsafe();
    }
  });
}
setTimeout(checkConn, 1500);
setInterval(checkConn, 3000);

/* ===================== ALL OFF (real hardware stop) ===================== */
function allOff(){
  if(systemOn) forceDisarm();
  sendServo('C');
  document.getElementById('lastResponse').textContent = 'ALL OFF';
}

/* init */
updateLockUI();
</script>

</body>
</html>
)rawliteral";

// ==============================
// PMS DASHBOARD PAGE
// Open http://ESP32-IP/pms in a browser on the same Wi-Fi network.
// ==============================
const char pmsWebpage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Amphibian Drone | Air Quality</title>
<style>
  *{box-sizing:border-box}
  body{margin:0;min-height:100vh;padding:24px 14px;background:#070b12;color:#eef3ff;font-family:Arial, sans-serif}
  main{max-width:760px;margin:auto}
  h1{font-size:clamp(22px,5vw,34px);margin:0 0 8px;color:#72f5ce}
  .sub{color:#a4aec5;margin-bottom:22px;line-height:1.5}
  .status{display:flex;align-items:center;gap:9px;margin:14px 0 20px;color:#a4aec5}
  .dot{width:10px;height:10px;border-radius:50%;background:#e0a33b}
  .dot.ok{background:#38e6a8;box-shadow:0 0 10px #38e6a8}
  .dot.bad{background:#ff5364}
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:14px}
  .card{background:linear-gradient(145deg,#171e2b,#10151f);border:1px solid #2c3749;border-radius:18px;padding:22px}
  .label{font-size:13px;color:#a4aec5;letter-spacing:1px;text-transform:uppercase}
  .value{font-size:clamp(34px,8vw,52px);font-weight:800;margin:12px 0 4px;color:#f5f7ff}
  .unit{font-size:14px;color:#a4aec5}
  .pm25 .value{color:#ffca72}
  .foot{margin-top:20px;padding:14px;border:1px solid #2c3749;border-radius:12px;color:#a4aec5;font-size:13px;line-height:1.6}
  a{color:#72f5ce}
</style>
</head>
<body>
<main>
  <h1>Air Quality Monitor</h1>
  <div class="sub">Amphibian Drone · Live particulate matter readings from the PMS sensor</div>
  <div class="status"><span class="dot" id="dot"></span><span id="status">Waiting for sensor data…</span></div>
  <section class="grid">
    <div class="card"><div class="label">PM1.0</div><div class="value" id="pm1">--</div><div class="unit">µg/m³</div></div>
    <div class="card pm25"><div class="label">PM2.5</div><div class="value" id="pm25">--</div><div class="unit">µg/m³</div></div>
    <div class="card"><div class="label">PM10</div><div class="value" id="pm10">--</div><div class="unit">µg/m³</div></div>
  </section>
  <div class="foot">
    Data refreshes every second. These values are particulate concentration readings, not an AQI score.
    <br><a href="/">Return to drone control page</a>
  </div>
</main>
<script>
async function updateReadings(){
  try{
    const response = await fetch('/pms-data', {cache:'no-store'});
    if(!response.ok) throw new Error('HTTP ' + response.status);
    const d = await response.json();
    if(d.valid){
      document.getElementById('pm1').textContent = d.pm1;
      document.getElementById('pm25').textContent = d.pm25;
      document.getElementById('pm10').textContent = d.pm10;
      document.getElementById('status').textContent = 'Live · updated ' + new Date().toLocaleTimeString();
      document.getElementById('dot').className = 'dot ok';
    }else{
      document.getElementById('status').textContent = 'No valid PMS data received from Arduino UNO';
      document.getElementById('dot').className = 'dot bad';
    }
  }catch(e){
    document.getElementById('status').textContent = 'Cannot reach ESP32 data endpoint';
    document.getElementById('dot').className = 'dot bad';
  }
}
updateReadings();
setInterval(updateReadings, 1000);
</script>
</body>
</html>
)rawliteral";

// ==============================
// ROUTES
// ==============================
void handlePmsPage() {
  server.send_P(200, "text/html", pmsWebpage);
}

// ==============================
// GPS (NEO-6M) on UART2 - GPIO16/17 were kept free for exactly this
// GPS TX -> ESP32 GPIO16 (RX2)   GPS RX -> ESP32 GPIO17 (TX2)
// GPS VCC -> 5V   GPS GND -> GND
// ==============================
TinyGPSPlus gps;
HardwareSerial GPSserial(2);
#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 9600

void pollGps() {
  while (GPSserial.available() > 0) gps.encode(GPSserial.read());
}

void handlePmsData() {
  String json = "{";
  json += "\"valid\":";
  json += (pmsDataValid ? "true" : "false");
  json += ",\"pm1\":" + String(pmsPM1);
  json += ",\"pm25\":" + String(pmsPM25);
  json += ",\"pm10\":" + String(pmsPM10);
  bool gpsFix = gps.location.isValid() && gps.location.age() < 5000;
  json += ",\"gpsFix\":";
  json += (gpsFix ? "true" : "false");
  if (gpsFix) {
    json += ",\"lat\":" + String(gps.location.lat(), 6);
    json += ",\"lon\":" + String(gps.location.lng(), 6);
    json += ",\"alt\":" + String(gps.altitude.isValid() ? gps.altitude.meters() : 0, 1);
  } else {
    json += ",\"lat\":0,\"lon\":0,\"alt\":0";
  }
  json += ",\"sats\":" + String(gps.satellites.isValid() ? gps.satellites.value() : 0);
  json += "}";
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

void handleRoot() {
  server.send(200, "text/html", webpage);
}

void handlePing() {
  server.send(200, "text/plain", "OK");
}

void handleCommand() {
  // Any hit to this endpoint counts as "the ESP32 heard from a client just now" -
  // this is what feeds the command-timeout watchdog in loop().
  lastCommandMillis = millis();

  if (!server.hasArg("cmd")) {
    server.send(400, "text/plain", "Missing command");
    return;
  }

  String command = server.arg("cmd");

  if (command == "START") {
    systemStarted = true;
    sendMotorCommand('S', 0);
    server.send(200, "text/plain", "STARTED");
    return;
  }

  if (command == "STOP") {
    systemStarted = false;
    sendMotorCommand('S', 0);
    stopAllEscs();
    server.send(200, "text/plain", "STOPPED");
    return;
  }

  // HALT = release-to-stop while driving (does NOT disarm the system)
  if (command == "HALT") {
    sendMotorCommand('S', 0);
    server.send(200, "text/plain", "HALT");
    return;
  }

  // Block movement until START pressed
  if (!systemStarted) {
    sendMotorCommand('S', 0);
    server.send(200, "text/plain", "NOT STARTED");
    return;
  }

  if (command == "F" || command == "B" || command == "L" || command == "R") {
    // Always max speed for N20 motors
    sendMotorCommand(command.charAt(0), MOTOR_SPEED);
    server.send(200, "text/plain", command);
    return;
  }

  server.send(400, "text/plain", "Unknown command");
}

void handleServo() {
  if (!server.hasArg("cmd")) {
    server.send(400, "text/plain", "Missing command");
    return;
  }

  String command = server.arg("cmd");
  command.toUpperCase();

  if (command == "O") {
    setServos(OPEN_ANGLE);
    armOpen = true;
    Serial.println("Arm: OPEN");
    server.send(200, "text/plain", "OPEN");
    return;
  }

  if (command == "C") {
    setServos(CLOSE_ANGLE);
    armOpen = false;
    Serial.println("Arm: CLOSED");
    server.send(200, "text/plain", "CLOSED");
    return;
  }

  server.send(400, "text/plain", "Unknown command");
}

void handleBldc() {
  // Same watchdog feed as /command - keeps BLDC input covered by the
  // vehicle-side failsafe too, not just ground drive.
  lastCommandMillis = millis();

  if (!systemStarted) {
    stopAllEscs();
    server.send(200, "text/plain", "NOT STARTED");
    return;
  }

  int m1 = server.hasArg("m1") ? server.arg("m1").toInt() : 0;
  int m2 = server.hasArg("m2") ? server.arg("m2").toInt() : 0;
  int m3 = server.hasArg("m3") ? server.arg("m3").toInt() : 0;
  int m4 = server.hasArg("m4") ? server.arg("m4").toInt() : 0;

  m1 = constrain(m1, 0, 100);
  m2 = constrain(m2, 0, 100);
  m3 = constrain(m3, 0, 100);
  m4 = constrain(m4, 0, 100);

  esc1.writeMicroseconds(map(m1, 0, 100, ESC_MIN_US, ESC_MAX_US));
  esc2.writeMicroseconds(map(m2, 0, 100, ESC_MIN_US, ESC_MAX_US));
  esc3.writeMicroseconds(map(m3, 0, 100, ESC_MIN_US, ESC_MAX_US));
  esc4.writeMicroseconds(map(m4, 0, 100, ESC_MIN_US, ESC_MAX_US));

  server.send(200, "text/plain", "OK");
}

// ==============================
// SETUP
// ==============================
void setup() {
  Serial.begin(115200);

  // I2C to Arduino UNO (N20 motor commands) - ESP32 is master, UNO is slave @ 0x08
  Wire.begin(I2C_SDA, I2C_SCL);

  // GPS UART2
  GPSserial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Wire.setClock(100000); // standard-mode 100kHz, reliable over longer wires

  // Attach servos, start closed
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
  setServos(CLOSE_ANGLE);

  // ---- ESC ARMING SEQUENCE ----
  // Real ESCs require minimum throttle to be held for a moment at power-up
  // before they'll accept commands (this is also when most ESCs beep their
  // arming tone). Skipping this means the motors simply won't respond later.
  esc1.attach(ESC1_PIN);
  esc2.attach(ESC2_PIN);
  esc3.attach(ESC3_PIN);
  esc4.attach(ESC4_PIN);
  stopAllEscs();
  Serial.println("Arming ESCs (holding minimum throttle)...");
  delay(2000);
  Serial.println("ESCs armed.");

  // Connect WiFi
  WiFi.begin(ssid, password);
  Serial.println();
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());

  // Routes
  server.on("/", handleRoot);
  server.on("/pms", handlePmsPage);
  server.on("/pms-data", handlePmsData);
  server.on("/ping", handlePing);
  server.on("/command", handleCommand);
  server.on("/servo", handleServo);
  server.on("/bldc", handleBldc);
  server.begin();
  Serial.println("Web server started");

  // Safety: ensure everything is stopped at boot
  sendMotorCommand('S', 0);
  stopAllEscs();
  lastCommandMillis = millis();
}

// ==============================
// LOOP
// ==============================
void loop() {
  server.handleClient();
  pollGps(); // feed NMEA sentences to TinyGPS++ every loop

  // Request the latest PMS values from the UNO once per second.
  if (millis() - lastPmsReadMillis >= PMS_READ_INTERVAL_MS) {
    lastPmsReadMillis = millis();
    readPmsDataFromUno();
  }

  // ---- VEHICLE-SIDE FAILSAFE (works even if the browser can't reach us) ----
  if (WiFi.status() != WL_CONNECTED) {
    // The ESP32 itself lost WiFi - stop everything immediately.
    if (systemStarted) {
      sendMotorCommand('S', 0);
      stopAllEscs();
      systemStarted = false;
      Serial.println("FAILSAFE: WiFi disconnected, motors + ESCs stopped");
    }
  } else if (systemStarted && (millis() - lastCommandMillis > COMMAND_TIMEOUT_MS)) {
    // WiFi is technically up, but nothing has reached us in a while
    // (phone out of range, browser tab closed, etc.) - stop anyway.
    sendMotorCommand('S', 0);
    stopAllEscs();
    systemStarted = false;
    Serial.println("FAILSAFE: command timeout, motors + ESCs stopped");
  }

  // NOTE: the UNO's "FORWARD"/"STOP"/etc debug prints now go out its own USB
  // Serial only - the ESP32 can no longer read them back the way it could
  // over the old UART link. Check the UNO's own Serial Monitor directly if
  // you need to confirm what it received.
}
