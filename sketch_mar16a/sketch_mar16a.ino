#include <WiFi.h>
#include <WiFiUdp.h>
#include <MAVLink.h>
#include <esp_task_wdt.h>

// --- WiFi / Network Config ---
const char* WIFI_SSID     = "PixRacer";
const char* WIFI_PASSWORD = "pixracer";
const char* FC_IP         = "192.168.4.1";
const uint16_t FC_PORT    = 14550;
const uint16_t RX_PORT    = 14550;
const uint16_t TX_PORT    = 14555;

// --- MAVLink IDs ---
const uint8_t SYS_ID  = 255;
const uint8_t COMP_ID = 190;
uint8_t target_sysid  = 1;

// --- UDP sockets ---
WiFiUDP udpRx;
WiFiUDP udpTx;

// --- State ---
bool is_armed    = false;
bool is_offboard = false;

// --- Timing ---
// 50Hz setpoint stream to compensate for WiFi packet loss
// PX4 needs >2Hz sustained — 50Hz means we can lose 24/25 packets and still pass
const int SETPOINT_HZ  = 50;
const int SETPOINT_MS  = 1000 / SETPOINT_HZ;
const int HEARTBEAT_MS = 1000;

// --- Stream tracking ---
unsigned long streamStartTime   = 0;
bool          streamingEnough   = false; // true after 3s of continuous streaming
unsigned long setpointCount     = 0;
unsigned long lastSetpointPrint = 0;

// --- Offboard sequence ---
bool          seq_active  = false;
int           seq_step    = 0;
unsigned long seq_start   = 0;

// --------------------------------------------------------------------------
// Forward declarations
// --------------------------------------------------------------------------
void sendMavlink(mavlink_message_t& msg);
void sendHeartbeat();
void sendVelocityBody(float vx, float vy, float vz, float yaw_rate_deg);
void setOffboardMode();
void readMavlink();

// --------------------------------------------------------------------------
// MAVLink Send
// --------------------------------------------------------------------------

void sendMavlink(mavlink_message_t& msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  int r1 = udpTx.beginPacket(FC_IP, FC_PORT);
  udpTx.write(buf, len);
  int r2 = udpTx.endPacket();
  if (r1 == 0 || r2 == 0)
    Serial.printf("  [UDP FAIL] r1=%d r2=%d\n", r1, r2);
}

void sendHeartbeat() {
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(SYS_ID, COMP_ID, &msg,
    MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, MAV_STATE_ACTIVE);
  sendMavlink(msg);
}

void sendVelocityBody(float vx, float vy, float vz, float yaw_rate_deg) {
  mavlink_message_t msg;
  float yaw_rate_rad = yaw_rate_deg * (M_PI / 180.0f);
  mavlink_msg_set_position_target_local_ned_pack(
    SYS_ID, COMP_ID, &msg, millis(),
    target_sysid, 1, MAV_FRAME_BODY_NED, 0x0FC7,
    0, 0, 0, vx, vy, vz, 0, 0, 0, 0, yaw_rate_rad
  );
  sendMavlink(msg);
  setpointCount++;
}

void setOffboardMode() {
  mavlink_message_t msg;
  mavlink_msg_set_mode_pack(
    SYS_ID, COMP_ID, &msg,
    target_sysid,
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    0x00060000
  );
  sendMavlink(msg);
  Serial.println("  [TX] SET_MODE offboard sent");
}

// --------------------------------------------------------------------------
// MAVLink Receive
// --------------------------------------------------------------------------

void readMavlink() {
  int packetSize = udpRx.parsePacket();
  if (!packetSize) return;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  udpRx.read(buf, packetSize);
  mavlink_message_t msg;
  mavlink_status_t  status;
  for (int i = 0; i < packetSize; i++) {
    if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) continue;
    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT && msg.sysid != SYS_ID) {
      target_sysid = msg.sysid;
      mavlink_heartbeat_t hb;
      mavlink_msg_heartbeat_decode(&msg, &hb);
      bool armed    = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
      bool offboard = (hb.custom_mode == 393216); // 0x00060000
      if (armed != is_armed) {
        is_armed = armed;
        Serial.printf("[STATE] %s\n", is_armed ? "Armed" : "Disarmed");
        if (is_armed) streamStartTime = millis(); // reset stream timer on arm
      }
      if (offboard != is_offboard) {
        is_offboard = offboard;
        Serial.printf("[MODE] %s\n", is_offboard ? "*** OFFBOARD ***" : "Not offboard");
        if (is_offboard && !seq_active) {
          Serial.println("[SEQ] Offboard detected — starting sequence!");
          seq_active = true;
          seq_step   = 0;
          seq_start  = millis();
        }
      }
    }
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
      mavlink_command_ack_t ack;
      mavlink_msg_command_ack_decode(&msg, &ack);
      Serial.printf("[ACK] cmd=%d result=%d\n", ack.command, ack.result);
    }
  }
}

// --------------------------------------------------------------------------
// Offboard Sequence (non-blocking)
// --------------------------------------------------------------------------

// Returns velocity command for current step
// All movement at 0.3 m/s — slow and safe for indoor testing
void runSequence() {
  if (!seq_active) return;

  // Abort if we leave offboard
  if (!is_offboard) {
    Serial.println("[SEQ] Left offboard — sequence aborted.");
    seq_active = false;
    return;
  }

  unsigned long elapsed = millis() - seq_start;
  float vx = 0, vy = 0, vz = 0;

  switch (seq_step) {
    case 0: // Hover 3s to stabilize
      if (elapsed > 3000) {
        Serial.println("[SEQ] Step 1: Moving forward 0.3m/s for 2s");
        seq_step++; seq_start = millis();
      }
      break;
    case 1: // Forward
      vx = 0.3f;
      if (elapsed > 2000) { seq_step++; seq_start = millis(); Serial.println("[SEQ] Stopping..."); }
      break;
    case 2: // Stop
      if (elapsed > 1000) { seq_step++; seq_start = millis(); Serial.println("[SEQ] Step 2: Moving backward"); }
      break;
    case 3: // Backward
      vx = -0.3f;
      if (elapsed > 2000) { seq_step++; seq_start = millis(); Serial.println("[SEQ] Stopping..."); }
      break;
    case 4: // Stop
      if (elapsed > 1000) { seq_step++; seq_start = millis(); Serial.println("[SEQ] Step 3: Moving right"); }
      break;
    case 5: // Right
      vy = 0.3f;
      if (elapsed > 2000) { seq_step++; seq_start = millis(); Serial.println("[SEQ] Stopping..."); }
      break;
    case 6: // Stop
      if (elapsed > 1000) { seq_step++; seq_start = millis(); Serial.println("[SEQ] Step 4: Moving left"); }
      break;
    case 7: // Left
      vy = -0.3f;
      if (elapsed > 2000) { seq_step++; seq_start = millis(); Serial.println("[SEQ] Stopping..."); }
      break;
    case 8: // Final hover
      if (elapsed > 3000) {
        Serial.println("[SEQ] Complete! Flip RC back to manual.");
        seq_active = false;
        seq_step++;
      }
      break;
    default:
      break;
  }

  sendVelocityBody(vx, vy, vz, 0);
}

// --------------------------------------------------------------------------
// SETUP
// --------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\nWiFi connected. IP: %s\n", WiFi.localIP().toString().c_str());

  udpRx.begin(RX_PORT);
  udpTx.begin(TX_PORT);
  esp_task_wdt_deinit();

  // Wait for FC heartbeat
  Serial.println("Waiting for FC heartbeat...");
  bool foundFC = false;
  while (!foundFC) {
    sendHeartbeat();
    int sz = udpRx.parsePacket();
    if (sz) {
      uint8_t buf[MAVLINK_MAX_PACKET_LEN];
      udpRx.read(buf, sz);
      mavlink_message_t msg;
      mavlink_status_t  status;
      for (int i = 0; i < sz; i++) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
          if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT && msg.sysid != SYS_ID) {
            target_sysid = msg.sysid;
            foundFC = true;
            Serial.printf("FC found! SysID: %d\n", target_sysid);
          }
        }
      }
    }
    delay(200);
  }

  Serial.println("\n=== INSTRUCTIONS ===");
  Serial.println("1. Arm with RC and take off");
  Serial.println("2. Hover stably at 1-2m");
  Serial.println("3. The ESP32 will stream setpoints at 50Hz continuously");
  Serial.println("4. After 3s of armed streaming, try flipping RC to offboard");
  Serial.println("5. Sequence starts automatically when offboard is detected");
  Serial.println("====================\n");

  streamStartTime = millis();
}

// --------------------------------------------------------------------------
// LOOP
// --------------------------------------------------------------------------

unsigned long lastHeartbeat  = 0;
unsigned long lastSetpoint   = 0;

void loop() {
  readMavlink();
  unsigned long now = millis();

  // Heartbeat every 1s
  if (now - lastHeartbeat >= HEARTBEAT_MS) {
    sendHeartbeat();
    lastHeartbeat = now;
  }

  // Setpoints at 50Hz — always stream once armed, hover command if not in sequence
  if (is_armed && now - lastSetpoint >= SETPOINT_MS) {
    lastSetpoint = now;

    // Track how long we've been streaming
    unsigned long streamDuration = now - streamStartTime;
    if (!streamingEnough && streamDuration >= 3000) {
      streamingEnough = true;
      Serial.println("[STREAM] 3s of continuous setpoints reached — try flipping to offboard now!");
    }

    // Run sequence if active, otherwise hover (zero velocity)
    if (seq_active) {
      runSequence(); // sends its own velocity command
    } else {
      sendVelocityBody(0, 0, 0, 0); // hover
    }
  }

  // Print setpoint rate every 5s for diagnostics
  if (now - lastSetpointPrint >= 5000) {
    float rate = setpointCount / ((now - lastSetpointPrint + 5000) / 1000.0f);
    Serial.printf("[DIAG] Setpoints sent in last 5s: %lu (~%.1f Hz)\n", setpointCount, rate);
    setpointCount      = 0;
    lastSetpointPrint  = now;
  }
}