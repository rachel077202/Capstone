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
bool offboard_active       = false;
bool is_armed              = false;
bool is_offboard           = false;
int  seq_step              = 0;
unsigned long seq_start    = 0;

const int LOOP_HZ = 20;
const int LOOP_MS = 1000 / LOOP_HZ;

// Velocity command struct
struct VelocityCmd {
  float vx, vy, vz, yaw_rate;
};

// --------------------------------------------------------------------------
// Forward declarations
// --------------------------------------------------------------------------
void sendMavlink(mavlink_message_t& msg);
void sendHeartbeat();
void sendHeartbeatOffboard();
void sendVelocityBody(float vx, float vy, float vz, float yaw_rate_deg);
void setOffboardMode();
void readMavlink();
VelocityCmd getSequenceCmd();

// --------------------------------------------------------------------------
// MAVLink Send Helpers
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

void sendHeartbeatOffboard() {
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(SYS_ID, COMP_ID, &msg,
    MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0x00060000, MAV_STATE_ACTIVE);
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
  mavlink_status_t status;
  for (int i = 0; i < packetSize; i++) {
    if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) continue;
    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT && msg.sysid != SYS_ID) {
      target_sysid = msg.sysid;
      mavlink_heartbeat_t hb;
      mavlink_msg_heartbeat_decode(&msg, &hb);
      bool armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
      if (armed != is_armed) {
        is_armed = armed;
        Serial.printf("[STATE] %s\n", is_armed ? "Armed" : "Disarmed");
      }
      bool offboard = (hb.custom_mode == 393216); // 0x00060000 = PX4 offboard
      if (offboard != is_offboard) {
        is_offboard = offboard;
        Serial.printf("[MODE] %s\n", is_offboard ? "Offboard!" : "Not offboard");
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
// Offboard Sequence (non-blocking state machine)
// --------------------------------------------------------------------------

VelocityCmd getSequenceCmd() {
  unsigned long elapsed = millis() - seq_start;
  VelocityCmd cmd = {0, 0, 0, 0};

  switch (seq_step) {

    case 0: // Hover and stream setpoints — wait for RC to confirm offboard
      if (is_offboard) {
        Serial.println("[SEQ] Offboard detected! Hovering 3s...");
        seq_step++; seq_start = millis();
      }
      break;

    case 1: // Hover 3s
      if (elapsed > 3000) {
        Serial.println("[SEQ] Moving forward...");
        seq_step++; seq_start = millis();
      }
      break;

    case 2: // Forward 2s
      cmd.vx = 0.3f;
      if (elapsed > 2000) {
        Serial.println("[SEQ] Stopping...");
        seq_step++; seq_start = millis();
      }
      break;

    case 3: // Stop 1s
      if (elapsed > 1000) {
        Serial.println("[SEQ] Moving backward...");
        seq_step++; seq_start = millis();
      }
      break;

    case 4: // Backward 2s
      cmd.vx = -0.3f;
      if (elapsed > 2000) {
        Serial.println("[SEQ] Stopping...");
        seq_step++; seq_start = millis();
      }
      break;

    case 5: // Stop 1s
      if (elapsed > 1000) {
        Serial.println("[SEQ] Moving right...");
        seq_step++; seq_start = millis();
      }
      break;

    case 6: // Right 2s
      cmd.vy = 0.3f;
      if (elapsed > 2000) {
        Serial.println("[SEQ] Stopping...");
        seq_step++; seq_start = millis();
      }
      break;

    case 7: // Stop 1s
      if (elapsed > 1000) {
        Serial.println("[SEQ] Moving left...");
        seq_step++; seq_start = millis();
      }
      break;

    case 8: // Left 2s
      cmd.vy = -0.3f;
      if (elapsed > 2000) {
        Serial.println("[SEQ] Final hover...");
        seq_step++; seq_start = millis();
      }
      break;

    case 9: // Final hover 3s
      if (elapsed > 3000) {
        Serial.println("[SEQ] Complete! Switch RC back to manual.");
        offboard_active = false;
        seq_step++;
      }
      break;

    default:
      break;
  }

  return cmd;
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

  Serial.println("Waiting for FC heartbeat...");
  bool foundFC = false;
  while (!foundFC) {
    sendHeartbeat();
    int packetSize = udpRx.parsePacket();
    if (packetSize) {
      uint8_t buf[MAVLINK_MAX_PACKET_LEN];
      udpRx.read(buf, packetSize);
      mavlink_message_t msg;
      mavlink_status_t status;
      for (int i = 0; i < packetSize; i++) {
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

  Serial.println("Waiting for arm via RC transmitter...");
}

// --------------------------------------------------------------------------
// LOOP
// --------------------------------------------------------------------------

unsigned long lastHeartbeat = 0;
unsigned long lastLoop      = 0;

void loop() {
  readMavlink();
  unsigned long now = millis();

  if (now - lastHeartbeat >= 1000) {
    sendHeartbeat();
    lastHeartbeat = now;
  }

  if (now - lastLoop < LOOP_MS) return;
  lastLoop = now;

  // Start streaming setpoints as soon as armed so PX4 accepts offboard switch
  if (!offboard_active && is_armed && seq_step == 0) {
    Serial.println("Armed! Switch RC to offboard mode to start sequence.");
    offboard_active = true;
    seq_start = millis();
  }

  // Abort if disarmed mid-sequence
  if (offboard_active && !is_armed) {
    Serial.println("[ABORT] Disarmed! Stopping.");
    offboard_active = false;
  }

  if (offboard_active) {
    VelocityCmd cmd = getSequenceCmd();
    sendVelocityBody(cmd.vx, cmd.vy, cmd.vz, cmd.yaw_rate);
  }
}
