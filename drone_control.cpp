#include <WiFi.h>
#include <WiFiUdp.h>
#include <MAVLink.h>
#include <esp_task_wdt.h>

// THIS IS JUST PLACEHOLDER EXAMPLE CODE FOR WHEN WE WANT TO TRY WHEN WE WANT TO UPLOAD TO AN ESP32 CONTROLLER

// --- WiFi / Network Config ---
const char* WIFI_SSID     = "PixRacer";
const char* WIFI_PASSWORD = "pixracer";
const char* FC_IP         = "192.168.4.3";  // IP of ESP8266/flight controller
const uint16_t FC_PORT    = 14555;  // WIFI_UDP_CPORT — ESP8266 receives on this port
const uint16_t LOCAL_PORT = 14550;  // WIFI_UDP_HPORT — ESP8266 sends telemetry to this port

// --- MAVLink IDs ---
const uint8_t SYS_ID  = 255;  // GCS system ID (255 = standard GCS)
const uint8_t COMP_ID = 0;    // 0 = generic GCS component (QGC uses 0)

// --- Speed Config ---
const float SPEED      = 2.0f;   // m/s
const float YAW_SPEED  = 30.0f;  // deg/s
const float VERT_SPEED = 1.5f;   // m/s
const int   LOOP_HZ    = 20;
const int   LOOP_MS    = 1000 / LOOP_HZ;

WiFiUDP udp;
bool offboard_active = false;

// --------------------------------------------------------------------------
// Send a raw MAVLink message over UDP
// --------------------------------------------------------------------------
void sendMavlink(mavlink_message_t& msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  udp.beginPacket(FC_IP, FC_PORT);
  udp.write(buf, len);
  udp.endPacket();
}

// --------------------------------------------------------------------------
// Read incoming MAVLink packets and print command ACKs
// --------------------------------------------------------------------------
void readMavlink() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    udp.read(buf, packetSize);
    mavlink_message_t msg;
    mavlink_status_t status;
    for (int i = 0; i < packetSize; i++) {
      if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
        if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
          mavlink_command_ack_t ack;
          mavlink_msg_command_ack_decode(&msg, &ack);
          Serial.print("ACK command: ");
          Serial.print(ack.command);
          Serial.print(" result: ");
          Serial.println(ack.result);  // 0=accepted, 1=temp rejected, 2=denied, 3=unsupported
        }
        if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
          mavlink_heartbeat_t hb;
          mavlink_msg_heartbeat_decode(&msg, &hb);
          Serial.print("FC Heartbeat — base_mode: ");
          Serial.print(hb.base_mode);
          Serial.print(" custom_mode: ");
          Serial.print(hb.custom_mode);
          Serial.print(" system_status: ");
          Serial.println(hb.system_status);
          // system_status: 0=uninit,1=boot,2=calibrating,3=standby,4=active,5=critical,6=emergency
        }
        if (msg.msgid == MAVLINK_MSG_ID_SYS_STATUS) {
          mavlink_sys_status_t sys;
          mavlink_msg_sys_status_decode(&msg, &sys);
          Serial.print("Pre-arm errors (onboard_control_sensors_health): 0x");
          Serial.println(sys.onboard_control_sensors_health, HEX);
        }
      }
    }
  }
}

// --------------------------------------------------------------------------
// Wait ms milliseconds while sending heartbeats and reading MAVLink
// --------------------------------------------------------------------------
void waitWithHeartbeat(unsigned long ms) {
  unsigned long start = millis();
  unsigned long lastHB = 0;
  while (millis() - start < ms) {
    readMavlink();
    if (millis() - lastHB >= 1000) {
      sendHeartbeat();
      lastHB = millis();
    }
    delay(20);
  }
}

// --------------------------------------------------------------------------
// HEARTBEAT — must be sent at ~1 Hz so PX4 keeps the GCS link alive
// --------------------------------------------------------------------------
void sendHeartbeat() {
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(
    SYS_ID, COMP_ID, &msg,
    MAV_TYPE_GCS,
    MAV_AUTOPILOT_INVALID,
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    0,
    MAV_STATE_ACTIVE
  );
  sendMavlink(msg);
}

// --------------------------------------------------------------------------
// SET_POSITION_TARGET_BODY_NED — velocity body frame
//   type_mask 0b0000111111000111 = 0x0FC7 ignores everything except vx,vy,vz,yaw_rate
// --------------------------------------------------------------------------
void sendVelocityBody(float vx, float vy, float vz, float yaw_rate_deg) {
  mavlink_message_t msg;
  float yaw_rate_rad = yaw_rate_deg * (M_PI / 180.0f);

  mavlink_msg_set_position_target_local_ned_pack(
    SYS_ID, COMP_ID, &msg,
    millis(),                        // time_boot_ms
    1,                               // target system (PX4 default)
    1,                               // target component
    MAV_FRAME_BODY_NED,              // body-relative frame
    0x0FC7,                          // type_mask: only vx, vy, vz, yaw_rate
    0, 0, 0,                         // x, y, z position (ignored)
    vx, vy, vz,                      // vx, vy, vz m/s
    0, 0, 0,                         // ax, ay, az (ignored)
    0,                               // yaw (ignored)
    yaw_rate_rad                     // yaw rate rad/s
  );
  sendMavlink(msg);
}

// --------------------------------------------------------------------------
// Command PX4 to switch into OFFBOARD mode
// --------------------------------------------------------------------------
void setOffboardMode() {
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
    SYS_ID, COMP_ID, &msg,
    1, 1,                                      // target sys/comp
    MAV_CMD_DO_SET_MODE,
    0,                                         // confirmation
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,         // param1
    6,                                         // param2: PX4 custom mode 6 = OFFBOARD
    0, 0, 0, 0, 0
  );
  sendMavlink(msg);
}

// --------------------------------------------------------------------------
// Set a PX4 parameter (float value)
// --------------------------------------------------------------------------
void setParam(const char* param_id, float value) {
  mavlink_message_t msg;
  char id[16] = {};
  strncpy(id, param_id, 16);
  mavlink_msg_param_set_pack(
    SYS_ID, COMP_ID, &msg,
    1, 1,           // target sys/comp
    id,
    value,
    MAV_PARAM_TYPE_REAL32
  );
  sendMavlink(msg);
  Serial.print("Set param: ");
  Serial.print(param_id);
  Serial.print(" = ");
  Serial.println(value);
}

// --------------------------------------------------------------------------
// Disable pre-arm checks so we can arm without GPS etc.
// --------------------------------------------------------------------------
void rebootFC() {
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
    SYS_ID, COMP_ID, &msg,
    1, 1,
    MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
    0,
    1,  // 1 = reboot autopilot
    0, 0, 0, 0, 0, 0
  );
  sendMavlink(msg);
  Serial.println("FC rebooting...");
}

void disablePrearmChecks() {
  setParam("COM_ARM_WO_GPS",   1);      delay(200);
  setParam("CBRK_USB_CHK",     197848); delay(200);
  setParam("CBRK_IO_SAFETY",   22027);  delay(200);
  setParam("CBRK_AIRSPD_CHK",  162128); delay(200);
  setParam("COM_PREARM_MODE",  0);      delay(200);
  rebootFC();
}

// --------------------------------------------------------------------------
// ARM the drone
// --------------------------------------------------------------------------
void armDrone(bool force = false) {
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
    SYS_ID, COMP_ID, &msg,
    1, 1,
    MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,                    // 1 = arm
    force ? 21196 : 0,    // 21196 = force arm magic number (bypasses pre-arm checks)
    0, 0, 0, 0, 0
  );
  sendMavlink(msg);
}

// --------------------------------------------------------------------------
// TAKEOFF
// --------------------------------------------------------------------------
void takeoff(float altitude_m) {
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
    SYS_ID, COMP_ID, &msg,
    1, 1,
    MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0,   // params 1-4 (unused)
    NAN, NAN,     // lat/lon (use current)
    altitude_m
  );
  sendMavlink(msg);
}

// --------------------------------------------------------------------------
// LAND
// --------------------------------------------------------------------------
void land() {
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
    SYS_ID, COMP_ID, &msg,
    1, 1,
    MAV_CMD_NAV_LAND,
    0,
    0, 0, 0, 0,
    NAN, NAN, 0
  );
  sendMavlink(msg);
}

// --------------------------------------------------------------------------
// SETUP
// --------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);  // wait for USB serial to connect
  }
  delay(1000);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected. IP: " + WiFi.localIP().toString());

  udp.begin(LOCAL_PORT);
  Serial.println("UDP started.");

  // Disable watchdog so long startup sequence doesn't trigger a reset
  esp_task_wdt_deinit();

  // Wait for a heartbeat FROM the FC specifically
  Serial.println("Waiting for FC heartbeat...");
  bool fcHeartbeat = false;
  unsigned long giveUp = millis() + 30000;
  while (!fcHeartbeat && millis() < giveUp) {
    sendHeartbeat();
    delay(200);
    int packetSize = udp.parsePacket();
    if (packetSize) {
      uint8_t buf[MAVLINK_MAX_PACKET_LEN];
      udp.read(buf, packetSize);
      mavlink_message_t msg;
      mavlink_status_t status;
      for (int i = 0; i < packetSize; i++) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
          if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT && msg.sysid != SYS_ID) {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
            Serial.print("FC heartbeat received! sysid: ");
            Serial.print(msg.sysid);
            Serial.print(" status: ");
            Serial.println(hb.system_status);
            fcHeartbeat = true;
          }
        }
      }
    }
  }

  if (!fcHeartbeat) {
    Serial.println("No FC heartbeat received. Halting.");
    while (true) delay(1000);
  }

  // Send heartbeats for 5 seconds so FC registers us as a trusted GCS
  Serial.println("Establishing GCS link...");
  waitWithHeartbeat(5000);

  // Now proceed with arm + takeoff
  // Wait for RC arm — user arms manually with RC controller
  Serial.println("Waiting for you to arm with RC controller...");
  bool armed = false;
  while (!armed) {
    readMavlink();
    sendHeartbeat();
    // Check heartbeat for armed state (base_mode bit 7 = MAV_MODE_FLAG_SAFETY_ARMED)
    int packetSize = udp.parsePacket();
    if (packetSize) {
      uint8_t buf[MAVLINK_MAX_PACKET_LEN];
      udp.read(buf, packetSize);
      mavlink_message_t msg;
      mavlink_status_t status;
      for (int i = 0; i < packetSize; i++) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
          if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT && msg.sysid != SYS_ID) {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
            if (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) {
              Serial.println("Drone is armed!");
              armed = true;
            }
          }
        }
      }
    }
    delay(50);
  }

  // Give it a moment to stabilize then switch to offboard
  Serial.println("Switching to offboard mode...");
  // Send several setpoints first (PX4 requires this before offboard will engage)
  for (int i = 0; i < 20; i++) {
    sendVelocityBody(0, 0, 0, 0);
    sendHeartbeat();
    delay(50);
  }
  setOffboardMode();
  waitWithHeartbeat(1000);

  offboard_active = true;
  Serial.println("Offboard active — sending velocity commands.");
}

// --------------------------------------------------------------------------
// LOOP
// --------------------------------------------------------------------------
unsigned long lastHeartbeat = 0;
unsigned long lastLoop      = 0;

void loop() {
  unsigned long now = millis();

  // Always read incoming MAVLink (catches ACKs and other messages)
  readMavlink();

  // Heartbeat at 1 Hz
  if (now - lastHeartbeat >= 1000) {
    sendHeartbeat();
    lastHeartbeat = now;
  }

  // Control loop at LOOP_HZ
  if (now - lastLoop >= LOOP_MS) {
    lastLoop = now;

    if (offboard_active) {
      // --- REPLACE THIS SECTION WITH YOUR IMU-DERIVED VALUES ---
      float forward  = 0.0f;   // +forward / -backward  (m/s)
      float right    = 0.0f;   // +right   / -left       (m/s)
      float vertical = 0.0f;   // +down    / -up         (m/s)  Note: NED convention
      float yaw      = 0.0f;   // +clockwise             (deg/s)
      // ----------------------------------------------------------

      sendVelocityBody(forward, right, vertical, yaw);
    }
  }
}
