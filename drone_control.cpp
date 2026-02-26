#include <WiFi.h>
#include <WiFiUdp.h>
#include "mavlink/common/mavlink.h"

// THIS IS JUST PLACEHOLDER EXAMPLE CODE FOR WHEN WE WANT TO TRY WHEN WE WANT TO UPLOAD TO AN ESP32 CONTROLLER

// --- WiFi / Network Config ---
const char* WIFI_SSID     = "your_ssid";
const char* WIFI_PASSWORD = "your_password";
const char* FC_IP         = "192.168.x.x";  // IP of ESP8266/flight controller
const uint16_t FC_PORT    = 14550;
const uint16_t LOCAL_PORT = 14551;

// --- MAVLink IDs ---
const uint8_t SYS_ID  = 255;  // GCS system ID
const uint8_t COMP_ID = 190;  // GCS component ID

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

  // PX4 expects SET_POSITION_TARGET_LOCAL_NED in body frame via coordinate_frame = MAV_FRAME_BODY_NED
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
  // MAV_CMD_DO_SET_MODE: base_mode=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, custom_mode=6 (OFFBOARD on PX4)
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
// ARM the drone
// --------------------------------------------------------------------------
void armDrone() {
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
    SYS_ID, COMP_ID, &msg,
    1, 1,
    MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,  // 1 = arm
    0, 0, 0, 0, 0, 0
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

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected. IP: " + WiFi.localIP().toString());

  udp.begin(LOCAL_PORT);
  Serial.println("UDP started.");

  // Give PX4 a moment then arm + takeoff
  delay(2000);
  Serial.println("Arming...");
  armDrone();
  delay(3000);

  Serial.println("Taking off...");
  takeoff(5.0);
  delay(6000);

  // Send a zero setpoint before requesting OFFBOARD mode (PX4 requires this)
  sendVelocityBody(0, 0, 0, 0);
  delay(100);
  setOffboardMode();
  delay(500);
  offboard_active = true;
  Serial.println("Offboard mode requested.");
}

// --------------------------------------------------------------------------
// LOOP
// --------------------------------------------------------------------------
unsigned long lastHeartbeat = 0;
unsigned long lastLoop      = 0;

void loop() {
  unsigned long now = millis();

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
