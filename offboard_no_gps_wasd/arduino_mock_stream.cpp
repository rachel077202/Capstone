// Timed flight sequence for ESP32
// Sends pitch, roll, yaw, thrust over Serial to PC
// Format: "pitch,roll,yaw,thrust\n"

// --- Tuning ---
constexpr float HOVER_THRUST  = 0.55f;
constexpr float FWD_PITCH     = 10.0f;   // degrees forward
constexpr float BACK_PITCH    = -10.0f;  // degrees back
constexpr float LEFT_ROLL     = -10.0f;  // degrees left
constexpr float RIGHT_ROLL    = 10.0f;   // degrees right
constexpr float YAW_RIGHT     = 15.0f;   // degrees CW
constexpr float YAW_LEFT      = -15.0f;  // degrees CCW
constexpr float THRUST_UP     = 0.65f;   // climb
constexpr float THRUST_DOWN   = 0.45f;   // descend
constexpr int   STEP_MS       = 50;      // send rate (20Hz)

// --- One flight step ---
struct Step {
    float pitch, roll, yaw, thrust;
    unsigned long durationMs;
    const char* label;
};

// --- Flight sequence ---
// Edit this to change what the drone does
Step sequence[] = {
    //  pitch        roll    yaw    thrust        ms      label
    {   0.0f,        0.0f,   0.0f,  HOVER_THRUST, 2000,  "Hover"        },
    {   FWD_PITCH,   0.0f,   0.0f,  HOVER_THRUST, 2000,  "Forward"      },
    {   0.0f,        0.0f,   0.0f,  HOVER_THRUST, 1000,  "Hover"        },
    {   BACK_PITCH,  0.0f,   0.0f,  HOVER_THRUST, 2000,  "Backward"     },
    {   0.0f,        0.0f,   0.0f,  HOVER_THRUST, 1000,  "Hover"        },
    {   0.0f,        LEFT_ROLL, 0.0f, HOVER_THRUST, 2000, "Strafe Left" },
    {   0.0f,        0.0f,   0.0f,  HOVER_THRUST, 1000,  "Hover"        },
    {   0.0f,        RIGHT_ROLL, 0.0f, HOVER_THRUST, 2000, "Strafe Right"},
    {   0.0f,        0.0f,   0.0f,  HOVER_THRUST, 1000,  "Hover"        },
    {   0.0f,        0.0f,   YAW_RIGHT, HOVER_THRUST, 2000, "Yaw Right" },
    {   0.0f,        0.0f,   0.0f,  HOVER_THRUST, 1000,  "Hover"        },
    {   0.0f,        0.0f,   YAW_LEFT, HOVER_THRUST, 2000, "Yaw Left"   },
    {   0.0f,        0.0f,   0.0f,  HOVER_THRUST, 1000,  "Hover"        },
    {   0.0f,        0.0f,   0.0f,  THRUST_UP,    2000,  "Climb"        },
    {   0.0f,        0.0f,   0.0f,  HOVER_THRUST, 1000,  "Hover"        },
    {   0.0f,        0.0f,   0.0f,  THRUST_DOWN,  2000,  "Descend"      },
    {   0.0f,        0.0f,   0.0f,  HOVER_THRUST, 3000,  "Final Hover"  },
};

constexpr int NUM_STEPS = sizeof(sequence) / sizeof(sequence[0]);

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("ESP32 Flight Sequence Ready");
}

void loop() {
    for (int i = 0; i < NUM_STEPS; i++) {
        Step& s = sequence[i];
        Serial.print("# ");
        Serial.println(s.label);  // label line (PC will ignore it, not CSV)

        unsigned long start = millis();
        while (millis() - start < s.durationMs) {
            Serial.printf("%.2f,%.2f,%.2f,%.2f\n",
                          s.pitch, s.roll, s.yaw, s.thrust);
            delay(STEP_MS);
        }
    }

    // Hold hover after sequence ends
    Serial.println("# Sequence complete — holding hover");
    while (true) {
        Serial.printf("%.2f,%.2f,%.2f,%.2f\n", 0.0f, 0.0f, 0.0f, HOVER_THRUST);
        delay(STEP_MS);
    }
}
