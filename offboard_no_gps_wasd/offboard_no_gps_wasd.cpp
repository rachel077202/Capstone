// Offboard control example for indoor flight without GPS
// Uses WASD controls - works like flying in Acro/Stabilize mode
// Safe for testing with manual RC backup

#include <atomic>
#include <chrono>
#include <iostream>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

// consts for WASD control (could be tuned for our specific drone)
constexpr float HOVER_THRUST  = 0.55f;
constexpr float MAX_PITCH_DEG = 15.0f;
constexpr float MAX_ROLL_DEG  = 15.0f;
constexpr float MAX_YAW_RATE  = 30.0f;  // degrees per second
constexpr float THRUST_STEP   = 0.02f;
constexpr float THRUST_MIN    = 0.30f;
constexpr float THRUST_MAX    = 0.80f;
constexpr int   LOOP_HZ       = 20;

// Attitude state (written by input thread, read by send thread)
struct SharedState {
    std::atomic<float> roll_deg   {0.0f};
    std::atomic<float> pitch_deg  {0.0f};
    std::atomic<float> yaw_deg    {0.0f};
    std::atomic<float> thrust     {HOVER_THRUST};
    std::atomic<bool>  running    {true};
    std::atomic<bool>  offboard_ok{false};
};

// RAII class to set terminal to raw mode for non-blocking input
struct RawTerminal {
    termios original;
    RawTerminal() {
        tcgetattr(STDIN_FILENO, &original);
        termios raw = original;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN]  = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }
    ~RawTerminal() { tcsetattr(STDIN_FILENO, TCSANOW, &original); }
};

char read_key() {
    char c = '\0';
    read(STDIN_FILENO, &c, 1);
    return c;
}

void print_status(const SharedState& s) {
    printf("\r\033[6A");
    printf("  Roll  : %+6.1f deg  (A = left,  D = right)   \n", s.roll_deg.load());
    printf("  Pitch  : %+6.1f deg  (W = fwd,   S = back)    \n", s.pitch_deg.load());
    printf("  Yaw    : %+6.1f deg  (Q = CCW,   E = CW)      \n", s.yaw_deg.load());
    printf("  Thrust : %6.3f      (R = up,    F = down)    \n", s.thrust.load());
    printf("  Mode   : %s                                    \n",
           s.offboard_ok ? "OFFBOARD ACTIVE" : "waiting...");
    printf("  Keys   : WASD/QE/RF  |  H = level  |  ESC = quit\n");
    fflush(stdout);
}

void sender_thread(Offboard& offboard, SharedState& state)
{
    const auto period = milliseconds(1000 / LOOP_HZ);

    Offboard::Attitude sp{};
    sp.roll_deg     = 0.0f;
    sp.pitch_deg    = 0.0f;
    sp.yaw_deg      = 0.0f;
    sp.thrust_value = HOVER_THRUST;

    // Pre-warm: send 30 setpoints (~1.5s) before attempting offboard start.
    // WiFi latency means 10 is often not enough — 30 is more reliable.
    std::cout << "Pre-warming setpoint stream...\n";
    for (int i = 0; i < 30; ++i) {
        offboard.set_attitude(sp);
        sleep_for(period);
    }

    // Attempt offboard start with retries (helps on flaky WiFi)
    Offboard::Result result = Offboard::Result::Unknown;
    for (int attempt = 1; attempt <= 5; ++attempt) {
        result = offboard.start();
        if (result == Offboard::Result::Success) break;
        std::cerr << "Offboard start attempt " << attempt << "/5 failed, retrying...\n";
        // Keep sending setpoints between retries so PX4 doesn't time out
        for (int i = 0; i < 10; ++i) {
            offboard.set_attitude(sp);
            sleep_for(period);
        }
    }

    if (result != Offboard::Result::Success) {
        std::cerr << "Offboard start failed after 5 attempts\n";
        state.running = false;
        return;
    }

    state.offboard_ok = true;
    print_status(state);

    // Main loop: send attitude setpoint at LOOP_HZ
    while (state.running) {
        sp.roll_deg     = state.roll_deg;
        sp.pitch_deg    = state.pitch_deg;
        sp.yaw_deg      = state.yaw_deg;
        sp.thrust_value = state.thrust;
        offboard.set_attitude(sp);
        sleep_for(period);
    }

    offboard.stop();
    state.offboard_ok = false;
}

void usage(const std::string bin_name)
{
    std::cout << "Usage: " << bin_name << " <connection_url>\n";
    std::cout << "Connection URL format should be:\n";
    std::cout << " For UDP (WiFi, listen mode): udpin://0.0.0.0:14550\n";
    std::cout << " For UDP (send to IP):        udpout://192.168.4.1:14550\n";
    std::cout << " For Serial:                  serial:///dev/ttyACM0:57600\n";
    std::cout << "Examples:\n";
    std::cout << "  " << bin_name << " udpin://0.0.0.0:14550\n";
    std::cout << "  " << bin_name << " serial:///dev/ttyACM0:57600\n";
}

void pitch(SharedState state, float percent)
{
    state.pitch_deg =  MAX_PITCH_DEG * percent;
}

void roll(SharedState state, float percent)
{
    state.roll_deg =  MAX_ROLL_DEG * percent;
}

void yaw(SharedState state, float percent) 
{
    state.yaw_deg = state.yaw_deg + MAX_YAW_RATE * percent * (1.0f / LOOP_HZ);
}

void thrust(SharedState state, float percent) 
{
    float t = state.thrust + THRUST_STEP * percent;
    state.thrust = (t > THRUST_MAX) ? THRUST_MAX : t;
}

int main(int argc, char** argv)
{
    if (argc != 2) { usage(argv[0]); return 1; }

    Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};
    if (mavsdk.add_any_connection(argv[1]) != ConnectionResult::Success) {
        std::cerr << "Connection failed\n";
        return 1;
    }

    std::cout << "Waiting for system to connect...\n";
    auto system = mavsdk.first_autopilot(5.0);
    if (!system) {
        std::cerr << "Timed out waiting for system\n";
        return 1;
    }

    auto action   = Action{system.value()};
    auto offboard = Offboard{system.value()};
    auto telemetry = Telemetry{system.value()};

    std::cout << "✓ System connected\n";

    // Check basic health
    std::cout << "\nChecking system health...\n";
    auto health = telemetry.health();
    std::cout << "  Accelerometer: " << (health.is_accelerometer_calibration_ok ? "✓" : "✗") << "\n";
    std::cout << "  Gyroscope:     " << (health.is_gyrometer_calibration_ok     ? "✓" : "✗") << "\n";
    std::cout << "  Magnetometer:  " << (health.is_magnetometer_calibration_ok  ? "✓" : "✗") << "\n";
    std::cout << "  Local position:" << (health.is_local_position_ok            ? "✓" : "✗") << "\n";
    std::cout << "  Armable:       " << (health.is_armable                      ? "✓" : "✗") << "\n";

    if (!health.is_armable) {
        std::cerr << "\n⚠ WARNING: System reports not armable.\n";
        std::cerr << "Check QGroundControl for specific failure reasons.\n";
        std::cerr << "Common issues: battery low, safety switch, calibration needed.\n";
        std::cout << "\nContinue anyway? (y/n): ";
        char response;
        std::cin >> response;
        if (response != 'y' && response != 'Y') return 1;
    }

    std::cout << "\n=== IMPORTANT SAFETY INFORMATION ===\n";
    std::cout << "1. This program uses OFFBOARD mode for direct control\n";
    std::cout << "2. Keep your RC transmitter ON as a safety backup\n";
    std::cout << "3. You can switch back to manual mode on your RC at any time\n";
    std::cout << "4. The drone responds to WASD keyboard input in real time\n";
    std::cout << "5. Make sure you have enough space for testing\n";
    std::cout << "\nREADY TO ARM AND FLY?\n";
    std::cout << "Press ENTER to continue (or Ctrl+C to cancel)...\n";
    std::cin.ignore();
    std::cin.get();

    // Wait for manual arm via RC / QGC
    if (!telemetry.armed()) {
        std::cout << "Waiting for arm signal (arm via RC or QGC)...\n";
        while (!telemetry.armed()) sleep_for(milliseconds(300));
    }
    std::cout << "✓ Armed\n";

    std::cout << "\nTake off manually to hover height (~1-2 m).\n"
              << "Press ENTER when hovering to start WASD control...\n";
    std::cin.get();

    printf("\n\n\n\n\n\n");

    SharedState state;
    print_status(state);

    // Start sender thread — pre-warm + offboard start happen inside
    std::thread sender(sender_thread, std::ref(offboard), std::ref(state));

    // Main input loop
    {
        RawTerminal raw;

        while (state.running) {
            char key = read_key();

            switch (key) {
                case 'w': case 'W': pitch(state, 1.0); break;
                case 's': case 'S': pitch(state, -1.0); break;
                case 'd': case 'D': roll(state, 1.0);  break;
                case 'a': case 'A': roll(state, -1.0);  break;

                case 'e': case 'E':
                    yaw(state, 1.0); break;
                case 'q': case 'Q':
                    yaw(state, -1.0); break;

                case 'r': case 'R': {
                    thrust(state, 1.0);
                    break;
                }
                case 'f': case 'F': {
                    thrust(state, -1.0);
                    break;
                }

                case 'h': case 'H':
                    state.roll_deg  = 0.0f;
                    state.pitch_deg = 0.0f;
                    break;

                case 27:  // ESC
                    std::cout << "\nESC — stopping offboard...\n";
                    state.running = false;
                    break;

                case '\0':
                    state.roll_deg  = 0.0f;
                    state.pitch_deg = 0.0f;
                    break;

                default: break;
            }

            print_status(state);
            sleep_for(milliseconds(1000 / LOOP_HZ));
        }
    }

    sender.join();

    std::cout << "\nOffboard control stopped.\n";
    std::cout << "Use your RC to land the drone manually.\n";
    std::cout << "Once landed, press ENTER to disarm...\n";
    std::cin.get();
    std::cout << "Disarming...\n";
    action.disarm();
    std::cout << "✓ Disarmed\n";
    std::cout << "\nProgram finished!\n";
    return 0;
}
