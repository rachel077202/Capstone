// Offboard control example for indoor flight without GPS
// Uses WASD controls OR glove IMU over Serial
// Safe for testing with manual RC backup

#include <atomic>
#include <chrono>
#include <iostream>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <fcntl.h>
#include <cstdio>
#include <string>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

// --- Constants ---
constexpr float       HOVER_THRUST  = 0.55f;
constexpr float       MAX_PITCH_DEG = 15.0f;
constexpr float       MAX_ROLL_DEG  = 15.0f;
constexpr float       MAX_YAW_RATE  = 30.0f;
constexpr float       THRUST_STEP   = 0.02f;
constexpr float       THRUST_MIN    = 0.30f;
constexpr float       THRUST_MAX    = 0.80f;
constexpr int         LOOP_HZ       = 20;
constexpr const char* SERIAL_PORT   = "/dev/ttyACM0";

// --- Shared attitude state ---
struct SharedState {
    std::atomic<float> roll_deg   {0.0f};
    std::atomic<float> pitch_deg  {0.0f};
    std::atomic<float> yaw_deg    {0.0f};
    std::atomic<float> thrust     {HOVER_THRUST};
    std::atomic<bool>  running    {true};
    std::atomic<bool>  offboard_ok{false};
};

// --- RAII raw terminal ---
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

// --- Sender thread: streams attitude setpoints to PX4 ---
void sender_thread(Offboard& offboard, SharedState& state)
{
    const auto period = milliseconds(1000 / LOOP_HZ);

    Offboard::Attitude sp{};
    sp.roll_deg     = 0.0f;
    sp.pitch_deg    = 0.0f;
    sp.yaw_deg      = 0.0f;
    sp.thrust_value = HOVER_THRUST;

    std::cout << "Pre-warming setpoint stream...\n";
    for (int i = 0; i < 30; ++i) {
        offboard.set_attitude(sp);
        sleep_for(period);
    }

    Offboard::Result result = Offboard::Result::Unknown;
    for (int attempt = 1; attempt <= 5; ++attempt) {
        result = offboard.start();
        if (result == Offboard::Result::Success) break;
        std::cerr << "Offboard start attempt " << attempt << "/5 failed, retrying...\n";
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

// --- Serial reader thread: reads glove IMU from ESP32 over USB ---
void serial_reader_thread(SharedState& state, const std::string& port)
{
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::cerr << "⚠ Could not open serial port " << port << " — falling back to WASD only\n";
        return;
    }

    termios tty{};
    cfsetspeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD | CS8);
    tty.c_lflag  = 0;
    tcsetattr(fd, TCSANOW, &tty);

    std::cout << "✓ Serial port " << port << " opened — reading glove IMU data\n";

    std::string line;
    char c;

    while (state.running) {
        if (read(fd, &c, 1) > 0) {
            if (c == '\n') {
                float pitch, roll, yaw, thrust;
                if (sscanf(line.c_str(), "%f,%f,%f,%f",
                           &pitch, &roll, &yaw, &thrust) == 4) {
                    state.pitch_deg = pitch;
                    state.roll_deg  = roll;
                    state.yaw_deg   = yaw;
                    state.thrust    = thrust;
                } else if (line.rfind("# ", 0) == 0) {
                    // Label line from ESP32 e.g. "# Forward"
                    std::cout << "\n[SEQ] " << line.substr(2) << "\n";
                }
                line.clear();
            } else if (c != '\r') {
                line += c;
            }
        }
    }

    close(fd);
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

    auto action    = Action{system.value()};
    auto offboard  = Offboard{system.value()};
    auto telemetry = Telemetry{system.value()};

    std::cout << "✓ System connected\n";

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
        std::cout << "\nContinue anyway? (y/n): ";
        char response;
        std::cin >> response;
        if (response != 'y' && response != 'Y') return 1;
    }

    std::cout << "\n=== IMPORTANT SAFETY INFORMATION ===\n";
    std::cout << "1. This program uses OFFBOARD mode for direct control\n";
    std::cout << "2. Keep your RC transmitter ON as a safety backup\n";
    std::cout << "3. You can switch back to manual mode on your RC at any time\n";
    std::cout << "4. Control input: glove IMU via Serial (WASD as fallback)\n";
    std::cout << "5. Make sure you have enough space for testing\n";
    std::cout << "\nREADY TO ARM AND FLY?\n";
    std::cout << "Press ENTER to continue (or Ctrl+C to cancel)...\n";
    std::cin.ignore();
    std::cin.get();

    if (!telemetry.armed()) {
        std::cout << "Waiting for arm signal (arm via RC or QGC)...\n";
        while (!telemetry.armed()) sleep_for(milliseconds(300));
    }
    std::cout << "✓ Armed\n";

    std::cout << "\nTake off manually to hover height (~1-2 m).\n"
              << "Press ENTER when hovering to start control...\n";
    std::cin.get();

    printf("\n\n\n\n\n\n");

    SharedState state;
    print_status(state);

    std::thread sender(sender_thread, std::ref(offboard), std::ref(state));
    std::thread serial(serial_reader_thread, std::ref(state), std::string(SERIAL_PORT));

    {
        RawTerminal raw;

        while (state.running) {
            char key = read_key();

            switch (key) {
                case 'w': case 'W': state.pitch_deg =  MAX_PITCH_DEG; break;
                case 's': case 'S': state.pitch_deg = -MAX_PITCH_DEG; break;
                case 'd': case 'D': state.roll_deg  =  MAX_ROLL_DEG;  break;
                case 'a': case 'A': state.roll_deg  = -MAX_ROLL_DEG;  break;

                case 'e': case 'E':
                    state.yaw_deg = state.yaw_deg + MAX_YAW_RATE * (1.0f / LOOP_HZ); break;
                case 'q': case 'Q':
                    state.yaw_deg = state.yaw_deg - MAX_YAW_RATE * (1.0f / LOOP_HZ); break;

                case 'r': case 'R': {
                    float t = state.thrust + THRUST_STEP;
                    state.thrust = (t > THRUST_MAX) ? THRUST_MAX : t;
                    break;
                }
                case 'f': case 'F': {
                    float t = state.thrust - THRUST_STEP;
                    state.thrust = (t < THRUST_MIN) ? THRUST_MIN : t;
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
    serial.join();

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
