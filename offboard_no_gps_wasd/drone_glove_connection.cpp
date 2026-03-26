// Offboard control example for indoor flight without GPS
// Uses WASD controls - works like flying in Acro/Stabilize mode
// Safe for testing with manual RC backup
#include <drone_control.h>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

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

// --- initialize ---
void initialize(const std::string connection_url, DroneContext& ctx)
{
    Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};
    if (mavsdk.add_any_connection(connection_url) != ConnectionResult::Success) {
        std::cerr << "Connection failed\n";
        return;
    }

    auto system = mavsdk.first_autopilot(5.0);
    if (!system) {
        std::cerr << "Timed out waiting for system\n";
        return;
    }

    ctx.action   = Action{system.value()};
    ctx.offboard = Offboard{system.value()};

    // ... health checks, safety prompt, arm/hover wait (same as before) ...

    // Spawn sender into the caller-owned ctx
    ctx.sender = std::thread(sender_thread,
                             std::ref(ctx.offboard),
                             std::ref(ctx.state));
    return;
}

// --- droneControl ---
// Takes ctx by reference so it can reach state and sender.
void droneControl(DroneContext& ctx, float pitch, float yaw, float roll, float thrust)
{
    if (ctx.state.running) {
        ctx.state.roll_deg  = roll;
        ctx.state.pitch_deg = pitch;
        ctx.state.yaw_deg   = yaw;
        ctx.state.thrust    = thrust;

        print_status(ctx.state);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / LOOP_HZ));
    }
}

// --- shutDown ---
// Also takes ctx so it can join the thread and disarm.
void shutDown(DroneContext& ctx)
{
    ctx.state.running = false;   // signals sender_thread to exit
    if (ctx.sender.joinable())
        ctx.sender.join();       // wait for it to finish cleanly

    std::cout << "Offboard control stopped.\n"
              << "Use your RC to land the drone manually.\n"
              << "Once landed, press ENTER to disarm...\n";
    std::cin.get();

    ctx.action.disarm();
    std::cout << "✓ Disarmed\n";
}
