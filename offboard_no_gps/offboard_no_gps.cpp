
// Offboard control example for indoor flight without GPS
// Uses attitude control mode - works like flying in Acro/Stabilize mode
// Safe for testing with manual RC backup

#include <chrono>
#include <iostream>
#include <thread>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect via serial: serial:///dev/ttyACM0:57600\n";
}

// Simple attitude-based hover and movement test
bool test_attitude_control(mavsdk::Offboard& offboard)
{
    std::cout << "\n=== Starting Attitude Control Test (No GPS Required) ===\n";

    // Initialize with hover attitude
    Offboard::Attitude hover;
    hover.roll_deg = 0.0f;
    hover.pitch_deg = 0.0f;
    hover.yaw_deg = 0.0f;
    hover.thrust_value = 0.55f;  // 55% throttle - adjust for your drone

    std::cout << "Setting initial hover attitude...\n";
    offboard.set_attitude(hover);

    // Start offboard mode
    std::cout << "Starting offboard mode...\n";
    // ADD: Send the setpoint multiple times before starting (bypassing wifi timeout issue)
    for (int i = 0; i < 10; i++) {
        offboard.set_attitude(hover);
        sleep_for(milliseconds(100));
    }
    // Same as before
    Offboard::Result offboard_result = offboard.start();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard start failed: " << offboard_result << '\n';
        return false;
    }
    std::cout << "✓ Offboard mode active!\n";

    // Test 1: Hover for 3 seconds
    std::cout << "\nTest 1: Hovering (3 seconds)...\n";
    for (int i = 0; i < 60; i++) {  // 3 seconds at 20Hz
        offboard.set_attitude(hover);
        sleep_for(milliseconds(50));
    }

    // Test 2: Roll right
    std::cout << "Test 2: Rolling 15° right (2 seconds)...\n";
    hover.roll_deg = 15.0f;
    for (int i = 0; i < 40; i++) {  // 2 seconds at 20Hz
        offboard.set_attitude(hover);
        sleep_for(milliseconds(50));
    }

    // Back to level
    std::cout << "Leveling out...\n";
    hover.roll_deg = 0.0f;
    for (int i = 0; i < 20; i++) {  // 1 second
        offboard.set_attitude(hover);
        sleep_for(milliseconds(50));
    }

    // Test 3: Roll left
    std::cout << "Test 3: Rolling 15° left (2 seconds)...\n";
    hover.roll_deg = -15.0f;
    for (int i = 0; i < 40; i++) {
        offboard.set_attitude(hover);
        sleep_for(milliseconds(50));
    }

    // Back to level
    std::cout << "Leveling out...\n";
    hover.roll_deg = 0.0f;
    for (int i = 0; i < 20; i++) {
        offboard.set_attitude(hover);
        sleep_for(milliseconds(50));
    }

    // Test 4: Pitch forward
    std::cout << "Test 4: Pitching 10° forward (2 seconds)...\n";
    hover.pitch_deg = 10.0f;
    for (int i = 0; i < 40; i++) {
        offboard.set_attitude(hover);
        sleep_for(milliseconds(50));
    }

    // Back to level
    std::cout << "Leveling out...\n";
    hover.pitch_deg = 0.0f;
    for (int i = 0; i < 20; i++) {
        offboard.set_attitude(hover);
        sleep_for(milliseconds(50));
    }

    // Test 5: Gentle climb
    std::cout << "Test 5: Gentle climb (2 seconds)...\n";
    hover.thrust_value = 0.60f;  // Increase throttle
    for (int i = 0; i < 40; i++) {
        offboard.set_attitude(hover);
        sleep_for(milliseconds(50));
    }

    // Back to hover throttle
    std::cout << "Returning to hover throttle...\n";
    hover.thrust_value = 0.55f;
    for (int i = 0; i < 20; i++) {
        offboard.set_attitude(hover);
        sleep_for(milliseconds(50));
    }

    // Final hover before stopping
    std::cout << "Final hover (2 seconds)...\n";
    for (int i = 0; i < 40; i++) {
        offboard.set_attitude(hover);
        sleep_for(milliseconds(50));
    }

    // Stop offboard mode
    std::cout << "\nStopping offboard mode...\n";
    offboard_result = offboard.stop();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard stop failed: " << offboard_result << '\n';
        return false;
    }
    std::cout << "✓ Offboard mode stopped\n";

    return true;
}

int main(int argc, char** argv)
{
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }

    // Create MAVSDK instance
    Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};
    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    std::cout << "Waiting for system to connect...\n";
    auto system = mavsdk.first_autopilot(3.0);
    if (!system) {
        std::cerr << "Timed out waiting for system\n";
        return 1;
    }

    // Instantiate plugins
    auto action = Action{system.value()};
    auto offboard = Offboard{system.value()};
    auto telemetry = Telemetry{system.value()};

    std::cout << "✓ System connected\n";

    // Check basic health (not all_ok - that requires GPS)
    std::cout << "\nChecking system health...\n";
    auto health = telemetry.health();
    std::cout << "  Accelerometer: " << (health.is_accelerometer_calibration_ok ? "✓" : "✗") << "\n";
    std::cout << "  Gyroscope: " << (health.is_gyrometer_calibration_ok ? "✓" : "✗") << "\n";
    std::cout << "  Magnetometer: " << (health.is_magnetometer_calibration_ok ? "✓" : "✗") << "\n";
    std::cout << "  Local position: " << (health.is_local_position_ok ? "✓" : "✗") << "\n";
    std::cout << "  Armable: " << (health.is_armable ? "✓" : "✗") << "\n";

    if (!health.is_armable) {
        std::cerr << "\n⚠ WARNING: System reports not armable.\n";
        std::cerr << "Check QGroundControl for specific failure reasons.\n";
        std::cerr << "Common issues: battery low, safety switch, calibration needed.\n";
        std::cout << "\nContinue anyway? (y/n): ";
        char response;
        std::cin >> response;
        if (response != 'y' && response != 'Y') {
            return 1;
        }
    }

    std::cout << "\n=== IMPORTANT SAFETY INFORMATION ===\n";
    std::cout << "1. This program uses OFFBOARD mode for direct control\n";
    std::cout << "2. Keep your RC transmitter ON as a safety backup\n";
    std::cout << "3. You can switch back to manual mode on your RC at any time\n";
    std::cout << "4. The drone will execute pre-programmed movements\n";
    std::cout << "5. Make sure you have enough space for testing\n";
    std::cout << "\nREADY TO ARM AND FLY?\n";
    std::cout << "Press ENTER to continue (or Ctrl+C to cancel)...\n";
    std::cin.ignore();
    std::cin.get();

    // // Arm the vehicle
    // std::cout << "\nArming...\n";
    // const Action::Result arm_result = action.arm();
    // if (arm_result != Action::Result::Success) {
    //     std::cerr << "✗ Arming failed: " << arm_result << '\n';
    //     std::cerr << "Check QGroundControl for error details\n";
    //     return 1;
    // }
    // std::cout << "✓ Armed\n";
    
    // Check if already armed
    if (!telemetry.armed()) {
        std::cout << "\n⚠ Drone is not armed.\n";
        std::cout << "Please arm using your RC transmitter or QGroundControl\n";
        std::cout << "Waiting for arm signal...\n";
        
        // Wait for manual arm
        while (!telemetry.armed()) {
            sleep_for(milliseconds(500));
            std::cout << "." << std::flush;
        }
        std::cout << "\n✓ Drone armed!\n";
    } else {
        std::cout << "✓ Already armed\n";
    }

    std::cout << "\n=== MANUAL TAKEOFF REQUIRED ===\n";
    std::cout << "Use your RC transmitter to take off to hover height (~1-2 meters)\n";
    std::cout << "Once hovering, press ENTER to start offboard control...\n";
    std::cin.get();

    // Run the attitude control test
    if (!test_attitude_control(offboard)) {
        std::cerr << "Offboard control test failed\n";
        std::cout << "Take control with RC and land manually\n";
        return 1;
    }

    std::cout << "\n=== TEST COMPLETE ===\n";
    std::cout << "Offboard control test finished successfully!\n";
    std::cout << "Use your RC to land the drone manually\n";
    std::cout << "\nOnce landed, press ENTER to disarm...\n";
    std::cin.get();

    // Disarm
    std::cout << "Disarming...\n";
    const Action::Result disarm_result = action.disarm();
    if (disarm_result != Action::Result::Success) {
        std::cerr << "Disarming failed: " << disarm_result << '\n';
        std::cout << "You may need to disarm manually through QGC\n";
    } else {
        std::cout << "✓ Disarmed\n";
    }

    std::cout << "\nProgram finished!\n";
    return 0;
}