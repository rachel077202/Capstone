import asyncio
import pygame
import sys
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

# --- Config ---
CONNECTION_URL = "udp://:14540"
SPEED       = 2.0   # m/s
YAW_SPEED   = 30.0  # deg/s
VERT_SPEED  = 1.5   # m/s
LOOP_HZ     = 20

# --------------------------------------------------------------------------

async def wait_for_disarm(drone, timeout=15):
    """Wait until drone reports it is no longer armed, up to `timeout` seconds."""
    async for is_armed in drone.telemetry.armed():
        if not is_armed:
            return True
        timeout -= 1 / LOOP_HZ
        if timeout <= 0:
            return False

async def run():
    pygame.init()
    screen = pygame.display.set_mode((420, 240))
    pygame.display.set_caption("Drone Keyboard Control")
    font = pygame.font.SysFont("monospace", 14)

    drone = System()
    print(f"Connecting to drone at {CONNECTION_URL} ...")
    await drone.connect(system_address=CONNECTION_URL)

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    print("Waiting for drone to be ready...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Drone is ready to arm!")
            break

    print("Arming...")
    await drone.action.arm()
    print("Armed. Press T to take off.")

    offboard_active = False  # Track offboard state ourselves
    clock = pygame.time.Clock()
    running = True

    while running:
        keys = pygame.key.get_pressed()

        forward  =  SPEED      if keys[pygame.K_w] else (-SPEED      if keys[pygame.K_s] else 0.0)
        right    =  SPEED      if keys[pygame.K_d] else (-SPEED      if keys[pygame.K_a] else 0.0)
        yaw      =  YAW_SPEED  if keys[pygame.K_e] else (-YAW_SPEED  if keys[pygame.K_q] else 0.0)
        vertical = -VERT_SPEED if keys[pygame.K_r] else ( VERT_SPEED if keys[pygame.K_f] else 0.0)

        # Only send velocity setpoints when offboard mode is active
        if offboard_active:
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(forward, right, vertical, yaw)
            )
        else:
            forward = right = yaw = vertical = 0.0  # Zero out HUD display too

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.KEYDOWN:

                if event.key == pygame.K_ESCAPE:
                    print("ESC — emergency stop!")
                    running = False

                elif event.key == pygame.K_t and not offboard_active:
                    print("Taking off...")
                    await drone.action.set_takeoff_altitude(5.0)
                    await drone.action.takeoff()
                    await asyncio.sleep(6)
                    # Send a zero setpoint first, then start offboard
                    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
                    try:
                        await drone.offboard.start()
                        offboard_active = True
                        print("Offboard mode active.")
                    except OffboardError as e:
                        print(f"Offboard start failed: {e}")

                elif event.key == pygame.K_l and offboard_active:
                    print("Landing...")
                    try:
                        await drone.offboard.stop()
                        offboard_active = False
                    except Exception as e:
                        print(f"Offboard stop error: {e}")
                    await asyncio.sleep(0.5)
                    await drone.action.land()
                    print("Waiting for drone to land and auto-disarm...")
                    # PX4 will auto-disarm after landing — just wait for it
                    await wait_for_disarm(drone, timeout=30)
                    print("Disarmed.")
                    running = False
                    print("Arming...")
                    
                    await drone.action.arm()
                    print("Armed. Press T to take off.")

                    offboard_active = False  # Track offboard state ourselves
                    clock = pygame.time.Clock()
                    running = True

                elif event.key == pygame.K_SPACE and offboard_active:
                    print("Hovering...")
                    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))

        # --- HUD ---
        screen.fill((20, 20, 20))
        status = "OFFBOARD" if offboard_active else "STANDBY — Press T to takeoff"
        lines = [
            "=== DRONE KEYBOARD CONTROL ===",
            f"  Status       : {status}",
            "",
            f"  Forward/Back : W / S   [{forward:+.1f} m/s]",
            f"  Left/Right   : A / D   [{right:+.1f} m/s]",
            f"  Up/Down      : R / F   [{vertical:+.1f} m/s]",
            f"  Yaw          : Q / E   [{yaw:+.1f} deg/s]",
            "",
            "  T=Takeoff  L=Land  SPACE=Hover  ESC=Stop",
        ]
        for i, line in enumerate(lines):
            color = (0, 255, 120) if i == 0 else (180, 180, 50) if i == 1 else (200, 200, 200)
            screen.blit(font.render(line, True, color), (10, 10 + i * 22))
        pygame.display.flip()

        await asyncio.sleep(1 / LOOP_HZ)

    # --- Cleanup ---
    print("Cleaning up...")
    if offboard_active:
        try:
            await drone.offboard.stop()
        except Exception:
            pass
    # Only force-disarm if drone is still armed (e.g. ESC pressed mid-flight)
    async for is_armed in drone.telemetry.armed():
        if is_armed:
            print("Force disarming...")
            await drone.action.disarm()
        break  # Only read one value
    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    asyncio.run(run())