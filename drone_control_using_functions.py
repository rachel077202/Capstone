import asyncio
from drone_control_functions import arm_takeoff, forward_backward, hover, land, right_left, up_down, yaw
import pygame
import sys
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

# --- Config ---
CONNECTION_URL = "udp://:14540"
LOOP_HZ     = 20

# --------------------------------------------------------------------------

async def run():
    pygame.init()
    screen = pygame.display.set_mode((420, 300))
    pygame.display.set_caption("Drone Keyboard Control")
    font = pygame.font.SysFont("monospace", 14)

    screen.fill((20, 20, 20))
    pygame.display.flip()

    drone = await arm_takeoff(CONNECTION_URL)

    offboard_active = True
    clock = pygame.time.Clock()
    running = True
    foward_speed=right_speed=up_speed=yaw_speed=0.0
    speed_pct = 0.5          # default 50%
    input_text = ""          # what the user is typing
    input_active = False     # is the input box focused?
    input_error = ""         # validation message

    while running:
        keys = pygame.key.get_pressed()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.KEYDOWN:
                # --- Input box handling ---
                if input_active:
                    if event.key == pygame.K_RETURN:
                        try:
                            val = float(input_text)
                            if -1.0 <= val <= 1.0:
                                speed_pct = val
                                input_error = ""
                            else:
                                input_error = "Must be between -1 and 1"
                        except ValueError:
                            input_error = "Invalid number"
                        input_text = ""
                        input_active = False
                    elif event.key == pygame.K_BACKSPACE:
                        input_text = input_text[:-1]
                    elif event.key == pygame.K_ESCAPE:
                        input_text = ""
                        input_active = False
                    else:
                        input_text += event.unicode
                else:
                    # --- Drone controls (only when not typing) ---
                    if event.key == pygame.K_ESCAPE:
                        print("ESC — emergency stop!")
                        running = False
                    elif event.key == pygame.K_l and offboard_active:
                        await land(drone)
                        offboard_active = False
                        running = False
                    elif event.key == pygame.K_SPACE and offboard_active:
                        await hover(drone)
                    elif event.key == pygame.K_s:
                        # Press S to open the speed input box
                        input_active = True

        # --- Movement (only when not typing) ---
        if offboard_active and not input_active:
            if keys[pygame.K_w]:
                forward_speed = speed_pct
                right_speed=up_speed=yaw_speed=0.0
            elif keys[pygame.K_d]:
                right_speed = speed_pct
                forward_speed=up_speed=yaw_speed=0.0
            elif keys[pygame.K_e]:
                yaw_speed = speed_pct
                forward_speed=right_speed=up_speed=0.0
            elif keys[pygame.K_r]:
                up_speed = speed_pct
                forward_speed=right_speed=yaw_speed=0.0
            else:
                forward_speed=right_speed=up_speed=yaw_speed=0.0

            # Always send a setpoint every loop tick
            max_speed = 2.0
            max_yaw = 30.0
            max_vert = 1.5
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(
                    max_speed * forward_speed,
                    max_speed * right_speed,
                    -1 * max_vert * up_speed,
                    max_yaw * yaw_speed
                )
            )

        # --- HUD ---
        screen.fill((20, 20, 20))
        status = "OFFBOARD" if offboard_active else "STANDBY"
        lines = [
            "=== DRONE KEYBOARD CONTROL ===",
            f"  Status       : {status}",
            f"  Speed %      : {speed_pct:+.2f}  (S to change)",
            "",
            f"  Forward/Back : W   [{abs(foward_speed):.1f} m/s]",
            f"  Left/Right   : D   [{abs(right_speed):.1f} m/s]",
            f"  Up/Down      : R   [{abs(up_speed):.1f} m/s]",
            f"  Yaw          : E   [{abs(yaw_speed):.1f} deg/s]",
            "",
            "  L=Land  SPACE=Hover  ESC=Stop",
        ]
        for i, line in enumerate(lines):
            color = (0, 255, 120) if i == 0 else (180, 180, 50) if i == 1 else (200, 200, 200)
            screen.blit(font.render(line, True, color), (10, 10 + i * 22))

        # --- Input box ---
        if input_active:
            pygame.draw.rect(screen, (50, 50, 80), (10, 260, 400, 28))
            pygame.draw.rect(screen, (100, 100, 200), (10, 260, 400, 28), 2)
            prompt = f"Speed (-1 to 1): {input_text}_"
            screen.blit(font.render(prompt, True, (255, 255, 255)), (15, 265))
        if input_error:
            screen.blit(font.render(input_error, True, (255, 80, 80)), (15, 265))

        pygame.display.flip()
        clock.tick(LOOP_HZ)
        await asyncio.sleep(0)

    # --- Cleanup ---
    print("Cleaning up...")
    if offboard_active:
        try:
            await drone.offboard.stop()
        except Exception:
            pass
    async for is_armed in drone.telemetry.armed():
        if is_armed:
            print("Force disarming...")
            await drone.action.disarm()
        break
    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    asyncio.run(run())