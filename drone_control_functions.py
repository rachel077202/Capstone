import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
from mavsdk.telemetry import LandedState

async def wait_for_disarm(drone, timeout=15):
    """Wait until drone reports it is no longer armed, up to `timeout` seconds.
    
    Args:
        drone (System): The connection to the drone
        timeout (int): Time until it times out
    """
    async for is_armed in drone.telemetry.armed():
        if not is_armed:
            return True
        timeout -= 1 / 20
        if timeout <= 0:
            return False

async def arm_takeoff(connection_url):
    """Connects to the drone, then arms the drone and takes off.
    
    Args:
        connection_url (str): The address of the drone to connect to

    Returns:
        System: The connection to the drone
    """
    drone = System()
    print(f"Connecting to drone at {connection_url} ...")
    await drone.connect(system_address=connection_url)

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    print("Waiting for drone to be ready...")
    # async for health in drone.telemetry.health():
    #     if health.is_global_position_ok and health.is_home_position_ok:
    #         print("Drone is ready to arm!")
    #         break

    print("Arming...")
    await drone.action.arm()
    print("Armed.")

    print("Taking off...")
    await drone.action.set_takeoff_altitude(5.0)
    await drone.action.takeoff()
    await asyncio.sleep(6)
    # Send a zero setpoint first, then start offboard
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
    try:
        await drone.offboard.start()
        # offboard_active = True
        print("Offboard mode active.")
        return drone
    except OffboardError as e:
        print(f"Offboard start failed: {e}")

async def land(drone):
    """Lands the drone.
    
    Args:
        drone (System): The connection to the drone
    """
    print("Landing...")
    try:
        await drone.offboard.stop()
    except Exception as e:
        print(f"Offboard stop error: {e}")

    await asyncio.sleep(0.5)
    await drone.action.land()
    print("Waiting for drone to land and auto-disarm...")
    async for state in drone.telemetry.landed_state():
        if state == LandedState.ON_GROUND:
            break

    # PX4 will auto-disarm after landing — just wait for it
    await wait_for_disarm(drone, timeout=30)
    print("Disarmed.")

async def hover(drone):
    """Tells the drone to hover in place.
    
    Args:
        drone (System): The connection to the drone
    """
    print("Hovering...")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))

async def forward_backward(drone, speed_percentage):
    """
    Tells the drone to either go forward or backward.
    Forward is represented by a positive speed and backward is represented by a negative speed.
    Takes a percentage of the max speed based on the given speed input which will be a range from -1 to 1.
    
    Args: 
        drone (System): The connection to the drone
        speed_percentage (float): The percentage of the maximum speed (-1 to 1)

    Returns:
        speed (float): The speed that the drone is moving at
    """
    max_speed = 2.0 # m/s

    start = asyncio.get_event_loop().time()
    while asyncio.get_event_loop().time() - start < 1.0:
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(max_speed * speed_percentage, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.1)
    return max_speed * speed_percentage

async def right_left(drone, speed_percentage):
    """
    Tells the drone to either go right or left.
    Right is represented by a positive speed and left is represented by a negative speed.
    Takes a percentage of the max speed based on the given speed input which will be a range from -1 to 1.
    
    Args: 
        drone (System): The connection to the drone
        speed_percentage (float): The percentage of the maximum speed (-1 to 1)

    Returns:
        speed (float): The speed that the drone is moving at
    """
    max_speed = 2.0 # m/s
    
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, max_speed * speed_percentage, 0.0, 0.0))
    return max_speed * speed_percentage

async def yaw(drone, speed_percentage):
    """
    Sets the yaw of the drone.
    Takes a percentage of the max speed based on the given speed input which will be a range from -1 to 1.
    
    Args: 
        drone (System): The connection to the drone
        speed_percentage (float): The percentage of the maximum speed (-1 to 1)

    Returns:
        speed (float): The speed that the drone is moving at
    """
    max_speed = 30.0  # deg/s
    
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, max_speed * speed_percentage))
    return max_speed * speed_percentage


async def up_down(drone, speed_percentage):
    """
    Tells the drone to either go up or down.
    Up is represented by a positive speed and down is represented by a negative speed.
    Takes a percentage of the max speed based on the given speed input which will be a range from -1 to 1.
    
    Args: 
        drone (System): The connection to the drone
        speed_percentage (float): The percentage of the maximum speed (-1 to 1)

    Returns:
        speed (float): The speed that the drone is moving at
    """
    max_speed = 1.5 # m/s
    
    # speed if multiplied by -1 because up is negative and down is positive in the VelocityBodyYawspeed function
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, -1*max_speed * speed_percentage, 0.0))
    return -1*max_speed * speed_percentage
