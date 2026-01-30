"""
Motor scanning utilities for LeRobot
Scans all serial ports for connected Dynamixel and Feetech motors
"""

import sys
import glob
import threading
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FuturesTimeoutError
from pathlib import Path
from typing import Optional
import typer


# Default timeout for port scanning operations (seconds)
PORT_SCAN_TIMEOUT = 5.0


def run_with_timeout(func, timeout: float, default=None):
    """
    Run a function with a timeout. Returns default if timeout is exceeded.
    Works on both Windows and Unix.
    """
    with ThreadPoolExecutor(max_workers=1) as executor:
        future = executor.submit(func)
        try:
            return future.result(timeout=timeout)
        except FuturesTimeoutError:
            return default
        except Exception:
            return default


# Motor model number to name mapping
DYNAMIXEL_MODELS = {
    1060: "XL430-W250",
    1190: "XL330-M077",
    1200: "XL330-M288",
    1020: "XM430-W350",
    1120: "XM540-W270",
    1070: "XC430-W150",
}

FEETECH_MODELS = {
    777: "STS3215",
    2825: "STS3250",
    11272: "SM8512BL",
    1284: "SCS0009",
}

# SO100/SO101 use STS3215 motors (model 777) for all joints
SO_LEADER_MODELS = {777}  # STS3215 for SO100/SO101 leader
SO_FOLLOWER_MODELS = {777, 2825}  # STS3215 or STS3250 for follower

# Motor models typically used in leader vs follower arms
# Leader arms use lighter motors (XL330-M077) for ease of manual control
# Follower arms use stronger motors (XL430, XL330-M288) for payload
KOCH_LEADER_MODELS = {1190}  # XL330-M077 only
KOCH_FOLLOWER_MODELS = {1060, 1200, 1020, 1120, 1070}  # XL430, XL330-M288, etc.


def get_serial_ports() -> list[str]:
    """Get available serial ports for motor scanning."""
    ports = []
    
    if sys.platform == "win32":
        # Windows - use pyserial to find COM ports
        try:
            from serial.tools import list_ports
            # Filter out Bluetooth serial ports (they hang during scanning)
            # and prioritize USB ports (those with VID:PID)
            usb_ports = []
            other_ports = []
            for port in list_ports.comports():
                # Skip Bluetooth ports - they cause hangs
                if "Bluetooth" in port.description:
                    continue
                # Prioritize USB ports (have VID/PID)
                if port.vid is not None:
                    usb_ports.append(port.device)
                else:
                    other_ports.append(port.device)
            # Put USB ports first, they're most likely to have motors
            ports = usb_ports + other_ports
        except ImportError:
            typer.echo("⚠️  pyserial required for Windows port detection")
    elif sys.platform == "darwin":
        # macOS - USB serial devices appear as tty.usbmodem* or cu.usbmodem*
        # Use glob.glob() instead of Path.glob() for better compatibility with /dev/ on macOS
        patterns = [
            "/dev/tty.usbmodem*",    # USB CDC ACM devices (robot arms)
            "/dev/cu.usbmodem*",     # Call-out version of usbmodem
            "/dev/tty.usbserial*",   # USB-to-serial adapters
            "/dev/cu.usbserial*",    # Call-out version of usbserial
            "/dev/tty.SLAB*",        # Silicon Labs USB-UART bridges
            "/dev/cu.SLAB*",         # Call-out version
            "/dev/tty.wchusbserial*", # WCH USB-UART bridges (common on some arms)
            "/dev/cu.wchusbserial*",
        ]
        for pattern in patterns:
            ports.extend(glob.glob(pattern))
        
        # Prefer tty.* over cu.* (tty handles hardware flow control better)
        # Filter out duplicates, keeping tty.* versions
        # Sort with tty.* before cu.* so tty.* gets processed first
        def sort_key(port):
            # tty.* should come before cu.* (return 0 for tty, 1 for cu)
            if "/tty." in port:
                return (0, port)
            elif "/cu." in port:
                return (1, port)
            return (2, port)
        
        seen_devices = set()
        filtered_ports = []
        for port in sorted(ports, key=sort_key):
            # Extract the unique identifier (everything after tty. or cu.)
            if ".usbmodem" in port or ".usbserial" in port or ".SLAB" in port or ".wchusbserial" in port:
                device_id = port.split(".")[-1]
                if device_id not in seen_devices:
                    seen_devices.add(device_id)
                    # Prefer tty.* over cu.*
                    if "/tty." in port:
                        filtered_ports.append(port)
                    elif "/cu." in port:
                        # Check if tty version exists
                        tty_version = port.replace("/cu.", "/tty.")
                        if not Path(tty_version).exists():
                            filtered_ports.append(port)
            else:
                filtered_ports.append(port)
        ports = filtered_ports
    else:
        # Linux
        patterns = [
            "/dev/ttyACM*",   # USB CDC ACM devices (most common for robot arms)
            "/dev/ttyUSB*",   # USB-to-serial adapters (FTDI, CH340, etc.)
        ]
        for pattern in patterns:
            ports.extend(glob.glob(pattern))
    
    return sorted(ports)


def scan_dynamixel_port(port: str, baudrate: int = 1_000_000, verbose: bool = False, timeout: float = PORT_SCAN_TIMEOUT) -> dict[int, int]:
    """Scan a port for Dynamixel motors. Returns {motor_id: model_number}."""
    
    def _scan():
        try:
            import dynamixel_sdk as dxl
        except ImportError:
            if verbose:
                typer.echo(f"   ⚠️  dynamixel_sdk not installed ")
            return {}
        
        found = {}
        try:
            port_handler = dxl.PortHandler(port)
            packet_handler = dxl.PacketHandler(2.0)  # Protocol 2.0
            
            if not port_handler.openPort():
                if verbose:
                    typer.echo(f"   ⚠️  Failed to open port {port} for Dynamixel scan (may be in use or access denied)")
                return {}
            
            port_handler.setBaudRate(baudrate)
            
            # Scan motor IDs 1-20
            for motor_id in range(1, 21):
                model_number, result, _ = packet_handler.ping(port_handler, motor_id)
                if result == dxl.COMM_SUCCESS:
                    found[motor_id] = model_number
            
            port_handler.closePort()
        except Exception as e:
            if verbose:
                typer.echo(f"   ⚠️  Dynamixel scan error on {port}: {e}")
        
        return found
    
    result = run_with_timeout(_scan, timeout, default={})
    if result is None:
        if verbose:
            typer.echo(f"   ⚠️  Dynamixel scan on {port} timed out after {timeout}s")
        return {}
    return result


def scan_feetech_port(port: str, baudrate: int = 1_000_000, protocol: int = 0, verbose: bool = False, timeout: float = PORT_SCAN_TIMEOUT) -> dict[int, int]:
    """
    Scan a port for Feetech motors. Returns {motor_id: model_number}.
    
    Args:
        port: Serial port path
        baudrate: Baud rate to use (default 1M)
        protocol: Feetech protocol version (0 for STS/SMS, 1 for SCS)
        verbose: Print detailed error messages
        timeout: Timeout in seconds for the scan operation
    """
    
    def _scan():
        try:
            import scservo_sdk as scs
        except ImportError:
            if verbose:
                typer.echo(f"   ⚠️  scservo_sdk (feetech-servo-sdk) not installed")
            return {}
        
        found = {}
        try:
            port_handler = scs.PortHandler(port)
            packet_handler = scs.PacketHandler(protocol)
            
            if not port_handler.openPort():
                if verbose:
                    typer.echo(f"   ⚠️  Failed to open port {port} for Feetech scan (may be in use or access denied)")
                return {}
            
            port_handler.setBaudRate(baudrate)
            
            if protocol == 0:
                # Protocol 0 (STS/SMS) - use broadcast ping
                # Simplified broadcast ping - scan individual IDs
                for motor_id in range(1, 21):
                    model_number, result, _ = packet_handler.ping(port_handler, motor_id)
                    if result == scs.COMM_SUCCESS:
                        # Read model number from register
                        model_nb, _, _ = packet_handler.read2ByteTxRx(port_handler, motor_id, 3)  # Model_Number at addr 3
                        if model_nb:
                            found[motor_id] = model_nb
                        else:
                            found[motor_id] = model_number
            else:
                # Protocol 1 (SCS) - scan individual IDs
                for motor_id in range(1, 21):
                    model_number, result, _ = packet_handler.ping(port_handler, motor_id)
                    if result == scs.COMM_SUCCESS:
                        model_nb, _, _ = packet_handler.read2ByteTxRx(port_handler, motor_id, 3)
                        if model_nb:
                            found[motor_id] = model_nb
                        else:
                            found[motor_id] = model_number
            
            port_handler.closePort()
        except Exception as e:
            if verbose:
                typer.echo(f"   ⚠️  Feetech scan error on {port}: {e}")
        
        return found
    
    result = run_with_timeout(_scan, timeout, default={})
    if result is None:
        if verbose:
            typer.echo(f"   ⚠️  Feetech scan on {port} timed out after {timeout}s")
        return {}
    return result


def read_feetech_voltage(port: str, motor_id: int = 1, baudrate: int = 1_000_000, protocol: int = 0) -> Optional[float]:
    """
    Read the present voltage from a Feetech motor.
    
    Args:
        port: Serial port path
        motor_id: Motor ID to read from (default 1)
        baudrate: Baud rate
        protocol: Feetech protocol version
    
    Returns:
        Voltage in volts, or None if read failed
    """
    try:
        import scservo_sdk as scs
    except ImportError:
        return None
    
    try:
        port_handler = scs.PortHandler(port)
        packet_handler = scs.PacketHandler(protocol)
        
        if not port_handler.openPort():
            return None
        
        port_handler.setBaudRate(baudrate)
        
        # Present_Voltage is at address 62, 1 byte
        # Value is in 0.1V units (e.g., 120 = 12.0V)
        voltage_raw, result, _ = packet_handler.read1ByteTxRx(port_handler, motor_id, 62)
        
        port_handler.closePort()
        
        if result == scs.COMM_SUCCESS and voltage_raw:
            return voltage_raw / 10.0  # Convert to volts
        
        return None
    except Exception:
        return None


def set_feetech_torque(port: str, motor_ids: list[int] = None, enable: bool = True, baudrate: int = 1_000_000, protocol: int = 0) -> bool:
    """
    Enable or disable torque on Feetech motors.
    
    Args:
        port: Serial port path
        motor_ids: List of motor IDs (default: 1-6)
        enable: True to enable torque, False to disable
        baudrate: Baud rate
        protocol: Feetech protocol version
    
    Returns:
        True if successful, False otherwise
    """
    if motor_ids is None:
        motor_ids = list(range(1, 7))
    
    try:
        import scservo_sdk as scs
    except ImportError:
        return False
    
    try:
        port_handler = scs.PortHandler(port)
        packet_handler = scs.PacketHandler(protocol)
        
        if not port_handler.openPort():
            return False
        
        port_handler.setBaudRate(baudrate)
        
        # Torque_Enable is at address 40 for STS series
        torque_value = 1 if enable else 0
        success = True
        for motor_id in motor_ids:
            result, _ = packet_handler.write1ByteTxRx(port_handler, motor_id, 40, torque_value)
            if result != scs.COMM_SUCCESS:
                success = False
        
        port_handler.closePort()
        return success
    except Exception:
        return False


def read_feetech_positions(port: str, motor_ids: list[int] = None, baudrate: int = 1_000_000, protocol: int = 0) -> dict[int, int]:
    """
    Read present positions from Feetech motors.
    
    Args:
        port: Serial port path
        motor_ids: List of motor IDs to read (default: 1-6)
        baudrate: Baud rate
        protocol: Feetech protocol version
    
    Returns:
        Dictionary of {motor_id: position}
    """
    if motor_ids is None:
        motor_ids = list(range(1, 7))
    
    try:
        import scservo_sdk as scs
    except ImportError:
        return {}
    
    positions = {}
    try:
        port_handler = scs.PortHandler(port)
        packet_handler = scs.PacketHandler(protocol)
        
        if not port_handler.openPort():
            return {}
        
        port_handler.setBaudRate(baudrate)
        
        for motor_id in motor_ids:
            # Present_Position is at address 56, 2 bytes for STS series
            position, result, _ = packet_handler.read2ByteTxRx(port_handler, motor_id, 56)
            if result == scs.COMM_SUCCESS:
                positions[motor_id] = position
        
        port_handler.closePort()
    except Exception:
        pass
    
    return positions


def detect_movement_on_port(port: str, duration: float = 2.0, threshold: int = 50, verbose: bool = False) -> bool:
    """
    Detect if motors on a port are being moved by comparing positions over time.
    
    Args:
        port: Serial port path
        duration: How long to monitor for movement (seconds)
        threshold: Minimum position change to count as movement
        verbose: Print debug info
    
    Returns:
        True if movement detected, False otherwise
    """
    import time
    
    # Read initial positions
    initial_positions = read_feetech_positions(port)
    if not initial_positions:
        return False
    
    # Wait and read again
    time.sleep(duration)
    
    # Read final positions
    final_positions = read_feetech_positions(port)
    if not final_positions:
        return False
    
    # Check for movement
    total_movement = 0
    for motor_id in initial_positions:
        if motor_id in final_positions:
            movement = abs(final_positions[motor_id] - initial_positions[motor_id])
            total_movement += movement
            if verbose and movement > 0:
                typer.echo(f"   Motor {motor_id}: moved {movement} units")
    
    if verbose:
        typer.echo(f"   Total movement: {total_movement} units (threshold: {threshold})")
    
    return total_movement >= threshold


def detect_moving_port(ports: list[str], duration: float = 3.0, threshold: int = 50, verbose: bool = False) -> Optional[str]:
    """
    Monitor multiple ports simultaneously and detect which one has movement.
    
    Args:
        ports: List of serial ports to monitor
        duration: How long to monitor for movement (seconds)
        threshold: Minimum position change to count as movement
        verbose: Print debug info
    
    Returns:
        Port with detected movement, or None if no movement detected
    """
    import time
    
    if not ports:
        return None
    
    # Read initial positions from all ports
    initial_positions = {}
    for port in ports:
        positions = read_feetech_positions(port)
        if positions:
            initial_positions[port] = positions
    
    if not initial_positions:
        if verbose:
            typer.echo("   ⚠️  Could not read positions from any port")
        return None
    
    # Wait while user moves the arm
    time.sleep(duration)
    
    # Read final positions and calculate movement for each port
    port_movements = {}
    for port in initial_positions:
        final_positions = read_feetech_positions(port)
        if not final_positions:
            continue
        
        total_movement = 0
        for motor_id in initial_positions[port]:
            if motor_id in final_positions:
                movement = abs(final_positions[motor_id] - initial_positions[port][motor_id])
                total_movement += movement
        
        port_movements[port] = total_movement
        if verbose:
            typer.echo(f"   {port}: {total_movement} units")
    
    # Find port with most movement above threshold
    best_port = None
    best_movement = 0
    for port, movement in port_movements.items():
        if movement >= threshold and movement > best_movement:
            best_port = port
            best_movement = movement
    
    return best_port


def detect_so_arm_type_by_voltage(port: str, verbose: bool = False) -> Optional[str]:
    """
    Detect if a SO100/SO101 arm is leader or follower based on motor voltage.
    
    Leader arms use 5V motors, follower arms use 12V motors.
    
    Returns "leader", "follower", or None if detection failed.
    """
    # Try to read voltage from motor ID 1
    voltage = read_feetech_voltage(port, motor_id=1)
    
    if voltage is None:
        if verbose:
            typer.echo(f"   ⚠️  Could not read voltage from {port}")
        return None
    
    if verbose:
        typer.echo(f"   📊 Voltage on {port}: {voltage:.1f}V")
    
    # Leader arms: ~5V (typically 4.5-6.5V range)
    # Follower arms: ~12V (typically 10-14V range)
    if voltage < 8.0:
        if verbose:
            typer.echo(f"   → Detected as LEADER arm (5V motor)")
        return "leader"
    else:
        if verbose:
            typer.echo(f"   → Detected as FOLLOWER arm (12V motor)")
        return "follower"


def detect_arm_type_from_models(models: set[int], motor_brand: str = "dynamixel") -> Optional[str]:
    """Detect whether motors indicate a leader or follower arm."""
    if not models:
        return None
    
    if motor_brand == "dynamixel":
        # Koch arms detection
        if models.issubset(KOCH_LEADER_MODELS):
            return "leader"
        elif models & KOCH_FOLLOWER_MODELS:  # Has any follower-type motors
            return "follower"
    elif motor_brand == "feetech":
        # SO100/SO101 - both leader and follower use same motor types
        # Cannot distinguish by model alone, would need additional heuristics
        if models & SO_LEADER_MODELS or models & SO_FOLLOWER_MODELS:
            return "unknown"  # Can't tell leader from follower on SO arms by model
    
    return None


def detect_robot_type_from_port(port: str, verbose: bool = False) -> tuple[Optional[str], Optional[str], dict[int, int]]:
    """
    Auto-detect robot type and motor brand by scanning a port.
    
    Returns (robot_type, motor_brand, motors_found) where:
        - robot_type: "koch", "so100", "so101", or None
        - motor_brand: "dynamixel", "feetech", or None
        - motors_found: {motor_id: model_number}
    """
    if verbose:
        typer.echo(f"   Scanning {port} for motors...")
    
    # Try Dynamixel first
    dynamixel_motors = scan_dynamixel_port(port)
    if dynamixel_motors:
        if verbose:
            model_names = [DYNAMIXEL_MODELS.get(m, f"Unknown({m})") for m in dynamixel_motors.values()]
            typer.echo(f"   ✅ Found Dynamixel motors: {model_names}")
        return "koch", "dynamixel", dynamixel_motors
    
    # Try Feetech (Protocol 0 - STS/SMS series)
    feetech_motors = scan_feetech_port(port, protocol=0)
    if feetech_motors:
        if verbose:
            model_names = [FEETECH_MODELS.get(m, f"Unknown({m})") for m in feetech_motors.values()]
            typer.echo(f"   ✅ Found Feetech motors: {model_names}")
        # SO100 and SO101 both use STS3215 - default to so101 as it's newer
        return "so101", "feetech", feetech_motors
    
    # Try Feetech Protocol 1 (SCS series)
    feetech_motors_p1 = scan_feetech_port(port, protocol=1)
    if feetech_motors_p1:
        if verbose:
            model_names = [FEETECH_MODELS.get(m, f"Unknown({m})") for m in feetech_motors_p1.values()]
            typer.echo(f"   ✅ Found Feetech SCS motors: {model_names}")
        return "so100", "feetech", feetech_motors_p1
    
    if verbose:
        typer.echo(f"   ⚠️  No motors found on {port}")
    
    return None, None, {}


def auto_detect_robot_type(verbose: bool = True) -> tuple[Optional[str], list[tuple[str, str, dict]]]:
    """
    Scan all ports to auto-detect robot type.
    
    Returns (robot_type, port_info) where:
        - robot_type: Detected robot type ("koch", "so100", "so101") or None
        - port_info: List of (port, motor_brand, motors_found) for ports with motors
    """
    if verbose:
        typer.echo("🔍 Auto-detecting robot type...")
    
    ports = get_serial_ports()
    if not ports:
        if verbose:
            typer.echo("❌ No serial ports found")
        return None, []
    
    port_info = []
    detected_types = set()
    
    for port in ports:
        robot_type, motor_brand, motors = detect_robot_type_from_port(port, verbose)
        if motors:
            port_info.append((port, motor_brand, motors))
            if robot_type:
                detected_types.add(robot_type)
    
    # Determine final robot type
    if len(detected_types) == 1:
        robot_type = detected_types.pop()
        
        # Check for bimanual configuration: 4 ports = full bimanual (2 leaders + 2 followers)
        # With 2 ports we can't distinguish single-arm (1L+1F) from bimanual half (2L or 2F)
        ports_with_motors = len([p for p in port_info if p[1] is not None])
        if ports_with_motors >= 4 and robot_type in ("so100", "so101"):
            robot_type = f"bi_{robot_type}"
            if verbose:
                typer.echo(f"✅ Detected robot type: {robot_type.upper()} (bimanual - {ports_with_motors} arms found)")
        else:
            if verbose:
                typer.echo(f"✅ Detected robot type: {robot_type.upper()}")
        return robot_type, port_info
    elif len(detected_types) > 1:
        if verbose:
            typer.echo(f"⚠️  Multiple robot types detected: {detected_types}")
            typer.echo("   Using first detected type...")
        return detected_types.pop(), port_info
    else:
        if verbose:
            typer.echo("⚠️  Could not auto-detect robot type")
        return None, port_info


def auto_detect_ports(robot_type: str = None, verbose: bool = True) -> tuple[Optional[str], Optional[str], Optional[str]]:
    """
    Auto-detect leader and follower ports based on connected motor types.
    Also auto-detects robot type if not specified.
    
    For Koch/Dynamixel arms: Can distinguish leader/follower by motor model.
    For SO100/SO101/Feetech arms: Distinguishes by voltage (5V=leader, 12V=follower).
    
    Returns (leader_port, follower_port, detected_robot_type) or (None, None, None) if detection failed.
    """
    if verbose:
        typer.echo("🔍 Auto-detecting arm ports...")
    
    ports = get_serial_ports()
    
    if not ports:
        if verbose:
            typer.echo("❌ No serial ports found")
        return None, None, None
    
    leader_port = None
    follower_port = None
    detected_robot_type = robot_type
    port_info = []
    ports_with_motors = []
    
    for port in ports:
        port_robot_type, motor_brand, motors = detect_robot_type_from_port(port, verbose=False)
        if motors:
            ports_with_motors.append((port, port_robot_type, motor_brand, motors))
            
            # Update detected robot type if not specified
            if detected_robot_type is None and port_robot_type:
                detected_robot_type = port_robot_type
            
            if motor_brand == "dynamixel":
                # Koch arms - can distinguish leader/follower by motor model
                models = set(motors.values())
                arm_type = detect_arm_type_from_models(models, "dynamixel")
                port_info.append((port, motors, arm_type, motor_brand))
                
                if arm_type == "leader" and leader_port is None:
                    leader_port = port
                elif arm_type == "follower" and follower_port is None:
                    follower_port = port
            else:
                # Feetech/SO arms - detect by voltage (5V=leader, 12V=follower)
                arm_type = detect_so_arm_type_by_voltage(port, verbose=verbose)
                port_info.append((port, motors, arm_type, motor_brand))
                
                if arm_type == "leader" and leader_port is None:
                    leader_port = port
                elif arm_type == "follower" and follower_port is None:
                    follower_port = port
    
    if verbose:
        if detected_robot_type:
            typer.echo(f"✅ Detected robot type: {detected_robot_type.upper()}")
        
        if leader_port or follower_port:
            typer.echo("✅ Auto-detected ports:")
            if leader_port:
                typer.echo(f"   • Leader:   {leader_port}")
            if follower_port:
                typer.echo(f"   • Follower: {follower_port}")
        else:
            typer.echo("⚠️  Could not auto-detect arm types")
            if port_info:
                typer.echo("   Found motors but couldn't determine leader/follower:")
                for port, motors, arm_type, brand in port_info:
                    typer.echo(f"   • {port}: {len(motors)} {brand} motors" + 
                              (f" ({arm_type})" if arm_type else ""))
    
    return leader_port, follower_port, detected_robot_type


def auto_detect_single_port(arm_type: str, robot_type: str = None, verbose: bool = True) -> tuple[Optional[str], Optional[str]]:
    """
    Auto-detect a single arm port (leader or follower).
    
    For Koch/Dynamixel: Can auto-detect based on motor models.
    For SO100/SO101/Feetech: Detects by voltage (5V=leader, 12V=follower).
    
    Args:
        arm_type: "leader" or "follower"
        robot_type: Robot type (e.g., "koch", "so101"). If None, will auto-detect.
        verbose: Whether to print status messages
    
    Returns (port, detected_robot_type) tuple.
    """
    leader_port, follower_port, detected_robot_type = auto_detect_ports(robot_type, verbose=False)
    
    if arm_type == "leader":
        port = leader_port
    elif arm_type == "follower":
        port = follower_port
    else:
        return None, detected_robot_type
    
    if verbose:
        if detected_robot_type and detected_robot_type != robot_type:
            typer.echo(f"   ✅ Auto-detected robot type: {detected_robot_type.upper()}")
        if port:
            # Show voltage info for SO arms
            if detected_robot_type in ("so100", "so101"):
                voltage = read_feetech_voltage(port, motor_id=1)
                if voltage:
                    voltage_type = "5V" if voltage < 8.0 else "12V"
                    typer.echo(f"   ✅ Found {arm_type} arm on {port} ({voltage_type} motors, {voltage:.1f}V)")
                else:
                    typer.echo(f"   ✅ Auto-detected {arm_type} arm on {port}")
            else:
                typer.echo(f"   ✅ Auto-detected {arm_type} arm on {port}")
        else:
            typer.echo(f"   ⚠️  Could not auto-detect {arm_type} arm")
    
    return port, detected_robot_type


def scan_motors():
    """Scan all serial ports for connected motors and display results."""
    typer.echo("🔍 Scanning for connected motors...\n")
    
    # Check SDK availability first
    sdk_status = []
    try:
        import dynamixel_sdk
        sdk_status.append("✅ dynamixel_sdk installed")
    except ImportError:
        sdk_status.append("❌ dynamixel_sdk NOT installed")
    
    try:
        import scservo_sdk
        sdk_status.append("✅ scservo_sdk installed")
    except ImportError:
        sdk_status.append("❌ scservo_sdk NOT installed")
    
    typer.echo("📦 SDK Status:")
    for status in sdk_status:
        typer.echo(f"   {status}")
    typer.echo("")
    
    ports = get_serial_ports()
    
    if not ports:
        typer.echo("❌ No serial ports found.")
        typer.echo("\n💡 Tips:")
        typer.echo("   • Make sure your robot arm is connected via USB")
        if sys.platform == "win32":
            typer.echo("   • Check Device Manager for COM ports")
            typer.echo("   • Make sure the correct drivers are installed")
            typer.echo("   • Try a different USB port or cable")
        else:
            typer.echo("   • Run 'solo setup-usb' to configure USB permissions")
        return
    
    typer.echo(f"📡 Found {len(ports)} serial port(s):\n")
    
    found_any = False
    detected_robot_type = None
    
    for port in ports:
        typer.echo(f"━━━ {port} ━━━")
        
        # Try Dynamixel first (Koch arms)
        dynamixel_motors = scan_dynamixel_port(port, verbose=True)
        
        if dynamixel_motors:
            found_any = True
            detected_robot_type = "koch"
            typer.echo("  Dynamixel motors (Protocol 2.0):")
            for motor_id, model_num in sorted(dynamixel_motors.items()):
                model_name = DYNAMIXEL_MODELS.get(model_num, f"Unknown ({model_num})")
                typer.echo(f"    ✅ ID {motor_id}: {model_name}")
            
            # Detect arm type based on motor models
            models = set(dynamixel_motors.values())
            if models == {1190}:  # All XL330-M077
                typer.echo("    → Likely: Koch Leader arm")
            elif 1060 in models or 1200 in models:  # XL430 or XL330-M288
                typer.echo("    → Likely: Koch Follower arm")
        else:
            # Try Feetech Protocol 0 (STS/SMS - SO100/SO101)
            feetech_motors = scan_feetech_port(port, protocol=0, verbose=True)
            
            if feetech_motors:
                found_any = True
                detected_robot_type = "so101"
                typer.echo("  Feetech motors (STS/SMS series):")
                for motor_id, model_num in sorted(feetech_motors.items()):
                    model_name = FEETECH_MODELS.get(model_num, f"Unknown ({model_num})")
                    typer.echo(f"    ✅ ID {motor_id}: {model_name}")
                
                # Read voltage to determine arm type
                voltage = read_feetech_voltage(port, motor_id=1)
                if voltage:
                    typer.echo(f"    📊 Voltage: {voltage:.1f}V")
                    if voltage < 8.0:
                        typer.echo("    → Likely: SO100/SO101 LEADER arm (5V)")
                    else:
                        typer.echo("    → Likely: SO100/SO101 FOLLOWER arm (12V)")
                else:
                    # SO100/SO101 detection without voltage
                    models = set(feetech_motors.values())
                    if 777 in models:  # STS3215
                        typer.echo("    → Likely: SO100/SO101 arm")
            else:
                # Try Feetech Protocol 1 (SCS series)
                feetech_motors_p1 = scan_feetech_port(port, protocol=1, verbose=True)
                
                if feetech_motors_p1:
                    found_any = True
                    detected_robot_type = "so100"
                    typer.echo("  Feetech motors (SCS series):")
                    for motor_id, model_num in sorted(feetech_motors_p1.items()):
                        model_name = FEETECH_MODELS.get(model_num, f"Unknown ({model_num})")
                        typer.echo(f"    ✅ ID {motor_id}: {model_name}")
                else:
                    typer.echo("  (No motors found on this port)")
        
        typer.echo("")
    
    if not found_any:
        typer.echo("⚠️  No motors responded on any port.\n")
        typer.echo("💡 Troubleshooting:")
        typer.echo("   1. Make sure the arm is POWERED")
        typer.echo("      • Dynamixel/Koch: 12V power supply")
        typer.echo("      • Feetech/SO100/SO101: 7.4V or 12V depending on model")
        typer.echo("   2. Check that motor cables are firmly connected")
        if sys.platform == "win32":
            typer.echo("   3. Windows-specific checks:")
            typer.echo("      • Open Device Manager → Ports (COM & LPT)")
            typer.echo("      • Check if COM port shows without errors")
            typer.echo("      • Try running terminal/cmd as Administrator")
            typer.echo("      • Close any other apps using the COM port (Arduino IDE, etc.)")
            typer.echo("      • Install CH340/FTDI drivers if needed")
        else:
            typer.echo("   3. Run 'solo setup-usb' if you have permission issues")
        typer.echo("   4. Try unplugging and replugging the USB cable")
    else:
        if detected_robot_type:
            typer.echo(f"🤖 Detected robot type: {detected_robot_type.upper()}")
        typer.echo("✅ Motor scan complete!")
        typer.echo("\n💡 To start teleoperation:")
        typer.echo("   solo robo --teleop")


def diagnose_connection(port: str, verbose: bool = True) -> dict:
    """
    Run detailed diagnostics on a motor connection.
    Returns dict with diagnostic results.
    """
    results = {
        "port": port,
        "ping_success": False,
        "motors_found": {},
        "arm_type": None,  # "leader" or "follower"
        "min_position_limit_read": False,
        "present_position_read": False,
        "errors": []
    }
    
    try:
        import dynamixel_sdk as dxl
    except ImportError:
        results["errors"].append("dynamixel_sdk not installed")
        return results
    
    if verbose:
        typer.echo(f"\n🔍 Diagnosing connection on {port}...")
    
    try:
        port_handler = dxl.PortHandler(port)
        packet_handler = dxl.PacketHandler(2.0)
        
        # Step 1: Open port
        if verbose:
            typer.echo("  1️⃣  Opening port...", nl=False)
        if not port_handler.openPort():
            results["errors"].append("Failed to open port")
            if verbose:
                typer.echo(" ❌")
            return results
        if verbose:
            typer.echo(" ✅")
        
        # Step 2: Set baud rate
        if verbose:
            typer.echo("  2️⃣  Setting baud rate (1M)...", nl=False)
        port_handler.setBaudRate(1_000_000)
        if verbose:
            typer.echo(" ✅")
        
        # Step 3: Ping motors
        if verbose:
            typer.echo("  3️⃣  Pinging motors...")
        for motor_id in range(1, 7):
            model_number, result, error = packet_handler.ping(port_handler, motor_id)
            if result == dxl.COMM_SUCCESS:
                model_name = DYNAMIXEL_MODELS.get(model_number, f"Unknown({model_number})")
                results["motors_found"][motor_id] = model_number
                if verbose:
                    typer.echo(f"       ID {motor_id}: {model_name} ✅")
            else:
                if verbose:
                    typer.echo(f"       ID {motor_id}: No response ❌")
        
        results["ping_success"] = len(results["motors_found"]) > 0
        
        if not results["ping_success"]:
            results["errors"].append("No motors responded to ping")
            port_handler.closePort()
            return results
        
        # Detect arm type based on motor models
        models = set(results["motors_found"].values())
        results["arm_type"] = detect_arm_type_from_models(models)
        
        if verbose and results["arm_type"]:
            arm_label = "🎮 LEADER" if results["arm_type"] == "leader" else "🤖 FOLLOWER"
            typer.echo(f"       → Detected: {arm_label} arm")
        
        # Step 4: Try reading Min_Position_Limit (the failing operation)
        if verbose:
            typer.echo("  4️⃣  Reading Min_Position_Limit...")
        
        # Address for Min_Position_Limit on X-series
        MIN_POS_ADDR = 52
        MIN_POS_LEN = 4
        
        for motor_id in results["motors_found"].keys():
            value, result, error = packet_handler.read4ByteTxRx(
                port_handler, motor_id, MIN_POS_ADDR
            )
            if result == dxl.COMM_SUCCESS:
                if verbose:
                    typer.echo(f"       ID {motor_id}: {value} ✅")
                results["min_position_limit_read"] = True
            else:
                error_msg = packet_handler.getTxRxResult(result)
                if verbose:
                    typer.echo(f"       ID {motor_id}: {error_msg} ❌")
                results["errors"].append(f"ID {motor_id} Min_Position_Limit read failed: {error_msg}")
        
        # Step 5: Try reading Present_Position
        if verbose:
            typer.echo("  5️⃣  Reading Present_Position...")
        
        PRESENT_POS_ADDR = 132
        PRESENT_POS_LEN = 4
        
        for motor_id in results["motors_found"].keys():
            value, result, error = packet_handler.read4ByteTxRx(
                port_handler, motor_id, PRESENT_POS_ADDR
            )
            if result == dxl.COMM_SUCCESS:
                if verbose:
                    typer.echo(f"       ID {motor_id}: {value} ✅")
                results["present_position_read"] = True
            else:
                error_msg = packet_handler.getTxRxResult(result)
                if verbose:
                    typer.echo(f"       ID {motor_id}: {error_msg} ❌")
                results["errors"].append(f"ID {motor_id} Present_Position read failed: {error_msg}")
        
        port_handler.closePort()
        
    except Exception as e:
        results["errors"].append(str(e))
        if verbose:
            typer.echo(f"  ❌ Error: {e}")
    
    if verbose:
        typer.echo("")
        if results["errors"]:
            typer.echo("⚠️  Issues found:")
            for err in results["errors"]:
                typer.echo(f"   • {err}")
        else:
            typer.echo("✅ All diagnostics passed!")
    
    return results


def diagnose_all_ports():
    """Run diagnostics on all detected serial ports."""
    typer.echo("🔬 Running connection diagnostics...\n")
    
    ports = get_serial_ports()
    
    if not ports:
        typer.echo("❌ No serial ports found")
        return
    
    for port in ports:
        diagnose_connection(port, verbose=True)
        typer.echo("")


if __name__ == "__main__":
    scan_motors()

