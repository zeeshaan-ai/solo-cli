"""
Port detection utilities for LeRobot
"""

import platform
import subprocess
import time
from pathlib import Path
from typing import List, Optional
import typer
from rich.prompt import Prompt


def find_available_ports() -> List[str]:
    """Find all available serial ports on the system.
    
    Returns ports specific to robot arm connections:
    - Linux: /dev/ttyACM*, /dev/ttyUSB*
    - macOS: /dev/tty.usbmodem*, /dev/tty.usbserial*, etc.
    - Windows: COM ports via pyserial
    """
    try:
        # Use the comprehensive port detection from scan.py
        from solo.commands.robots.lerobot.scan import get_serial_ports
        return get_serial_ports()
    except ImportError:
        # Fallback to basic detection
        try:
            from serial.tools import list_ports
            
            if platform.system() == "Windows":
                ports = [port.device for port in list_ports.comports()]
            elif platform.system() == "Darwin":  # macOS
                # Look for USB modem/serial devices
                ports = []
                for pattern in ["tty.usbmodem*", "tty.usbserial*", "cu.usbmodem*", "cu.usbserial*"]:
                    ports.extend(str(p) for p in Path("/dev").glob(pattern))
            else:  # Linux
                ports = []
                for pattern in ["ttyACM*", "ttyUSB*"]:
                    ports.extend(str(p) for p in Path("/dev").glob(pattern))
            return sorted(set(ports))
        except ImportError:
            typer.echo("❌ pyserial is required for port detection. Installing...")
            try:
                subprocess.run(["pip", "install", "pyserial"], check=True)
                from serial.tools import list_ports
                if platform.system() == "Windows":
                    return [port.device for port in list_ports.comports()]
                else:
                    ports = []
                    patterns = ["ttyACM*", "ttyUSB*"] if platform.system() != "Darwin" else ["tty.usbmodem*", "tty.usbserial*"]
                    for pattern in patterns:
                        ports.extend(str(p) for p in Path("/dev").glob(pattern))
                    return sorted(ports)
            except Exception as e:
                typer.echo(f"❌ Failed to install pyserial: {e}")
                return []


def detect_arm_port(arm_type: str, robot_type: str = None, use_auto_detect: bool = True) -> tuple[Optional[str], Optional[str]]:
    """
    Detect the port for a specific arm (leader or follower)
    
    First tries auto-detection based on motor types.
    Falls back to manual plug/unplug detection if auto-detection fails.
    
    Returns (detected_port, detected_robot_type) tuple
    """
    detected_robot_type = robot_type
    
    # Try auto-detection first (for all robot types)
    if use_auto_detect:
        typer.echo(f"\n🔍 Auto-detecting {arm_type} arm port...")
        try:
            from solo.commands.robots.lerobot.scan import auto_detect_single_port
            port, auto_robot_type = auto_detect_single_port(arm_type, robot_type, verbose=True)
            if port:
                # Update robot type if auto-detected
                if auto_robot_type and detected_robot_type is None:
                    detected_robot_type = auto_robot_type
                return port, detected_robot_type
            typer.echo("   Falling back to manual detection...")
        except Exception as e:
            typer.echo(f"   Auto-detection failed: {e}")
            typer.echo("   Falling back to manual detection...")
    
    # Manual detection via plug/unplug
    typer.echo(f"\n🔍 Detecting port for {arm_type} arm...")
    
    # Get initial ports
    ports_before = find_available_ports()
    typer.echo(f"Available ports: {ports_before}")
    
    # Ask user to plug in the arm
    typer.echo(f"\n📱 Please plug in your {arm_type} arm and press Enter when connected.")
    input()
    
    time.sleep(1.0)  # Allow time for port to be detected
    
    # Get ports after connection
    ports_after = find_available_ports()
    new_ports = list(set(ports_after) - set(ports_before))
    
    if len(new_ports) == 1:
        port = new_ports[0]
        typer.echo(f"✅ Detected {arm_type} arm on port: {port}")
        # Try to detect robot type from the newly connected port
        if detected_robot_type is None:
            try:
                from solo.commands.robots.lerobot.scan import detect_robot_type_from_port
                auto_robot_type, _, _ = detect_robot_type_from_port(port, verbose=True)
                if auto_robot_type:
                    detected_robot_type = auto_robot_type
            except Exception:
                pass
        return port, detected_robot_type
    elif len(new_ports) == 0:
        # If no new ports detected but there are existing ports,
        # the arm might already be connected. Try unplug/replug method.
        if len(ports_before) > 0:
            typer.echo(f"⚠️  No new port detected. The {arm_type} arm might already be connected.")
            typer.echo(f"Let's identify the correct port by unplugging and replugging.")
            
            # Ask user to unplug the arm
            typer.echo(f"\n📱 Please UNPLUG your {arm_type} arm and press Enter when disconnected.")
            input()
            
            time.sleep(1.0)  # Allow time for port to be released
            
            # Get ports after disconnection
            ports_unplugged = find_available_ports()
            missing_ports = list(set(ports_before) - set(ports_unplugged))
            
            if len(missing_ports) == 1:
                # Found the port that disappeared
                port = missing_ports[0]
                typer.echo(f"✅ Identified {arm_type} arm port: {port}")
                typer.echo(f"📱 Please plug your {arm_type} arm back in and press Enter.")
                input()
                time.sleep(1.0)  # Allow time for reconnection
                # Try to detect robot type
                if detected_robot_type is None:
                    try:
                        from solo.commands.robots.lerobot.scan import detect_robot_type_from_port
                        auto_robot_type, _, _ = detect_robot_type_from_port(port, verbose=True)
                        if auto_robot_type:
                            detected_robot_type = auto_robot_type
                    except Exception:
                        pass
                return port, detected_robot_type
            elif len(missing_ports) == 0:
                typer.echo(f"❌ No port disappeared when unplugging {arm_type} arm. Please check connection.")
                return None, detected_robot_type
            else:
                typer.echo(f"⚠️  Multiple ports disappeared: {missing_ports}")
                typer.echo("Please select which port corresponds to your arm:")
                for i, port in enumerate(missing_ports, 1):
                    typer.echo(f"  {i}. {port}")
                
                choice = int(Prompt.ask("Enter port number", default="1"))
                if 1 <= choice <= len(missing_ports):
                    port = missing_ports[choice - 1]
                    typer.echo(f"📱 Please plug your {arm_type} arm back in and press Enter.")
                    input()
                    time.sleep(1.0)
                    return port, detected_robot_type
                else:
                    port = missing_ports[0]
                    typer.echo(f"📱 Please plug your {arm_type} arm back in and press Enter.")
                    input()
                    time.sleep(1.0)
                    return port, detected_robot_type
        else:
            typer.echo(f"❌ No ports available and no new port detected for {arm_type} arm. Please check connection.")
            return None, detected_robot_type
    else:
        typer.echo(f"⚠️  Multiple new ports detected: {new_ports}")
        typer.echo("Please select the correct port:")
        for i, port in enumerate(new_ports, 1):
            typer.echo(f"  {i}. {port}")
        
        choice = int(Prompt.ask("Enter port number", default="1"))
        if 1 <= choice <= len(new_ports):
            return new_ports[choice - 1], detected_robot_type
        else:
            return new_ports[0], detected_robot_type


def auto_detect_both_ports(robot_type: str = None) -> tuple[Optional[str], Optional[str], Optional[str]]:
    """
    Auto-detect both leader and follower ports based on motor types.
    Also auto-detects robot type if not specified.
    
    Returns (leader_port, follower_port, detected_robot_type).
    """
    try:
        from solo.commands.robots.lerobot.scan import auto_detect_ports
        return auto_detect_ports(robot_type, verbose=True)
    except Exception as e:
        typer.echo(f"⚠️  Auto-detection failed: {e}")
        return None, None, robot_type


def detect_bimanual_arm_ports(arm_type: str) -> tuple[Optional[str], Optional[str]]:
    """
    Detect ports for bimanual arm (left and right) using voltage detection and movement.
    Returns (left_port, right_port)
    """
    from solo.commands.robots.lerobot.scan import (
        get_serial_ports, 
        scan_feetech_port, 
        detect_moving_port,
        detect_so_arm_type_by_voltage,
        set_feetech_torque
    )
    
    typer.echo(f"\n🔍 Detecting ports for bimanual {arm_type} arms...")
    
    # Find all ports with Feetech motors and filter by voltage
    ports = get_serial_ports()
    candidate_ports = []
    
    for port in ports:
        motors = scan_feetech_port(port, protocol=0)
        if motors:
            # Check voltage to determine if leader (5V) or follower (12V)
            detected_arm_type = detect_so_arm_type_by_voltage(port, verbose=False)
            if detected_arm_type == arm_type or detected_arm_type == "unknown":
                candidate_ports.append(port)
                typer.echo(f"   ✅ {port}: {detected_arm_type or 'unknown'} arm (matches {arm_type})")
            else:
                typer.echo(f"   ⏭️  {port}: {detected_arm_type} arm (skipping, looking for {arm_type})")
    
    if len(candidate_ports) < 2:
        typer.echo(f"❌ Need at least 2 {arm_type} arms connected, found {len(candidate_ports)}")
        if len(candidate_ports) == 1:
            typer.echo(f"   Found: {candidate_ports[0]}")
        return None, None
    
    typer.echo(f"   Found {len(candidate_ports)} {arm_type} ports: {candidate_ports}")
    feetech_ports = candidate_ports
    
    # Disable torque on all candidate ports for movement detection (only needed for followers)
    if arm_type == "follower":
        typer.echo("   🔓 Disabling torque for movement detection...")
        for port in feetech_ports:
            set_feetech_torque(port, enable=False)
    
    try:
        # Detect LEFT arm by movement - monitor all ports simultaneously
        typer.echo(f"\n👈 Move the LEFT {arm_type} arm now...")
        typer.echo("   (Move any joint back and forth continuously)")
        input("   Press Enter, then KEEP MOVING the arm...")
        
        left_port = detect_moving_port(feetech_ports, duration=3.0, threshold=50, verbose=True)
        
        if left_port:
            typer.echo(f"   ✅ Detected LEFT {arm_type} arm on {left_port}")
        else:
            typer.echo(f"❌ No movement detected. Please try again with larger movements.")
            return None, None
        
        # Detect RIGHT arm by movement (from remaining ports)
        remaining_ports = [p for p in feetech_ports if p != left_port]
        
        if len(remaining_ports) == 1:
            # Only one port left, must be the right arm
            right_port = remaining_ports[0]
            typer.echo(f"\n👉 RIGHT {arm_type} arm detected on {right_port} (only remaining port)")
        else:
            typer.echo(f"\n👉 Move the RIGHT {arm_type} arm now...")
            typer.echo("   (Move any joint back and forth continuously)")
            input("   Press Enter, then KEEP MOVING the arm...")
            
            right_port = detect_moving_port(remaining_ports, duration=3.0, threshold=50, verbose=True)
            
            if right_port:
                typer.echo(f"   ✅ Detected RIGHT {arm_type} arm on {right_port}")
            else:
                typer.echo(f"❌ No movement detected for right arm.")
                return left_port, None
        
        typer.echo(f"\n✅ Detected bimanual {arm_type} arms:")
        typer.echo(f"   • Left {arm_type}: {left_port}")
        typer.echo(f"   • Right {arm_type}: {right_port}")
        
        return left_port, right_port
    
    finally:
        # Re-enable torque only on follower arms (leaders should stay torque-free)
        if arm_type == "follower":
            typer.echo("   🔒 Re-enabling torque...")
            for port in feetech_ports:
                set_feetech_torque(port, enable=True)


def detect_and_retry_ports(leader_port: str, follower_port: str, config: dict = None) -> tuple[str, str]:
    """
    Detect new ports if connection fails and update config.
    Also updates all preconfigured mode settings (teleop, recording, inference, replay).
    
    Returns (new_leader_port, new_follower_port)
    """
    typer.echo("🔍 Detecting new ports...")
    
    # Detect new ports
    new_leader_port, _ = detect_arm_port("leader")
    new_follower_port, _ = detect_arm_port("follower")
    
    if new_leader_port and new_follower_port:
        typer.echo(f"✅ Found new ports:")
        typer.echo(f"   • Leader: {new_leader_port}")
        typer.echo(f"   • Follower: {new_follower_port}")
        
        # Update config with new ports if provided
        if config:
            from solo.commands.robots.lerobot.config import save_lerobot_config
            from solo.commands.robots.lerobot.mode_config import update_all_mode_config_ports
            
            # Update general config
            save_lerobot_config(config, {
                'leader_port': new_leader_port,
                'follower_port': new_follower_port
            })
            
            # Also update all preconfigured mode settings
            update_all_mode_config_ports(config, new_leader_port, new_follower_port)
        
        return new_leader_port, new_follower_port
    else:
        error_msg = "Could not detect new ports automatically."
        if leader_port is None:
            error_msg += " Leader port is not set."
        if follower_port is None:
            error_msg += " Follower port is not set."
        raise ValueError(error_msg) 