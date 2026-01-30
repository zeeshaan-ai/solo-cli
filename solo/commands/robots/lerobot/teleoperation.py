"""
Teleoperation utilities for LeRobot

Note: Heavy lerobot imports are done lazily inside functions to speed up CLI startup.
"""

import typer
from rich.prompt import Confirm, Prompt
from typing import Optional

from solo.commands.robots.lerobot.config import (
    get_robot_config_classes,
    create_follower_config,
    get_known_ids,
    save_lerobot_config,
    is_bimanual_robot,
    is_realman_robot,
    create_bimanual_leader_config,
    create_bimanual_follower_config,
    validate_lerobot_config,
)
from solo.commands.robots.lerobot.mode_config import use_preconfigured_args
from solo.commands.robots.lerobot.ports import detect_arm_port, detect_and_retry_ports, detect_bimanual_arm_ports

# Heavy lerobot imports are done lazily inside functions:
# - from lerobot.scripts.lerobot_teleoperate import TeleoperateConfig, teleoperate

def teleoperation(config: dict = None, auto_use: bool = False) -> bool:
    leader_id = None
    follower_id = None
    camera_config = None

    preconfigured, detected_robot_type = use_preconfigured_args(config, 'teleop', 'Teleoperation', auto_use=auto_use)
    if preconfigured:
        leader_port = preconfigured.get('leader_port')
        follower_port = preconfigured.get('follower_port')
        robot_type = preconfigured.get('robot_type')
        camera_config = preconfigured.get('camera_config')
        leader_id = preconfigured.get('leader_id')
        follower_id = preconfigured.get('follower_id')
    

    if not preconfigured:
        # Validate configuration using utility function
        leader_port, follower_port, leader_calibrated, follower_calibrated, saved_robot_type = validate_lerobot_config(config)
        
        # Use detected robot type if available (e.g., from mismatch detection), otherwise use saved
        robot_type = detected_robot_type if detected_robot_type else saved_robot_type
        
        if not robot_type:
            # Try auto-detection first
            try:
                from solo.commands.robots.lerobot.scan import auto_detect_robot_type
                detected_type, port_info = auto_detect_robot_type(verbose=True)
                
                if detected_type:
                    # Ask user to confirm auto-detected type
                    typer.echo(f"\n🤖 Auto-detected robot type: {detected_type.upper()}")
                    use_detected = Confirm.ask("Use this robot type?", default=True)
                    if use_detected:
                        robot_type = detected_type
                    else:
                        detected_type = None
                
                if not detected_type:
                    # Manual selection
                    typer.echo("\n🤖 Select your robot type:")
                    typer.echo("1. SO100 (single arm)")
                    typer.echo("2. SO101 (single arm)")
                    typer.echo("3. Koch (single arm)")
                    typer.echo("4. RealMan R1D2 (follower with SO101 leader)")
                    typer.echo("5. Bimanual SO100")
                    typer.echo("6. Bimanual SO101")
                    robot_choice = int(Prompt.ask("Enter robot type", default="2"))
                    robot_type_map = {
                        1: "so100",
                        2: "so101",
                        3: "koch",
                        4: "realman_r1d2",
                        5: "bi_so100",
                        6: "bi_so101"
                    }
                    robot_type = robot_type_map.get(robot_choice, "so101")
            except Exception as e:
                typer.echo(f"⚠️  Auto-detection failed: {e}")
                # Fall back to manual selection
                typer.echo("\n🤖 Select your robot type:")
                typer.echo("1. SO100 (single arm)")
                typer.echo("2. SO101 (single arm)")
                typer.echo("3. Koch (single arm)")
                typer.echo("4. RealMan R1D2 (follower with SO101 leader)")
                typer.echo("5. Bimanual SO100")
                typer.echo("6. Bimanual SO101")
                robot_choice = int(Prompt.ask("Enter robot type", default="2"))
                robot_type_map = {
                    1: "so100",
                    2: "so101",
                    3: "koch",
                    4: "realman_r1d2",
                    5: "bi_so100",
                    6: "bi_so101"
                }
                robot_type = robot_type_map.get(robot_choice, "so101")
            
            config['robot_type'] = robot_type
        
        # Check if RealMan and handle network-based follower
        if is_realman_robot(robot_type):
            lerobot_config = config.get('lerobot', {})
            
            # Leader is SO101 (USB)
            if not leader_port:
                leader_port, _ = detect_arm_port("leader", robot_type="so101")
                config['leader_port'] = leader_port
            
            # Follower is RealMan (network) - always load fresh config from YAML
            # to pick up any configuration changes (like invert_joints)
            from solo.commands.robots.lerobot.realman_config import load_realman_config
            realman_config = load_realman_config()
            # Merge with any saved network settings (ip/port) if they exist
            saved_realman = lerobot_config.get('realman_config', {})
            if saved_realman:
                realman_config['ip'] = saved_realman.get('ip', realman_config['ip'])
                realman_config['port'] = saved_realman.get('port', realman_config['port'])
            config['realman_config'] = realman_config
            
            # For RealMan, follower_port is not used (network-based)
            follower_port = None
        
        # Check if bimanual and handle port detection accordingly
        elif is_bimanual_robot(robot_type):
            import os
            lerobot_config = config.get('lerobot', {})
            left_leader_port = lerobot_config.get('left_leader_port')
            right_leader_port = lerobot_config.get('right_leader_port')
            left_follower_port = lerobot_config.get('left_follower_port')
            right_follower_port = lerobot_config.get('right_follower_port')
            
            # Validate that saved ports actually exist
            def port_exists(port): return port and os.path.exists(port)
            
            if not port_exists(left_leader_port) or not port_exists(right_leader_port):
                if left_leader_port or right_leader_port:
                    typer.echo("⚠️  Saved leader ports not connected, detecting...")
                left_leader_port, right_leader_port = detect_bimanual_arm_ports("leader")
                if 'lerobot' not in config:
                    config['lerobot'] = {}
                config['lerobot']['left_leader_port'] = left_leader_port
                config['lerobot']['right_leader_port'] = right_leader_port
            if not port_exists(left_follower_port) or not port_exists(right_follower_port):
                if left_follower_port or right_follower_port:
                    typer.echo("⚠️  Saved follower ports not connected, detecting...")
                left_follower_port, right_follower_port = detect_bimanual_arm_ports("follower")
                if 'lerobot' not in config:
                    config['lerobot'] = {}
                config['lerobot']['left_follower_port'] = left_follower_port
                config['lerobot']['right_follower_port'] = right_follower_port
        else:
            import os
            if not leader_port or not os.path.exists(leader_port):
                if leader_port:
                    typer.echo("⚠️  Saved leader port not connected, detecting...")
                leader_port, _ = detect_arm_port("leader", robot_type=robot_type)
                config['leader_port'] = leader_port
            if not follower_port or not os.path.exists(follower_port):
                if follower_port:
                    typer.echo("⚠️  Saved follower port not connected, detecting...")
                follower_port, _ = detect_arm_port("follower", robot_type=robot_type)
                config['follower_port'] = follower_port
    
        # Prompt/select ids if not provided
        known_leader_ids, known_follower_ids = get_known_ids(config, robot_type=robot_type)
        default_leader_id = config.get('lerobot', {}).get('leader_id') or f"{robot_type}_leader"
        default_follower_id = config.get('lerobot', {}).get('follower_id') or f"{robot_type}_follower"

        from solo.commands.robots.lerobot.config import display_known_ids
        if not leader_id:
            display_known_ids(known_leader_ids, "leader", detected_robot_type=robot_type, config=config)
            leader_id = Prompt.ask("Enter leader id", default=default_leader_id)
        if not follower_id:
            display_known_ids(known_follower_ids, "follower", detected_robot_type=robot_type, config=config)
            follower_id = Prompt.ask("Enter follower id", default=default_follower_id)
        
        # Setup cameras if not provided
        if camera_config is None:
            # Check if cameras were previously configured
            lerobot_config = config.get('lerobot', {})
            prev_camera_config = lerobot_config.get('camera_config', {})
            cameras_previously_enabled = prev_camera_config.get('enabled', False)
            
            # Default to previous setting (no if never configured)
            use_camera = Confirm.ask("Would you like to setup cameras?", default=cameras_previously_enabled)
            if use_camera:
                from solo.commands.robots.lerobot.cameras import setup_cameras
                camera_config = setup_cameras()
            else:
                # Set empty camera config when user chooses not to use cameras
                camera_config = {'enabled': False, 'cameras': []}

    try:
        # Determine config classes based on robot type
        leader_config_class, follower_config_class = get_robot_config_classes(robot_type)
        
        if leader_config_class is None or follower_config_class is None:
            typer.echo(f"❌ Unsupported robot type for teleoperation: {robot_type}")
            return False
        
        # Debug: Show port/connection assignments
        typer.echo(f"\n🔌 Connection Configuration:")
        typer.echo(f"   • Leader port:   {leader_port}")
        if is_realman_robot(robot_type):
            realman_config = config.get('realman_config', {})
            typer.echo(f"   • Follower:      RealMan @ {realman_config.get('ip', 'N/A')}:{realman_config.get('port', 'N/A')}")
        else:
            typer.echo(f"   • Follower port: {follower_port}")
        
        # Create configurations based on robot type
        if is_realman_robot(robot_type):
            # RealMan: SO101 leader (USB) + RealMan follower (network)
            from solo.commands.robots.lerobot.realman_config import create_realman_follower_config
            
            # Create SO101 leader config
            leader_config = leader_config_class(port=leader_port, id=leader_id or "so101_leader")
            
            # Create RealMan follower config
            realman_config = config.get('realman_config', {})
            follower_config = create_realman_follower_config(
                realman_config,
                camera_config,
                follower_id=follower_id or "realman_r1d2_follower"
            )
        
        elif is_bimanual_robot(robot_type):
            # Create bimanual configurations
            lerobot_config = config.get('lerobot', {})
            left_leader_port = lerobot_config.get('left_leader_port')
            right_leader_port = lerobot_config.get('right_leader_port')
            left_follower_port = lerobot_config.get('left_follower_port')
            right_follower_port = lerobot_config.get('right_follower_port')
            
            leader_config = create_bimanual_leader_config(
                leader_config_class,
                left_leader_port,
                right_leader_port,
                robot_type,
                leader_id=leader_id
            )
            
            follower_config = create_bimanual_follower_config(
                follower_config_class,
                left_follower_port,
                right_follower_port,
                robot_type,
                camera_config,
                follower_id=follower_id
            )
        else:
            # Create single-arm configurations
            leader_config = leader_config_class(port=leader_port, id=leader_id)
            
            # Create robot config with cameras if enabled
            follower_config = create_follower_config(
                follower_config_class,
                follower_port,
                robot_type,
                camera_config,
                follower_id=follower_id,
            )
        
        # Lazy import heavy lerobot modules
        typer.echo("\n⏳ Loading LeRobot modules...")
        from lerobot.scripts.lerobot_teleoperate import TeleoperateConfig, teleoperate
        typer.echo("✅ LeRobot modules loaded.\n")
        
        # Create teleoperation config
        teleop_config = TeleoperateConfig(
            teleop=leader_config,
            robot=follower_config,
            fps=60,
            display_data=True
        )
        
        # Save configuration before execution (if not using preconfigured settings)
        if config and not preconfigured:
            from .mode_config import save_teleop_config
            if is_bimanual_robot(robot_type):
                # For bimanual, we don't use the standard save_teleop_config
                # Configuration is already saved via save_lerobot_config
                pass
            else:
                save_teleop_config(
                    config,
                    leader_port,
                    follower_port,
                    robot_type,
                    camera_config,
                    leader_id,
                    follower_id,
                )
        
        if is_realman_robot(robot_type):
            typer.echo("🎮 Starting RealMan teleoperation... Press Ctrl+C to stop.")
            typer.echo("📋 Move the SO101 leader arm to control the RealMan follower arm.")
            typer.echo("⚠️  Note: RealMan uses network connection - ensure robot is powered and connected.")
        elif is_bimanual_robot(robot_type):
            typer.echo("🎮 Starting bimanual teleoperation... Press Ctrl+C to stop.")
            typer.echo("📋 Move BOTH leader arms to control BOTH follower arms.")
        else:
            typer.echo("🎮 Starting teleoperation... Press Ctrl+C to stop.")
            typer.echo("📋 Move the leader arm to control the follower arm.")
        
        # Start teleoperation with retry logic
        max_retries = 1
        for attempt in range(max_retries + 1):
            try:
                # Use standard lerobot teleoperate (stability is now handled in lerobot itself)
                teleoperate(teleop_config)
                
                return True
                
            except Exception as e:
                error_msg = str(e)
                
                # Enhanced error reporting for sync_read failures
                if "sync read" in error_msg.lower() or "sync_read" in error_msg.lower():
                    typer.echo(f"\n⚠️  Motor communication error detected!")
                    typer.echo(f"   Error: {error_msg}")
                    typer.echo("")
                    typer.echo("🔍 Running quick diagnostics...")
                    try:
                        from solo.commands.robots.lerobot.scan import diagnose_connection
                        typer.echo(f"\n   Leader port ({leader_port}):")
                        diagnose_connection(leader_port, verbose=True)
                        typer.echo(f"\n   Follower port ({follower_port}):")
                        diagnose_connection(follower_port, verbose=True)
                    except Exception as diag_err:
                        typer.echo(f"   (Diagnostic failed: {diag_err})")
                    typer.echo("")
                    typer.echo("💡 Possible causes:")
                    typer.echo("   1. Motor power issue - check 12V supply")
                    typer.echo("   2. USB timing - try unplugging and waiting 2 seconds")
                    typer.echo("   3. Port swapped - run 'solo robo --scan' to verify")
                    typer.echo("   4. Loose cable in daisy chain")
                    return False
                
                # Check if it's a port connection error
                if "Could not connect on port" in error_msg or "Make sure you are using the correct port" in error_msg:
                    if attempt < max_retries:
                        typer.echo(f"❌ Connection failed: {error_msg}")
                        typer.echo("🔄 Attempting to detect new ports...")
                        
                        # Detect new ports and retry
                        new_leader_port, new_follower_port = detect_and_retry_ports(leader_port, follower_port, config)
                        
                        if new_leader_port != leader_port or new_follower_port != follower_port:
                            # Update ports and recreate configs
                            leader_port, follower_port = new_leader_port, new_follower_port
                            leader_config = leader_config_class(port=leader_port, id=leader_id)
                            follower_config = create_follower_config(
                                follower_config_class,
                                follower_port,
                                robot_type,
                                camera_config,
                                follower_id=follower_id,
                            )
                            teleop_config = TeleoperateConfig(
                                teleop=leader_config,
                                robot=follower_config,
                                fps=60,
                                display_data=True
                            )
                            typer.echo("🔄 Retrying teleoperation with new ports...")
                            continue
                        else:
                            typer.echo("❌ Could not find new ports. Please check connections.")
                            return False
                    else:
                        typer.echo(f"❌ Teleoperation failed after retry: {error_msg}")
                        return False
                else:
                    # Non-port related error
                    typer.echo(f"❌ Teleoperation failed: {error_msg}")
                    return False
        
    except KeyboardInterrupt:
        typer.echo("\n🛑 Teleoperation stopped by user.")
        return True
