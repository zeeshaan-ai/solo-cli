"""
Replay mode for LeRobot
Handles replaying recorded dataset episodes on the robot
"""

import time
import typer
from rich.prompt import Prompt, Confirm

from solo.commands.robots.lerobot.config import (
    validate_lerobot_config,
    get_robot_config_classes,
    save_lerobot_config,
    is_bimanual_robot,
    is_realman_robot,
    create_follower_config,
    create_bimanual_follower_config,
)
from solo.commands.robots.lerobot.mode_config import use_preconfigured_args, load_mode_config
from solo.commands.robots.lerobot.ports import detect_arm_port, detect_bimanual_arm_ports
from solo.commands.robots.lerobot.utils.text_cleaning import clean_ansi_codes


def replay_mode(config: dict, auto_use: bool = False, replay_options: dict = None):
    """Handle LeRobot replay mode - replay actions from a recorded dataset episode"""
    typer.echo("🔄 Starting LeRobot replay mode...")
    
    # Check if CLI arguments were provided (non-interactive mode)
    if replay_options and replay_options.get('dataset'):
        # Use CLI arguments
        _, follower_port, _, _, robot_type = validate_lerobot_config(config)
        follower_id = replay_options.get('follower_id')
        dataset_repo_id = replay_options.get('dataset')
        episode = replay_options.get('episode', 0)
        fps = replay_options.get('fps', 30)
        play_sounds = True
        
        typer.echo(f"📦 Dataset: {dataset_repo_id}")
        typer.echo(f"📹 Episode: {episode}")
        if follower_id:
            typer.echo(f"🤖 Follower ID: {follower_id}")
    else:
        # Check for preconfigured replay settings
        preconfigured, detected_robot_type = use_preconfigured_args(config, 'replay', 'Replay', auto_use=auto_use)
        
        if preconfigured and preconfigured.get('follower_port') and preconfigured.get('dataset_repo_id'):
            robot_type = preconfigured.get('robot_type')
            follower_port = preconfigured.get('follower_port')
            follower_id = preconfigured.get('follower_id')
            dataset_repo_id = clean_ansi_codes(preconfigured.get('dataset_repo_id', ''))
            episode = preconfigured.get('episode', 0)
            fps = preconfigured.get('fps', 30)
            play_sounds = preconfigured.get('play_sounds', True)
        else:
            # Get robot config
            _, follower_port, _, _, saved_robot_type = validate_lerobot_config(config)
            
            # Use detected robot type if available (e.g., from mismatch detection), otherwise use saved
            robot_type = detected_robot_type if detected_robot_type else saved_robot_type
        
            if not robot_type:
                from solo.commands.robots.lerobot.utils.helper import auto_detect_robot
                robot_type = auto_detect_robot(default="so101")
            
            # Handle port detection based on robot type
            if is_realman_robot(robot_type):
                from solo.commands.robots.lerobot.utils.helper import get_realman_configs
                realman_config = get_realman_configs(config)
                config['realman_config'] = realman_config
                follower_port = None  # Network-based
                typer.echo(f"\n🔌 RealMan follower: {realman_config.get('ip')}:{realman_config.get('port')}")
            
            elif is_bimanual_robot(robot_type):
                # Bimanual port detection
                lerobot_config = config.get('lerobot', {})
                left_follower_port = lerobot_config.get('left_follower_port')
                right_follower_port = lerobot_config.get('right_follower_port')
                
                if not left_follower_port or not right_follower_port:
                    left_follower_port, right_follower_port = detect_bimanual_arm_ports("follower")
                    if 'lerobot' not in config:
                        config['lerobot'] = {}
                    config['lerobot']['left_follower_port'] = left_follower_port
                    config['lerobot']['right_follower_port'] = right_follower_port
            else:
                # Single-arm port detection
                from solo.commands.robots.lerobot.utils.helper import port_detection
                follower_port = port_detection(config, "follower", robot_type, follower_port)
            
            # Get follower ID
            from solo.commands.robots.lerobot.utils.helper import prompt_arm_id
            follower_id = prompt_arm_id(config, "follower", robot_type)
            
            # Get default dataset from recording config if available
            recording_config = load_mode_config(config, 'recording')
            default_dataset = recording_config.get('dataset_repo_id') if recording_config else None
            
            dataset_repo_id = clean_ansi_codes(Prompt.ask("Enter dataset repository ID", default=default_dataset or ""))
            if '/' not in dataset_repo_id:
                dataset_repo_id = f"local/{dataset_repo_id}"
            
            episode = int(Prompt.ask("Enter episode number to replay", default="0"))
            fps = 30
            play_sounds = True
            
            # Save config
            from solo.commands.robots.lerobot.mode_config import save_replay_config
            save_replay_config(config, {
                'robot_type': robot_type, 'follower_port': follower_port, 'follower_id': follower_id,
                'dataset_repo_id': dataset_repo_id, 'episode': episode, 'fps': fps, 'play_sounds': play_sounds
            })
    
    typer.echo(f"📊 Replaying episode {episode} from {dataset_repo_id}")
    
    # Import lerobot components
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.processor import make_default_robot_action_processor
    from lerobot.robots import make_robot_from_config
    from lerobot.utils.constants import ACTION, HF_LEROBOT_HOME
    from lerobot.utils.robot_utils import precise_sleep
    from lerobot.utils.utils import log_say
    
    robot = None
    try:
        # Setup robot
        _, follower_config_class = get_robot_config_classes(robot_type)
        if not follower_config_class:
            raise ValueError(f"Unsupported robot type: {robot_type}")
        
        # Create follower config based on robot type
        if is_realman_robot(robot_type):
            # RealMan: Create network-based follower config
            from solo.commands.robots.lerobot.realman_config import create_realman_follower_config
            realman_cfg = config.get('realman_config')
            follower_config = create_realman_follower_config(
                realman_cfg,
                camera_config=None,
                follower_id=follower_id
            )
        
        elif is_bimanual_robot(robot_type):
            lerobot_config = config.get('lerobot', {})
            left_follower_port = lerobot_config.get('left_follower_port')
            right_follower_port = lerobot_config.get('right_follower_port')
            
            follower_config = create_bimanual_follower_config(
                follower_config_class,
                left_follower_port,
                right_follower_port,
                robot_type,
                camera_config=None,
                follower_id=follower_id
            )
        else:
            follower_config = create_follower_config(
                follower_config_class,
                follower_port,
                robot_type,
                follower_id=follower_id
            )
        
        # Load dataset - handle local datasets properly
        # For local datasets (starting with "local/"), check if the path exists
        # and verify metadata before loading to avoid HuggingFace Hub lookup
        import json
        local_dataset_path = HF_LEROBOT_HOME / dataset_repo_id
        is_local_dataset = dataset_repo_id.startswith("local/")
        
        if is_local_dataset:
            if not local_dataset_path.exists():
                raise FileNotFoundError(
                    f"Local dataset not found at: {local_dataset_path}\n"
                    f"Please check the dataset name and ensure it was recorded locally."
                )
            # Verify metadata exists
            meta_info_path = local_dataset_path / "meta" / "info.json"
            if not meta_info_path.exists():
                raise FileNotFoundError(
                    f"Dataset metadata not found at: {meta_info_path}\n"
                    f"The dataset may be incomplete or corrupted."
                )
            # Load info.json to validate episode number
            with open(meta_info_path, 'r') as f:
                info = json.load(f)
            total_episodes = info.get('total_episodes', 0)
            if episode >= total_episodes:
                raise ValueError(
                    f"Episode {episode} does not exist. Dataset has {total_episodes} episode(s).\n"
                    f"Valid episode range: 0 to {total_episodes - 1}"
                )
            # Check for data files
            data_path = local_dataset_path / "data"
            if not data_path.exists() or not any(data_path.rglob("*.parquet")):
                raise FileNotFoundError(
                    f"Dataset data files not found at: {data_path}\n"
                    f"The dataset may be incomplete or corrupted."
                )
            typer.echo(f"📂 Loading local dataset from: {local_dataset_path}")
            typer.echo(f"📊 Dataset has {total_episodes} episode(s)")
        
        # Load dataset (default behavior uses HF_LEROBOT_HOME / repo_id as root)
        try:
            dataset = LeRobotDataset(dataset_repo_id, episodes=[episode])
        except Exception as e:
            error_msg = str(e)
            # Catch HuggingFace Hub errors for local datasets
            if is_local_dataset and ("404 Client Error" in error_msg or "Repository Not Found" in error_msg):
                raise RuntimeError(
                    f"Failed to load local dataset '{dataset_repo_id}'.\n"
                    f"Dataset path: {local_dataset_path}\n"
                    f"This may be due to version compatibility issues or corrupted metadata.\n"
                    f"Try re-recording the dataset or check the dataset files."
                ) from e
            raise
        episode_frames = dataset.hf_dataset.filter(lambda x: x["episode_index"] == episode)
        actions = episode_frames.select_columns(ACTION)
        typer.echo(f"📥 Loaded {len(episode_frames)} frames")
        
        # Connect and replay with retry logic
        robot_action_processor = make_default_robot_action_processor()
        
        max_retries = 1
        for attempt in range(max_retries + 1):
            try:
                robot = make_robot_from_config(follower_config)
                robot.connect()
                
                log_say("Replaying episode", play_sounds, blocking=True)
                
                for idx in range(len(episode_frames)):
                    start_t = time.perf_counter()
                    
                    action = {name: actions[idx][ACTION][i] for i, name in enumerate(dataset.features[ACTION]["names"])}
                    processed_action = robot_action_processor((action, robot.get_observation()))
                    robot.send_action(processed_action)
                    
                    precise_sleep(1 / fps - (time.perf_counter() - start_t))
                
                robot.disconnect()
                typer.echo(f"✅ Replay completed! ({len(episode_frames)} frames)")
                break  # Success, exit retry loop
                
            except Exception as e:
                error_msg = str(e)
                # Check if it's a port connection error
                if "Could not connect on port" in error_msg or "Make sure you are using the correct port" in error_msg:
                    if attempt < max_retries:
                        typer.echo(f"❌ Connection failed: {error_msg}")
                        typer.echo("🔄 Attempting to detect new port...")
                        
                        # Detect new follower port(s)
                        if is_bimanual_robot(robot_type):
                            left_follower_port, right_follower_port = detect_bimanual_arm_ports("follower")
                            
                            if left_follower_port and right_follower_port:
                                typer.echo(f"✅ Found new follower ports: {left_follower_port}, {right_follower_port}")
                                
                                # Save updated ports to main lerobot config
                                save_lerobot_config(config, {
                                    'left_follower_port': left_follower_port,
                                    'right_follower_port': right_follower_port
                                })
                                
                                # Recreate follower config
                                follower_config = create_bimanual_follower_config(
                                    follower_config_class,
                                    left_follower_port,
                                    right_follower_port,
                                    robot_type,
                                    camera_config=None,
                                    follower_id=follower_id
                                )
                                typer.echo("🔄 Retrying replay with new ports...")
                                continue
                            else:
                                typer.echo("❌ Could not find new ports. Please check connections.")
                                return
                        else:
                            new_follower_port, _ = detect_arm_port("follower", robot_type=robot_type)
                            
                            if new_follower_port and new_follower_port != follower_port:
                                follower_port = new_follower_port
                                typer.echo(f"✅ Found new follower port: {follower_port}")
                                
                                # Save updated port to main lerobot config (shared across all modes)
                                save_lerobot_config(config, {'follower_port': follower_port})
                                
                                # Save updated port to replay config
                                from solo.commands.robots.lerobot.mode_config import save_replay_config
                                save_replay_config(config, {
                                    'robot_type': robot_type, 'follower_port': follower_port, 'follower_id': follower_id,
                                    'dataset_repo_id': dataset_repo_id, 'episode': episode, 'fps': fps, 'play_sounds': play_sounds
                                })
                                
                                follower_config = create_follower_config(follower_config_class, follower_port, robot_type, follower_id=follower_id)
                                typer.echo("🔄 Retrying replay with new port...")
                                continue
                            else:
                                typer.echo("❌ Could not find new port. Please check connections.")
                                return
                    else:
                        typer.echo(f"❌ Replay failed after retry: {error_msg}")
                        return
                else:
                    raise  # Re-raise non-port errors
        
    except KeyboardInterrupt:
        typer.echo("\n🛑 Stopped by user.")
    except Exception as e:
        typer.echo(f"❌ Replay failed: {e}")
    finally:
        if robot:
            try:
                robot.disconnect()
            except Exception:
                pass

