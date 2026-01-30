#!/usr/bin/env python3
"""
Full motor reset - fixes extended position mode and offset issues.

Usage:
    1. Move arms to their physical middle position
    2. Run: python scripts/full_motor_reset.py
"""
import sys
import os
import glob

lerobot_path = os.path.expanduser('~/projects/stick/bimanual-arms/lerobot/src')
if os.path.exists(lerobot_path):
    sys.path.insert(0, lerobot_path)

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors.motors_bus import Motor, MotorNormMode

SO101_MOTORS = {
    "shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
    "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
    "elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
    "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
}


def full_reset_port(port):
    print(f"\n{'='*60}")
    print(f"🔧 Full reset on {port}")
    print(f"{'='*60}")
    
    try:
        motors = {k: Motor(v.id, v.model, v.norm_mode) for k, v in SO101_MOTORS.items()}
        bus = FeetechMotorsBus(port=port, motors=motors)
        
        # Set baud rate to 1M before connecting
        bus.default_baudrate = 1_000_000
        print(f"\n0️⃣  Connecting at 1M baud rate...")
        bus.connect()
        
        print("1️⃣  Disabling torque...")
        bus.disable_torque()
        
        print("2️⃣  Setting Operating_Mode to Position mode (0)...")
        for motor in bus.motors:
            try:
                bus.write("Operating_Mode", motor, 0)  # 0 = Position mode
            except Exception as e:
                print(f"   Warning: {motor}: {e}")
        
        print("3️⃣  Resetting Homing_Offset to 0...")
        for motor in bus.motors:
            try:
                bus.write("Homing_Offset", motor, 0)
            except Exception as e:
                print(f"   Warning: {motor}: {e}")
        
        print("4️⃣  Setting Min/Max Position Limits to full range (0-4095)...")
        for motor in bus.motors:
            try:
                bus.write("Min_Position_Limit", motor, 0, normalize=False)
                bus.write("Max_Position_Limit", motor, 4095, normalize=False)
            except Exception as e:
                print(f"   Warning: {motor}: {e}")
        
        print("\n5️⃣  Reading raw positions...")
        print("-" * 50)
        print(f"{'Joint':<15} {'Position':>10} {'Status':>15}")
        print("-" * 50)
        
        all_ok = True
        for motor in bus.motors:
            try:
                pos = bus.read("Present_Position", motor, normalize=False)
                if 0 <= pos <= 4095:
                    status = "✓ OK"
                else:
                    status = f"⚠️  OUT OF RANGE"
                    all_ok = False
                print(f"{motor:<15} {pos:>10} {status:>15}")
            except Exception as e:
                print(f"{motor:<15} {'ERROR':>10} {str(e)[:15]}")
                all_ok = False
        print("-" * 50)
        
        if all_ok:
            print("\n6️⃣  Centering positions to 2048...")
            bus.set_half_turn_homings()
            
            print("\n✅ Final positions (should be ~2048):")
            print("-" * 50)
            for motor in bus.motors:
                pos = bus.read("Present_Position", motor, normalize=False)
                offset = bus.read("Homing_Offset", motor, normalize=False)
                print(f"{motor:<15} pos={pos:>5}  offset={offset:>5}")
            print("-" * 50)
        else:
            print("\n⚠️  Some motors have out-of-range positions.")
            print("   Try power cycling the arm (unplug USB, wait 5 sec, plug back in)")
        
        bus.disconnect()
        return all_ok
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    print("=" * 60)
    print("🔧 Full Motor Reset Tool")
    print("=" * 60)
    print("\n⚠️  Make sure arms are in their physical MIDDLE position!")
    
    ports = sorted(glob.glob('/dev/ttyACM*'))
    
    if not ports:
        print("\n❌ No /dev/ttyACM* ports found.")
        return 1
    
    print(f"\n📡 Found ports: {ports}")
    
    response = input("\nProceed with full reset? [y/N]: ").strip().lower()
    if response != 'y':
        print("Cancelled.")
        return 0
    
    for port in ports:
        full_reset_port(port)
    
    print("\n" + "=" * 60)
    print("🎉 Done! Now try: solo robots lerobot calibrate")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())

