import json
from pathlib import Path

from robot.motion import RobotMotion


CALIBRATION_FILE = Path(__file__).resolve().parent / "robot" / "motion_calibration.json"


def float_input(prompt, default=None, minimum=0.0):
    """Prompt until the user enters a valid number at or above minimum."""
    while True:
        suffix = f" [{default}]" if default is not None else ""
        value = input(f"{prompt}{suffix}: ").strip()
        if not value and default is not None:
            return float(default)
        try:
            parsed = float(value)
            if parsed >= minimum:
                return parsed
        except ValueError:
            pass
        print("Enter a number equal to or above", minimum)


def load_profiles():
    """Load every floor profile, or return an empty dictionary if none can be read."""
    try:
        with CALIBRATION_FILE.open("r", encoding="utf-8") as calibration_file:
            return json.load(calibration_file)
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return {}


def save_profile(name, profile):
    """Save or replace one floor profile in the calibration JSON file."""
    profiles = load_profiles()
    profiles[name] = profile
    with CALIBRATION_FILE.open("w", encoding="utf-8") as calibration_file:
        json.dump(profiles, calibration_file, indent=2)
    print("Saved", name, "to", CALIBRATION_FILE)


def turn_duration(old_duration, measured_angle):
    """Scale the turn duration so an angle of 90 degrees takes the expected duration."""
    if not 1.0 <= measured_angle <= 180.0:
        raise ValueError("Measured angle must be between 1 and 180 degrees")
    return old_duration * (90.0 / measured_angle)


def calibrate_turn(motion, direction, profile):
    """Run repeated 90-degree turns and replace the profile's duration using their average."""
    is_left = direction == "left"
    duration_key = "left_turn_duration" if is_left else "right_turn_duration"
    old_duration = float(profile.get(duration_key, 0.30 if is_left else 0.21))
    repetitions = int(float_input("Turns to run", 3, 1))
    speed = float_input("Turn speed", 0.30, 0.05)

    if is_left:
        motion.turn_90_left(speed=speed, duration=old_duration)
    else:
        motion.turn_90_right(speed=speed, duration=old_duration)

    # Running more than one turn lets the final value use an average measurement.
    for _ in range(repetitions - 1):
        input("Position the robot for the next turn, then press Enter.")
        if is_left:
            motion.turn_90_left(speed=speed, duration=old_duration)
        else:
            motion.turn_90_right(speed=speed, duration=old_duration)

    angles = []
    for index in range(repetitions):
        angle = float_input(f"Measured angle for turn {index + 1}", 90.0, 1.0)
        angles.append(angle)

    # Each run only affects this calibration if it is measured and included here.
    measured_angle = sum(angles) / len(angles)
    corrected_duration = turn_duration(old_duration, measured_angle)
    profile[duration_key] = round(corrected_duration, 4)
    print(f"Average angle: {measured_angle:.1f} degrees")
    print(f"New {direction} duration: {corrected_duration:.4f} seconds")


def calibrate_straight_profile(profile):
    """Adjust one motor correction from a manually measured forward-test drift."""
    distance_cm = float_input("Forward test distance in cm", 200.0, 50.0)
    drift_cm = float_input("Absolute sideways drift in cm", 0.0, 0.0)
    if drift_cm == 0:
        print("No sideways drift entered; corrections unchanged.")
        return

    drift_side = input("Did it drift left or right? ").strip().lower()
    if drift_side not in ("left", "right"):
        print("Side not recognised; corrections unchanged.")
        return

    # This factor estimates how much motor imbalance caused the observed drift.
    change = 1.0 - (drift_cm / distance_cm)
    change = max(0.80, min(1.20, change))
    if drift_side == "left":
        # A left drift is corrected by reducing the left motor's correction.
        profile["left_correction"] = round(max(0.5, min(1.5, float(profile.get("left_correction", 1.0)) / change)), 4)
    else:
        # A right drift is corrected by reducing the right motor's correction.
        profile["right_correction"] = round(max(0.5, min(1.5, float(profile.get("right_correction", 1.0)) / change)), 4)
    print("Updated correction:", profile["left_correction"], profile["right_correction"])


def apply_profile(motion, profile):
    """Copy the in-memory profile values onto the RobotMotion instance."""
    motion.standard_voltage = float(profile["standard_voltage"])
    motion.left_correction = float(profile["left_correction"])
    motion.right_correction = float(profile["right_correction"])
    motion.left_turn_duration = float(profile["left_turn_duration"])
    motion.right_turn_duration = float(profile["right_turn_duration"])


def main():
    """Run the interactive calibration menu; values remain unsaved until Save is chosen."""
    print("Motion calibration")
    floor = input("Floor/profile name [default]: ").strip().lower() or "default"
    profiles = load_profiles()
    profile = dict(profiles.get(floor, {
        "standard_voltage": 11.796,
        "left_correction": 1.0,
        "right_correction": 0.885,
        "left_turn_duration": 0.30,
        "right_turn_duration": 0.21,
    }))
    profile["standard_voltage"] = float_input(
        "Calibration battery voltage",
        profile.get("standard_voltage", 11.796),
        9.0,
    )

    motion = RobotMotion(floor=floor)
    apply_profile(motion, profile)

    while True:
        print("\n1. Test left 90-degree turn")
        print("2. Test right 90-degree turn")
        print("3. Calibrate left 90-degree turn")
        print("4. Calibrate right 90-degree turn")
        print("5. Calibrate straight driving")
        print("6. Save and exit")
        print("7. Exit without saving")
        choice = input("Choose: ").strip()

        if choice == "1":
            motion.turn_90_left()
        elif choice == "2":
            motion.turn_90_right()
        elif choice == "3":
            calibrate_turn(motion, "left", profile)
            apply_profile(motion, profile)
        elif choice == "4":
            calibrate_turn(motion, "right", profile)
            apply_profile(motion, profile)
        elif choice == "5":
            calibrate_straight_profile(profile)
            apply_profile(motion, profile)
        elif choice == "6":
            save_profile(floor, profile)
            break
        elif choice == "7":
            break


if __name__ == "__main__":
    main()
