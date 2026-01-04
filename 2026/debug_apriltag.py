import sys

print("--- DIAGNOSTIC START ---")

# 1. Check if the library exists
try:
    import robotpy_apriltag as ra
    print(f"SUCCESS: robotpy_apriltag imported.")
except ImportError:
    print("CRITICAL FAILURE: robotpy_apriltag is NOT installed.")
    sys.exit(1)

# 2. Check if we can load the 2025 Field Layout
print("\n--- Checking Field Layout ---")
try:
    # This is the specific line from your detectors.py
    field_enum = ra.AprilTagField.k2025ReefscapeWelded
    print(f"Found Enum: {field_enum}")
    
    layout = ra.AprilTagFieldLayout.loadField(field_enum)
    print(f"SUCCESS: Layout loaded with {len(layout.getTags())} tags.")
    
    # Check a specific tag (e.g. Tag 1)
    pose = layout.getTagPose(1)
    print(f"Tag 1 Pose: {pose}")

except AttributeError:
    print("FAILURE: 'k2025ReefscapeWelded' not found in robotpy_apriltag.")
    print("Your robotpy-apriltag version might be too old for the 2025 game.")
    print("Available Fields:")
    for x in dir(ra.AprilTagField):
        if not x.startswith('_'): print(f"  - {x}")

except Exception as e:
    print(f"FAILURE: Layout load threw an exception: {e}")

print("\n--- DIAGNOSTIC END ---")