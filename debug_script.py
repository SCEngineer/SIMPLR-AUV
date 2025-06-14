#!/usr/bin/env python3
"""
Debug script to identify issues with SIMPLR-AUV startup
"""

import sys
import os
import traceback

print("=== SIMPLR-AUV Debug Script ===")
print(f"Python version: {sys.version}")
print(f"Working directory: {os.getcwd()}")
print(f"Python path: {sys.path[0]}")

# Test 1: Check file existence
required_files = [
    'main_integration.py',
    'hardware_abstraction.py', 
    'navigation_system.py',
    'executive_system.py',
    'control_system.py',
    'guidance_system.py',
    'ballast_tank.py',
    'actuators.py',
    'failsafe_system.py',
    'core_types.py'
]

print("\n=== File Check ===")
missing_files = []
for file in required_files:
    if os.path.exists(file):
        print(f"✓ {file}")
    else:
        print(f"✗ {file} - MISSING")
        missing_files.append(file)

if missing_files:
    print(f"\nMISSING FILES: {missing_files}")
    print("Please ensure all required files are in the current directory.")
    sys.exit(1)

# Test 2: Try imports one by one
print("\n=== Import Test ===")
try:
    print("Importing core_types...")
    import core_types
    print("✓ core_types imported")
except Exception as e:
    print(f"✗ core_types failed: {e}")
    traceback.print_exc()
    sys.exit(1)

try:
    print("Importing hardware_abstraction...")
    import hardware_abstraction
    print("✓ hardware_abstraction imported")
except Exception as e:
    print(f"✗ hardware_abstraction failed: {e}")
    traceback.print_exc()
    sys.exit(1)

try:
    print("Importing navigation_system...")
    import navigation_system
    print("✓ navigation_system imported")
except Exception as e:
    print(f"✗ navigation_system failed: {e}")
    traceback.print_exc()
    sys.exit(1)

try:
    print("Importing main_integration...")
    import main_integration
    print("✓ main_integration imported")
except Exception as e:
    print(f"✗ main_integration failed: {e}")
    traceback.print_exc()
    sys.exit(1)

# Test 3: Check websockets availability
print("\n=== WebSocket Check ===")
try:
    import websockets
    print("✓ websockets package available")
except ImportError:
    print("✗ websockets package not installed")
    print("  This is optional for GUI functionality")

# Test 4: Try module import function
print("\n=== Module Import Function Test ===")
try:
    result = main_integration.import_auv_modules()
    print(f"import_auv_modules() result: {result}")
    if result:
        print("✓ All AUV modules imported successfully")
    else:
        print("✗ AUV module import failed")
except Exception as e:
    print(f"✗ import_auv_modules() failed: {e}")
    traceback.print_exc()

# Test 5: Try creating AUV system
print("\n=== AUV System Creation Test ===")
try:
    system = main_integration.SIMPLRAUVSystem(use_simulation=True, enable_gui=False)
    print("✓ SIMPLRAUVSystem created successfully")
except Exception as e:
    print(f"✗ SIMPLRAUVSystem creation failed: {e}")
    traceback.print_exc()

print("\n=== Debug Complete ===")
print("If all tests pass, try running the simulation again.")
print("If any test fails, that's likely the source of the issue.")
