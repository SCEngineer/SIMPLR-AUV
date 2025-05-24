import json
import os
from modules.navigation import Navigation
from modules.control import Control
from modules.guidance import Guidance

class Executive:
    def __init__(self):
        self.navigation = Navigation()
        self.control = Control(self.navigation)
        self.guidance = Guidance(self.navigation, self.control)

    def run(self):
        print("[EXECUTIVE] Starting mission execution")
        mission = self.load_mission_file()

        print(f"[EXECUTIVE] Mission loaded: {len(mission)} steps.")
        for i, step in enumerate(mission):
            print(f"[EXECUTIVE] Step {i + 1}: {step}")
            action = step.get("action")

            if action == "submerge":
                print("[EXECUTIVE] Submerging to operational trim")
                self.control.set_ballast("flood")
                self.control.submerge_trim()

            elif action == "dive":
                depth = step.get("depth", 1.0)
                print(f"[EXECUTIVE] Diving to {depth} meters")
                self.control.dive_to_depth(depth)

            elif action == "navigate":
                waypoint = step.get("waypoint")
                speed = step.get("speed", 1.0)
                depth = step.get("depth", 1.0)
                if waypoint:
                    print(f"[EXECUTIVE] Navigating to waypoint {waypoint}")
                    self.guidance.navigate_to_waypoint(waypoint, speed, depth)

            elif action == "surface":
                print("[EXECUTIVE] Surfacing")
                self.control.set_ballast("empty")
                self.control.surface()

            else:
                print(f"[EXECUTIVE] Unknown action: {action}")

    def load_mission_file(self):
        path = os.path.join(os.path.dirname(__file__), "../mission.json")
        try:
            with open(path, 'r') as file:
                return json.load(file)
        except Exception as e:
            print(f"[EXECUTIVE] Failed to load mission file: {e}")
            return []
