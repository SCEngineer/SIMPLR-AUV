class Control:
    def __init__(self, navigation):
        self.navigation = navigation
        print("[CONTROL] Initialized")

    def set_ballast(self, mode):
        if mode == "flood":
            print("[CONTROL] Commanding ballast to flood")
        elif mode == "empty":
            print("[CONTROL] Commanding ballast to empty")
        else:
            print(f"[CONTROL] Unknown ballast mode: {mode}")

    def submerge_trim(self):
        print("[CONTROL] Submerging to operational trim")

    def surface(self):
        print("[CONTROL] Surfacing")

    def dive_to_depth(self, depth):
        print(f"[CONTROL] Adjusting diving fins to reach {depth} meters")

    def climb_to_depth(self, depth):
        print(f"[CONTROL] Adjusting diving fins to climb to {depth} meters")

    def turn_to_heading(self, heading):
        print(f"[CONTROL] Turning to heading {heading} degrees")

    def set_speed(self, speed):
        print(f"[CONTROL] Setting speed to {speed} m/s")

    def hold_course(self):
        print("[CONTROL] Holding current heading, depth, and speed")
