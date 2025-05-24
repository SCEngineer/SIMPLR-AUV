class Guidance:
    def __init__(self, navigation, control):
        self.navigation = navigation  # correct: Navigation instance
        self.control = control        # correct: Control instance
        print("[GUIDANCE] Initialized")

    def navigate_to_waypoint(self, waypoint, speed, depth):
        print(f"[GUIDANCE] Planning route to waypoint {waypoint} at depth {depth} and speed {speed}")
        
        self.control.set_speed(speed)
        self.control.dive_to_depth(depth)

        state = self.navigation.get_state()  # <== this now works as intended

        while not self._reached_waypoint(state, waypoint, depth):
            heading = self._calculate_heading(state, waypoint)
            self.control.turn_to_heading(heading)
            self.control.hold_course()
            state = self.navigation.get_state()  # Update state each loop

        print(f"[GUIDANCE] Waypoint {waypoint} reached. Holding position.")
        self.control.hold_course()

    def _reached_waypoint(self, state, waypoint, target_depth, position_threshold=0.5, depth_threshold=0.2):
        distance = ((state["lat"] - waypoint[0])**2 + (state["lon"] - waypoint[1])**2)**0.5
        depth_error = abs(state["depth"] - target_depth)
        return distance < position_threshold and depth_error < depth_threshold

    def _calculate_heading(self, state, waypoint):
        import math
        dy = waypoint[1] - state["lon"]
        dx = waypoint[0] - state["lat"]
        angle_rad = math.atan2(dy, dx)
        angle_deg = math.degrees(angle_rad)
        return (angle_deg + 360) % 360
