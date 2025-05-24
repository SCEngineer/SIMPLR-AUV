# failsafe.py

class Failsafe:
    def check(self, state):
        # Simple condition: if too deep, trigger failsafe
        return abs(state.get("z", 0)) > 10
