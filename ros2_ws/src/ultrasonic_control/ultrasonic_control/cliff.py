"""Cliff (edge) sensor logic, kept free of ROS/GPIO imports so it can be unit tested."""


class CliffDebounce:
    """Debounce one downward-facing edge sensor, erring on the side of "drop".

    A drop is reported on the first reading that sees one; the floor is reported again
    only after ``clear_count`` floor readings in a row. It starts as "drop" so the robot
    does not move until the sensor has proven it can see the floor.
    """

    def __init__(self, clear_count=3):
        self.clear_count = max(1, int(clear_count))
        self._floor_run = 0
        self.drop = True

    def update(self, drop_now):
        if drop_now:
            self._floor_run = 0
            self.drop = True
        else:
            self._floor_run += 1
            if self._floor_run >= self.clear_count:
                self.drop = False
        return self.drop


def summarise(states):
    """Turn {sensor_name: drop} into the /cliff message body.

    Sensor names start with "front" or "rear"; "front"/"rear" are True when any sensor
    on that end sees a drop.
    """
    return {
        'front': any(v for k, v in states.items() if k.startswith('front')),
        'rear': any(v for k, v in states.items() if k.startswith('rear')),
        'sensors': dict(states),
    }
