from gpiozero import DistanceSensor, PWMSoftwareFallback, DistanceSensorNoEcho
import warnings


class Ultrasonic:
    def __init__(self, trigger_pin: int = 27, echo_pin: int = 22, max_distance: float = 3.0):
        warnings.filterwarnings("ignore", category=DistanceSensorNoEcho)
        warnings.filterwarnings("ignore", category=PWMSoftwareFallback)
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.max_distance = max_distance
        self.sensor = DistanceSensor(echo=self.echo_pin, trigger=self.trigger_pin, max_distance=self.max_distance)

    def get_distance_cm(self):
        try:
            return round(float(self.sensor.distance * 100.0), 1)
        except RuntimeWarning:
            return None

    def close(self):
        self.sensor.close()

