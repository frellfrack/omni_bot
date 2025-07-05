import machine  
import time
import micropython

micropython.alloc_emergency_exception_buf(100)

class SimpleEncoder:
    def __init__(self, pin_a, pin_b, edges='both', reverse=False):
        self.pin_a = machine.Pin(pin_a, machine.Pin.IN, machine.Pin.PULL_UP)
        self.pin_b = machine.Pin(pin_b, machine.Pin.IN, machine.Pin.PULL_UP)
        self.position = 0
        self._dir = -1 if reverse else 1
        self._last_time = time.ticks_us()
        self._last_pos = 0
        self._velocity = 0
        self._vel_buffer = [0] * 5

        if edges == 'rising':
            trigger = machine.Pin.IRQ_RISING
        elif edges == 'falling':
            trigger = machine.Pin.IRQ_FALLING
        else:
            trigger = machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING

        self.pin_a.irq(trigger=trigger, handler=self._update)

    def _update(self, pin):
        a = self.pin_a.value()
        b = self.pin_b.value()
        if a == b:
            self.position += self._dir
        else:
            self.position -= self._dir

    def read(self):
        irq_state = machine.disable_irq()
        pos = self.position
        machine.enable_irq(irq_state)
        return pos

    def reset(self):
        irq_state = machine.disable_irq()
        self.position = 0
        machine.enable_irq(irq_state)
        self._last_time = time.ticks_us()
        self._last_pos = 0
        self._velocity = 0
        self._vel_buffer = [0] * 5

    def velocity(self):
        now = time.ticks_us()
        dt = time.ticks_diff(now, self._last_time) / 1_000_000
        if dt == 0:
            return self._velocity

        pos = self.read()
        dp = pos - self._last_pos
        self._last_pos = pos
        self._last_time = now

        new_velocity = dp / dt
        self._vel_buffer.pop(0)
        self._vel_buffer.append(new_velocity)
        self._velocity = sum(self._vel_buffer) / len(self._vel_buffer)
        return self._velocity

# Test block
if __name__ == "__main__":
    enc = SimpleEncoder(20, 21)
    while True:
        print("Position:", enc.read(), "Velocity:", enc.velocity(), "c/s")
        time.sleep(0.1)
