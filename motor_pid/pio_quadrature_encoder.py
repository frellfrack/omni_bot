import time
import machine
import rp2
from machine import Pin

@rp2.asm_pio(
    in_shiftdir=rp2.PIO.SHIFT_LEFT,
    autopush=True,
    push_thresh=2
)
def quadrature_decoder():
    label("start")
    in_(pins, 2)
    push()
    jmp("start")

class PIOQuadratureEncoder:
    _transition_table = [
         0,  1, -1,  0,
        -1,  0,  0,  1,
         1,  0,  0, -1,
         0, -1,  1,  0
    ]

    def __init__(self, pin_a, pin_b, sm_id=0):
        self.pin_a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self.pin_b = Pin(pin_b, Pin.IN, Pin.PULL_UP)
        self.position = 0
        self.last_state = None

        # Ensure both pins are adjacent (required by PIO in_base)
        if pin_b != pin_a + 1:
            raise ValueError("pin_b must be pin_a + 1 for in_(pins, 2) to work.")

        self.sm = rp2.StateMachine(
            sm_id,
            quadrature_decoder,
            in_base=self.pin_a,
            freq=100000
        )
        self.sm.active(1)

    def update(self):
        while self.sm.rx_fifo():
            state = self.sm.get() & 0b11
            if self.last_state is not None:
                transition = (self.last_state << 2) | state
                delta = self._transition_table[transition]
                self.position += delta
            self.last_state = state

    def read(self):
        self.update()
        return self.position

    def reset(self):
        self.position = 0
        self.last_state = None

if __name__ == "__main__":
    try:
        encoder = PIOQuadratureEncoder(10, 11)
        print("Test Encoder")
        while True:
            print("Position:", encoder.read())
            time.sleep(0.2)
    except Exception as e:
        print("Exception:", e)

