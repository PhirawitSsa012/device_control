import time
import serial

class SerialManager:
    def __init__(self, port='COM3', baud=9600):
        self.port = port
        self.baud = baud
        self.ser = None
        self._connect()

    def _connect(self):
        try:
            self.ser= serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)
            print(f"[Serial] Could not open {self.port}@{self.baud}")
        except Exception as e:
            print(f"[Serial] Could not open {self.port} : {e}")
            self.ser = None

    def _is_open(self):
        return self.ser is not None and self.ser.is_open

    def send_angle(self, angle:int):
        angle = max(0, min(100, int(angle)))
        msg = f"{angle}\n".encode("utf-8")
        try:
            if not self._is_open():
                self._connect()
            if self._is_open():
                self.ser.write(msg)
        except Exception as e:
            print(f"[Serial] write failed : {e}")

    def close(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print("[Serial] closed")
        except Exception as e :
            pass