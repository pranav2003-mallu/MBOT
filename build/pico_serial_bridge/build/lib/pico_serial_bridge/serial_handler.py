import serial
import time

class SerialHandler:
    def __init__(self, port="/dev/ttyACM0", baud=115200, timeout=0.1, reconnect_delay=2.0, logger=None):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.reconnect_delay = reconnect_delay
        self.ser = None
        self.logger = logger
        self.connect()

    def connect(self):
        """Try to open serial connection."""
        while True:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
                if self.logger:
                    self.logger.info(f"✅ Connected to Pico on {self.port} @ {self.baud}")
                time.sleep(1.0)  # Allow Pico to reset
                break
            except serial.SerialException as e:
                if self.logger:
                    self.logger.warn(f"⚠️ Pico not found on {self.port}. Retrying in {self.reconnect_delay}s...")
                time.sleep(self.reconnect_delay)

    def write(self, data):
        """Send data to Pico."""
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(data.encode())
            except serial.SerialException:
                self.reconnect()

    def read_line(self):
        """Read a line from Pico (non-blocking)."""
        if self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
                return line
            except serial.SerialException:
                self.reconnect()
        return ""

    def reconnect(self):
        """Try to reconnect to the serial port."""
        if self.logger:
            self.logger.warn("🔌 Lost connection to Pico. Reconnecting...")
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.connect()

    def flush(self):
        """Flush serial input/output."""
        if self.ser and self.ser.is_open:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
