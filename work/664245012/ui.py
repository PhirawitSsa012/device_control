import sys
import serial
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QMessageBox, QLabel

# เชื่อมต่อ Serial
try:
    arduino = serial.Serial('COM3', 9600, timeout=1)
except Exception as a:
    arduino = None
    print("เชื่อมต่อไม่ได้ :", a)

class ArduinoControl(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Arduino Controller")   # ชื่อหน้าต่าง
        self.setGeometry(200, 200, 300, 300)       # ขนาดหน้าต่าง

        layout = QVBoxLayout()

        # Label แสดงค่าที่ส่ง
        self.label = QLabel("สถานะ: ยังไม่ได้ส่งคำสั่ง")
        layout.addWidget(self.label)

        # สร้างปุ่มหลายค่า
        angles = ["15", "30", "60", "90", "115", "125", "145", "0"]
        for angle in angles:
            btn = QPushButton(angle if angle != "0" else "หยุด (0)")
            btn.clicked.connect(lambda _, a=angle: self.send_command(a))
            layout.addWidget(btn)

        self.setLayout(layout)

    def send_command(self, command):
        if arduino:
            arduino.write((command + '\n').encode())
            self.label.setText(f"ส่งค่า: {command}° ไปยัง Arduino")
        else:
            QMessageBox.critical(self, "Error", "ไม่พบการเชื่อมต่อ")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    windows = ArduinoControl()
    windows.show()
    sys.exit(app.exec_())
