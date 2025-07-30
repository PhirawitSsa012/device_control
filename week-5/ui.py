import sys
import serial

from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QMessageBox

try :
    arduino = serial.Serial('COM3', 9600, timeout=1)
except Exception as a:
    arduino = None
    print("เชื่อมต่อไม่ได้ :", a)


class ArduinoControl(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Arduino Controller ") #ชื่อ
        self.setGeometry(200, 200, 300, 150) #ขนาด

        layout = QVBoxLayout()
        self.btn_on = QPushButton("LED ON") #Buttonเปิด
        self.btn_on.clicked.connect(lambda:self.send_command("ON"))
        layout.addWidget(self.btn_on)

        self.btn_off = QPushButton("LED OFF") #Buttonปิด
        self.btn_off.clicked.connect(lambda:self.send_command("OFF"))
        layout.addWidget(self.btn_off)

        self.btn_blink = QPushButton("LED BLINK") #Buttonกระพริบ
        self.btn_blink.clicked.connect(lambda:self.send_command("BLINK"))
        layout.addWidget(self.btn_blink)

        self.setLayout(layout)

    def send_command(self, command):
        if(arduino):
            arduino.write((command + '\n').encode())
        else:
            QMessageBox.critical(self, "Error", "ไม่พบการเชื่อมต่อ")

#การสร้าง GUI output
if __name__ == "__main__":
    app = QApplication(sys.argv)
    windows = ArduinoControl()
    windows.show()
    sys.exit(app.exec_())