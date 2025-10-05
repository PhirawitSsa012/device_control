import sys
from PyQt5 import QtWidgets, uic, QtGui, QtCore


from serial_manager import SerialManager
from workers import FaceScanWorker, HandAxisWorker


class MainWindow(QtWidgets.QMainWindow):
    def __init(self):
        super().__init__()
        uic.loadUi("main_window", self)



if __name__=="__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())