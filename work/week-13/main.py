# -*- coding: utf-8 -*-
import sys, time, traceback
import cv2
import serial
import serial.tools.list_ports
import numpy as np
import math  # เพิ่มบรรทัดนี้
import mediapipe as mp  # เพิ่มบรรทัดนี้
import face_recognition  # ต้องติดตั้ง: pip install face_recognition

from PyQt5 import QtCore, QtGui, QtWidgets

# ========================= Serial Manager (Updated) =========================
class SerialManager:
    def __init__(self, port='COM3', baud=9600):
        self.port = port
        self.baud = baud
        self.ser = None
        self._connect()

    def _connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)
            print(f"[Serial] Connected to {self.port} @ {self.baud}")
        except Exception as e:
            print(f"[Serial] Could not open {self.port}: {e}")
            self.ser = None

    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def send_angle(self, angle: int):
        """ยังคงไว้เพื่อความเข้ากันได้กับ HandAxisWorker"""
        angle = max(0, min(180, int(angle)))
        msg = f"{angle}\n".encode('utf-8')
        try:
            if self.is_open():
                self.ser.write(msg)
        except Exception as e:
            print(f"[Serial] Write failed: {e}")

    def send_command(self, cmd: str):
        """ส่งคำสั่งตัวอักษร เช่น 'O' หรือ 'C'"""
        try:
            if self.is_open():
                self.ser.write(cmd.encode('utf-8'))
                print(f"[Serial] Sent command: {cmd}")
        except Exception as e:
            print(f"[Serial] Command failed: {e}")

    def close(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print("[Serial] Closed")
        except:
            pass


# ========================= Utilities =========================
def cvimg_to_qimage(frame_bgr):
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    h, w, ch = frame_rgb.shape
    bytes_per_line = ch * w
    return QtGui.QImage(frame_rgb.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888).copy()

def open_camera_reliably(preferred_index=0, warmup_frames=5, width=640, height=480):
    backends = []
    if hasattr(cv2, "CAP_DSHOW"):
        backends.append(cv2.CAP_DSHOW)
    if hasattr(cv2, "CAP_MSMF"):
        backends.append(cv2.CAP_MSMF)
    backends.append(cv2.CAP_ANY)

    tried = []
    indices = [preferred_index] + [i for i in range(4) if i != preferred_index]

    for idx in indices:
        for be in backends:
            cap = cv2.VideoCapture(idx, be)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

                ok_warm = True
                for _ in range(warmup_frames):
                    ok, _frame = cap.read()
                    if not ok:
                        ok_warm = False
                        break
                    time.sleep(0.02)

                if ok_warm:
                    print(f"[Camera] Opened index={idx} backend={be}")
                    return cap
                else:
                    cap.release()
                    tried.append((idx, be, "warmup-failed"))
            else:
                tried.append((idx, be, "open-failed"))

    print("[Camera] Could not open any camera. Tried:", tried)
    return None


# ========================= Face Recognition Worker (New) =========================
class FaceRecognitionWorker(QtCore.QThread):
    frame_signal = QtCore.pyqtSignal(QtGui.QImage)
    info_signal  = QtCore.pyqtSignal(str)

    def __init__(self, ser: SerialManager, parent=None, cam_index=0):
        super().__init__(parent)
        self.ser = ser
        self._running = False
        self.cam_index = cam_index

        # โหลดรูปเจ้าของหลายคน
        self.known_face_encodings = []
        self.known_face_names = []

        # แก้ไข path ตามที่คุณมีไฟล์จริง
        people = [
            ("week-13/user.jpg", "Phirawit Srisaart"),
            # เพิ่มได้มากกว่า 1 คน เช่น:
            # ("user2.jpg", "Somchai"),
        ]

        try:
            for img_path, name in people:
                image = face_recognition.load_image_file(img_path)
                encodings = face_recognition.face_encodings(image)
                if encodings:
                    self.known_face_encodings.append(encodings[0])
                    self.known_face_names.append(name)
                    print(f"[Face] Loaded: {name} from {img_path}")
                else:
                    print(f"[Face] ⚠️ No face found in {img_path}")
        except Exception as e:
            print(f"[Face] Error loading faces: {e}")
            self.info_signal.emit(f"⚠️ โหลดรูปไม่ได้: {e}")

    def run(self):
        self._running = True
        cap = open_camera_reliably(preferred_index=self.cam_index)
        if cap is None or not cap.isOpened():
            self.info_signal.emit("Camera not available. (โปรดเช็คสิทธิ์/ปิดโปรแกรมที่ใช้กล้องอยู่)")
            return

        self.info_signal.emit("Face Recognition running…")
        try:
            while self._running:
                ok, frame = cap.read()
                if not ok:
                    self.info_signal.emit("Frame grab failed. (กล้องหลุด)")
                    break

                # แปลงเป็น RGB (face_recognition ต้องการ RGB)
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                rgb_frame = np.array(rgb_frame, dtype=np.uint8)

                # หาตำแหน่งและ encoding ใบหน้า
                face_locations = face_recognition.face_locations(rgb_frame)
                face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)

                if len(face_encodings) == 0:
                    # ไม่เจอหน้า → ส่ง 'C' (ล็อก)
                    self.ser.send_command('C')
                    self.info_signal.emit("No face detected → Locking (C)")
                else:
                    for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
                        matches = face_recognition.compare_faces(
                            self.known_face_encodings, face_encoding, tolerance=0.35
                        )
                        face_distances = face_recognition.face_distance(
                            self.known_face_encodings, face_encoding
                        )

                        best_match_index = face_distances.argmin() if len(face_distances) > 0 else -1

                        if best_match_index >= 0 and matches[best_match_index]:
                            name = self.known_face_names[best_match_index]
                            color = (0, 255, 0)  # เขียว
                            self.ser.send_command('O')  # ปลดล็อก
                            self.info_signal.emit(f"✅ Recognized: {name} → Unlock (O)")
                        else:
                            name = "Unknown"
                            color = (0, 0, 255)  # แดง
                            self.ser.send_command('C')  # ล็อก
                            self.info_signal.emit("❌ Unknown face → Locking (C)")

                        # วาดกรอบและชื่อ
                        cv2.rectangle(frame, (left, top), (right, bottom), color, 2)
                        cv2.putText(frame, name, (left, top - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

                # ส่งเฟรมไปยัง GUI
                qimg = cvimg_to_qimage(frame)
                self.frame_signal.emit(qimg)
                self.msleep(50)  # ~20 FPS (face_recognition ช้า)

        except Exception as e:
            self.info_signal.emit(f"Face recognition error: {e}")
            traceback.print_exc()
        finally:
            cap.release()
            self.info_signal.emit("Face Recognition stopped.")

    def stop(self):
        self._running = False
        self.wait(1000)


# ========================= Hand Axis Worker (คงเดิม) =========================
class HandAxisWorker(QtCore.QThread):
    frame_signal = QtCore.pyqtSignal(QtGui.QImage)
    info_signal  = QtCore.pyqtSignal(str)

    def __init__(self, ser: SerialManager, parent=None, cam_index=0):
        super().__init__(parent)
        self.ser = ser
        self._running = False
        self.cam_index = cam_index
        self.mp_hands = mp.solutions.hands
        self.mp_draw  = mp.solutions.drawing_utils

        self.hands = self.mp_hands.Hands(
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7,
            max_num_hands=1
        )

    def calculate_angle(self, wrist, thumb, index):
        v1 = [thumb[0] - wrist[0], thumb[1] - wrist[1]]
        v2 = [index[0] - wrist[0], index[1] - wrist[1]]
        dot = v1[0]*v2[0] + v1[1]*v2[1]
        mag1 = math.sqrt(v1[0]**2 + v1[1]**2)
        mag2 = math.sqrt(v2[0]**2 + v2[1]**2)
        if mag1 * mag2 != 0:
            cos_val = dot / (mag1 * mag2)
            cos_val = max(-1.0, min(1.0, cos_val))
            return math.degrees(math.acos(cos_val))
        return 0.0

    def run(self):
        self._running = True
        cap = open_camera_reliably(preferred_index=self.cam_index)
        if cap is None or not cap.isOpened():
            self.info_signal.emit("Camera not available.")
            return

        self.info_signal.emit("Hand Axis running…")
        try:
            while self._running:
                ok, frame = cap.read()
                if not ok: break
                frame = cv2.flip(frame, 1)
                h, w, _ = frame.shape
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = self.hands.process(rgb)

                display_angle = 0
                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                        wrist_lm = hand_landmarks.landmark[2]
                        thumb_tip = hand_landmarks.landmark[4]
                        index_tip = hand_landmarks.landmark[8]
                        x0, y0 = int(wrist_lm.x * w), int(wrist_lm.y * h)
                        x1, y1 = int(thumb_tip.x * w), int(thumb_tip.y * h)
                        x2, y2 = int(index_tip.x * w), int(index_tip.y * h)
                        angle = self.calculate_angle((x0, y0), (x1, y1), (x2, y2))
                        angle = max(0.0, min(90.0, angle))
                        display_angle = angle
                        self.ser.send_angle(int(angle))
                        cv2.circle(frame, (x1, y1), 8, (255, 0, 0), -1)
                        cv2.circle(frame, (x2, y2), 8, (0, 0, 255), -1)
                        break

                cv2.putText(frame, f"Angle: {int(display_angle)} deg", (50, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
                qimg = cvimg_to_qimage(frame)
                self.frame_signal.emit(qimg)
                self.msleep(30)
        except Exception as e:
            self.info_signal.emit(f"Hand worker error: {e}")
            traceback.print_exc()
        finally:
            cap.release()
            self.info_signal.emit("Hand Axis stopped.")

    def stop(self):
        self._running = False
        self.wait(1000)


# ========================= Main Window =========================
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Face Recognition + Hand Control → COM3")
        self.resize(980, 700)

        self.ser = SerialManager(port="COM3", baud=9600)

        # Widgets
        self.video_label = QtWidgets.QLabel()
        self.video_label.setAlignment(QtCore.Qt.AlignCenter)
        self.video_label.setMinimumSize(800, 500)
        self.video_label.setStyleSheet("background:#111; color:#ccc;")

        self.btn_face  = QtWidgets.QPushButton("Face Recognition (Start)")
        self.btn_hand  = QtWidgets.QPushButton("Hand Axis (Start)")
        self.status    = QtWidgets.QLabel("Ready.")
        self.status.setStyleSheet("color:#0a0;")

        # Layout
        btn_row = QtWidgets.QHBoxLayout()
        btn_row.addWidget(self.btn_face)
        btn_row.addWidget(self.btn_hand)
        wrapper = QtWidgets.QVBoxLayout()
        wrapper.addWidget(self.video_label, 1)
        wrapper.addLayout(btn_row)
        wrapper.addWidget(self.status)

        central = QtWidgets.QWidget()
        central.setLayout(wrapper)
        self.setCentralWidget(central)

        # Workers
        self.face_worker = None
        self.hand_worker = None

        # Signals
        self.btn_face.clicked.connect(self.toggle_face)
        self.btn_hand.clicked.connect(self.toggle_hand)

    def set_status(self, text, ok=True):
        self.status.setText(text)
        self.status.setStyleSheet(f"color:{'#0a0' if ok else '#f55'};")

    def show_frame(self, qimg: QtGui.QImage):
        pix = QtGui.QPixmap.fromImage(qimg).scaled(
            self.video_label.width(), self.video_label.height(),
            QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation
        )
        self.video_label.setPixmap(pix)

    def toggle_face(self):
        if self.hand_worker is not None:
            self.stop_hand()

        if self.face_worker is None:
            self.start_face()
        else:
            self.stop_face()

    def toggle_hand(self):
        if self.face_worker is not None:
            self.stop_face()

        if self.hand_worker is None:
            self.start_hand()
        else:
            self.stop_hand()

    def start_face(self):
        self.face_worker = FaceRecognitionWorker(self.ser, cam_index=0)
        self.face_worker.frame_signal.connect(self.show_frame)
        self.face_worker.info_signal.connect(lambda m: self.set_status(m, ok=True))
        self.face_worker.finished.connect(lambda: self._face_finished())
        self.face_worker.start()
        self.btn_face.setText("Face Recognition (Stop)")
        self.set_status("Face Recognition started.")

    def stop_face(self):
        if self.face_worker:
            self.face_worker.stop()
            self.face_worker = None
            self.btn_face.setText("Face Recognition (Start)")
            self.set_status("Face Recognition stopped.")

    def _face_finished(self):
        self.face_worker = None
        self.btn_face.setText("Face Recognition (Start)")

    def start_hand(self):
        self.hand_worker = HandAxisWorker(self.ser, cam_index=0)
        self.hand_worker.frame_signal.connect(self.show_frame)
        self.hand_worker.info_signal.connect(lambda m: self.set_status(m, ok=True))
        self.hand_worker.finished.connect(lambda: self._hand_finished())
        self.hand_worker.start()
        self.btn_hand.setText("Hand Axis (Stop)")
        self.set_status("Hand Axis started.")

    def stop_hand(self):
        if self.hand_worker:
            self.hand_worker.stop()
            self.hand_worker = None
            self.btn_hand.setText("Hand Axis (Start)")
            self.set_status("Hand Axis stopped.")

    def _hand_finished(self):
        self.hand_worker = None
        self.btn_hand.setText("Hand Axis (Start)")

    def closeEvent(self, event):
        try:
            if self.face_worker: self.face_worker.stop()
            if self.hand_worker: self.hand_worker.stop()
            self.ser.close()
        finally:
            super().closeEvent(event)


# ========================= Entry =========================
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())