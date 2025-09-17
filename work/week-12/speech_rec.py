import re, speech_recognition as sr
from datetime import datetime
import time
import winsound   # ใช้ได้ใน Windows ถ้า Linux/Mac เปลี่ยนเป็น os.system("play ...")

r = sr.Recognizer()
alarms = []   # เก็บเวลาปลุกที่ตั้งไว้

with sr.Microphone() as mic:
    r.adjust_for_ambient_noise(mic, duration=0.6)
    print("พูดคำสั่ง เช่น 'สั่งงาน ปลุก 07:30' หรือ 'Alarm 07:30'")

    while True:
        # --- เช็คเวลาปลุก ---
        now = datetime.now().strftime("%H:%M")
        if now in alarms:
            print(f"ALARM {now} 🔔")
            winsound.Beep(1000, 800)   # beep 800 ms
            alarms.remove(now)         # ลบออก (ไม่ให้ดังซ้ำ)

        try:
            audio = r.listen(mic, timeout=6, phrase_time_limit=10)
            text = r.recognize_google(audio, language="th-TH")
            print("ได้ยิน :", text)

            # ----- ภาษาไทย -----
            if text.startswith("สั่งงาน"):
                cmd = text.replace("สั่งงาน", "", 1).strip()

                # จับเวลา เช่น "ปลุก 07:30"
                m = re.search(r"ปลุก\s*(\d{1,2}:\d{2})", cmd)
                if m:
                    alarm_time = m.group(1)
                    alarms.append(alarm_time)
                    print("⏰ ตั้งปลุกไว้ที่:", alarm_time)

            # ----- ภาษาอังกฤษ -----
            elif text.lower().startswith("alarm"):
                m = re.search(r"(\d{1,2}:\d{2})", text)
                if m:
                    alarm_time = m.group(1)
                    alarms.append(alarm_time)
                    print("⏰ Alarm set at:", alarm_time)

        except sr.WaitTimeoutError:
            print("ไม่มีเสียงพูดมาได้เลยครับ")

        except sr.UnknownValueError:
            print("พูดมาได้เลยครับ......")

        time.sleep(1)