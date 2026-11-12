import time
import cv2
from jetbot import Camera
from pupil_apriltags import Detector

# 1. Start kameraet i en god oploesning til AprilTags (640x480)
camera = Camera.instance(width=224, height=224)

# 2. Konfigurer AprilTag-detektoren til standard-familien 'tag36h11'
detector = Detector(families='tag36h11')

print("Kamera koerer. Scanner efter AprilTags... Tryk paa Ctrl+C i konsollen for at stoppe.")

try:
    while True:
        # Hent det nyeste billede direkte fra JetBot-kameraet
        frame = camera.value
        
        # Konverter til graatoner, som detektoren kraever
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Scan billedet for AprilTags
        results = detector.detect(gray)

        # Hvis der findes et eller flere tags, print dem til konsollen
        for r in results:
            cX = int(r.center[0])
            cY = int(r.center[1])
            print(f"Fundet AprilTag -> ID: {r.tag_id}")

        # Kort pause saa CPU'en ikke arbejder unoedigt haardt
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nProgrammet blev stoppet.")

finally:
    # VIGTIGT: Luk kameraet ordentligt ned, saa det ikke laaser naeste gang du koerer scriptet
    camera.stop()
    print("Kamera lukket ned.")
