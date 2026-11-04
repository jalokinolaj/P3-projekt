import cv2
from pupil_apriltags import Detector

# 1. Konfigurer AprilTag-detektoren
# Vi bruger standard-familien 'tag36h11' (den mest almindelige)
detector = Detector(families='tag36h11')

# 2. Start webkameraet (0 er normalt det indbyggede kamera)
cap = cv2.VideoCapture(0)

print("Programmet kører. Tryk på 'q' for at afslutte...")

while True:
    # Læs et billede fra kameraet
    ret, frame = cap.read()
    if not ret:
        print("Kunne ikke hente billede fra webcam.")
        break

    # AprilTag-detektoren kræver et gråtonebillede
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Find tags i billedet
    results = detector.detect(gray)

    # Loop igennem alle de tags, der blev fundet
    for r in results:
        # Hent hjørnerne på tagget (skal bruges til at tegne boksen)
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))

        # Tegn streger mellem hjørnerne for at lave en boks omkring tagget
        cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
        cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
        cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
        cv2.line(frame, ptD, ptA, (0, 255, 0), 2)

        # Hent centrum af tagget for at placere teksten
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        
        # Skriv taggets ID-nummer på skærmen
        tag_id = f"ID: {r.tag_id}"
        cv2.putText(frame, tag_id, (cX - 10, cY - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Vis det endelige billede i et vindue
    cv2.imshow("AprilTag Detektor", frame)

    # Stop programmet, hvis du trykker på tasten 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Ryd op og luk vinduerne ordentligt
cap.release()
cv2.destroyAllWindows()
