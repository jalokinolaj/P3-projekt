import time

import cv2
from jetbot import Camera
from pupil_apriltags import Detector


def apriltag_detection():
    """Capture AprilTags until interrupted and return newly detected results."""
    camera = Camera.instance(width=224, height=224)
    detector = Detector(families="tag36h11")
    detections = []
    last_detection = None

    try:
        while True:
            frame = camera.value
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            results = detector.detect(gray)
            current_detection = tuple(result.tag_id for result in results)

            if current_detection and current_detection != last_detection:
                detections.append(results)
                last_detection = current_detection

            time.sleep(0.1)
    except KeyboardInterrupt:
        return detections
    finally:
        camera.stop()
        print("Camera stopped.")
