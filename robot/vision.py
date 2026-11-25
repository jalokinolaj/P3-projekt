import time
import cv2
import threading
from jetbot import Camera
from pupil_apriltags import Detector


class VisionSystem:
    def __init__(self):
        self.detector = Detector(families='tag36h11')
        self.running = False
        self.thread = None

        self._lock = threading.Lock()
        self._latest_tag = None

    def start(self):
        if not self.running:
            self.camera = Camera.instance(width = 640, height = 480)
            self.running = True
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()

    def stop(self):
        if self.running:
            self.running = False
            if self.thread is not None:
                self.thread.join()
            if self.camera is not None:
                self.camera.stop()

    def get_latest_tag(self):
        with self._lock:
            return self._latest_tag
        
    def reset_latest_tag(self):
        with self._lock:
            self._latest_tag = None
    def run(self):
        while self.running:
            frame = self.camera.value

            if frame is None:
                time.sleep(0.05)   # Add a small delay to reduce CPU usage
                continue
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            results = self.detector.detect(gray_frame)

            with self._lock:
                if results:
                    self._latest_tag = results[0].tag_id
                else:
                    self._latest_tag = None

                time.sleep(0.05)  # Add a small delay to reduce CPU usage



