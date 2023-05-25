import cv2
import time

class DeviceCamera:
    """docstring for DeviceCamera."""
    def __init__(self, device_id = None, winname = 'Camera output'):
        print("Loading camera")

        self.device_id = device_id
        self.device_ids = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        self.winname = winname
        self.capture = None

        if self.device_id is None:
            self.device_id = self.search_available_device_id()
        

    def validate_device_id(self, device_id):
        try:
            cap = cv2.VideoCapture(device_id)
            is_opened = cap.isOpened()
            cap.release()
            return is_opened
        except Exception as e:
            return False
    
    def search_available_device_id(self):
        for device_id in self.device_ids:
            if self.validate_device_id(device_id):
                return device_id
        return None
    
    def available_device_id(self):
        available_device = [device_id for device_id in self.device_ids if self.validate_device_id(device_id)]
        return available_device

def main():
    camera = DeviceCamera()
    print(camera.available_device_id())

if __name__ == "__main__":
    main()


        