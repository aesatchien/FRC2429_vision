# this is a standalone that should be able to inherit any grip pipeline and decorate it
# intended to be used with the wpi multicamera server and a grip pipeline
# note that GRIP is old so sometimes you have to correct the cv2 return values on some of the auto-generated code
# 2/8/2020 CJH FRC team 2429
from grip import GripPipeline
import cv2
import time

class SpartanOverlay(GripPipeline):

    def __init__(self):
        super().__init__()
        # can override the GRIP parameters here if we need to
        self.targets = 0
        self.image = None

    def sort_contours(self):
        """Sort our filtered contours based on size"""
        for ix, contour in enumerate(self.filter_contours_output):
            pass
           # x, y, w, h = cv2.boundingRect(contour)
           # print(f"Contour {ix} has area {cv2.contourArea(contour)} with width {w} and height {h} at location ({x},{y})")

        self.filter_contours_output = sorted(self.filter_contours_output, key=cv2.contourArea, reverse=True)[:5]

        for ix, contour in enumerate(self.filter_contours_output):
            x, y, w, h = cv2.boundingRect(contour)
            #print(f"Contour {ix} has area {cv2.contourArea(contour)} with width {w} and height {h} at location ({x},{y})")

        return (self.filter_contours_output)

    def paint_contours(self):
        """Draw a box around all of our contours with the main one emphasized"""
        for ix, contour in enumerate(self.filter_contours_output):
            if ix == 0:
                color = (255, 0, 0)
                thickness = 2
            else:
                color = (0, 255, 0)
                thickness = 1
            rect = cv2.boundingRect(contour)
            #print(rect)
            self.image = cv2.rectangle(self.image, (int(rect[0]), int(rect[1])), (int(rect[0] + rect[2]), int(rect[1] + rect[3])), color, thickness)
        pass

    def process(self, source0):
        """Run the parent pipeline and then do custom overlays and reporting"""
        super(self.__class__, self).process(source0)
        # we just processed the incoming image with the parent GRIP pipeline and have our filtered contours.  now sort
        self.image = source0
        self.sort_contours()
        self.paint_contours()


if __name__ == "__main__":

    start_time = time.time()
    count = 0
    run_time = 10
    while time.time() - start_time < run_time:
        count += 1
        cam = cv2.VideoCapture(0)
        s, im = cam.read()  # captures image
        pipeline = SpartanOverlay()
        pipeline.process(im)
        cv2.imshow("Test Picture", pipeline.image)  # displays captured image
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    #cv2.waitKey(0)
    cam.release()
    print(f"Processed {count} images in {run_time} second")
