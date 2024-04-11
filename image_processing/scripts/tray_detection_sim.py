#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge 
import cv2 
import numpy as np


class Camera(Node):
    def __init__(self):
        super().__init__("color_image_calibration")
        self.subscription_colour_image = self.create_subscription(Image, '/camera/image_raw', self.callback_colour, 10)
        self.bridge = CvBridge()
        self.bounding_box = []
        self.distance = 0
        
    def callback_colour(self, data):
        image = self.bridge.imgmsg_to_cv2(data)

        image = cv2.GaussianBlur(image, (3, 3), 0)
        imgHSV= cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

        lower_HSV = np.array([93,0,119])
        upper_HSV = np.array([255,255,255])


        mask = cv2.inRange(imgHSV,lower_HSV,upper_HSV)
        imgResult_HSV = cv2.bitwise_and(image,image,mask=mask)

        coins = image.copy()

        blurred = cv2.GaussianBlur(imgResult_HSV, (5, 5), 0)
        canny = cv2.Canny(blurred, 50, 100)
        cnts,_ = cv2.findContours(canny.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in cnts:
            x, y, w, h = cv2.boundingRect(cnt)
            self.bounding_box = [x, y, w, h]
            cv2.rectangle(coins,(x, y), (x+w, y+h), (0, 255, 0), 1)
            # Fining center of the bounding box
            cv2.circle(coins, (int(x+w/2), int(y+h/2)), 1, (0,0,255), 1)
            cv2.putText(coins, f"({640-round(x+w/2, -1)}, {360-round(y+h/2, -1)})", (x+h, y+h),cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255,255,255), 1)

        try:
            cv2.putText(coins, f"Distance: {self.distance}", (int(x+w/2), 
                                                                        int(y+h/2)),
                                                                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
            
        except:
            print("no distance value")

        # print(coins.shape)

        coins = cv2.resize(coins, (800, 600)) 
        cv2.imshow("tennis ball tracker", coins)
        cv2.imshow("edge detection", canny)


        key = cv2.waitKey(1)
        if key == 27:
            cv2.destroyAllWindows()
            rclpy.shutdown()   
    


def main(args=None):
    rclpy.init(args=args)
    node = Camera()
    rclpy.spin(node)

    node.shutdown()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    