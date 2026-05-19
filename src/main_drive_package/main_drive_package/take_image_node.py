import json
import os

import cv2
import joblib
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy
from skimage.feature import hog
from std_msgs.msg import String

# Classifying Parameters
IMG_SIZE = (64, 64)

SAVE_DIR = os.path.expanduser('~/detections')
os.makedirs(SAVE_DIR, exist_ok=True)

bundle = joblib.load("hog_svm_greek_letters.pkl")
model = bundle["model"]
le = bundle["label_encoder"]

#////////////////////////////////////////////////////////////////////////////////////
#                     IMAGE PROCESSING
#////////////////////////////////////////////////////////////////////////////////////
        
def order_points(pts):
    # Rearrange points: top-left, top-right, bottom-right, bottom-left
    rect = np.zeros((4, 2), dtype="float32")

    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]

    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]

    return rect

def four_point_transform(image, pts):
    rect = order_points(pts)
    (tl, tr, br, bl) = rect

    widthA = np.linalg.norm(br - bl)
    widthB = np.linalg.norm(tr - tl)
    maxWidth = int(max(widthA, widthB))

    heightA = np.linalg.norm(tr - br)
    heightB = np.linalg.norm(tl - bl)
    maxHeight = int(max(heightA, heightB))

    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]
    ], dtype="float32")

    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))

    return warped

def imagePreprocess(image):
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(gray, 50, 150)
    kernel = np.ones((5,5), np.uint8)
    edges = cv2.dilate(edges, kernel, iterations=1)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    edges = cv2.dilate(edges, kernel, iterations=1)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    #edges = cv2.dilate(edges, kernel, iterations=1)
    #edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    return edges

#////////////////////////////////////////////////////////////////////////////////////
#                     HOG AND SVM
#////////////////////////////////////////////////////////////////////////////////////

def preprocess_image_array(img):
    img = cv2.resize(img, IMG_SIZE)
    img = cv2.equalizeHist(img)
    return img

def extract_hog(img):
    return hog(
        img,
        orientations=9,
        pixels_per_cell=(8, 8),
        cells_per_block=(2, 2),
        block_norm='L2-Hys',
        transform_sqrt=True,
        feature_vector=True
    )

#////////////////////////////////////////////////////////////////////////////////////
#                 CAPTURING IMAGE
#////////////////////////////////////////////////////////////////////////////////////

class JoyImageCapture(Node):
    def __init__(self):
        super().__init__('joy_image_capture')

        self.bridge = CvBridge()

        self.latest_image = None
        self.last_button_state = 0

        self.image_sub = self.create_subscription(Image, '/oak/rgb/image_raw', self.image_callback, 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.create_subscription(String, '/perception/trigger', self._trigger_cb, 10)
        self._det_pub = self.create_publisher(String, '/detections', 10)
        self._current_pos = (0.0, 0.0)
        self.get_logger().info("TakeImageNode ready — waiting for /perception/trigger")

    def image_callback(self, msg):
        self.latest_image = msg

    def joy_callback(self, msg):
        pass

    def _trigger_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._current_pos = (float(data['x']), float(data['y']))
        except Exception:
            pass
        self.save_image()

    def save_image(self):
        if self.latest_image is None:
            self.get_logger().info("No image received yet")
            return

        img = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        
        #Top Black Border to Detect Paper close to top
        image = cv2.copyMakeBorder(
            img,
            top=100,
            bottom=0,
            left=0,
            right=0,
            borderType=cv2.BORDER_CONSTANT,
            value=(0,255,0)
        )
        
        orig = image.copy()    
        edges = imagePreprocess(image)
        
        # ---- Find contours ----
        """Find Connected Shapes
        Only External (doesn't do inside shapes) cv2.RETR_EXTERNAL or cv2.RETR_LIST returns all
        cv2.CHAIN_APPROX_SIMPLE returns a list of key points rather than every pixel on the outline"""
        contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Sort largest to smallest, A4 paper should be the largest
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        # Eventually stores the contour of the paper
        doc_contour = None

        # Check every found contour
        for i, c in enumerate(contours):

            # Find perimeter
            peri = cv2.arcLength(c, True)

            # Simplify contour
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)
            
            if cv2.contourArea(c) < 14000:
                break
            if cv2.contourArea(c) > 210000:
                continue
            
            area = cv2.contourArea(c)
            rect = cv2.minAreaRect(c)
            # Get 4 rectangle corners
            box = cv2.boxPoints(rect)
            box = np.int32(box)
            
            w, h = rect[1]
            if w == 0 or h == 0:
                continue
            box_area = w * h
            rectangularity = area / box_area
            aspect = h / w
            
            if rectangularity > 0.7: #and aspect > 1.2:
                doc_contour = box
                break   

        if doc_contour is None:
            #raise Exception("Could not find A4 sheet")
            print("No A4 Found")
            #output_dir2 = os.path.join("notable", filename)
            #cv2.imwrite(output_dir2, orig)

        ##insert colour det

            return

        # ---- Transform ----
        warped = four_point_transform(orig, doc_contour.reshape(4, 2))
        
        # ---- Save ----
        filename = f"letter_{self.get_clock().now().to_msg().sec}.png"
        fpath = os.path.join(SAVE_DIR, filename)
        cv2.imwrite(fpath, warped)

        # PREDICT
        img = cv2.imread(fpath, cv2.IMREAD_GRAYSCALE)
        img = preprocess_image_array(img)
        feat = extract_hog(img).reshape(1, -1)

        pred_idx = model.predict(feat)[0]
        pred_label = le.inverse_transform([pred_idx])[0]
        proba = model.predict_proba(feat)[0]
        confidence = float(proba[pred_idx])

        self.get_logger().info(f'[LETTER] Predicted: {pred_label}  Confidence: {confidence:.2f}')

        ox, oy = self._current_pos
        entry = {'label': f'letter_{pred_label}', 'x': round(ox, 2), 'y': round(oy, 2),
                 'confidence': round(confidence, 3), 'image': fpath}
        out = String()
        out.data = json.dumps(entry)
        self._det_pub.publish(out)

        

#////////////////////////////////////////////////////////////////////////////////////
#				MAIN
#////////////////////////////////////////////////////////////////////////////////////

def main():
    rclpy.init()
    node = JoyImageCapture()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()