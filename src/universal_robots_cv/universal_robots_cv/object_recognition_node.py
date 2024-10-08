import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from object_recognition_msgs.msg import ObjectInformation
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import torch
import torchvision
from torchvision.models.detection import FasterRCNN_ResNet50_FPN_Weights

class CameraTransformerNode(Node):
    def __init__(self):
        super().__init__('object_recognition_node')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
                                                    Image,
                                                    '/camera/image_raw',
                                                    self.listener_callback,
                                                    10)
        self.image_sub 
        self.tracked_image_pub = self.create_publisher(Image, '/camera/tracked_image', 10)
        self.object_info_pub = self.create_publisher(ObjectInformation, '/object_info', 10)
        self.model = torchvision.models.detection.fasterrcnn_resnet50_fpn(weights=FasterRCNN_ResNet50_FPN_Weights.DEFAULT)
        self.model.eval()
        
        self.coco_labels = {
            1: 'person', 2: 'bicycle', 3: 'car', 4: 'motorcycle', 5: 'airplane',
            6: 'bus', 7: 'train', 8: 'truck', 9: 'boat', 10: 'traffic light',
            11: 'fire hydrant', 13: 'stop sign', 14: 'parking meter', 15: 'bench',
            16: 'bird', 17: 'cat', 18: 'dog', 19: 'horse', 20: 'sheep',
            21: 'cow', 22: 'elephant', 23: 'bear', 24: 'zebra', 25: 'giraffe',
            27: 'backpack', 28: 'umbrella', 31: 'handbag', 32: 'tie',
            33: 'suitcase', 34: 'frisbee', 35: 'skis', 36: 'snowboard',
            37: 'sports ball', 38: 'kite', 39: 'baseball bat', 40: 'baseball glove',
            41: 'skateboard', 42: 'surfboard', 43: 'tennis racket',
            44: 'bottle', 46: 'wine glass', 47: 'cup', 48: 'fork',
            49: 'knife', 50: 'spoon', 51: 'bowl', 52: 'banana',
            53: 'apple', 54: 'sandwich', 55: 'orange', 56: 'broccoli',
            57: 'carrot', 58: 'hot dog', 59: 'pizza', 60: 'donut',
            61: 'cake', 62: 'chair', 63: 'couch', 64: 'potted plant',
            65: 'bed', 67: 'dining table', 70: 'toilet', 72: 'tv',
            73: 'laptop', 74: 'mouse', 75: 'remote', 76: 'keyboard',
            77: 'cell phone', 78: 'microwave', 79: 'oven', 80: 'toaster',
            81: 'sink', 82: 'refrigerator', 84: 'book', 85: 'clock',
            86: 'vase', 87: 'scissors', 88: 'teddy bear', 89: 'hair drier',
            90: 'toothbrush'
        }

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error('CvBridge Error: {0}'.format(e))
            return
        self.get_logger().info('Received image')
        transform = torchvision.transforms.Compose([torchvision.transforms.ToTensor()])
        image_tensor = transform(cv_image)
        with torch.no_grad():
            outputs = self.model([image_tensor])[0]
        for i, (box, score, label) in enumerate(zip(outputs['boxes'], outputs['scores'], outputs['labels'])):
            if score >= 0.5:
                x1, y1, x2, y2 = box.int().tolist()
                label_name = self.coco_labels.get(label.item(), 'Unknown')
                cv.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                overlay = cv_image.copy()
                cv.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), -1)
                alpha = 0.4
                cv_image = cv.addWeighted(overlay, alpha, cv_image, 1 - alpha, 0)
                cv.putText(cv_image, label_name, (x1, y1 - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                self.get_logger().info('Detected object: %s' % label_name)
                
                # Publish ObjectInformation
                object_info = ObjectInformation()
                object_info.name = label_name
                self.object_info_pub.publish(object_info)
                
                try:
                    detection_image = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
                    self.tracked_image_pub.publish(detection_image)
                except CvBridgeError as e:
                    self.get_logger().error('Failed to convert image: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)
    node = CameraTransformerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()