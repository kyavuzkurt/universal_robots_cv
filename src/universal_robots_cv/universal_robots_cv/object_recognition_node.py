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
        super().__init__('camera_transformer_node')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/camera_raw', self.image_callback, 10)
        self.tracked_image_pub = self.create_publisher(Image, '/camera/tracked_image', 10)
        self.object_info_pub = self.create_publisher(ObjectInformation, '/object_info', 10)
        self.model = torchvision.models.detection.fasterrcnn_resnet50_fpn(weights=FasterRCNN_ResNet50_FPN_Weights.DEFAULT)
        self.model.eval()
        self.coco_labels = {v: k for k, v in self.model.coco_labels.items()}

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error('CvBridge Error: {0}'.format(e))
            return
        self.get_logger().info('Received image')
        self.detect_objects(cv_image)

    def detect_objects(self, cv_image):
        transform = torchvision.transforms.Compose([torchvision.transforms.ToTensor()])
        image_tensor = transform(cv_image)
        outputs = self.model([image_tensor])[0]
        for i, (box, score, label) in enumerate(zip(outputs['boxes'], outputs['scores'], outputs['labels'])):
            if score >= 0.5:
                x1, y1, x2, y2 = box.int().tolist()
                label_name = self.coco_labels[label.item()]
                cv.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                overlay = cv_image.copy()
                cv.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), -1)
                alpha = 0.4
                cv_image = cv.addWeighted(overlay, alpha, cv_image, 1 - alpha, 0)
                cv.putText(cv_image, label_name, (x1, y1 - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                self.get_logger().info('Detected object: %s' % label_name)
                self.object_info_pub.publish(label_name)
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

        