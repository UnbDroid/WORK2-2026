import cv2
import numpy as np
import apriltag
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from robot_vision.msg import Cube, CubeArray


# ==============================================================================
# 1. CLASSE DE DETECÇÃO DE CORES
# ==============================================================================
class ColorDetector:
    def __init__(self):
        self.red_lower1 = np.array([0, 120, 70])
        self.red_upper1 = np.array([10, 255, 255])
        
        self.red_lower2 = np.array([170, 120, 70])
        self.red_upper2 = np.array([180, 255, 255])
        
        self.blue_lower = np.array([90, 50, 50])
        self.blue_upper = np.array([130, 255, 255])
        
        self.kernel = np.ones((3, 3), np.uint8)

    def detect(self, frame):
        suave = cv2.GaussianBlur(frame, (7, 7), 0)
        img_hsv = cv2.cvtColor(suave, cv2.COLOR_BGR2HSV)

        # Filtro Vermelho
        red_mask1 = cv2.inRange(img_hsv, self.red_lower1, self.red_upper1)
        red_mask2 = cv2.inRange(img_hsv, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        red_erode = cv2.erode(red_mask, self.kernel, iterations=1)
        red_dilate = cv2.dilate(red_erode, self.kernel, iterations=1)

        # Filtro Azul
        blue_mask = cv2.inRange(img_hsv, self.blue_lower, self.blue_upper)
        blue_erode = cv2.erode(blue_mask, self.kernel, iterations=1)
        blue_dilate = cv2.dilate(blue_erode, self.kernel, iterations=1)

        color_objects = []
        for mask, color_name in [(red_dilate, "vermelho"), (blue_dilate, "azul")]:
            contours_info = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = contours_info[0] if len(contours_info) == 2 else contours_info[1]

            for cnt in contours:
                if cv2.contourArea(cnt) > 1500:
                    color_objects.append({
                        "color": color_name,
                        "contour": cnt,
                        "bbox": cv2.boundingRect(cnt),
                        "processed": False
                    })

        return color_objects


# ==============================================================================
# 2. CLASSE DE DETECÇÃO DE APRILTAGS
# ==============================================================================
class AprilTagDetector:
    def __init__(self):
        self.options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(self.options)
        self.camera_params = [1121.40, 118.81, 649.17, 364.85]
        self.tag_size = 0.04

    def detect(self, frame):
        if frame is None:
            return []
            
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)
        tags = []

        for r in results:
            pose, _, _ = self.detector.detection_pose(r, self.camera_params, self.tag_size)
            corners = r.corners.astype(int)

            tags.append({
                "id": int(r.tag_id),
                "center": (float(r.center[0]), float(r.center[1])),
                "bbox": cv2.boundingRect(corners),
                "pose_3d": (float(pose[0][3]), float(pose[1][3]), float(pose[2][3]))
            })

        return tags


# ==============================================================================
# 3. MOTOR DE CLASSIFICAÇÃO
# ==============================================================================
class ObjectClassifier:
    @staticmethod
    def is_tag_inside_color(contour, tag_center, tag_bbox, color_bbox, margin=15):
        pt = (int(tag_center[0]), int(tag_center[1]))
        
        if cv2.pointPolygonTest(contour, pt, False) >= 0:
            return True

        x, y, w, h = color_bbox
        tx, ty, tw, th = tag_bbox
        
        tx_min, ty_min = tx - margin, ty - margin
        tx_max, ty_max = tx + tw + margin, ty + th + margin

        return (x < tx_max and x + w > tx_min and y < ty_max and y + h > ty_min)

    def classify(self, tags, color_objects):
        classified_objects = []

        # 1. Associa Tags a Cores (Cubos ou Zonas)
        for tag in tags:
            matched = False
            for color_obj in color_objects:
                if color_obj["processed"]:
                    continue

                if self.is_tag_inside_color(
                    color_obj["contour"], 
                    tag["center"], 
                    tag["bbox"], 
                    color_obj["bbox"]
                ):
                    color_obj["processed"] = True
                    matched = True
                    classified_objects.append({
                        "id": tag["id"],
                        "color": color_obj["color"],
                        "waypoint": "cubo",
                        "bbox": color_obj["bbox"],
                        "pose_3d": tag["pose_3d"]
                    })
                    break

            if not matched:
                classified_objects.append({
                    "id": tag["id"],
                    "color": "sem_cor",
                    "waypoint": "zona",
                    "bbox": tag["bbox"],
                    "pose_3d": tag["pose_3d"]
                })

        # 2. Registra Cores Restantes sem Tag (Contêineres)
        for color_obj in color_objects:
            if not color_obj["processed"]:
                x, y, w, h = color_obj["bbox"]
                cx, cy = x + w // 2, y + h // 2
                pos_x = (cx - 320) / 500.0
                pos_y = (cy - 240) / 500.0

                classified_objects.append({
                    "id": -1,
                    "color": color_obj["color"],
                    "waypoint": "conteiner",
                    "bbox": color_obj["bbox"],
                    "pose_3d": (pos_x, pos_y, 1.0)
                })

        return classified_objects


# ==============================================================================
# 4. NÓ PRINCIPAL ROS 2
# ==============================================================================
class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.bridge = CvBridge()
        self.color_detector = ColorDetector()
        self.tag_detector = AprilTagDetector()
        self.classifier = ObjectClassifier()

        self.sub = self.create_subscription(Image, '/cam/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(CubeArray, '/detected_objects', 10)

        # Cria a janela de exibição com tamanho ajustável para evitar travamentos gráficos
        cv2.namedWindow("Visão do Robô", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Visão do Robô", 640, 480)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            display_frame = frame.copy()

            color_objects = self.color_detector.detect(frame)
            tags = self.tag_detector.detect(frame)
            results = self.classifier.classify(tags, color_objects)

            array_msg = CubeArray()
            array_msg.header.stamp = self.get_clock().now().to_msg()
            array_msg.header.frame_id = 'camera_link'

            for item in results:
                # Preenche a mensagem do ROS
                cube = Cube()
                cube.id = item["id"]
                cube.color = item["color"]
                cube.waypoint = item["waypoint"]
                cube.position_relative_to_camera.x = float(item["pose_3d"][0])
                cube.position_relative_to_camera.y = float(item["pose_3d"][1])
                cube.position_relative_to_camera.z = float(item["pose_3d"][2])
                array_msg.cubes.append(cube)

                # Desenha o objeto detectado no frame de visualização
                x, y, w, h = item["bbox"]
                cor_bgr = (0, 0, 255) if item["color"] == "vermelho" else (255, 0, 0) if item["color"] == "azul" else (255, 255, 255)
                
                cv2.rectangle(display_frame, (x, y), (x + w, y + h), cor_bgr, 2)
                label = f"{item['waypoint'].upper()} | ID:{item['id']} | {item['color']}"
                cv2.putText(display_frame, label, (x, max(y - 10, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, cor_bgr, 2)

            if len(array_msg.cubes) > 0:
                self.pub.publish(array_msg)

            # Exibe o frame processado e atualiza os eventos gráficos
            cv2.imshow("Visão do Robô", display_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Erro no processamento da imagem: {e}')


# ==============================================================================
# PONTO DE ENTRADA DO SCRIPT
# ==============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Garante que as janelas sejam fechadas ao pressionar Ctrl+C no terminal
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()