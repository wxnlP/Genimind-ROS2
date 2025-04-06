import cv2
import numpy as np
from time import time
from hobot_vio import libsrcampy as srcampy
from hobot_dnn import pyeasy_dnn as dnn
from genimind_visual.utils.RDK import YOLO11_Detect

# sensor 
sensor_width = 1920
sensor_height = 1080

def get_display_res():
    disp_w_small=1920
    disp_h_small=1080
    disp = srcampy.Display()
    resolution_list = disp.get_display_res()
    if (sensor_width, sensor_height) in resolution_list:
        print(f"Resolution {sensor_width}x{sensor_height} exists in the list.")
        return int(sensor_width), int(sensor_height)
    else:
        print(f"Resolution {sensor_width}x{sensor_height} does not exist in the list.")
        for res in resolution_list:
            # Exclude 0 resolution first.
            if res[0] == 0 and res[1] == 0:
                break
            else:
                disp_w_small=res[0]
                disp_h_small=res[1]

            # If the disp_w、disp_h is not set or not in the list, default to iterating to the smallest resolution for use.
            if res[0] <= sensor_width and res[1] <= sensor_height:
                print(f"Resolution {res[0]}x{res[1]}.")
                return int(res[0]), int(res[1])

    disp.close()
    return disp_w_small, disp_h_small

coco_names = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", 
    "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", 
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", 
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", 
    "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", 
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", 
    "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", 
    "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
    ]

rdk_colors = [
    (56, 56, 255), (151, 157, 255), (31, 112, 255), (29, 178, 255),(49, 210, 207), (10, 249, 72), (23, 204, 146), (134, 219, 61),
    (52, 147, 26), (187, 212, 0), (168, 153, 44), (255, 194, 0),(147, 69, 52), (255, 115, 100), (236, 24, 0), (255, 56, 132),
    (133, 0, 82), (255, 56, 203), (200, 149, 255), (199, 55, 255)]

def draw_detection(img: np.array, 
                   bbox: tuple[int, int, int, int],
                   score: float, 
                   class_id: int) -> None:
    """
    Draws a detection bounding box and label on the image.

    Parameters:
        img (np.array): The input image.
        bbox (tuple[int, int, int, int]): A tuple containing the bounding box coordinates (x1, y1, x2, y2).
        score (float): The detection score of the object.
        class_id (int): The class ID of the detected object.
    """
    x1, y1, x2, y2 = bbox
    color = rdk_colors[class_id%20]
    cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
    label = f"{coco_names[class_id]}: {score:.2f}"
    (label_width, label_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    label_x, label_y = x1, y1 - 10 if y1 - 10 > label_height else y1 + 10
    cv2.rectangle(
        img, (label_x, label_y - label_height), (label_x + label_width, label_y + label_height), color, cv2.FILLED
    )
    cv2.putText(img, label, (label_x, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)


class BPU_Detect:
    def __init__(self, model_path, conf=0.45, iou=0.25, display=True):
        try:
            begin_time = time()
            # self.models = dnn.load(model_path)
            self.models = YOLO11_Detect(model_path, 0.25, 0.45)
            print(f'[INFO] 模型加载时间: {np.round(1000*(time() - begin_time), 2)}ms')
        except Exception as e:
            print(f'[INFO] 加载模型失败: {e}')
            exit(1)
        # 从模型输入获取输入尺寸
        # self.input_shape = self.models[0].inputs[0].properties.shape
        # self.input_w = self.input_shape[2]  # NCHW格式
        # self.input_h = self.input_shape[3]
        self.input_w = self.models.model_input_weight
        self.input_h = self.models.model_input_height
        self.display = display

    def static_image_init(self, img_path):
        """静态图片准备"""
        img_file = cv2.imread(img_path)
        h = self.models[0].inputs[0].properties.shape[2]
        w = self.models[0].inputs[0].properties.shape[3]
        resized_data = cv2.resize(img_file, (w, h), interpolation=cv2.INTER_AREA)
        self.nv12_data = self.bgr2nv12_opencv(resized_data)

    def static_image_detect(self):
        outputs = self.models[0].forward(self.nv12_data)

    def camera_init(self):
        """MIPI摄像头初始化"""
        # 创建MIPI对象
        self.cam = srcampy.Camera()
        # 打开MIPI
        disp_w, disp_h = get_display_res()
        self.cam.open_cam(0, -1, 30, [self.input_w, disp_w], [self.input_h, disp_h], sensor_height, sensor_width)
        if self.display:
            self.disp = srcampy.Display()
            self.disp.display(0, disp_w, disp_h)
            srcampy.bind(self.cam, self.disp)
            self.disp.display(3, disp_w, disp_h)

    def camera_detect(self):
        """MIPI检测"""
        try:
            while True:
                img_bytes = self.cam.get_img(2, 640, 640)
                if img_bytes is None:
                    print("WARN: Get image timeout")
                    continue
                img_nv12 = np.frombuffer(img_bytes, dtype=np.uint8)
                img_bgr = cv2.cvtColor(img_nv12.reshape(640*3//2, 640), cv2.COLOR_YUV2BGR_NV12)
                # cv2.imwrite("result.jpg", img_bgr)

                # 额外数据处理
                input_tensor = self.models.bgr2nv12(img_bgr)
                # 1.推理
                # outputs = self.models[0].forward(img_nv12)
                # self.print_properties(outputs[0].properties)
                # print(f'[INFO] {len(self.models[0].outputs)}')
                # 2.推理
                outputs = self.models.c2numpy(self.models.forward(input_tensor))
                ids, scores, bboxes = self.models.postProcess(outputs)
                for class_id, score, bbox in zip(ids, scores, bboxes):
                    x1, y1, x2, y2 = bbox
                    print("(%d, %d, %d, %d) -> %s: %.2f"%(x1,y1,x2,y2, coco_names[class_id], score))
                    # draw_detection(img_bgr, (x1, y1, x2, y2), score, class_id)
                    if self.display:
                        self.disp.set_graph_rect(x1, y1, x2, y2, chn = 2, flush = 1,  color = 0xffff00ff)
        except Exception as e:
            print(f"[WARN] {e}")
            self.cam.close_cam()
            if self.display:
                self.disp.close()


if __name__ == "__main__":
    detect = BPU_Detect('models/yolo11n_detect_bayese_640x640_nv12.bin', display=True)
    detect.camera_init()
    detect.camera_detect()