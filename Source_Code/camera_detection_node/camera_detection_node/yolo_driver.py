import cv2
import numpy as np
import onnxruntime as ort


class YoloDriver:

    def __init__(self):

        print("====================================")
        print(" Loading YOLO ONNX Model")
        print("====================================")

        self.session = ort.InferenceSession(
            "/root/models/best.onnx",
            providers=["CPUExecutionProvider"]
        )

        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

        print(f"Input  : {self.input_name}")
        print(f"Output : {self.output_name}")

        print("YOLO Driver Ready")

    def letterbox(self, image, new_shape=(640, 640), color=(114, 114, 114)):
        """
        Ultralytics-compatible LetterBox preprocessing.
        """

        shape = image.shape[:2]  # (height, width)

        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)

        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])

        new_unpad = (
            int(round(shape[1] * r)),
            int(round(shape[0] * r))
        )

        dw = new_shape[1] - new_unpad[0]
        dh = new_shape[0] - new_unpad[1]

        dw /= 2
        dh /= 2

        if shape[::-1] != new_unpad:
           image = cv2.resize(image, new_unpad, interpolation=cv2.INTER_LINEAR)

        top = int(round(dh - 0.1))
        bottom = int(round(dh + 0.1))
        left = int(round(dw - 0.1))
        right = int(round(dw + 0.1))

        image = cv2.copyMakeBorder(
            image,
            top,
            bottom,
            left,
            right,
            cv2.BORDER_CONSTANT,
            value=color
        )

        return image



    def detect(self, frame):

        # Ultralytics-compatible preprocessing
        img = self.letterbox(frame, (640, 640))

        # ---------- DEBUG SAVE ----------
        cv2.imwrite("/tmp/onnx_input.jpg", img)
        # -------------------------------
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        img = img.astype(np.float32)

        img /= 255.0

        img = np.transpose(img, (2, 0, 1))

        img = np.expand_dims(img, axis=0)

        img = np.ascontiguousarray(img)
        #Debug Point
        print(f"Input Tensor: {img.shape}  {img.dtype}")

        # Run inference
        outputs = self.session.run(
            [self.output_name],
            {self.input_name: img}
        )

        detections = outputs[0]

        print("\n============= RAW OUTPUT =============")
        print(detections[0][:10])
        print("======================================\n")

        best_conf = 0.0
        best_det = None

        for det in detections[0]:

            x1, y1, x2, y2, conf, cls = det

            if conf > best_conf:
                best_conf = float(conf)
                best_det = det

        if best_det is not None and best_conf > 0.30:

            x1, y1, x2, y2, conf, cls = best_det

            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0

            return {
                "detected": True,
                "class_name": "obstacle",
                "confidence": float(conf),
                "distance": 999.0,
                "center_x": float(center_x),
                "center_y": float(center_y)
            }

        return {
            "detected": False,
            "class_name": "",
            "confidence": 0.0,
            "distance": 999.0,
            "center_x": 0.0,
            "center_y": 0.0
        }
