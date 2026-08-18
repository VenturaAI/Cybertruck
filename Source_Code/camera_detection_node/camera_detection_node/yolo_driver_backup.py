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

    def detect(self, frame):

        # Resize to model input
        img = cv2.resize(frame, (640, 640))

        # BGR -> RGB
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # Normalize
        img = img.astype(np.float32) / 255.0

        # HWC -> CHW
        img = np.transpose(img, (2, 0, 1))

        # Add batch dimension
        img = np.expand_dims(img, axis=0)

        # Run inference
        outputs = self.session.run(
            [self.output_name],
            {self.input_name: img}
        )

        detections = outputs[0]

        # Print only valid detections
        for det in detections[0]:

            x1, y1, x2, y2, conf, cls = det

            if conf > 0.40:

                print(
                    f"Class={int(cls)} "
                    f"Conf={conf:.2f} "
                    f"Box=({x1:.0f},{y1:.0f}) "
                    f"({x2:.0f},{y2:.0f})"
                )

        return {
            "detected": False,
            "class_name": "",
            "confidence": 0.0,
            "distance": 999.0,
            "center_x": 0.0,
            "center_y": 0.0
        }
