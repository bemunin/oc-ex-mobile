import os

import cv2
import rclpy

# import requests
from cv_bridge import CvBridge
from oc_interfaces.srv import ImageToText
from PIL import Image as PillowImage
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from transformers import BlipForConditionalGeneration, BlipProcessor

DIR_NAME = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.join(DIR_NAME, "../../../../../../..")
IMG_FILE_NAME = "iwhub_left_camera_image.jpg"


class ImageToTextService(Node):
    def __init__(self):
        super().__init__("image_to_text_service")
        self.declare_parameter("device", "cpu")  # cpu, gpu,serverless_gpu
        self.service = self.create_service(
            ImageToText,
            "image_to_text_service",
            self.desc_image_callback,
        )

        # bridge
        self._cv_bridge = CvBridge()

        # vlm model
        self._processor = BlipProcessor.from_pretrained(
            f"{ROOT_DIR}/.models/blip-image-captioning-large"
        )
        self._model = BlipForConditionalGeneration.from_pretrained(
            f"{ROOT_DIR}/.models/blip-image-captioning-large",
        )

    def desc_image_callback(self, request: Image, response: String):
        self.get_logger().info("Received image request")
        device = self._params("device").string_value
        result_text = ""
        # Convert ROS Image message to OpenCV format
        cv_image = self._cv_bridge.imgmsg_to_cv2(request.image, desired_encoding="bgr8")
        self._save_image(cv_image)

        # Call the AI model to process the image
        if device == "serverless_gpu":
            result_text = self.ai_serverless_gpu()
        else:
            result_text = self.ai(device)

        self._save_image(cv_image, result_text)

        response.text = result_text
        return response

    def ai_serverless_gpu(self) -> str:
        # TODO: Send post request to serverless GPU
        return "Not implemented yet"

    def ai(self, device="cpu") -> str:
        image_path = f"{ROOT_DIR}/.execute/{IMG_FILE_NAME}"
        raw_image = PillowImage.open(image_path).convert("RGB")
        leading_text = "A photography of"
        inputs = self._processor(raw_image, leading_text, return_tensors="pt")

        if device == "gpu":
            self.get_logger().info("Using GPU for inference")
            self._model.to("cuda")
            inputs.to("cuda")
        else:
            self.get_logger().info("Using CPU for inference")
            self._model.to("cpu")

        output = self._model.generate(**inputs)
        text_output = self._processor.decode(output[0], skip_special_tokens=True)

        return text_output

    def _save_image(self, cv_image, text: str = None):
        self.get_logger().info("Saving image to file...")

        # If text is provided, overlay it on the image with a highlighted background
        if text is not None:
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1
            font_color = (255, 255, 255)  # White text
            thickness = 2
            position = (10, 50)  # Top-left corner of the text

            # Calculate text size
            text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
            text_width, text_height = text_size[0], text_size[1]
            margin = 5  # Margin around the text

            # Define the rectangle coordinates
            rect_x1 = position[0] - margin
            rect_y1 = position[1] - text_height - margin
            rect_x2 = position[0] + text_width + margin
            rect_y2 = position[1] + margin

            # Add a semi-transparent black rectangle
            overlay = cv_image.copy()
            cv2.rectangle(
                overlay, (rect_x1, rect_y1), (rect_x2, rect_y2), (0, 0, 0), -1
            )
            alpha = 0.3  # Opacity level (30%)
            cv_image = cv2.addWeighted(overlay, alpha, cv_image, 1 - alpha, 0)

            # Add the text on top of the rectangle
            cv2.putText(
                cv_image,
                text,
                position,
                font,
                font_scale,
                font_color,
                thickness,
                cv2.LINE_AA,
            )

        # Save the image to a file
        if not os.path.exists(f"{ROOT_DIR}/.execute"):
            os.makedirs(f"{ROOT_DIR}/.execute")

        file_path = f"{ROOT_DIR}/.execute/{IMG_FILE_NAME}"

        cv2.imwrite(file_path, cv_image)
        self.get_logger().info(f"Image saved to {file_path}")
        return cv_image

    def _params(self, name: str) -> str:
        """Getter for the device parameter."""
        return self.get_parameter(name).get_parameter_value()


def main(args=None):
    rclpy.init(args=args)

    service = ImageToTextService()

    rclpy.spin(service)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
