#import helper libraries
import time
import urllib.request
from urllib.parse import urlparse
import os
import sys
import requests
import io
from PIL import Image
import cv2
import numpy as np

#custom patch libraries
import retired.patch as patch

class ImageProcessor():
    def __init__(self, output_size):
        self.output_size = output_size

    def detect_faces(self, image):
      # Convert the image from PIL.Image format to a NumPy array
      image = np.array(image)

      # Convert the image to grayscale
      gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

      # Load the face detector
      face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

      # Detect faces in the image
      faces = face_cascade.detectMultiScale(gray, 1.3, 5)

      # Return the list of face bounding boxes
      return faces

    def process_image(self, image):
        width, height = image.size

        if width < self.output_size or height < self.output_size:
            raise Exception("Has smaller resolution than output_size, skipping")

        if width < height:
            new_width = self.output_size
            new_height = int(height * self.output_size / width)
        else:
            new_width = int(width * self.output_size / height)
            new_height = self.output_size

        image = image.resize((new_width, new_height), Image.ANTIALIAS)
        face_boxes = self.detect_faces(image)

        images = []
        
        if len(face_boxes) == 0:
            print("[INFO] No faces found...")

        for face_box in face_boxes:
            center_x = face_box[0] + face_box[2] / 2
            center_y = face_box[1] + face_box[3] / 2
            top = center_y - self.output_size / 2
            left = center_x - self.output_size / 2
            bottom = center_y + self.output_size / 2
            right = center_x + self.output_size / 2

            # Adjust top and left values to ensure they do not go outside the bounds of the original image
            if top < 0:
                bottom = bottom + abs(top)
                top = 0
            if left < 0:
                right = right + abs(left)
                left = 0

            # Adjust bottom and right values to ensure they do not go outside the bounds of the original image
            if bottom > new_height:
                rest = bottom - new_height
                top = top - rest
                bottom = new_height
            if right > new_width:
                rest = right - new_width
                left = left - rest
                right = new_width

            new_image = image.copy()
            new_image = new_image.crop((left, top, right, bottom))
            images.append(new_image)
        
        return images

    def process_url(self, image_url):
        image = requests.get(image_url,timeout=5)
        if image.status_code != 200:
            raise Exception("Discarded due to error code %s"%(image.status_code))
        
        image_from_web = Image.open(io.BytesIO(image.content))
        return image_from_web
#                    except OSError:
#                        print("[WARNING] OS Error: %s, trying anyway", %(e))
#                        rgb_im = image_from_web.convert('RGB')
#                        process_image(rgb_im, output_size, image_path)
