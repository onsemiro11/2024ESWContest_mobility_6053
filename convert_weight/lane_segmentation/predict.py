import cv2
import matplotlib.pyplot as plt
from ultralytics import YOLO

# Load a pretrained YOLO11n-seg Segment model
model = YOLO("best.pt")

# Run inference on an image
results = model("lane.jpg")  # results list

# Load the original image
img = cv2.imread("lane.jpg")

# Check if the results contain masks
for r in results:
    if r.masks is not None:
        # Extract the masks
        masks = r.masks.data.cpu().numpy()  # Convert the mask data to NumPy array

        # For visualization, we'll overlay masks on the original image
        for mask in masks:
            mask_resized = cv2.resize(mask, (img.shape[1], img.shape[0]))
            img[mask_resized > 0.5] = [0, 255, 0]  # Overlay the mask in green

# Convert BGR to RGB for display in matplotlib
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

# Display the result
plt.imshow(img_rgb)
plt.axis('off')  # Hide axis
plt.show()

