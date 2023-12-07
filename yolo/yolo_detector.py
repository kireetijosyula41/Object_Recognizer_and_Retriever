import torch
from PIL import Image, ImageDraw
import numpy as np

from torchvision import transforms


def load_model(model_path, device):
    # Load the trained model from file
    # trained_model = torch.load(model_path)
    trained_model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)


    trained_model = trained_model.to(device)
    trained_model.eval()  # Set the model to evaluation mode
    return trained_model


def process_image(image_path):
    # Load image
    image = Image.open(image_path).convert("RGB")

    # Define transformations
    transform = transforms.Compose([
        transforms.Resize((640, 640)),
        transforms.ToTensor(),
    ])

    # Apply transformations
    image = transform(image)
    return image


def detect_objects(model, image, device):
    # Add an extra batch dimension since pytorch treats all inputs as batches
    image = image.unsqueeze(0)

    # Move the image to the selected device
    image = image.to(device)

    with torch.no_grad():
        # Forward pass
        outputs = model(image)

    return outputs


def main():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # Path to your trained YOLO model and the image
    model_path = "./runs/train/yolo_custom24/weights/best.pt"  # Update with your model path
    image_path = "../dataset/images/floor3.png"  # Update with your image path

    # Load the model
    model = load_model(model_path, device)

    # Process the image
    image = process_image(image_path)

    # Detect objects
    outputs = detect_objects(model, image, device)

    # Print results
    print(len(outputs[0][0]))
    # Process outputs
    outputs = outputs[0][0]  # Get detections

    # Load image for drawing
    image = Image.open(image_path).convert("RGB")
    draw = ImageDraw.Draw(image)

    # Define classes (replace with your classes)
    class_names = ['ball', 'cube', 'bottle', 'pen', 'nothing']
    filtered_outputs = sorted(outputs, key=lambda row: row[4])[-5:]
    for output in filtered_outputs:
        # Unpack the bounding box coordinates and confidence
        x_center, y_center, width, height, confidence, *class_probs = output.tolist()

        # Get the class with the highest probability
        class_pred = np.argmax(class_probs)
        class_conf = class_probs[class_pred]
        print(output)
        # Rescale bounding box back to image size
        x1, y1 = int((x_center - width / 2) * image.width), int((y_center - height / 2) * image.height)
        x2, y2 = int((x_center + width / 2) * image.width), int((y_center + height / 2) * image.height)
        print(x1, y1, x2, y2)
        # Draw rectangle to image
        draw.rectangle([x1, y1, x2, y2], outline="red", width=2)

        # Draw label (class name and confidence)
        label = f"{class_names[class_pred]} {class_conf:.2f}"
        draw.text((x1, y1), label, fill="white")
        #print(x1, y1, x2, y2)
    # Display image
    image.show()

    # Save image
    image.save("output.jpg")


if __name__ == "__main__":
    main()
