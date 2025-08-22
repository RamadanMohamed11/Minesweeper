import cv2
import easyocr

# Enable CUDA if available
reader = easyocr.Reader(['en'], gpu=True)  # Set gpu=True to use CUDA

# Initialize the camera (0 for default webcam)
camera = cv2.VideoCapture(0)

if not camera.isOpened():
    print("Error: Could not open camera.")
    exit()

while True:
    # Capture a frame
    ret, frame = camera.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Convert frame to grayscale (optional, EasyOCR works with color too)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect text using EasyOCR
    result = reader.readtext(gray)

    # Draw bounding boxes and detected text on the frame
    for (bbox, text, prob) in result:
        top_left, top_right, bottom_right, bottom_left = bbox
        top_left = tuple(map(int, top_left))
        bottom_right = tuple(map(int, bottom_right))

        # Draw rectangle around detected text
        cv2.rectangle(frame, top_left, bottom_right, (0, 255, 0), 2)

        # Display the detected text
        cv2.putText(frame, text, (top_left[0], top_left[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    # Show the frame with detections
    cv2.imshow("CUDA-Accelerated OCR", frame)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close windows
camera.release()
cv2.destroyAllWindows()
