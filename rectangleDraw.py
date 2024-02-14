import cv2

def nott(x):
    pass

video_path1 = "videos/88c81ff7-9c48-4826-811b-e9d02f5fcc10.mp4"
cam = cv2.VideoCapture(video_path1)

ESCAPE = 27
status, frame = cam.read()
im_height = frame.shape[0]//2
im_width = frame.shape[1]//2
cv2.namedWindow("result")
cv2.createTrackbar("y", "result", 0, im_height, nott)
cv2.createTrackbar("y2", "result", 0, im_height, nott)
cv2.createTrackbar("x", "result", 0, im_width, nott)
cv2.createTrackbar("x2", "result", 0, im_width, nott)

while True:
    status, frame = cam.read()
    if status == False:
        print("Have't frame1")
        cam.release()
        cam = cv2.VideoCapture(video_path1)
    else:
        frame = cv2.resize(frame, (frame.shape[1]//2, frame.shape[0]//2))

        x = cv2.getTrackbarPos("x", "result")
        y = cv2.getTrackbarPos("y", "result")
        x2 = cv2.getTrackbarPos("x2", "result")
        y2 = cv2.getTrackbarPos("y2", "result")

        cv2.rectangle(frame, (x, y), (x2, y2), (0, 255, 0), 2)
        cv2.imshow('frame', frame)

        key = cv2.waitKey(10)
        if key == ESCAPE:
            break

cam.release()
cv2.destroyAllWindows()
