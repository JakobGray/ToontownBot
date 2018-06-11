# import the necessary packages
import os
from subprocess import Popen, PIPE
from threading import Thread
from time import sleep
import numpy as np
import argparse
import cv2
from PIL import ImageGrab

def round_down(num, divisor):
    return num - (num%divisor)

index = -1
thickness = 4
color = (255, 0, 255)
yessir = True
target_x = 450
target_y = 200
boot_template = cv2.imread('boot.jpg')
fishing = False
bucketcount = 0
circlesaver = {}
ignored = {}

def get_image_from_screenshot():
    printscreen_pil =  ImageGrab.grab(bbox=(0,0,900,600))
    image =   np.array(printscreen_pil, dtype='uint8')\
    .reshape((printscreen_pil.size[1],printscreen_pil.size[0],3))
    image = image[:,:,::-1]
    return image

def process_image(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 115, 4)
    thresh = cv2.bitwise_not(thresh)    # Invert the binary image

    # Preprocess with erosion and dilation
    kernel = np.ones((5,5), 'uint8')
    thresh = cv2.erode(thresh,kernel,iterations=1)
    thresh = cv2.dilate(thresh,kernel,iterations=4)
    thresh = cv2.erode(thresh,kernel,iterations=3)
    # cv2.imshow("Thresh processed", thresh)

    return thresh

def display_reduced(image):
    winname = "Detection"
    shrunken = cv2.resize(image, (0,0), fx=0.5, fy=0.5)
    cv2.namedWindow(winname)
    cv2.moveWindow(winname, 400, 700)
    cv2.imshow("Detection", shrunken)
    return

def check_if_caught():
    child = Popen(["fishcaughtcolor.exe"], stdout=PIPE, stderr=PIPE)
    streamdata = child.communicate()[0]
    rc = child.returncode
    if rc == 2:
        print("New Species Found")
    elif rc == 3:
        print("New Record")
    elif rc == 0:
        return False

    return True

def straight_toss():
    child = Popen(["throws.exe"], stdout=PIPE, stderr=PIPE, shell=True)
    streamdata = child.communicate()[0]
    return

def toss_rod(x,y):
    child = Popen(["target_fish.exe", str(x), str(y)], stdout=PIPE, stderr=PIPE, shell=True)
    streamdata = child.communicate()[0]
    return

def close_fish_box():
    child = Popen(["closeFishBox.exe"], stdout=PIPE, stderr=PIPE, shell=True)
    streamdata = child.communicate()[0]
    return

def close_boot_box():
    child = Popen(["closeBootBox.exe"], stdout=PIPE, stderr=PIPE, shell=True)
    streamdata = child.communicate()[0]
    return

def sell_fish():
    child = Popen(["walrusway.exe"], stdout=PIPE, stderr=PIPE, shell=True)
    streamdata = child.communicate()[0]
    return

def full_fish(target_x, target_y, image):
    global bucketcount
    global fishing

    toss_rod(int(target_x),int(target_y))
    for _ in range(5):
        if check_if_caught():
            match_result = cv2.matchTemplate(image, boot_template, cv2.TM_CCOEFF_NORMED)
            minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(match_result)
            print(maxVal)
            bucketcount += 1
            print("Bucket capacity: " + str(bucketcount))
            if maxVal >= 0.99:
                close_boot_box()
            else:
                close_fish_box()
                sleep(0.5)
            if bucketcount >= 15:
                straight_toss()
                sleep(3)
                sell_fish()
                bucketcount = 0
            fishing = False
            return
        sleep(0.5)
    fishing = False
    return

# Grab first frame for dimensions
init_image = get_image_from_screenshot()

height = init_image.shape[0]
width = init_image.shape[1]
minheight = int(height * .1)
maxheight = int(height * .66)
minwidth = int(width * .20)
maxwidth = int(width * .8)

print("Detecting static objects...")
for _ in range(200):
    image = get_image_from_screenshot()

    # load the image, clone it for output, and then convert it to grayscale
    output = image.copy()
    processed_image = process_image(image)

    # Find contours from theshold image
    _, contours, hierarchy = cv2.findContours(processed_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
        if len(c) < 5:
            continue

        # Filter by area
        area = cv2.contourArea(c)
        if area > 3000 or area < 100:
            continue

        # Filter by perimeter
        perimeter = cv2.arcLength(c, True)
        if perimeter > 350 or perimeter < 80:
            continue

        # Fit ellipse to object
        (x, y), (MA, ma), angle = cv2.fitEllipse(c)
        ellipse = ((x, y), (MA, ma), angle)

        # Check that object is in bounding box
        if x < minwidth or x > maxwidth or y < minheight or y > maxheight:
            continue

        # Check that ellipse is somewhat circular
        if ma > 3*MA:
            continue

        xround = round_down(int(x),5)
        yround = round_down(int(y),5)
        if ((xround, yround)) not in circlesaver:
            circlesaver[(xround, yround)] = 1
        else:
            circlesaver[(xround, yround)] += 1


print("Blocking out bad zones")
# Determine locations to ignore
for key in circlesaver:
    if circlesaver[key] > 50:
        xround = key[0]
        yround = key[1]
        for xpixel in range(xround-1,xround+7):
            for ypixel in range(yround-1, yround+7):
                if (xpixel,ypixel) not in ignored:
                    ignored[(xpixel,ypixel)] = True
for x in range(404,497):
    for y in range(357,485):
        if (x,y) not in ignored:
            ignored[(x,y)] = True

while(yessir):

    image = get_image_from_screenshot()

    # load the image, clone it for output, and then convert it to grayscale
    output = image.copy()
    processed_image = process_image(image)

    # Find contours from theshold image
    _, contours, hierarchy = cv2.findContours(processed_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
        if len(c) < 5:
            continue

        # Filter by area
        area = cv2.contourArea(c)
        if area > 3000 or area < 100:
            continue

        # Filter by perimeter
        perimeter = cv2.arcLength(c, True)
        if perimeter > 350 or perimeter < 80:
            continue

        # Fit ellipse to object
        (x, y), (MA, ma), angle = cv2.fitEllipse(c)
        ellipse = ((x, y), (MA, ma), angle)

        if (int(x),int(y)) in ignored:
            continue

        # Check that object is in bounding box
        if x < minwidth or x > maxwidth or y < minheight or y > maxheight:
            continue

        # Check that ellipse is somewhat circular
        if ma > 3*MA:
            continue

        target_x = int(x)
        target_y = int(y)
        cv2.ellipse(output, ellipse, (0, 255, 0), 2)

        display_reduced(output)

        # End on 'q' input
        if cv2.waitKey(20) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            yessir = False
            break

        if(not fishing):
            fishing = True
            Thread(target=full_fish, args=([target_x,target_y,image])).start()
