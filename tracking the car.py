import cv2
import numpy as np
 
# Create a VideoCapture object and read from input file
# If the input is the camera, pass 0 instead of the video file name
cap = cv2.VideoCapture('FroggerHighway.mp4')

LEFT_TO_RIGHT= 1
RIGHT_TO_LEFT= 2
UNKNOWN = 0
direction = UNKNOWN 

kernelSize=1
x=0
y=0
x2=0
dX=0
t1=0
t2=0
DISTANCE= 13.395
#DISTANCE= 43.89
#kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
#sift = cv2.xfeatures2d.SIFT_create()
fgbg = cv2.createBackgroundSubtractorMOG2()
# Check if camera opened successfully
if (cap.isOpened()== False): 
  print("Error opening video stream or file")
 
# Read until video is completed
while(cap.isOpened()):
  # Capture frame-by-frame
  ret, frame = cap.read()
  width = cap.get(3)
  height = cap.get(4)
  print(width)
  #resize = cv2.resize(frame, (640, 480))
  ftperpixel= DISTANCE/ width
  
  
  
  if ret == True:
    time = cap.get(cv2.CAP_PROP_POS_MSEC)
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = cap.get(cv2.CAP_PROP_FRAME_COUNT)
    
    cv2.circle(frame,(245,5),5,(0,0,255),-1)
    cv2.circle(frame,(1115,5),5,(0,0,255),-1)
    cv2.circle(frame,(75,730),5,(0,0,255),-1)
    cv2.circle(frame,(1290,730),5,(0,0,255),-1)


    pts1 = np.float32([[245,5],[1115,5],[75,730],[1290,730]])
    pts2 = np.float32([[0,0],[1330,0],[0,730],[1330,730]])
    M = cv2.getPerspectiveTransform(pts1,pts2)
    dst = cv2.warpPerspective(frame,M,(1500,1500))
    
    kernel = np.ones((kernelSize,kernelSize),np.uint8)
    
    gray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray,(5,5),0)
    fgmask = fgbg.apply(blur)
    fgmask[fgmask==127]=0
    dilation = cv2.dilate(fgmask,kernel,iterations = 1)
    ret,thresh = cv2.threshold(fgmask,127,255,0)
    
    opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
    image,contours, hier = cv2.findContours(closing,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    # look for motion 
    motion_found = False
    biggest_area = 0
    MIN_AREA= 40000
    # examine the contours, looking for the largest one
   
    
    for c in contours:
        (x1, y1, w1, h1) = cv2.boundingRect(c)
        # get an approximate area of the contour
        found_area = w1*h1 
        # find the largest bounding rectangle
        if (found_area > MIN_AREA) and (found_area > biggest_area):  
            biggest_area = found_area
            
            x = x1
            y = y1
            h = h1
            w = w1
            cv2.circle(dst, ((x+w/2),(y+h/2)),8,(0,0,255),-1)
            cv2.rectangle(dst,(x,y),(x+w,y+h),(0,255,0),2)
            dX=x-x2
            dt= t1-t2
            vel = ((np.abs(dX) * ftperpixel)/ np.abs(dt)) * 3.6
            #vel = ((np.abs(dX) * ftperpixel)/ np.abs(dt)) * 0.681818  
            t1= int(time)
            if dX<0 :
              direction = LEFT_TO_RIGHT
              font = cv2.FONT_HERSHEY_SIMPLEX
              text= "left to right"
              cv2.putText(dst,text,((x+w/2),(y+h/2)), font, 2,(0,0,255),3,cv2.LINE_AA)
              #'Waited {} seconds'.format(waited)
              if dt>0:
                text2= '{} kmph'.format(vel)
                #text2= '{} mph'.format(vel)
                cv2.putText(dst,text2,(x,y), font, 2,(0,255,0),3,cv2.LINE_AA)
              else:
                pass
            else:
              direction = RIGHT_TO_LEFT
              font = cv2.FONT_HERSHEY_SIMPLEX
              text= "right to left"
              cv2.putText(dst,text,((x+w/2),(y+h/2)), font, 2,(255,0,0),3,cv2.LINE_AA)
              if dt>0:
                text2= '{} kmph'.format(vel)
                #text2= '{} mph'.format(vel)
                cv2.putText(dst,text2,(x,y), font, 2,(0,255,0),3,cv2.LINE_AA)
              else:
                pass
              


    x2=x1
    t2=t1
    

   


           
  

    
    cv2.imshow('frame',dst)


    # Press Q on keyboard to  exit
    if cv2.waitKey(25) & 0xFF == ord('q'):
      break
 
  # Break the loop
  else: 
    break
 
# When everything done, release the video capture object
cap.release()
 
# Closes all the frames
cv2.destroyAllWindows()
