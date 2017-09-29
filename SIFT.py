# Dependencies
import sys
import rospy
import cv2
import numpy as np
import math
import time

# ROS Sensor messages type 
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError



class Image_Tracker:
    def __init__(self):
        self.pub = rospy.Publisher('objective', Point, queue_size=1000)
        rospy.init_node('objective', anonymous=True)
        self.bridge = CvBridge()
        self.vid = rospy.Subscriber("/camera/rgb/image_color", Image, self.video_callback)
        self.depth = rospy.Subscriber("/camera/depth/image_rect", Image, self.depth_image)
        self.camera_info = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.C_Info)
        self.Z = 1
        self.P = 1

        # images used
        self.cv2_vid = 1
        self.Wanted = 1
        self.QueryImgBGR = 1

        # CInfo -- object coordinates in world
        self.X = 1
        self.Y = 1
        # depth_image -- object coordinates in world
        self.Z = 1

        # slider value
        self.Tracking = False

        # ROI image selection
        self.freeze = False
        self.refPt = (0,0)

        # SIFT tracking -- object coordinates in px
        self.object_x = 0
        self.object_y = 0
        

    def video_callback(self,msg):
	# Convert Ros video input into opencv image 
        if not self.freeze:        
            self.cv2_vid = self.bridge.imgmsg_to_cv2(msg,'bgr8')

            cv2.imshow("image", self.cv2_vid)
            switch = '0 : OFF \n1 : ON'
            cv2.createTrackbar(switch, "image",0,1,self.Slider_Callback)
            cv2.setMouseCallback("image",self.Selected_ROI)

        if not self.Tracking:
            cv2.destroyWindow('result')
        else:
            # Display location of object 
            self.SIFT()
            Position = "x:{0} y:{1} z:{2}".format(str(self.object_x), str(self.object_y), str(self.Z))
            Font = cv2.FONT_HERSHEY_COMPLEX_SMALL
            cv2.putText(self.QueryImgBGR, Position, (10,450), Font, 0.5,(255,255,255),1,cv2.LINE_AA)
            # Open the result windows where object is detected
            cv2.imshow("result",self.QueryImgBGR)

	    
        cv2.waitKey(100)

    def Selected_ROI(self,event, x, y, flags, param):
	# get (x, y) when mouse drag 
	# draw a rectangle ROI
        
        if event == cv2.EVENT_LBUTTONDOWN:
            self.freeze = True
            self.refPt = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            XLow = min(self.refPt[0], x)
            XHigh = max(self.refPt[0], x)
            YLow = min(self.refPt[1], y)
            YHigh = max(self.refPt[1], y)

            clone = self.cv2_vid.copy()
            roi = clone[YLow:YHigh, XLow:XHigh]
            cv2.imshow("Object Tracked", roi)
            self.Wanted = roi
            self.freeze = False

        elif self.freeze:
            frame = self.cv2_vid.copy()
            cv2.rectangle(frame, self.refPt, (x, y), (0, 255, 0), 2)
            cv2.imshow("image", frame)

    def SIFT(self):
        
            vid_s = self.cv2_vid.copy()
            
            MIN_MATCH_COUNT=30

            # Load SIFT function
            detector=cv2.xfeatures2d.SIFT_create()

            # Parameter for the KNN classifier
            FLANN_INDEX_KDITREE=2
            flannParam=dict(algorithm=FLANN_INDEX_KDITREE,tree=5)
            flann=cv2.FlannBasedMatcher(flannParam,{})

            # load the wanted image for the classifier
            trainKP,trainDesc=detector.detectAndCompute(self.Wanted, None)
            flann.add([trainDesc])
            flann.train()
            
            ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##
            
              
            QueryImg=cv2.cvtColor(vid_s,cv2.COLOR_BGR2GRAY)
        
            
    
            queryKP,queryDesc=detector.detectAndCompute(QueryImg,None)
       
            matches=flann.knnMatch(queryDesc,k=2)


            # find good matched based on ratio test
            goodMatch=[]
            for m,n in matches:
                if(m.distance<0.75*n.distance):
                    goodMatch.append(m)
            if(len(goodMatch)>MIN_MATCH_COUNT):
                tp=[]
                qp=[]
                for m in goodMatch:
	            tp.append(trainKP[m.trainIdx].pt)
                    qp.append(queryKP[m.queryIdx].pt)
                tp,qp=np.float32((tp,qp))

	            # Draw bounding box
                H,status=cv2.findHomography(tp,qp,cv2.RANSAC,3.0)
                
                h, w, _ = self.Wanted.shape
                trainBorder=np.float32([[[0,0],[0,h-1],[w-1,h-1],[w-1,0]]])
                queryBorder=cv2.perspectiveTransform(trainBorder,H)
                cv2.polylines(vid_s,[np.int32(queryBorder)],True,(0,255,0),5)

                self.object_x, self.object_y = np.mean(queryBorder[0],0).tolist()

                cv2.putText(vid_s, "Selected object",(50, 50),cv2.FONT_HERSHEY_COMPLEX_SMALL,2,(0,0,255))
                self.QueryImgBGR = vid_s

                # publishs x,y,z coordinates 
                self.pub.publish(self.object_x,self.object_y,self.Z)
          
            else:
                print "Not Enough match found- %d/%d"%(len(goodMatch),MIN_MATCH_COUNT)

    

    def depth_image(self, msg):
        D_Img = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        np.clip(D_Img, 0, 2, D_Img)
        self.Z = D_Img[int(self.object_y)][int(self.object_x)]
        
            

    def C_Info(self, msg):
        P = msg.P
        # return focal lenght and centre of Image 
        self.X = (self.object_x - P[2]) / P[0]
        self.Y = (self.object_y- P[6]) / P[5]

    def Slider_Callback(self, slider_value):
        if slider_value == 1:
            print('Tracking Object')
            self.Tracking = True
        else:
            self.Tracking = False
            print('Not Tracking')


        

def main(args):
    ic = Image_Tracker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


################## ############################### ########################## 


if __name__ == '__main__':
    main(sys.argv)

