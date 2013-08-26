#!/usr/bin/env python
import sys
import time  #for delay
import pygst  #for playing mp3 stream
import gst
import os
import sys
import hashlib
import roslib.packages

''' ============================================================== '''
''' Calculates the Offset from the Dart to the Center of the Image '''
''' ============================================================== '''
def get_dart_center_offset(self, req):
    
  dartboard_with_arrow_array = []
  dartboard_without_arrow_array = []
  
  for file in os.listdir(roslib.packages.get_pkg_dir(PACKAGE) + '/test_images/'):
    if fnmatch.fnmatch(file, '*_dartboard_with_arrow*' ):
        dartboard_with_arrow_array.append(file)
    if fnmatch.fnmatch(file, '*_dartbpard_full_grey*' ):   
        dartboard_without_arrow_array.append(file)
        
  
  for image_number in range(len(dartboard_with_arrow_array)):
    
    print "get_dart_center_offset()"
     
    timestamp = str(datetime.datetime.now())
    self.package_dir = roslib.packages.get_pkg_dir(PACKAGE) + '/test_images_output/'+ timestamp + '_'
    
  
    if self.last_reference_picture is None:
      raise Exception("The function take_reference_picture was not called")
  
    dartboard = cv2.imread(dartboard_without_arrow_array[image_number]) 
      
    dartboard_with_arrow = cv2.imread(dartboard_with_arrow_array[image_number])
    ''' Ab hier --- '''  
    #extract circle from the dartboard
    circles = self.detect_circles(dartboard, False)
    detected_middle = self.getAverageCircleMiddle(circles)
  
      
  
    # Convert to GreyScale:
    dartboard = cv2.cvtColor(dartboard, cv2.COLOR_RGBA2GRAY)
    dartboard_with_arrow = cv2.cvtColor(dartboard_with_arrow, cv2.COLOR_RGBA2GRAY)
  
  
    cv2.imwrite(self.package_dir + "dartboard_full_grey.png", dartboard) 
    #TODO: somewhere around here make a circular mask to get rid of the reflections
  
    length = int(self.dartboard_radius_pixel * 2)
      
    print 'length1', length
      
    xStart = detected_middle[0] - self.dartboard_radius_pixel
    yStart = detected_middle[1] - self.dartboard_radius_pixel
  
    xEnd = xStart+length
    yEnd = yStart+length
      
    print 'length2', length
      
    '''TODO check this and the block below if its needed, 
    check why template is not created probably if dartboard is at the side of camera view'''
    #Avoid Error if image is too small to cut out the whole template
    '''
    if xStart > len(dartboard[0]):
      xStart = len(dartboard[0])-1
    if yStart > len(dartboard):
      yStart = len(dartboard)-1
    '''
  
    # Filter negativ Values and set them to zero
    xStart = max(xStart, 0)
    yStart = max(yStart, 0)
  
    template = dartboard[yStart:yEnd, xStart:xEnd]
  
    cv2.imwrite(self.package_dir + "template.png", template)
    cv2.imwrite(self.package_dir + "dartboard_with_arrow.png", dartboard_with_arrow) 
      
    # match template
    try:
      result = cv2.matchTemplate(dartboard_with_arrow,template,cv2.TM_SQDIFF)
    except:
      print "Unexpected error: see template.png and dartboard_with_arrow.png", sys.exc_info()[0]
      return [0,0]
  
    (min_x,max_y,minloc,maxloc)=cv2.minMaxLoc(result)
    (min_loc_x, min_loc_y) = minloc
     
    xEndLoc = min_loc_x+length
    yEndLoc = min_loc_y+length
      
    print 'length3', length
      
    #Avoid Error if image is too small to cut out the whole template
    '''
    if xEnd > len(dartboard_with_arrow[0]):
      xEnd = len(dartboard_with_arrow[0])-1
    if yEnd > len(dartboard_with_arrow):
      yEnd = len(dartboard_with_arrow)-1
    '''
    cut_from_dartboard = dartboard_with_arrow[min_loc_y:yEndLoc, min_loc_x:xEndLoc]
    cv2.imwrite(self.package_dir + "cut_from_dartboard.png", cut_from_dartboard) 
      
  
    div = cv2.absdiff(template, cut_from_dartboard)
    cv2.imwrite(self.package_dir + "div.png", div)
  
    #threshold(src, threshold, pixel_color_if_above_threshold, thresholdType)
    (retval, binary_threshold) = cv2.threshold(div, self.threshold_value, 255, cv2.THRESH_BINARY)  
  
    #binary_pic = cv.fromarray(binary_threshold[1])
    #cv.SaveImage(self.package_dir + "div_threshold.jpg", binary_threshold)
  
    cv2.imwrite(self.package_dir + "binary_threshold.png", binary_threshold)
  
  
    #element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
    #eroded = cv2.erode(binary_threshold, element)
  
    #cv2.imwrite(self.package_dir + "eroded.png", eroded)
  
    #dilated = cv2.dilate(eroded, element)
  
    #cv2.imwrite(self.package_dir + "dilated.png", dilated)
  
    (y_non_zero_array,x_non_zero_array) = np.nonzero(binary_threshold > 0)
      
    if len(x_non_zero_array) > 300:
      element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2,2))
      eroded = cv2.erode(binary_threshold, element)
  
      cv2.imwrite(self.package_dir + "eroded.png", eroded)
        
      (y_non_zero_array,x_non_zero_array) = np.nonzero(eroded > 0)
      rospy.loginfo("Number of detected Pixels: " + str(len(x_non_zero_array)))
    else:
      '''
      0,0 is in the left top corner of the image, min_loc_y are rows, min_loc_x are cols
      '''
      rospy.loginfo("Number of detected Pixels: " + str(len(x_non_zero_array)))
        
        
      if len(x_non_zero_array) < 17:
        print 'No Dart detected, has a Dart been dropped?'
        return [0,0]
  
        
    y_median = np.median(y_non_zero_array)
    x_median = np.median(x_non_zero_array)
      
    print 'x_median' , x_median
  
    xPos = x_median + min_loc_x
    yPos = y_median + min_loc_y
      
    cv2.circle(dartboard_with_arrow,(int(xPos),int(yPos)),10, (255,255, 255),10) #white
  
    print 'x_median + min_loc_x' , x_median
      
    cv2.circle(dartboard_with_arrow,(int(detected_middle[0]),int(detected_middle[1])),10, (255,0, 0),10) #red
      
    cv2.imwrite(self.package_dir + "dartboard_with_detected_arrow.png", dartboard_with_arrow)
    '''
    image = dartboard_with_arrow
      
    image = cv.fromarray(image)
    #cv.SaveImage(self.package_dir + "CircleImage.png", image)
    self.eventImage = image  
    self.eventType = cv.CV_8UC1    
    small = cv.CreateMat(image.rows / 4, image.cols / 4, cv.CV_8UC1)
    print "image:", image
    print "small:", small    
    cv.Resize( image, small);
    '''
      
      
    xOffset = xPos - detected_middle[0]
    yOffset = yPos - detected_middle[1]
      
   
      
    print 'Pixel per meter' , self.pixel_per_meter
    print 'xOffset' , xOffset
    print 'yOffset' , yOffset
      
      
    xOffsetMeter = float(xOffset) / float(self.pixel_per_meter)
    yOffsetMeter = float(yOffset) / float(self.pixel_per_meter)
    
    #set image for stream
    #self.event_image = binary_threshold
      
    #Conversion from image coordinate system to robot coordinate system
    OffsetInRobotFrame = [0,0]
    OffsetInRobotFrame[0] = xOffsetMeter
    OffsetInRobotFrame[1] = -yOffsetMeter
      
    print 'XYOffsetInRObotFrame' , OffsetInRobotFrame
     
    return OffsetInRobotFrame