from pickle import TRUE
import cv2
import numpy as np
import time
import matplotlib.pyplot as plt


import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32
from geometry_msgs.msg import Point

class StopLineDetect():
    def __init__(self):
        self.bridge = CvBridge() 
        
        R_veh2cam = np.transpose(rotation_from_euler(0., np.radians(7.0), 0.))
        T_veh2cam = translation_matrix((2., 0.0, -1.0))

        # Rotate to camera coordinates
        R = np.array([[0., -1., 0., 0.],
                    [0., 0., -1., 0.],
                    [1., 0., 0., 0.],
                    [0., 0., 0., 1.]])

        self.RT = R @ R_veh2cam @ T_veh2cam
        
        """ ROS """
        rospy.init_node('stopline_detect')
        
        self.img_raw_sub = rospy.Subscriber('/image_raw', Image, self.callback_img_raw)
        
        self.img_stop_line_pub = rospy.Publisher('/stopline_img', Image, queue_size=3)
        self.ate_pub = rospy.Publisher('/at_error_local', Float32, queue_size=10)

        return

    def callback_img_raw(self, img_raw_msg):
        start_time = time.time()  
    
        """ 이미지 받아오기 """
        img_raw_stop = self.bridge.imgmsg_to_cv2(img_raw_msg, desired_encoding='bgr8')

        """ Stop Line detect"""
        stop_img = self.stopline(img_raw_stop)
        
        img_msg_stop = self.bridge.cv2_to_imgmsg(stop_img, encoding="bgr8")
        img_msg_stop.header.stamp = rospy.Time.now()
        self.img_stop_line_pub.publish(img_msg_stop)
        
        end_time = time.time() 
        #print(f"stopline 실행 시간: {end_time - start_time} 초") 
    
    
        return
    

    def inverse_perspective_mapping(self, img):
        
        camera_matrix = np.array([[530.91193822 ,  0.     ,    323.94406173 , 0.],
                                [  0.       ,  480.85427703, 260.42251396, 0.],
                                [  0.     ,      0.     ,      1.      ,0.0  ]])
        
        world_x_max = 10.0
        world_x_min = 1.0
        world_y_max = 2
        world_y_min = -2

        world_x_interval = 0.05 / 2.0
        world_y_interval = 0.08 / 2.0

        # Calculate the number of rows and columns in the output image
        output_width = int(np.ceil((world_y_max - world_y_min) / world_y_interval))
        output_height = int(np.ceil((world_x_max - world_x_min) / world_x_interval))
        
        #print("(width, height) :", "(", output_width, ",",  output_height, ")")
        map_x, map_y = self.generate_direct_backward_mapping(world_x_min, world_x_max, world_x_interval, world_y_min, world_y_max
                                                             , world_y_interval, extrinsic=self.RT, intrinsic = camera_matrix)
        
        output_image = cv2.remap(img, map_x, map_y, cv2.INTER_CUBIC, borderMode=cv2.BORDER_CONSTANT)

        return output_image
    
    def generate_direct_backward_mapping(self,
        world_x_min, world_x_max, world_x_interval, 
        world_y_min, world_y_max, world_y_interval, extrinsic, intrinsic):
        
        world_x_coords = np.arange(world_x_max, world_x_min, -world_x_interval)
        world_y_coords = np.arange(world_y_max, world_y_min, -world_y_interval)
        
        output_height = len(world_x_coords)
        output_width = len(world_y_coords)
        
        world_x_grid, world_y_grid = np.meshgrid(world_x_coords, world_y_coords)
        world_points = np.dstack([world_x_grid, world_y_grid, np.zeros_like(world_x_grid), np.ones_like(world_x_grid)])
        world_points = world_points.reshape(-1, 4).T

        points_c = intrinsic @ (extrinsic @ world_points)
        points_pixel = points_c / points_c[2]
        points_pixel = points_pixel[:2]
        
        map_x = points_pixel[0].reshape((output_height, output_width), order ='F').astype(np.float32)
        map_y = points_pixel[1].reshape((output_height, output_width), order = 'F').astype(np.float32)

        return map_x, map_y

    def detect_stop_line_using_contours(self, binary_img, original_img, min_width_ratio=0.1, max_width_ratio=1, min_area=300):
        contours, _ = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        img_width = binary_img.shape[1]
        valid_contours = []
        center_points = []
        
        for contour in contours:
            # 면적 필터링 추가
            area = cv2.contourArea(contour)
            if area < min_area:
                continue  # 너무 작은 컨투어는 무시
            
            x, y, width, height = cv2.boundingRect(contour)
            
            aspect_ratio = float(width) / height
            if min_width_ratio * img_width < width < max_width_ratio * img_width and aspect_ratio > 1.5:
                valid_contours.append(contour)
                center_x = x + width / 2
                center_y = y + height / 2
                center_points.append((center_x, center_y))
                
                cv2.drawContours(original_img, valid_contours, -1, (0, 255, 0), 2)
            
        return original_img, center_points
     
    
    def stopline(self, img):
        img_raw = img
        #bev_img = self.inverse_perspective_mapping(img_raw)
        bev_img = self.bev(img_raw)
        
        clahe_img = self.clahe(bev_img)
        white_img = self.select_white(clahe_img)
        gray_img = cv2.cvtColor(white_img,cv2.COLOR_BGR2GRAY)
        ret, binary_img = cv2.threshold(gray_img, 200, 255, cv2.THRESH_BINARY)
        #binary_img = binary_img[100:,:]
        #bev_img = bev_img[100:,:]
        
        # contour_img, center_points = self.detect_stop_line_using_contours(binary_img,bev_img)
        # #print(center_points)
        
        # if center_points:  # center_points가 비어 있지 않으면 실행
        #     pixel_coordinates = np.mean(np.array(center_points), axis=0)  # 중심점들의 평균 위치 계산

        #     y_scale = 0.025
            
        #     # pixel_coordinates[1] 대신 pixel_coordinates의 두 번째 값 사용
        #     lane_world_x = y_scale * (bev_img.shape[0] - pixel_coordinates[1])
            
        #     lane_world_x += 4.0
            
        #     #print("y좌표 변환: ", lane_world_x)
            
        #     at_error = lane_world_x
            
        #     ate_msg = Float32()
        #     ate_msg.data = at_error
        #     self.ate_pub.publish(ate_msg)
        # else:
        #     pass
            #print("정지선이 검출되지 않았습니다.")
        # kernel_size = (3, 3)
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, kernel_size)
        # closing = cv2.morphologyEx(gray_img, cv2.MORPH_CLOSE, kernel)
        # canny_img = cv2.Canny(binary_img,50,150)
        
        # lines = cv2.HoughLinesP(canny_img,rho=1, theta=np.pi, threshold=20, minLineLength=100, maxLineGap=10)

        # if lines is not None:
        #     for line in lines:
        #         for x1, y1, x2, y2 in line:
        #             angle = np.abs(np.arctan2(y2 - y1, x2 - x1) * 180. / np.pi)
        #             if angle <= 5:
        #                 cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    

        # histogram = np.sum(closing[:,60:580], axis=1)
        # stop_line_y = np.argmax(histogram)
        
        # if histogram[stop_line_y] > 40000:
        #     cv2.line(bev_img, (100, stop_line_y), (500, stop_line_y), (0, 255, 0), 2)
            
        #     stop_x = 320
        #     stop_y = stop_line_y
            
        #     x_scale = 0.007
        #     y_scale = 0.055

        #     # 'pixel_coordinates'는 변환하려는 픽셀 좌표입니다.
        #     pixel_coordinates = np.array([stop_x, stop_y]) 

        #     # 픽셀 좌표를 미터로 변환합니다.
        #     #meter_coordinates = np.array([pixel_coordinates[0] * pixels_per_meter_x, pixel_coordinates[1] * pixels_per_meter_y])
            
        #     lane_world_x = y_scale * (bev_img.shape[0] - pixel_coordinates[1]) 
        #     lane_world_y = x_scale * (bev_img.shape[1]/2 - pixel_coordinates[0])
            
        #     lane_world_x += 1.5
        
        #     # x축으로 비율 ==> 정지선 10(2.8m):400 pixel => 0.007m : 1pixel
        #     # y축으로 비율 ==> 흰색 점선 7.3(2m): 80pixel => 0.025m : 1pixel
        #     print("x좌표 변환: ", lane_world_y)
        #     print("y좌표 변환: ", lane_world_x)
            
        #     y_error = lane_world_x
            
        #     ye_msg = Float32()
        #     ye_msg.data = y_error
        #     self.ye_pub.publish(ye_msg)

        #print(histogram[stop_line_y])
        
        # plt.plot(histogram)
        # plt.show()
 
        bgr_img = cv2.cvtColor(binary_img,cv2.COLOR_GRAY2BGR)
        
        result_img = np.hstack([bev_img,bgr_img])
        
        
        return result_img
    
    def bev(self, img):
        h,w = img.shape[0],img.shape[1]
   
        # pts1 = np.float32([(222,265), (430,265), (620,350), (30,350)])
        # pts2 = np.float32([(10,40),(90,40),(95,360),(5,360)])
        
        pts1 = np.float32([(250,245), (410,245), (620,350), (30,350)])
        pts2 = np.float32([(10,10),(90,10),(95,360),(5,360)])
        
        mtrx = cv2.getPerspectiveTransform(pts1, pts2)
        result = cv2.warpPerspective(img, mtrx, (100, 360))
        
        return result
        
    def clahe(self,img):
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        Ycrcb = cv2.cvtColor(img,cv2.COLOR_BGR2YCrCb)
        Y,cr,cb = cv2.split(Ycrcb)
        Y_clahe = clahe.apply(Y)
        Ycrcb_clahe = cv2.merge((Y_clahe,cr,cb))
        res = cv2.cvtColor(Ycrcb_clahe,cv2.COLOR_YCrCb2BGR)
        
        return res
    

    def select_white(self,image):

        hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
        h,w = image.shape[0],image.shape[1]
        
        white_lower = np.array([20, 170, 0]) 
        white_upper = np.array([179, 255, 255])

        white_mask = cv2.inRange(hls, white_lower, white_upper)

        white_masked = cv2.bitwise_and(image, image, mask=white_mask)
        
        result_img = cv2.cvtColor(white_masked,cv2.COLOR_HLS2BGR)

        return result_img
    
def rotation_from_euler(roll=1., pitch=1., yaw=1.):
    """
    Get rotation matrix
    Args:
        roll, pitch, yaw:       In radians

    Returns:
        R:          [4, 4]
    """
    si, sj, sk = np.sin(roll), np.sin(pitch), np.sin(yaw)
    ci, cj, ck = np.cos(roll), np.cos(pitch), np.cos(yaw)
    cc, cs = ci * ck, ci * sk
    sc, ss = si * ck, si * sk

    R = np.identity(4)
    R[0, 0] = cj * ck
    R[0, 1] = sj * sc - cs
    R[0, 2] = sj * cc + ss
    R[1, 0] = cj * sk
    R[1, 1] = sj * ss + cc
    R[1, 2] = sj * cs - sc
    R[2, 0] = -sj
    R[2, 1] = cj * si
    R[2, 2] = cj * ci
    return R


def translation_matrix(vector):
    """
    Translation matrix

    Args:
        vector list[float]:     (x, y, z)

    Returns:
        T:      [4, 4]
    """
    M = np.identity(4)
    M[:3, 3] = vector[:3]
    return M
    
if __name__ == "__main__":
    try:
        # ROS
        stopline_detect = StopLineDetect()
        rospy.spin()

    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass