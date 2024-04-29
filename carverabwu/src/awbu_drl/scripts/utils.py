#!/usr/bin/python3

from scipy.optimize import linear_sum_assignment
import math
import numpy as np
import copy



def transform_local_to_global(robot_pose,local_point):
        """
        Transform coordinates from the local frame of a mobile robot to the global frame.

        Parameters:
            robot_pose (tuple): (x, y, theta) representing the robot's pose in the global frame.
                                (x, y) are the coordinates of the robot's position,
                                and theta is the orientation angle (in radians).
            local_point (tuple): (x_local, y_local) representing the coordinates of a point
                                in the robot's local frame.

        Returns:
            tuple: (x_global, y_global) representing the coordinates of the point in the global frame.
        """
        x_robot, y_robot, yaw = robot_pose

        theta_robot = yaw
        x_local, y_local = local_point

        # Transformation matrix from local frame to global frame
        T = np.array([[np.cos(theta_robot), -np.sin(theta_robot), x_robot],
                    [np.sin(theta_robot), np.cos(theta_robot), y_robot],
                    [0, 0, 1]])

        # Homogeneous coordinates of the local point
        local_homogeneous = np.array([[x_local], [y_local], [1]])

        # Transform the local point to global coordinates
        global_homogeneous = np.dot(T, local_homogeneous)

        # Convert homogeneous coordinates to Cartesian coordinates
        x_global = global_homogeneous[0, 0]
        y_global = global_homogeneous[1, 0]

        return (x_global, y_global)

import numpy as np

def transform_global_to_local(robot_pose, global_point):
    """
    Transform coordinates from the global frame to the local frame of a mobile robot.

    Parameters:
        robot_pose (tuple): (x, y, theta) representing the robot's pose in the global frame.
                            (x, y) are the coordinates of the robot's position,
                            and theta is the orientation angle (in radians).
        global_point (tuple): (x_global, y_global) representing the coordinates of a point
                            in the global frame.

    Returns:
        tuple: (x_local, y_local) representing the coordinates of the point in the robot's local frame.
    """
    global_x = global_point[0]
    global_y = global_point[1]

    robot_x , robot_y , robot_theta = robot_pose

    delta_x = global_x - robot_x
    delta_y = global_y - robot_y
    
    # Rotation
    local_x = delta_x * math.cos(-robot_theta) - delta_y * math.sin(-robot_theta)
    local_y = delta_x * math.sin(-robot_theta) + delta_y * math.cos(-robot_theta)

    return (local_x, local_y)




def Clustering(scan_in , dth = 0.2):
    """
        Clustering laser_scan to Group in local frame.

        Parameters:  
        scan_in: scan range 
        dth    : thresold ranges
    """
    clusters = []
    scan = scan_in
    dth = dth
    c_points = 0
    
    # Find the number of non inf laser scan values and save them in c_points
    for i in range(len(scan.ranges)):
        if not math.isinf(scan.ranges[i]):
            c_points += 1
    
    polar = [[0.0, 0.0] for _ in range(c_points + 1)]  # c_points+1 for wrapping

    j = 0
    for i in range(len(scan.ranges)):
        if not math.isinf(scan.ranges[i]):
            polar[j][0] = scan.ranges[i]  # first column is the range 
            polar[j][1] = scan.angle_min + i * scan.angle_increment  # second angle in rad
            j += 1
    
    # Complete the circle
    polar[-1][0] = polar[0][0]
    polar[-1][1] = polar[0][1]

    clustered1 = [False] * (c_points + 1)  # change to true when it is the first of the cluster
    clustered2 = [False] * (c_points + 1)  # change to true when it is clustered by another one

    for i in range(c_points):
        d = math.sqrt(polar[i][0]**2 + polar[i+1][0]**2 - 2 * polar[i][0] * polar[i+1][0] * math.cos(polar[i+1][1] - polar[i][1]))
        if d < dth:
            clustered1[i] = True  # both points belong to clusters
            clustered2[i+1] = True

    # If the last(first also) point is clustered and the first is not, make the first clustered
    if clustered2[-1] and not clustered2[0]:
        clustered2[0] = True
        clustered1[0] = False

    begin = []
    nclus = []
    i = 0
    flag = True

    while i < c_points and flag:
        if clustered1[i] and not clustered2[i] and flag:
            begin.append(i)
            nclus.append(1)
            while i+1 < c_points and clustered2[i+1] and clustered1[i+1]:
                i += 1
                nclus[-1] += 1
                if i == c_points-1 and flag:
                    i = -1
                    flag = False
            nclus[-1] += 1
        i += 1

    polar.pop()  # remove the wrapping element
    len_polar = len(polar)

    for i in range(len(begin)):
        cluster = []
        j = begin[i]
        flag = True
        while j < nclus[i] + begin[i]:
            if j == len_polar and flag:
                flag = False
            if flag:
                x =  polar[j][0] * math.cos(polar[j][1])
                y =  polar[j][0] * math.sin(polar[j][1])
            else:
                x =  polar[j-len_polar][0] * math.cos(polar[j-len_polar][1])
                y =  polar[j-len_polar][0] * math.sin(polar[j-len_polar][1])
            cluster.append((x, y))
            j += 1
        clusters.append(cluster)

    return clusters

# def laser_scan_to_coordinates(laser_scan_msg):
#     """
#     Convert a LaserScan message to a list of 2D coordinates (x, y).
    
#     Parameters:
#         laser_scan_msg (sensor_msgs.msg.LaserScan): The LaserScan message to convert.

#     Returns:
#         list: A list of 2D coordinates (x, y).
#     """
#     coordinates = []
#     angle_min = laser_scan_msg.angle_min
#     angle_increment = laser_scan_msg.angle_increment
#     range_values = laser_scan_msg.ranges

#     for i, range_value in enumerate(range_values):
#         angle = angle_min + angle_increment * i
#         x = range_value * np.cos(angle)  # Assuming laser is mounted at the origin
#         y = range_value * np.sin(angle)  # Assuming laser is mounted at the origin
#         coordinates.append((x, y))

#     return coordinates



# def Clustering(scan_in , Dmax = 0.4):

#     pos = laser_scan_to_coordinates(scan_in)
#     cluster = []
#     group = []
#     for i in range(1,len(pos)) : 
        
#         pn = np.array(pos[i])
#         pn_1 = np.array(pos[i-1])
        
#         ## Euclidean
#         dist = np.linalg.norm(pn - pn_1)

#         if dist > Dmax :
#             group.append((pn_1[0],pn_1[1]))
#             cluster.append(group)
#             group = []

#         else : group.append((pn_1[0],pn_1[1]))
    
#     #for last point
#     pn      = np.array(pos[0])
#     pn_1    = np.array(pos[-1])
#     pn_2    = np.array(pos[-2])

#     dist1   = np.linalg.norm(pn - pn_1)
#     dist2   = np.linalg.norm(pn_2 - pn_1)

#     if dist1 < Dmax and dist2 < Dmax : 
#         temm = []
#         temm += cluster[0] + cluster[-1]
#         temm.append((pn_1[0],pn_1[1]))
#         del cluster[0]
#         del cluster[-1]
#         cluster.append(temm)
#     elif dist1 < Dmax : cluster[0].append((pn_1[0],pn_1[1]))
#     elif dist2 < Dmax : cluster[-1].append((pn_1[0],pn_1[1]))
#     else : cluster.append([(pn_1[0],pn_1[1])])

#     return cluster


from Hungarian import Hungarian 

def association(previous_group,group):
        '''
        Check association of object in previous frame and current frame
        '''
        
        previous_mean = []
        current_mean  = []
        ID_List = []
        for g in previous_group:
            if previous_group[g].type == "Obstacle" : 
                x = sum([x[0] for x in previous_group[g].position])/len(previous_group[g].position)
                y = sum([x[1] for x in previous_group[g].position])/len(previous_group[g].position)
                previous_mean.append((x,y))

                ID_List.append(previous_group[g].ID)

        for g in group:
            current_mean.append(g)

        euclidean = np.zeros((len(previous_mean),len(group)))
        

        for i in range(len(previous_mean)):
            for j in range(len(group)):
                euclidean[i, j] = np.linalg.norm(np.array(previous_mean[i]) - np.array(current_mean[j]))

                
        hungarian = Hungarian(euclidean)
        hungarian.calculate()
        t = hungarian.get_results()

        row_temp = []
        col_temp = []
        for r , c in t :
            row_temp.append(r)
            col_temp.append(c)

        associations = []
        for i in range(len(row_temp)):
            if euclidean[row_temp[i],col_temp[i]] <= 2:
                associations.append((ID_List[row_temp[i]], col_temp[i]))

        return associations


########### Classifying object

def check_line_conditions(slopes , thres_vertical=80 , thres_horizontal=0.01, error_ =  0.1 , thres_t = 0.9):
    '''
    thres_vertical  :  threshold of grad for vertical line
    thres_horizontal : threshold of grad for horizontal line
    error_           : threshold of error -> all of grad should similar if it's line
    thres_t          : threshold for proportion -> to check most of all value should have huge value if it's vertical line. horizontal line otherwise
    '''
    check = 0
    check2 = 0
    # print(max(slopes))
    for i in slopes : 
        if abs(i) > thres_vertical : check +=1
        if abs(i) < thres_horizontal : check2 +=1

    # print("1," , check)
    # print("2," , check2)
    if check >= len(slopes)//3 or check2 >= len(slopes)//3 : return True


    slopes = np.array(slopes)
    num_elements = len(slopes)

    greater         = np.sum(np.abs(slopes) > thres_vertical)
    less            = np.sum(np.abs(slopes) < thres_horizontal)

    abs_mean = np.abs(np.mean(slopes))
    similarly       = np.sum(np.abs(np.abs(slopes) - abs_mean)) < error_
    
    proportion_greater = greater / num_elements
    proportion_less = less / num_elements
    proportion_similarly = similarly / num_elements
    if proportion_less > thres_t or proportion_greater > thres_t or proportion_similarly > thres_t:
        return True
    else:
        return False
    

    
def is_line(points):
    """
    Check if the given points form a straight line.

    Parameters:
    - points: A list of tuples representing the (x, y) coordinates of the points.

    Returns:
    - True if the points form a straight line, False otherwise.
    """
    if len(points) < 5:
        return True
    
    # Calculate the slopes between consecutive points
    slopes = []
    for i in range(len(points)-1):
        dx = points[i+1][0] - points[i][0]
        if dx == 0:
            slope = np.inf  # vertical line
        else:
            slope = (points[i+1][1] - points[i][1]) / dx

        slopes.append(slope)
    
    return check_line_conditions(slopes)


def fit_line_to_points(points):
    # Extract x and y coordinates from the list of points
    x_coords, y_coords = zip(*points)
    
    # Perform linear regression to fit a line to the points
    A = np.vstack([x_coords, np.ones(len(x_coords))]).T
    m, c = np.linalg.lstsq(A, y_coords, rcond=None)[0]
    
    return m, c

def calculate_distance(point, m, c):
    # Calculate the distance of a point to the line defined by the slope (m) and intercept (c)
    x, y = point
    return abs(y - (m * x + c)) / np.sqrt(1 + m**2)

def is_oblique_line(points, distance_threshold = 0.01 , percentage_threshold = 0.8 ):
    # Fit a line to the points
    ###### 

    all_point = list(points)
    front_half = list(points)[:int(len(points)//2)]
    back_half = list(points)[int(len(points)//2):]

    m1, c1 = fit_line_to_points(all_point)

    m2, c2 = fit_line_to_points(front_half)

    m3, c3 = fit_line_to_points(back_half)
    
    # Calculate distances of all points to the line
    distances_all = [calculate_distance(point, m1, c1) for point in all_point]
    distances_front_front = [calculate_distance(point, m2, c2) for point in front_half]
    distances_back_back = [calculate_distance(point, m3, c3) for point in back_half]

    #### use slope of front_half to check back
    distances_back_front = [calculate_distance(point, m2, c2) for point in back_half]

    #### use slope of back_half to check front
    distances_front_back = [calculate_distance(point, m3, c3) for point in front_half]

    # Count the number of points within the distance threshold
    num_inliers_all = sum(1 for dist in distances_all if dist < distance_threshold)
    num_inliers_front_front = sum(1 for dist in distances_front_front if dist < distance_threshold)
    num_inliers_back_back = sum(1 for dist in distances_back_back if dist < distance_threshold)
    num_inliers_back_front = sum(1 for dist in distances_back_front if dist < distance_threshold)
    num_inliers_front_back = sum(1 for dist in distances_front_back if dist < distance_threshold)

    
    # Calculate the percentage of inliers
    inlier_percentage_all = num_inliers_all / len(all_point)
    inlier_percentage_back_back = num_inliers_back_back / len(back_half)
    inlier_percentage_front_front  = num_inliers_front_front / len(front_half)
    inlier_percentage_back_front  = num_inliers_front_back / len(back_half)
    inlier_percentage_front_back  = num_inliers_back_front / len(front_half)
    

    return inlier_percentage_all > percentage_threshold \
            or inlier_percentage_back_back > percentage_threshold \
            or inlier_percentage_front_front > percentage_threshold\
            or inlier_percentage_back_front > percentage_threshold \
            or inlier_percentage_front_back > percentage_threshold


from scipy.optimize import curve_fit

############## Reconstruct Circle #############

def circle_equation(x, a, b, r):
    return np.sqrt((x[0]-a)**2 + (x[1]-b)**2) - r


def Reconstruct_Circle(points):
    # Define the equation of a circle

    # Extract x and y coordinates from the points
    x_data = [point[0] for point in points]
    y_data = [point[1] for point in points]

    # Initial guess for circle parameters (center and radius)
    x0 = np.mean(x_data)
    y0 = np.mean(y_data)
    r0 = np.mean(np.sqrt((np.array(x_data) - x0)**2 + (np.array(y_data) - y0)**2))

    # Fit the circle using least squares method
    popt, pcov = curve_fit(circle_equation, (x_data, y_data), np.zeros(len(points)), p0=(x0, y0, r0))

    # Extract fitted circle parameters
    a_fit, b_fit, r_fit = popt

    return (a_fit, b_fit), r_fit



def classify_object(points):
    # if is_L_shape(points):

    x = sum([i[0] for i in points])/len(points)
    y = sum([i[1] for i in points])/len(points)

    #     print(points)
    if is_oblique_line(points):
        return ["WALL", (x,y) , None]

    # else :
    try : 
        (h,k) , r = Reconstruct_Circle(points) 
        return ["Obstacle", (h,k) , r]
    except : 

        return ["WALL", (x,y) , None]
    


######### Kalman filter Constant Velocity ########

##https://github.com/NickNair/Multiple-Object-Tracking-using-Kalman-Filter
##https://github.com/zziz/kalman-filter


class KalmanFilter(object):
    def __init__(self, F = None, B = None, H = None, Q = None, R = None, P = None, x0 = None):

        if(F is None or H is None):
            raise ValueError("Set proper system dynamics.")

        self.n = F.shape[1]
        self.m = H.shape[1]

        self.F = F
        self.H = H
        self.B = 0 if B is None else B
        self.Q = np.eye(self.n) if Q is None else Q
        self.R = np.eye(self.n) if R is None else R
        self.P = np.eye(self.n) if P is None else P
        self.x = np.zeros((self.n, 1)) if x0 is None else x0

    def predict(self, u = 0):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.n)
        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P), 
        	(I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)