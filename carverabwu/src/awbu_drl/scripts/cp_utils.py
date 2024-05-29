#!/usr/bin/python3
import math
import numpy as np
from scipy.optimize import curve_fit


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

        # print(f'Euclidean Distance Matrix : \n{euclidean}')   
        
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
        #     if euclidean[row_temp[i],col_temp[i]] <= 2:
            associations.append((ID_List[row_temp[i]], col_temp[i]))

        return associations


########### Classifying object

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

def is_oblique_line(points, distance_threshold = 0.01 , percentage_threshold = 0.85):
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

    #  Line
    if is_oblique_line(points):
        return ["WALL", (x,y) , None]


    # Circle
    try :   
        (h,k) , r = Reconstruct_Circle(points) 
        return ["Obstacle", (h,k) , r]
    except : 
        return ["WALL", (x,y) , None]
    

"""
Implementation of the Hungarian (Munkres) Algorithm using Python and NumPy
References: http://www.ams.jhu.edu/~castello/362/Handouts/hungarian.pdf
        http://weber.ucsd.edu/~vcrawfor/hungar.pdf
        http://en.wikipedia.org/wiki/Hungarian_algorithm
        http://www.public.iastate.edu/~ddoty/HungarianAlgorithm.html
        http://www.clapper.org/software/python/munkres/
"""
##https://github.com/tdedecko/hungarian-algorithm/blob/master/hungarian.py

# Module Information.
__version__ = "1.1.1"
__author__ = "Thom Dedecko"
__url__ = "http://github.com/tdedecko/hungarian-algorithm"
__copyright__ = "(c) 2010 Thom Dedecko"
__license__ = "MIT License"


class HungarianError(Exception):
    pass

# Import numpy. Error if fails
try:
    import numpy as np
except ImportError:
    raise HungarianError("NumPy is not installed.")


class Hungarian:
    """
    Implementation of the Hungarian (Munkres) Algorithm using np.

    Usage:
        hungarian = Hungarian(cost_matrix)
        hungarian.calculate()
    or
        hungarian = Hungarian()
        hungarian.calculate(cost_matrix)

    Handle Profit matrix:
        hungarian = Hungarian(profit_matrix, is_profit_matrix=True)
    or
        cost_matrix = Hungarian.make_cost_matrix(profit_matrix)

    The matrix will be automatically padded if it is not square.
    For that numpy's resize function is used, which automatically adds 0's to any row/column that is added

    Get results and total potential after calculation:
        hungarian.get_results()
        hungarian.get_total_potential()
    """

    def __init__(self, input_matrix=None, is_profit_matrix=False):
        """
        input_matrix is a List of Lists.
        input_matrix is assumed to be a cost matrix unless is_profit_matrix is True.
        """
        if input_matrix is not None:
            # Save input
            my_matrix = np.array(input_matrix)
            self._input_matrix = np.array(input_matrix)
            self._maxColumn = my_matrix.shape[1]
            self._maxRow = my_matrix.shape[0]

            # Adds 0s if any columns/rows are added. Otherwise stays unaltered
            matrix_size = max(self._maxColumn, self._maxRow)
            pad_columns = matrix_size - self._maxRow
            pad_rows = matrix_size - self._maxColumn
            my_matrix = np.pad(my_matrix, ((0,pad_columns),(0,pad_rows)), 'constant', constant_values=(0))

            # Convert matrix to profit matrix if necessary
            if is_profit_matrix:
                my_matrix = self.make_cost_matrix(my_matrix)

            self._cost_matrix = my_matrix
            self._size = len(my_matrix)
            self._shape = my_matrix.shape

            # Results from algorithm.
            self._results = []
            self._totalPotential = 0
        else:
            self._cost_matrix = None

    def get_results(self):
        """Get results after calculation."""
        return self._results

    def get_total_potential(self):
        """Returns expected value after calculation."""
        return self._totalPotential

    def calculate(self, input_matrix=None, is_profit_matrix=False):
        """
        Implementation of the Hungarian (Munkres) Algorithm.

        input_matrix is a List of Lists.
        input_matrix is assumed to be a cost matrix unless is_profit_matrix is True.
        """
        # Handle invalid and new matrix inputs.
        if input_matrix is None and self._cost_matrix is None:
            raise HungarianError("Invalid input")
        elif input_matrix is not None:
            self.__init__(input_matrix, is_profit_matrix)

        result_matrix = self._cost_matrix.copy()

        # Step 1: Subtract row mins from each row.
        for index, row in enumerate(result_matrix):
            result_matrix[index] -= row.min()

        # Step 2: Subtract column mins from each column.
        for index, column in enumerate(result_matrix.T):
            result_matrix[:, index] -= column.min()

        # Step 3: Use minimum number of lines to cover all zeros in the matrix.
        # If the total covered rows+columns is not equal to the matrix size then adjust matrix and repeat.
        total_covered = 0
        while total_covered < self._size:
            # Find minimum number of lines to cover all zeros in the matrix and find total covered rows and columns.
            cover_zeros = CoverZeros(result_matrix)
            covered_rows = cover_zeros.get_covered_rows()
            covered_columns = cover_zeros.get_covered_columns()
            total_covered = len(covered_rows) + len(covered_columns)

            # if the total covered rows+columns is not equal to the matrix size then adjust it by min uncovered num (m).
            if total_covered < self._size:
                result_matrix = self._adjust_matrix_by_min_uncovered_num(result_matrix, covered_rows, covered_columns)

        # Step 4: Starting with the top row, work your way downwards as you make assignments.
        # Find single zeros in rows or columns.
        # Add them to final result and remove them and their associated row/column from the matrix.
        expected_results = min(self._maxColumn, self._maxRow)
        zero_locations = (result_matrix == 0)
        while len(self._results) != expected_results:

            # If number of zeros in the matrix is zero before finding all the results then an error has occurred.
            if not zero_locations.any():
                # raise HungarianError("Unable to find results. Algorithm has failed.")
                print("Unable to find results. Algorithm has failed.")
                break

            # Find results and mark rows and columns for deletion
            matched_rows, matched_columns = self.__find_matches(zero_locations)

            # Make arbitrary selection
            total_matched = len(matched_rows) + len(matched_columns)
            if total_matched == 0:
                matched_rows, matched_columns = self.select_arbitrary_match(zero_locations)

            # Delete rows and columns
            for row in matched_rows:
                zero_locations[row] = False
            for column in matched_columns:
                zero_locations[:, column] = False

            # Save Results
            self.__set_results(zip(matched_rows, matched_columns))

        # Calculate total potential
        value = 0
        for row, column in self._results:
            value += self._input_matrix[row, column]
        self._totalPotential = value

    @staticmethod
    def make_cost_matrix(profit_matrix):
        """
        Converts a profit matrix into a cost matrix.
        Expects NumPy objects as input.
        """
        # subtract profit matrix from a matrix made of the max value of the profit matrix
        matrix_shape = profit_matrix.shape
        offset_matrix = np.ones(matrix_shape, dtype=int) * profit_matrix.max()
        cost_matrix = offset_matrix - profit_matrix
        return cost_matrix

    def _adjust_matrix_by_min_uncovered_num(self, result_matrix, covered_rows, covered_columns):
        """Subtract m from every uncovered number and add m to every element covered with two lines."""
        # Calculate minimum uncovered number (m)
        elements = []
        for row_index, row in enumerate(result_matrix):
            if row_index not in covered_rows:
                for index, element in enumerate(row):
                    if index not in covered_columns:
                        elements.append(element)
        min_uncovered_num = min(elements)

        # Add m to every covered element
        adjusted_matrix = result_matrix
        for row in covered_rows:
            adjusted_matrix[row] += min_uncovered_num
        for column in covered_columns:
            adjusted_matrix[:, column] += min_uncovered_num

        # Subtract m from every element
        m_matrix = np.ones(self._shape, dtype=int) * min_uncovered_num
        adjusted_matrix -= m_matrix

        return adjusted_matrix

    def __find_matches(self, zero_locations):
        """Returns rows and columns with matches in them."""
        marked_rows = np.array([], dtype=int)
        marked_columns = np.array([], dtype=int)

        # Mark rows and columns with matches
        # Iterate over rows
        for index, row in enumerate(zero_locations):
            row_index = np.array([index])
            if np.sum(row) == 1:
                column_index, = np.where(row)
                marked_rows, marked_columns = self.__mark_rows_and_columns(marked_rows, marked_columns, row_index,
                                                                           column_index)

        # Iterate over columns
        for index, column in enumerate(zero_locations.T):
            column_index = np.array([index])
            if np.sum(column) == 1:
                row_index, = np.where(column)
                marked_rows, marked_columns = self.__mark_rows_and_columns(marked_rows, marked_columns, row_index,
                                                                           column_index)

        return marked_rows, marked_columns

    @staticmethod
    def __mark_rows_and_columns(marked_rows, marked_columns, row_index, column_index):
        """Check if column or row is marked. If not marked then mark it."""
        new_marked_rows = marked_rows
        new_marked_columns = marked_columns
        if not (marked_rows == row_index).any() and not (marked_columns == column_index).any():
            new_marked_rows = np.insert(marked_rows, len(marked_rows), row_index)
            new_marked_columns = np.insert(marked_columns, len(marked_columns), column_index)
        return new_marked_rows, new_marked_columns

    @staticmethod
    def select_arbitrary_match(zero_locations):
        """Selects row column combination with minimum number of zeros in it."""
        # Count number of zeros in row and column combinations
        rows, columns = np.where(zero_locations)
        zero_count = []
        for index, row in enumerate(rows):
            total_zeros = np.sum(zero_locations[row]) + np.sum(zero_locations[:, columns[index]])
            zero_count.append(total_zeros)

        # Get the row column combination with the minimum number of zeros.
        indices = zero_count.index(min(zero_count))
        row = np.array([rows[indices]])
        column = np.array([columns[indices]])

        return row, column

    def __set_results(self, result_lists):
        """Set results during calculation."""
        # Check if results values are out of bound from input matrix (because of matrix being padded).
        # Add results to results list.
        for result in result_lists:
            row, column = result
            if row < self._maxRow and column < self._maxColumn:
                new_result = (int(row), int(column))
                self._results.append(new_result)


class CoverZeros:
    """
    Use minimum number of lines to cover all zeros in the matrix.
    Algorithm based on: http://weber.ucsd.edu/~vcrawfor/hungar.pdf
    """

    def __init__(self, matrix):
        """
        Input a matrix and save it as a boolean matrix to designate zero locations.
        Run calculation procedure to generate results.
        """
        # Find zeros in matrix
        self._zero_locations = (matrix == 0)
        self._shape = matrix.shape

        # Choices starts without any choices made.
        self._choices = np.zeros(self._shape, dtype=bool)

        self._marked_rows = []
        self._marked_columns = []

        # marks rows and columns
        self.__calculate()

        # Draw lines through all unmarked rows and all marked columns.
        self._covered_rows = list(set(range(self._shape[0])) - set(self._marked_rows))
        self._covered_columns = self._marked_columns

    def get_covered_rows(self):
        """Return list of covered rows."""
        return self._covered_rows

    def get_covered_columns(self):
        """Return list of covered columns."""
        return self._covered_columns

    def __calculate(self):
        """
        Calculates minimum number of lines necessary to cover all zeros in a matrix.
        Algorithm based on: http://weber.ucsd.edu/~vcrawfor/hungar.pdf
        """
        while True:
            # Erase all marks.
            self._marked_rows = []
            self._marked_columns = []

            # Mark all rows in which no choice has been made.
            for index, row in enumerate(self._choices):
                if not row.any():
                    self._marked_rows.append(index)

            # If no marked rows then finish.
            if not self._marked_rows:
                return True

            # Mark all columns not already marked which have zeros in marked rows.
            num_marked_columns = self.__mark_new_columns_with_zeros_in_marked_rows()

            # If no new marked columns then finish.
            if num_marked_columns == 0:
                return True

            # While there is some choice in every marked column.
            while self.__choice_in_all_marked_columns():
                # Some Choice in every marked column.

                # Mark all rows not already marked which have choices in marked columns.
                num_marked_rows = self.__mark_new_rows_with_choices_in_marked_columns()

                # If no new marks then Finish.
                if num_marked_rows == 0:
                    return True

                # Mark all columns not already marked which have zeros in marked rows.
                num_marked_columns = self.__mark_new_columns_with_zeros_in_marked_rows()

                # If no new marked columns then finish.
                if num_marked_columns == 0:
                    return True

            # No choice in one or more marked columns.
            # Find a marked column that does not have a choice.
            choice_column_index = self.__find_marked_column_without_choice()

            while choice_column_index is not None:
                # Find a zero in the column indexed that does not have a row with a choice.
                choice_row_index = self.__find_row_without_choice(choice_column_index)

                # Check if an available row was found.
                new_choice_column_index = None
                if choice_row_index is None:
                    # Find a good row to accomodate swap. Find its column pair.
                    choice_row_index, new_choice_column_index = \
                        self.__find_best_choice_row_and_new_column(choice_column_index)

                    # Delete old choice.
                    self._choices[choice_row_index, new_choice_column_index] = False

                # Set zero to choice.
                self._choices[choice_row_index, choice_column_index] = True

                # Loop again if choice is added to a row with a choice already in it.
                choice_column_index = new_choice_column_index

    def __mark_new_columns_with_zeros_in_marked_rows(self):
        """Mark all columns not already marked which have zeros in marked rows."""
        num_marked_columns = 0
        for index, column in enumerate(self._zero_locations.T):
            if index not in self._marked_columns:
                if column.any():
                    row_indices, = np.where(column)
                    zeros_in_marked_rows = (set(self._marked_rows) & set(row_indices)) != set([])
                    if zeros_in_marked_rows:
                        self._marked_columns.append(index)
                        num_marked_columns += 1
        return num_marked_columns

    def __mark_new_rows_with_choices_in_marked_columns(self):
        """Mark all rows not already marked which have choices in marked columns."""
        num_marked_rows = 0
        for index, row in enumerate(self._choices):
            if index not in self._marked_rows:
                if row.any():
                    column_index, = np.where(row)
                    if column_index in self._marked_columns:
                        self._marked_rows.append(index)
                        num_marked_rows += 1
        return num_marked_rows

    def __choice_in_all_marked_columns(self):
        """Return Boolean True if there is a choice in all marked columns. Returns boolean False otherwise."""
        for column_index in self._marked_columns:
            if not self._choices[:, column_index].any():
                return False
        return True

    def __find_marked_column_without_choice(self):
        """Find a marked column that does not have a choice."""
        for column_index in self._marked_columns:
            if not self._choices[:, column_index].any():
                return column_index

        raise HungarianError(
            "Could not find a column without a choice. Failed to cover matrix zeros. Algorithm has failed.")

    def __find_row_without_choice(self, choice_column_index):
        """Find a row without a choice in it for the column indexed. If a row does not exist then return None."""
        row_indices, = np.where(self._zero_locations[:, choice_column_index])
        for row_index in row_indices:
            if not self._choices[row_index].any():
                return row_index

        # All rows have choices. Return None.
        return None

    def __find_best_choice_row_and_new_column(self, choice_column_index):
        """
        Find a row index to use for the choice so that the column that needs to be changed is optimal.
        Return a random row and column if unable to find an optimal selection.
        """
        row_indices, = np.where(self._zero_locations[:, choice_column_index])
        for row_index in row_indices:
            column_indices, = np.where(self._choices[row_index])
            column_index = column_indices[0]
            if self.__find_row_without_choice(column_index) is not None:
                return row_index, column_index

        # Cannot find optimal row and column. Return a random row and column.
        from random import shuffle

        shuffle(row_indices)
        column_index, = np.where(self._choices[row_indices[0]])
        return row_indices[0], column_index[0]