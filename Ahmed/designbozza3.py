import Rhino.Geometry as rg
import ghpythonlib.treehelpers as th
import random
import math
from copy import deepcopy

"""
Pending:
1. adjust account for movement to create a more homoegeous shape 
2. adjust the t2 parameters when it comes after t3 !!
or make a new assumption that after t3 t1 must come !!
3. add the value of retreat to create a more robust shape in the end (also 
can start from the beginning to account for the angle)
"""

#this class is one of the modules for the impact printing desing of columns
class DesignColumnModule1(object):
    """A generic module of the impact printing that starts with top curve shape
    and 2 (graph mapper values) remapped to fit a given cantilever length on each side to guide the subsequent layers downwards towards reaching the 
    ground and the total required height
    """
    def __init__(self,upper_curve, height, graph_mapper_left_vals, graph_mapper_right_vals,x_spacing, max_limit_t1 = 4, max_limit_t2 = 4,
                 layerheight = 5, part_length = 15):
        self.layer_height = layerheight
        self.part_length = part_length
        self.initial_curve = upper_curve
        self.height = height * 100 #in cms
        self.x_spacing = x_spacing
        self.limit_t1 = max_limit_t1
        self.limit_t2 = max_limit_t2
        self.left_steps = self.transform_graphmapper_vals_into_hzl_steps(graph_mapper_left_vals, 1)
        self.right_steps = self.transform_graphmapper_vals_into_hzl_steps(graph_mapper_right_vals, 1)
        self.left_end_types = self.determine_end_type(self.left_steps)
        self.right_end_types = self.determine_end_type(self.right_steps)
        self.impactlayers = []
        
    

    def transform_graphmapper_vals_into_hzl_steps(self, graph_mapper_vals, type_division =1):
        """
        provides the amount of steps needed on each impact level that approximates global design
        """
        #takes the values and approximates them into the actual spacing converted into number of steps
        if type_division == 1:
            step_val = self.x_spacing /200
            inc_steps_list = []
            prev_val = 0
            #iterate over values and approximates them
            for val in graph_mapper_vals:
                #print(val)
                divided_value = math.ceil(val / step_val)
                rounded_value = max(prev_val + 1, divided_value)
                if ((rounded_value - prev_val) %2 == 0): #makes sure that there will be a smooth shift by an odd number given the material constraint
                    rounded_value -= 1
                inc_steps_list.append(rounded_value) 
                prev_val = rounded_value

            #steps_list = inc_steps_list
            steps_list = transform_incremental_step_into_unitary(inc_steps_list, self.limit_t1 + self.limit_t2)
            return steps_list

    
    
    def create_contour_with_end_type_top_layer(self, ref_point_st, ref_point_end, x_spacing, length, perp_dist_thresh, type1Dist):
        """
        creates contours till we reach the final sides ONLY for first top layer 
        """
        #get the curve lowest and highest points in our direction
        tmin = rg.Curve.ClosestPoint(self.initial_curve, ref_point_st)[1]
        crv_pt_st = self.initial_curve.PointAt(tmin)

        tmax = rg.Curve.ClosestPoint(self.initial_curve, ref_point_end)[1]
        crv_pt_end = self.initial_curve.PointAt(tmax)

        #Create a line joining the extreme points and then getting the midpoint
        #in that case the contour points will propagate from the middle of the shape outwards
        extreme_line = rg.Line(crv_pt_st,crv_pt_end)
        mid_pt = extreme_line.PointAt(0.5)
        #create contours 
        endSideContourPairs = []
        stSideContourPairs = []
        midContour = [] 

        #create contours on one side 
        #adjust the mid_pt level to be on the same level of end1 
        mid_pt_for_end = rg.Point3d(mid_pt.X, crv_pt_end.Y, mid_pt.Z)   
        contour_pts_end = rg.Curve.DivideAsContour(self.initial_curve,mid_pt_for_end, crv_pt_end, x_spacing)
        endSideContourPairs = create_contour_pairs(contour_pts_end)
        insert_end_type_procedure(endSideContourPairs, length, perp_dist_thresh, False, type1Dist)

        mid_pt_for_start = rg.Point3d(mid_pt.X, crv_pt_st.Y, mid_pt.Z) 
        contour_pts_st = rg.Curve.DivideAsContour(self.initial_curve, mid_pt_for_start,crv_pt_st, x_spacing)
        stSideContourPairs = create_contour_pairs(contour_pts_st)
        insert_end_type_procedure(stSideContourPairs, length, perp_dist_thresh, True, type1Dist)

        #initialize an instance of impact layer 
        first_layer = ImpactLayer(1)
        first_layer.startPairs = stSideContourPairs
        first_layer.endPairs = endSideContourPairs
        self.impactlayers.append(first_layer)
        return first_layer,contour_pts_end , contour_pts_st
    
    def determine_end_type(self, steps_list):
        """
        determines the end type in each layer based on the steps_list
        """
        types_numbers = []
        counter = 0
        for index, step in enumerate(steps_list):
            if index == len(steps_list) -1 and step == 1:
                counter += 1
                types_numbers.append(counter)
            elif index == 0 or (step <= 1 and steps_list[index -1] > 1):#first time after type 3 or at beginning of list
                counter += 1
            elif step == 1 and steps_list[index -1] <= 1:
                counter +=1
            elif step > 1 and steps_list[index-1] > 1:
                types_numbers.append(0)
            elif step > 1:
                types_numbers.append(counter)
                counter = 0
                types_numbers.append(0)

        end_type_list = []
        for num in types_numbers:
            if num == 0:
               end_type_list.append("t3")
            elif num == 1:
                end_type_list.append("t2")
            elif num == 2:
                end_type_list.append("t1")
                end_type_list.append("t2")
            else:
                end_of_t1s =  math.floor((self.limit_t1 * num)/ (self.limit_t1 + self.limit_t2))
                for i in range(num):
                    if i < end_of_t1s:
                        end_type_list.append("t1")
                    else:
                        end_type_list.append("t2")

        return end_type_list
                              
    def generate_impact_layers(self, avg_nudge_inside_t2, scissors_step_t1, scissors_step_t2, upper_limit_y_move, lower_limit_y_move, average_step_y):
        """
        Generates all layers based on the topolgy type given from previous calculations:
        """ 
        #start with the start pairs (left)
        startpairs = self.impactlayers[0].startPairs
        for i  in range(1, len(self.left_end_types)):
            #print("i = {0}".format(i))
            #initialize first impact layer -1 coz we start from 1 and it should be even as a start (0)
            new_layer = ImpactLayer(get_even_odd(i-1)) 
            new_layer.move_one_step(startpairs , True, self.x_spacing/2, self.layer_height) 
            startpairs = new_layer.startPairs
            #add the end type for this layer 
            new_layer.apply_end_type_for_module_1(-1, self.left_end_types[i], self.left_end_types[i-1], self.part_length,
                                                    self.x_spacing, avg_nudge_inside_t2, scissors_step_t1, scissors_step_t2,
                                                    upper_limit_y_move, lower_limit_y_move, average_step_y)
            
            self.impactlayers.append(new_layer)

        endpairs = self.impactlayers[0].endPairs
        for i  in range(1, len(self.right_end_types)):
            self.impactlayers[i].move_one_step(endpairs , False, self.x_spacing/2, self.layer_height)
            
            endpairs = self.impactlayers[i].endPairs
            #add the end type for this layer 
            self.impactlayers[i].apply_end_type_for_module_1(1, self.right_end_types[i], self.right_end_types[i-1], self.part_length,
                                                    self.x_spacing, avg_nudge_inside_t2, scissors_step_t1, scissors_step_t2,
                                                    upper_limit_y_move, lower_limit_y_move, average_step_y)
            
            self.impactlayers[i].adjust_points_for_fabrication()


class ImpactLayer(object):

    def __init__(self, odd_even): #0 or 1 for odd_even (odd meaning we have only 1 )
        self.startPairs = []
        self.endPairs = []
        self.even_or_odd = odd_even
        self.impactPoints = []

    def move_one_step(self, list_pairs, isStartPart, moveIncrement, moveDownIncrement):
        """
        Move all points from upper layer into lower layer and adjust for initial checkerboard 
        """
        if isStartPart:
            moveIncrement *= -1
        if self.even_or_odd == 1:
            pointA = rg.Point3d(list_pairs[0][0].X - moveIncrement, list_pairs[0][0].Y ,list_pairs[0][0].Z - moveDownIncrement) 
            pointB = rg.Point3d(list_pairs[0][1].X - moveIncrement, list_pairs[0][1].Y ,list_pairs[0][1].Z - moveDownIncrement) 
            if isStartPart:
                self.startPairs.append([pointA,pointB])
            else:
                self.endPairs.append([pointA,pointB]) 
        for pair in list_pairs:   
                              
            pointA = rg.Point3d(pair[0].X + moveIncrement, pair[0].Y ,pair[0].Z - moveDownIncrement) 
            list_pairs = [pointA] 
            if len(pair) > 1:
                pointB = rg.Point3d(pair[1].X + moveIncrement, pair[1].Y ,pair[1].Z - moveDownIncrement)  
                list_pairs = [pointA, pointB]

            #check if there is a proble in sequencing:
            #if len(pair) > 1 and  (pair[0].Y - pair[1].Y) < 0:
                #print("we have a fucking problem!!")
            if isStartPart:
                self.startPairs.append(list_pairs)
            else:
                self.endPairs.append(list_pairs)

       

    def apply_end_type_for_module_1(self,dir, end_type, end_type_before, length, x_space, avg_nudge_inside_t2, scissors_step_t1, scissors_step_t2,
                                    upper_limit_y_move, lower_limit_y_move, average_step_y):
        """
        dir = direction on same level wether left or right !!
        """
        if dir == 1:
            studied_pairs = self.endPairs
        else:
            studied_pairs = self.startPairs
        
        #print("end type is {0}".format(end_type))

        if end_type == "t1":
            #t1 comes always after t3 or t1 so we dont need to say that again
            #adjust the distance based on even or odd !!            
            self.move_pairs_up_down_scissors(studied_pairs[-2], scissors_step_t1)
            #self.propagate_movement(1, 4, 0.5,studied_pairs[-2], studied_pairs)
        elif end_type == "t2":
            
            if end_type_before != "t2": #we need to apply t2 to the current config
                pointA = rg.Point3d(studied_pairs[-1][0].X, studied_pairs[-1][0].Y + length/2 , studied_pairs[-1][0].Z)
                pointB = rg.Point3d(studied_pairs[-1][0].X, studied_pairs[-1][0].Y - length/2 , studied_pairs[-1][0].Z) 
                studied_pairs[-1] = [pointA, pointB]  

            #print (len(studied_pairs[-1]))    
            #adjust the distance based on even or odd !!
            self.move_pairs_up_down_scissors(studied_pairs[-1], scissors_step_t2)
            #self.propagate_movement(1, 4, 0.5,studied_pairs[-2], studied_pairs)
        elif end_type == "t3":
            #print ("END TYPE IS {0} AND DIR IS {1}".format(end_type, dir) )
            #if end_type_before != "t3":
            pointA = rg.Point3d(studied_pairs[-1][0].X, studied_pairs[-1][0].Y , studied_pairs[-1][0].Z)
            pointB = rg.Point3d(studied_pairs[-1][1].X, studied_pairs[-1][1].Y , studied_pairs[-1][1].Z) 
            studied_pairs[-1] = [pointA, pointB] 
            pointC = rg.Point3d(pointA.X + x_space * dir , pointB.Y + (pointA.Y - pointB.Y)/2 , pointA.Z)
            
            studied_pairs.append([pointC])
                   
            self.move_pairs_up_down_scissors(studied_pairs[-2], scissors_step_t1) 
            
            #!!!! may arise problems later on !!
            #!!! needs further development in case of consecutive t3 but for now ok 
        self.propagate_movement(average_step_y, upper_limit_y_move, lower_limit_y_move,studied_pairs[-2], studied_pairs)



    def propagate_movement(self, actual_step, upper_limit_move, lower_limit_move, edging_pair, studied_pairs):
        """propagates the movement done at the edging pair inwards towards the rest of the elements
        edging pair: the last pair in the module of "t"s that wont be moved
        """
        #get index of the edging pair and start from them
        stop_index = studied_pairs.index(edging_pair) 
        #print("stop index is : {0}".format(stop_index))
        #check first with edging pair:
        previous_pair = edging_pair
        #make a function that prevents the pairs from edging more than a limit and if so then propagate 
        #the movement from the second shit to the next shit has reach the limit, then dont move  on
        upper_i = 1
        lower_i = 1
        #print("stop index is {0}:".format(stop_index))
        levels_upper = stop_index
        levels_lower = stop_index

        for pair in reversed(studied_pairs[:stop_index]):
            
            #print("length of the list is {0}, and the index of the pair is {1}".format(len(studied_pairs), studied_pairs.index(pair)))
            step_upper = calc_lateral_inward_step(lower_limit_move, upper_limit_move, levels_upper, upper_i,  True)
            upper_bullet_move, step_upper = check_lateral_movement_limit(step_upper, pair[0], previous_pair[0], upper_limit_move, )
            step_lower = calc_lateral_inward_step(lower_limit_move, upper_limit_move, levels_lower, lower_i,  True)  
            lower_bullet_move, step_lower = check_lateral_movement_limit(step_lower, pair[1], previous_pair[1], upper_limit_move, False)
            print("step upper is {0}, and step down is {1}".format(step_upper,step_lower))
            #if levels > i:
                #print("there is a prpblem in here reduce by 1")
            if upper_bullet_move:                          
                pair[0].Y += step_upper
                upper_i+=1
            else:
                levels_upper -= 1
        
            if lower_bullet_move:               
                pair[1].Y -= step_lower
                lower_i+=1
            else:
                levels_lower -= 1

            previous_pair = pair
            
  
    def adjust_points_for_fabrication(self, stop_index = -2):
    #choose the type of adjustment but for now just add all the points incrementally
        
        #self.impactPoints.append(pair[0] for pair in self.startPairs[:stop_index]) 
        for pair in self.startPairs[:stop_index]:
            self.impactPoints.append(pair[0])
        for pair in self.startPairs[:stop_index]:
            self.impactPoints.append(pair[1])
        #self.impactPoints.append(pair[1] for pair in self.startPairs[:stop_index]) weirdly enough this didnt work !!
        for pair in self.startPairs[stop_index:]:
            self.impactPoints.append(pair[0])
            if len(pair) > 1:
                self.impactPoints.append(pair[1])    
       
        start_index = self.even_or_odd
        for pair in self.endPairs[start_index:stop_index]:
            self.impactPoints.append(pair[0])
        for pair in self.endPairs[start_index:stop_index]:
            self.impactPoints.append(pair[1])
        #self.impactPoints.append(pair[0] for pair in self.endPairs[start_index:stop_index]) 
        #self.impactPoints.append(pair[1] for pair in self.endPairs[start_index:stop_index]) 
        for pair in self.endPairs[stop_index:]:
            self.impactPoints.append(pair[0])
            if len(pair) > 1:
                self.impactPoints.append(pair[1])   
               

    def replace_last_pair_with_more(self, dir, number_last_): #in the future in case of more than 2 2 be added
        pass

    def move_pairs_up_down_scissors(self, pair, step):
        """
        dir == 1 positive / -1 == negative
        This function follows module 1 column , and makes the "pants scissors",
        it needs a direction +/- based on which the end pairs get either closer or further from each other
        """
        if self.even_or_odd == 0: #(move forward)
            pair[0].Y += step
            pair[1].Y -= step
        else:
            pair[0].Y -= step
            pair[1].Y += step

            
    def move_several():
        pass


def remap(value, old_min, old_max, new_min, new_max):
    old_range = old_max - old_min
    new_range = new_max - new_min
    return (((value - old_min) * new_range) / old_range) + new_min

#!!! needs to be further adjusted as it diest take into consideration if the step is actully in same direction as the y difference or not
def check_lateral_movement_limit(step, bullet,  previous_bullet, limit, is_upper = True):
    move_bullet = True 
    #new function that checks y up and down : 
    if (bullet.Y - previous_bullet.Y) > limit:
            move_bullet = False
            bullet.Y -= (bullet.Y - previous_bullet.Y) - limit
    
    elif (previous_bullet.Y - bullet.Y ) > limit:
            move_bullet = False
            bullet.Y += (previous_bullet.Y - bullet.Y ) - limit
    
    
    if is_upper:       
        if (bullet.Y - previous_bullet.Y) + step >= limit :
            step =  limit - (bullet.Y - previous_bullet.Y) 
        
    #
    else: #(is lower)
       

        if (bullet.Y - previous_bullet.Y) - step <= -limit :
            step =  limit - (previous_bullet.Y - bullet.Y) 


    if step <= 0.3: #!!!hardcoded
        move_bullet = True
    

    print("limit is {0} and the abs. is {1} ".format(limit, abs(bullet.Y - previous_bullet.Y)) )
    return move_bullet, step
         
    
def calc_lateral_inward_step(st_lb, st_hb,levels, currentLevel, reverse = False):
    """
    if reversed then will start by the long steps then the shorter steps would follow in sigmoid way
    """
    if reverse:
        temp = st_lb
        st_lb = st_hb
        st_hb = temp
    trel = currentLevel/levels
    result = st_lb * ( 1/st_lb * st_hb ) ** trel
    #print("result is {0}".format(result))
    return result

def transform_incremental_step_into_unitary(incremental_step_values, max_limit):
    """
    max_limit: limit of end_types after which it is obligatory to increase the width
    step1: takes in the list of incremental steps and generates the new list of the equivalent step at each impact layer
    step2: if a streak of 1s is achieved and exceeds the sum of 1 or 2  
    """
    print("+++++++++++++++++")
    step_per_layer_list = [0] * len(incremental_step_values)
    for i in range(1, len(incremental_step_values)):
        step_per_layer_list[i] = incremental_step_values[i] - incremental_step_values[i - 1]
    n = 0
    prev_item = 1
    for i in range(len(step_per_layer_list)):
        if step_per_layer_list[i] <= 1 and prev_item <= 1 :
            n += 1
            if n == max_limit: #chsnge the value to 3 as obligation to accomodate to local design
                step_per_layer_list[i] = 3
                n = 0
                if i < len(step_per_layer_list) - 2: #meaning that there is an additional value afterwards to check
                    for j in range(i +1, len(step_per_layer_list)): #need to remove the added value we added upstairs from the next one
                        if step_per_layer_list[j] >= 3:
                            step_per_layer_list[j] -= 2
                            break
        else:
            n = 1
        prev_item = step_per_layer_list[i]
        #print (n)
    return step_per_layer_list   

def move_pairs_up_down(pair, step, length, threshold):
    actual_step = step
    #check the distance between the pair Y-Y abs - 1 length shit
    distance = abs(pair[0].Y - pair[1].Y) - length
    #print (distance)
    if distance >= threshold: #we cant add more and need to move to new conf.
        return False, actual_step
    else:
        if distance + 2 * step > threshold:
            actual_step = (threshold - distance) / 2.0
        
        pair[0].Y += actual_step
        pair[1].Y -= actual_step
        return True, actual_step

def move_pairs_up_down2(pair, step, length, threshold, reduction = 0.75):
    actual_step = step
    #check the distance between the pair Y-Y abs - 1 length shit
    distance = abs(pair[0].Y - pair[1].Y) - length * reduction
    #print (distance)
    if distance >= threshold: #we cant add more and need to move to new conf.
        return False, actual_step
    else:
        #if distance + 2 * step > threshold:
            #actual_step = (threshold - distance) / 2.0
        
        pair[0].Y += actual_step
        pair[1].Y -= actual_step
        return True, actual_step

    #if the sum of steps + distance < threshold , then move distance
    #else move remaining of the threshold - distance /2
 

def get_even_odd(number):
        if number % 2 == 0:
            return 0
        else:
            return 1


def create_contour_pairs(contour_pts):
    """Creates pairs of each 2 pts on the same contour line 
    then creates a new list containing them 
    if the contours yield odd number in the end then the last 
    item is appended in the end asa list of 1 item
    """
    contour_pairs = []
    for i in range(0, len(contour_pts),2):
        #check to make sure high point is inserted first:
        if contour_pts[i].Y < contour_pts[i+1].Y:
            pair = [contour_pts[i+1], contour_pts[i]]
        else:
            pair = [contour_pts[i], contour_pts[i+1]]
        contour_pairs.append(pair)

    #if len(contour_pts) % 2 != 0:
        #contour_pairs.append([contour_pts[-1]])

    return contour_pairs

def insert_end_type_procedure(list_pairs, length, converge_thresh, startSide, type1Dist = 10):
    """checks each pair excluding the last pair, updates their position based on stability check 
    and then makes sure that the end pair matches the end type 1 (for now)
    """
    for i in range(len(list_pairs) - 1): #we are excluding the last pair (arbitrary) or the last unit in case of odd num
        move_part_stability_check(list_pairs[i],list_pairs[i+1],converge_thresh)

    #remove the last pair in the list coz we no longer rely on it
    
    #check if now we reach the right distance betwee them for type 1
    readyForInsert = False
    if startSide:
        length *= -1

    last_distance = abs(list_pairs[-1][0].Y - list_pairs[-1][1].Y)
    if last_distance < type1Dist:
        list_pairs.pop(-1)
    #make a loop to add more parts on the sides till reaching type t1
    for i in range(50): #!!!hardcoded
        last_distance = abs(list_pairs[-1][0].Y - list_pairs[-1][1].Y) #since we removed the last pair
        if last_distance <= type1Dist: #add another point
            #check that they dont coincide:
            if last_distance < 10.0: #pt to pt hzl
                val_to_deduct = (10.0 - last_distance)/2.0
                last_distance += val_to_deduct
                list_pairs[-1][0].Y += val_to_deduct
                list_pairs[-1][1].Y -= val_to_deduct
            readyForInsert = True
            break
        else:              
            pointA = rg.Point3d(list_pairs[-1][0].X + length, list_pairs[-1][0].Y - converge_thresh * 0.9, list_pairs[-1][0].Z) #!!!Hardcoded
            pointB = rg.Point3d(list_pairs[-1][1].X + length, list_pairs[-1][1].Y + converge_thresh * 0.9, list_pairs[-1][1].Z) #!!!Hardcoded
            list_pairs.append([pointA, pointB])

    #now we formulate t1
    if readyForInsert:
        #pointCSafe = rg.Point3d(list_pairs[-1][0].X, list_pairs[-1][0].Y -  last_distance / 2.0, list_pairs[-1][0].Z) #!!! not safe just fr small mockup
        pointC = rg.Point3d(list_pairs[-1][0].X + length, list_pairs[-1][0].Y -  last_distance / 2.0, list_pairs[-1][0].Z) 
        list_pairs.append([pointC])


#!!!this function would need further future enhancement to accomodate a smooth curve by variating the curve based on the 
# curvature tangent at this point and comparing it to the maximum 
#!!! this fx misses some lines that check the distance between each pair themselves and makes sure they dont collide  
def move_part_stability_check(pair1, pair2, threshold):
    """Checks all the elements next to each other and then tries to adjust the movement of each one 
    so that eventually we reach the distance needed for implementing T1 
    """
    #check the absolute distance, if greater than threshold make it threshold, otherwise leave as is 
    #upper check
    upper_dist = pair1[0].Y - pair2[0].Y 
    #print(upper_dist)
    if  abs(upper_dist) > threshold:
        if upper_dist > 0: #means the value is positive 
            pair2[0].Y = pair1[0].Y - threshold
        else: #means the value is negative
            pair2[0].Y = pair1[0].Y + threshold
    #lower check
    lower_dist = pair1[1].Y - pair2[1].Y
    if  abs(lower_dist) > threshold:
        if lower_dist> 0: #means the value is positive 
            pair2[1].Y = pair1[1].Y - threshold
        else: #means the value is negative
            pair2[1].Y = pair1[1].Y + threshold

     

    

  
tolerance = 0.001

#unitize the vector of the contour direction
x_dir.Unitize()
y_dir.Unitize()

#re adjust the domain of the curve
curve.Domain = rg.Interval(0,1)

#get the curve lowest and highest points in our direction
tmin = rg.Curve.ClosestPoint(curve, ref_point_st)[1]
crv_pt_st = curve.PointAt(tmin)

tmax = rg.Curve.ClosestPoint(curve, ref_point_end)[1]
crv_pt_end = curve.PointAt(tmax)

column_mod_1 = DesignColumnModule1(curve, height, GM_values_L, GM_values_R,x_spacing, max_consec_t1, max_consec_t2, layer_height )

#apply function that creates the points and contours on the upper most layer:
upper_layer,_,_ = column_mod_1.create_contour_with_end_type_top_layer(crv_pt_st, crv_pt_end, length, x_spacing, perp_dist_thresh, type1Dist)
column_mod_1.generate_impact_layers(2,2,2, upper_limit_y_move, lower_limit_y_move, average_step_y)

#print(column_mod_1.impactlayers)
upper_layer.adjust_points_for_fabrication()

stSideContourPairs = upper_layer.startPairs
endSideContourPairs = upper_layer.endPairs

all_fab_points = []
for layer in column_mod_1.impactlayers:
    all_fab_points.append(layer.impactPoints) 





#print ("this is shit")
# all_fab_points.append(upper_layer.impactPoints)

# prev_layer = upper_layer
# all_layers = []


# for i in range(5):
#     new_layer = ImpactLayer(even_odd(i))
#     new_layer.move_one_step(prev_layer.startPairs , True, 3.5, 5)
#     new_layer.move_one_step(prev_layer.endPairs ,False, 3.5, 5)
#     new_layer.adjust_for_end_type(-1,15, thresh_case1_end,thresh_case2_end, 7,reduction, step_y_expand)
#     new_layer.adjust_for_end_type(1,15, thresh_case1_end, thresh_case2_end,7, reduction, step_y_expand)
#     prev_layer = new_layer
#     all_layers.append(new_layer)

# all_points = []
# all_points.append(upper_layer.startPairs)
# all_points.append(upper_layer.endPairs)


# for layer in all_layers:
#     points_in_layer = []
#     #points_in_layer.append(th.list_to_tree(pair) for pair in layer.startPairs)
#     #points_in_layer.append(th.list_to_tree(pair)  for pair in layer.endPairs)
#     #print(points_in_layer)
#     #all_points.append(points_in_layer) 
#     all_points.append((layer.startPairs)) 
#     all_points.append((layer.endPairs)) 
#     layer.adjust_points_for_fabrication()
#     all_fab_points.append(layer.impactPoints)

# #all_points = [th.list_to_tree(points) for points in all_points]    
# endSideContourPairs = th.list_to_tree(endSideContourPairs)
# stSideContourPairs = th.list_to_tree(stSideContourPairs)  
# all_fab_points = th.list_to_tree(all_fab_points)

# stop_index = 2
# my_list = [1,2,3,4,5]
# start_division = 2
# #iterate of the list
# for item in reversed(my_list[stop_index +1:]):
#     pass #print (item)

steps_list = column_mod_1.transform_graphmapper_vals_into_hzl_steps(GM_values_L)

incremental_list2 = column_mod_1.left_steps
incremental_list1 = column_mod_1.right_steps
inc3 = column_mod_1.determine_end_type(incremental_list2)
#phase1: trying to get the shape to work based on the division and the classes needed  
# there is part of the class that would take over a shape continue slicing it then leaving the last part like the bread.


"""Ideas:
1. the last line of contour insertion whether there will be one or 2 bullets, we could reduce the x-spacing in this one 
to accomodate for the end shape of the bulle t that tends to be filleted and hence might need further intact with the other parts
"""


"""Pendings:
- Graph mappers for everything includeing horizontal distances (done)
- stability check with upper layers (pending)
"""