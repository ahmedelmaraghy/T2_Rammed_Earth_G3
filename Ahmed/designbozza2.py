import Rhino.Geometry as rg
import ghpythonlib.treehelpers as th
import random
import math
from copy import deepcopy


#this class is one of the modules for the impact printing desing of columns
class DesignColumnModule1(object):
    """A generic module of the impact printing that starts with top curve shape
    and 2 guiding curves to guide the subsequent layers downwards towards reaching the 
    ground
    """
    def __init__(self,upper_curve):
        self.initial_curve = upper_curve
    
    def divide_main_curve_into_contours(self, ref_point_st, ref_point_end):
        #re-adjust the domain of the curve
        self.initial_curve.Domain = rg.Interval(0,1)
        
        #get the curve lowest and highest points in our direction
        tmin = rg.Curve.ClosestPoint(self.initial_curve, ref_point_st)[1]
        crv_pt_st = self.initial_curve.PointAt(tmin)

        tmax = rg.Curve.ClosestPoint(self.initial_curve, ref_point_end)[1]
        crv_pt_end = self.initial_curve.PointAt(tmax)

        #Create a line joining the extreme points and then getting the midpoint
        #in that case the contour points will propagate from the middle of the shape outwards
        extreme_line = rg.Line(crv_pt_st,crv_pt_end)
        mid_pt = extreme_line.PointAt(0.5)
        
        #fx that creates contours and checks the list backwards 

    

class ImpactLayer(object):

    def __init__(self, odd_even): #0 or 1 for odd_even (odd meaning we have only 1 )
        self.startPairs = []
        self.endPairs = []
        self.even_or_odd = odd_even

        #self.endShape


    def move_one_step(self, list_pairs, isStartPart, moveIncrement, moveDownIncrement):
        """Move all points half a step to the outer direction
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
            if isStartPart:
                self.startPairs.append(list_pairs)
            else:
                self.endPairs.append(list_pairs)
        
    
    def adjust_for_end_type(self, dir,length , threshold_case_1,threshold_case_2, x_space, step = 1 ):
        #takes the pairs, check the distance (in case last one is 1 brick)
        if dir == 1:
            studied_pairs = self.endPairs
        else:
            studied_pairs = self.startPairs

        if len(studied_pairs[-1]) == 1: #case 1 brick
            
            is_move_possible, actual_step = move_pairs_up_down(studied_pairs[-2],step,length, threshold_case_1)
            if is_move_possible: #we still have room for movement
                pass
                #!!!propagate the movement over all other elements
            else: #!!! we need to move to new config 
                pointA = rg.Point3d(studied_pairs[-1][0].X, studied_pairs[-1][0].Y + length/2, studied_pairs[-1][0].Z)
                pointB = rg.Point3d(studied_pairs[-1][0].X, studied_pairs[-1][0].Y - length/2, studied_pairs[-1][0].Z) 
                studied_pairs[-1] = [pointA, pointB]       

        elif len(studied_pairs[-1]) == 2: #case 2 bricks
            is_move_possible, actual_step = move_pairs_up_down(studied_pairs[-2], step, length, threshold_case_2)
            if is_move_possible:
                pass
            else:
                #additional_move = (threshold_case_1/2.0) * 0.5 #!!!hardcoded 0.5 
                additional_move = 0
                pointA = rg.Point3d(studied_pairs[-1][0].X, studied_pairs[-1][0].Y + additional_move, studied_pairs[-1][0].Z)
                pointB = rg.Point3d(studied_pairs[-1][1].X, studied_pairs[-1][1].Y - additional_move, studied_pairs[-1][1].Z) 
                studied_pairs[-1] = [pointA, pointB] 
                pointC = rg.Point3d(pointA.X + x_space * dir , pointB.Y + (pointA.Y - pointB.Y)/2 , pointA.Z)
                studied_pairs.append([pointC])

        else: #to be further developed
            if is_move_possible: #we still have room for movement
                pass
            else: #!!! we need to move to new config by adding 
               pass
        #if the distance is still not within threshold move all elements one step up and down
        #in theory i should propagate afters 
        #check if threshold is reached ? then replace last shit with 2 shits and keep adding till threshold
        #if threshold reached? add 1 more pair of 1 shit and !!  
        #always propagate the addition to the rest of the particles (increment for loop for now)


    def replace_last_pair_with_more(self, dir, number_last_): #in the future in case of more than 2 2 be added
        pass



            
    def move_several():
        pass

def move_pairs_up_down(pair, step, length, threshold):
    actual_step = step
    #check the distance between the pair Y-Y abs - 1 length shit
    distance = abs(pair[0].Y - pair[1].Y) - length
    print (distance)
    if distance >= threshold: #we cant add more and need to move to new conf.
        return False, actual_step
    else:
        if distance + 2 * step > threshold:
            actual_step = (threshold - distance) / 2.0
        
        pair[0].Y += actual_step
        pair[1].Y -= actual_step
        return True, actual_step



    #if the sum of steps + distance < threshold , then move distance
    #else move remaining of the threshold - distance /2

def even_odd(number):
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
    """checks each pair excluding the last pair, updates theur position based on stability check 
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
            if last_distance < 15.0: #pt to pt hzl
                val_to_deduct = (15.0 - last_distance)/2.0
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

     
#creates contours till we reach the final sides and checks 
def create_contour_with_end_type(curve, ref_point_st, ref_point_end, contours_dir, perp_dir, x_spacing, perp_dist_thresh, type1Dist):
    #get the curve lowest and highest points in our direction
    tmin = rg.Curve.ClosestPoint(curve, ref_point_st)[1]
    crv_pt_st = curve.PointAt(tmin)

    tmax = rg.Curve.ClosestPoint(curve, ref_point_end)[1]
    crv_pt_end = curve.PointAt(tmax)

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
    contour_pts_end = rg.Curve.DivideAsContour(curve,mid_pt_for_end, crv_pt_end, x_spacing)
    endSideContourPairs = create_contour_pairs(contour_pts_end)
    insert_end_type_procedure(endSideContourPairs, 7, perp_dist_thresh, False, type1Dist)

    mid_pt_for_start = rg.Point3d(mid_pt.X, crv_pt_st.Y, mid_pt.Z) 
    contour_pts_st = rg.Curve.DivideAsContour(curve, mid_pt_for_start,crv_pt_st, x_spacing)
    stSideContourPairs = create_contour_pairs(contour_pts_st)
    insert_end_type_procedure(stSideContourPairs, 7, perp_dist_thresh, True, type1Dist)

    return stSideContourPairs, endSideContourPairs,contour_pts_end , contour_pts_st

    #create contour on the other side
    #make a function that checks that each side is comptible with the next bullet in terms of points
    #get the last 2 levels in the list of lists 
    #put on 2 points and check for compatibiility with the range of module 1 
    #if not add more levels that concide with the maximum limit that we have
    #once achieved , insert type 1 
     
#after visual check, start creating classes for these elements and turn the methods into 
#their designated classes
  
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

#apply the function that takes contours and produces the points on both sides
#contour_pts = rg.Curve.DivideAsContour(curve,crv_pt_st, crv_pt_end, x_spacing)
stSideContourPairs, endSideContourPairs, contour_pts_end , contour_pts_st = create_contour_with_end_type(curve, crv_pt_st, crv_pt_end, x_dir, y_dir, x_spacing, perp_dist_thresh, type1Dist)
#check all points and return the non overlap points
#contour_pts = OverlapResolutionSameLevel(contour_pts, threshold_dist,y_dir, decision = 0)

#print ("this is shit")
upper_layer = ImpactLayer(1)
upper_layer.startPairs = stSideContourPairs
upper_layer.endPairs = endSideContourPairs

prev_layer = upper_layer
all_layers = []


for i in range(5):
    new_layer = ImpactLayer(even_odd(i))
    new_layer.move_one_step(prev_layer.startPairs , True, 3.5, 5)
    new_layer.move_one_step(prev_layer.endPairs ,False, 3.5, 5)
    new_layer.adjust_for_end_type(-1,15, thresh_case1_end,thresh_case2_end, 7, step_y_expand)
    new_layer.adjust_for_end_type(1,15, thresh_case1_end, thresh_case2_end,7,  step_y_expand)
    prev_layer = new_layer
    all_layers.append(new_layer)

all_points = []
all_points.append(upper_layer.startPairs)
all_points.append(upper_layer.endPairs)

for layer in all_layers:
    points_in_layer = []
    #points_in_layer.append(th.list_to_tree(pair) for pair in layer.startPairs)
    #points_in_layer.append(th.list_to_tree(pair)  for pair in layer.endPairs)
    print(points_in_layer)
    #all_points.append(points_in_layer) 
    all_points.append((layer.startPairs)) 
    all_points.append((layer.endPairs)) 

#all_points = [th.list_to_tree(points) for points in all_points]    
endSideContourPairs = th.list_to_tree(endSideContourPairs)
stSideContourPairs = th.list_to_tree(stSideContourPairs)  






#phase1: trying to get the shape to work based on the division and the classes needed  
# there is part of the class that would take over a shape continue slicing it then leaving the last part like the bread.


"""Ideas:
1. the last line of contour insertion whether there will be one or 2 bullets, we could reduce the x-spacing in this one 
to accomodate for the end shape of the bullet that tends to be filleted and hence might need further intact with the other parts
"""


"""Pendings:
- Graph mappers for everything includeing horizontal distances
- stability check with upper layers 
"""