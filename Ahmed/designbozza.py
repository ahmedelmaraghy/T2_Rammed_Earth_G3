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

 
def create_contour_tuples(contour_pts):
    """Creates pairs of each 2 pts on the same contour line 
    then creates a new list containing them 
    if the contours yield odd number in the end then the last 
    item is appended in the end asa list of 1 item
    """
    contour_pairs = []
    for i in range(0, len(contour_pts),2):
        pair = [contour_pts[i], contour_pts[i+1]]
        contour_pairs.append(pair)

    if len(contour_pts) % 2 != 0:
        contour_pairs.append([contour_pts[-1]])

    return contour_pairs


        
def move_part_stability_check():
    pass
        #divide into 2 parts and work on each of them 
#creates contours till we reach the final sides and checks 
def create_contour_with_end_type(curve, ref_point_st, ref_point_end, contours_dir, perp_dir, x_spacing):
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
    endSideContours = []
    strtSideContours = []
    midContour = [] 

    #create contours on one side 
    #adjust the mid_pt level to be on the same level of end1 
    mid_pt_for_end = rg.Point3d(mid_pt.X, crv_pt_end.Y, mid_pt.Z)
    
    contour_pts_end = contour_pts = rg.Curve.DivideAsContour(curve,mid_pt_for_end, crv_pt_end, x_spacing)
    endSideContours = create_contour_tuples(contour_pts_end)
    return contour_pts_end, endSideContours

    #create contour on the other side
    #make a function that checks that each side is comptible with the next bullet in terms of points
    #get the last 2 levels in the list of lists 
    #put on 2 points and check for compatibiility with the range of module 1 
    #if not add more levels that concide with the maximum limit that we have
    #once achieved , insert type 1 
     
#after visual check, start creating classes for these elements and turn the methods into 
#their designated classes




#calculate distance between points and if they are close to a certain level it either takes the average or just take one point
def OverlapResolutionSameLevel(points, threshold_dist, governing_direction, decision = 0):
    deleted_pts = []
    final_pts_list = []
    for i in range(len(points)):
        for j in range(i +1, len(points)):
            vector = points[i] - points[j]
            print (type(rg.Vector3d.Multiply(vector , governing_direction)))
            print (type(vector))
            print (type(governing_direction))           
            distance = abs(vector * governing_direction)
            if distance < threshold_dist:
                print ("warning we have a clash")
                deleted_pts.append(points[i])
    for pt in points:
        if pt in deleted_pts:
            continue
        final_pts_list.append(pt)
    return final_pts_list

            

#hellooo
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
contour_pts, endSideContours = create_contour_with_end_type(curve, crv_pt_st, crv_pt_end, x_dir, y_dir, x_spacing)
#check all points and return the non overlap points
#contour_pts = OverlapResolutionSameLevel(contour_pts, threshold_dist,y_dir, decision = 0)
endSideContours = th.list_to_tree(endSideContours)
print ("this is shit")


#layerTree = th.list_to_tree(layerTree, source=[0,0])
#phase1: trying to get the shape to work based on the division and the classes needed  
# there is part of the class that would take over a shape continue slicing it then leaving the last part like the bread.
# after cutting these parts, we would go for changing the 


