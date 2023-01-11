import Rhino.Geometry as rg
import ghpythonlib.treehelpers as th
import random
import math
from copy import deepcopy


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
contour_pts = rg.Curve.DivideAsContour(curve,crv_pt_st, crv_pt_end, x_spacing)

#check all points and return the non overlap points
contour_pts = OverlapResolutionSameLevel(contour_pts, threshold_dist,y_dir, decision = 0)

print ("this is shit")

#rg.Curve.DivideEquidistant()