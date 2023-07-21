#!/usr/bin/env python
import rospy
import path_planner
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from tf import TransformListener, ExtrapolationException, LookupException
from tf2_msgs.msg import TFMessage
from path_planner.bspline_curve import calccccc , mid_point
from geometry_msgs.msg import  Point
import numpy as np
from nav_msgs.msg import GridCells
from std_msgs.msg import ColorRGBA

class PathPlanner:
    def __init__(self):
        self.map = None
        self.start = None
        self.goal = None
        self.node = path_planner.Node
        self.mover = path_planner.GotoMover(self)
        self.is_goal_cancelled = False
        self.is_goal_reached = False
        self.is_map_loaded = False

        self.map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.start_subscriber = rospy.Subscriber("/tf", TFMessage, self.start_callback)
        self.goal_subscriber = rospy.Subscriber("/goal", PoseStamped, self.goal_callback)
        self.transform_listener = TransformListener()

        self.path_publisher = rospy.Publisher("/path", Path, queue_size=10)
        self.path_publisher_1 = rospy.Publisher("/path1", Path, queue_size=10)
        self.mover_publisher = rospy.Publisher("/move", Path, queue_size=10)
        self.grid_cell_1 = rospy.Publisher("/cell1",GridCells,queue_size=10)
        self.grid_cell_2 = rospy.Publisher("/cell2",GridCells,queue_size=10)

    def map_callback(self, data: OccupancyGrid):
        self.map = path_planner.Map(data)
        self.is_map_loaded = True

    
    def start_callback(self, data: TFMessage):
        try:
            position, quaternion = self.transform_listener.\
                lookupTransform("/map", "/base_link", rospy.Time())
        except (ExtrapolationException, LookupException):
            return

        self.start = path_planner.Node.from_tf(position, quaternion)
        self.mover.robot_position = self.start

    def goal_callback(self, data: PoseStamped) -> bool:
        if self.mover.is_moving:
            self.cancel_goal()

        rospy.loginfo("Received new goal")
        self.goal = path_planner.Node.from_pose(data.pose)
        self.is_goal_cancelled = False
        self.is_goal_reached = False

        if not self.map.is_node_free(self.goal) or not self.map or not self.start:
            rospy.loginfo("Goal can't be reached")
            self.display_path([])  # Clearing path
            return False

        return self.calculate_path()
    def rebuild(self,path_original):
        rebuild_path =[]
        for i in path_original:
            rebuild_path.append(i)
        z= (len(rebuild_path)-2)
        while z > 2 :
            
            a= [rebuild_path[z].x,rebuild_path[z-2].x]
            b= [rebuild_path[z].y,rebuild_path[z-2].y]

            if self.checkline(a,b)== True:
                # rebuild_path[z].parent = rebuild_path[z-2]
                rebuild_path.remove(rebuild_path[z-1])
            z =z -1
                
            
        z= (len(rebuild_path)-1)
        print('node count: ' + str(len(rebuild_path)))
        return rebuild_path
    def rebuild_1(self,path_original):
        rebuild_path =[]
        for i in path_original:
            rebuild_path.append(i)
        z= (len(rebuild_path)-2)
        while z > 1 :
            
            a= [rebuild_path[z].x,rebuild_path[z-2].x]
            b= [rebuild_path[z].y,rebuild_path[z-2].y]

            if self.checkline(a,b)== True:
                # rebuild_path[z].parent = rebuild_path[z-2]
                rebuild_path.remove(rebuild_path[z-1])
            z =z -1
        z= (len(rebuild_path)-1)
        add_midpoint = []
        for i in range(0,z-1,1):
            add_midpoint.append(rebuild_path[i])
            add_midpoint.append(mid_point(rebuild_path[i],rebuild_path[i+1]))
        add_midpoint.append(rebuild_path[-1])
        print('node count: ' + str(len(add_midpoint)))
        return add_midpoint

    def calculate_path(self):
        rospy.loginfo("Calculating path...")
        path_original = path_planner.find_path(self.map, self.start, self.goal)
        
        m = self.display_path_1(path_original)
        print('node original path')
        print(len(path_original))
        rebuild_path = self.rebuild(path_original)
        # add_midpoint = self.rebuild_1(rebuild_path)

        m = self.display_path(rebuild_path)
        print('node rebuild path')
        print(len(rebuild_path))
        if len(rebuild_path) > 0:
            rospy.loginfo("Path calculated")
            path_list = calccccc(rebuild_path)
            m = self.display_path
            path_msg =self.display_path_notshowpath(path_list)
            self.mover_publisher.publish(path_msg)
            self.display_risk_1(path_original)
            self.display_risk_2(path_original)
            # path_show = self.display_test(path_list,rebuild_path)
            return True
        rospy.loginfo("No path found")
        self.display_path([])
        
        return False

    def wait_for_map(self):
        while not self.is_map_loaded and not rospy.is_shutdown(): 
            rospy.sleep(0.2)
    
    def wait_for_result(self, duration: float) -> bool:
        sending_time = rospy.Time.now().to_sec()
        rospy.sleep(1)

        while self.mover.is_moving and not self.is_goal_cancelled and not rospy.is_shutdown():
            if rospy.Time.now().to_sec() - sending_time > duration:
                self.cancel_goal()
                return False
        
        return self.is_goal_reached
    
    def send_goal(self, pose: Pose) -> bool:
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose = pose

        return self.goal_callback(goal)

    def cancel_goal(self):
        self.mover.initialize_stop()
        self.is_goal_cancelled = True

    
    def moving_average(self, path: list, window: int = 4) -> list:
        window_queue = []
        smoothed_path = [path[0]]

        for node in path:
            if len(window_queue) == window:
                smoothed_path.append(sum(window_queue) / window)  # Mean
                window_queue.pop(0)

            window_queue.append(node)
        print('smoothed_path')
        for item in smoothed_path:
            print([item.x,item.y])

        return smoothed_path + [self.goal]

    def display_path(self, path_nodes: list) -> Path:
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        for node in path_nodes:
            path.poses.append(node.to_pose_stamped())

        self.path_publisher.publish(path)

        return path
    def display_path_1(self, path_nodes: list) -> Path:
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        for node in path_nodes:
            path.poses.append(node.to_pose_stamped())
        self.path_publisher_1.publish(path)
        return path
    def display_path_notshowpath(self, path_nodes: list) -> Path:
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        for node in path_nodes:
            path.poses.append(node.to_pose_stamped())
        
        return path
    

    # def checkline(self,a:list,b:list):
    #     print(a,b)
    #     a0,b0 = self.map.coordinates_to_indices(a[0],b[0])
    #     a1,b1 = self.map.coordinates_to_indices(a[1],b[1])
    #     c= a0 -a1
    #     c1 = abs(c)
    #     d= b0 - b1
    #     d1 = abs(d)
    #     if c == 0 or d == 0:
    #         return True
    #     for t in range(1,int(c1)+1):
    #         x = a0 + t
    #         y = (x-a1)*d/c+b1
            
    #         if self.map.get_by_indices(int(x),int(y)) != 0:
    #             return False
            
    #     for t in range(1,int(d1)+1):
    #         y = b0 + t
    #         x = (y-b1)*c/d+a1
    #         if self.map.get_by_indices(int(x),int(y)) != 0:
    #             return False
    #     return True    

    def checkline(self,a:list,b:list):
        class cube:
            def __init__(self,x,y) -> None:
                self.x =  x
                self.y =  y
        c= a[0] -a[1]
        c1 = max(a) -min(a)
        d= b[0] - b[1]
        d1 = max(b) - min(b)
        if c == 0 and d != 0:
            for t in np.arange(0.0,float(d1),0.001):
                x= a[0]
                y= min(b) +t
                if self.map.is_node_free(self.node.__add__(cube(x,y),cube(0,0))) is False:
                    return False
        elif d == 0 and c !=0:
            for t in np.arange(0.0,float(c1),0.001):
                x = min(a) + t
                y = b[0]
                if self.map.is_node_free(self.node.__add__(cube(x,y),cube(0,0))) is False:
                    return False

        elif d ==0 and c ==0:
            return True
        else :
            for t in np.arange(0.0,float(c1),0.001):
                x = min(a) + t
                y = (x-a[1])*d/c+b[1]
                if self.map.is_node_free(self.node.__add__(cube(x,y),cube(0,0))) is False:
                    return False
            
            for t in np.arange(0.0,float(d1),0.001):
                y = min(b) + t
                x = (y-b[1])*c/d+a[1]
                if self.map.is_node_free(self.node.__add__(cube(x,y),cube(0,0))) is False:
                    return False
        return True
    def display_risk_1(self,path):
        print('launch_display')
        grid_cells_msg = GridCells()
        grid_cells_msg.header.frame_id = "map"
        grid_cells_msg.header.stamp = rospy.Time.now()
        grid_cells_msg.cell_width = 0.05  # in meters
        grid_cells_msg.cell_height = 0.05  # in meters
        class cube:
            def __init__(self,x,y) -> None:
                self.x =  x
                self.y =  y
        list_x = [node.x for node in path]
        list_y = [node.y for node in path]
        for t in np.arange(min(list_x),max(list_x),0.05):
            for k in np.arange(min(list_y),max(list_y),0.05):
                cell = Point()
                if self.map.is_node_risk_1(self.node.__add__(cube(t,k),cube(0,0))) is True:
                    cell.x = t
                    cell.y =k
                    cell.z = 0
                    grid_cells_msg.cells.append(cell)
        self.grid_cell_1.publish(grid_cells_msg)
    def display_risk_2(self,path):
        print('launch_display')
        grid_cells_msg = GridCells()
        grid_cells_msg.header.frame_id = "map"
        grid_cells_msg.header.stamp = rospy.Time.now()
        grid_cells_msg.cell_width = 0.05  # in meters
        grid_cells_msg.cell_height = 0.05  # in meters
        class cube:
            def __init__(self,x,y) -> None:
                self.x =  x
                self.y =  y
        list_x = [node.x for node in path]
        list_y = [node.y for node in path]
        for t in np.arange(min(list_x),max(list_x),0.05):
            for k in np.arange(min(list_y),max(list_y),0.05):
                cell = Point()
                if self.map.is_node_risk_2(self.node.__add__(cube(t,k),cube(0,0))) is True:
                    cell.x = t
                    cell.y =k
                    cell.z = 0
                    grid_cells_msg.cells.append(cell)
                    
        self.grid_cell_2.publish(grid_cells_msg)



