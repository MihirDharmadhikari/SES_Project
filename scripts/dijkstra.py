import rospy
from std_msgs.msg import Float32MultiArray 
from nav_msgs.msg import *
import math
import tf
import time
import heapq
import numpy


class Trajectory:
    def __init__(self):
        self.coordinates = { 'a':{'x':0,'y':0}, 'b':{'x':0,'y':1}, 'c':{'x':0,'y':2}, 'd':{'x':1,'y':2}, 'e':{'x':1,'y':1}, 'f':{'x':2,'y':2}, 'g':{'x':2,'y':4}, 'h':{'x':0,'y':3}, 'i':{'x':1,'y':4}}
        self.goal = '%'
        self.goal_xcoordinate = 0
        self.goal_ycoordinate = 0 
        self.final_path = []
        self.nearest = " "
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0
        self.index = 1
        self.main()

    def init_pub_sub(self):
        rospy.init_node('dijkstra')
        self.pub = rospy.Publisher('/Global_Waypoint',Float32MultiArray ,queue_size=1)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_data_callback)
        time.sleep(1)

    def odom_data_callback(self,data):
    	self.robot_x = data.pose.pose.position.x
    	self.robot_y = data.pose.pose.position.y

    	robot_orientation = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    	self.robot_yaw = robot_orientation[2]

    def get_nearest_node(self):
        mini = 9999999
        for node in self.coordinates:
            dist = math.sqrt((self.robot_x - int(self.coordinates[node]['x']))**2 + (self.robot_y - self.coordinates[node]['y'])**2)
            print dist
            if dist<mini:
                mini = dist
                self.nearest = node
        self.final_path = path_plan(self.goal,self.nearest)

    def get_goal(self):
        self.goal = raw_input()

        try:
            goal_xcoordinate = self.coordinates[self.goal]['x']
            goal_ycoordinate = self.coordinates[self.goal]['y'] 
            print self.goal
            self.get_nearest_node()
        except:
            print "goal not on graph"

    def main(self):
       while not rospy.is_shutdown():
        self.init_pub_sub()
        if self.goal=='%':
            self.get_goal()
            self.get_nearest_node()
        print self.nearest
        print self.final_path
        dat = Float32MultiArray()
        near_x = float(self.coordinates[self.nearest]['x'])
        near_y = float(self.coordinates[self.nearest]['y'])
        near_list = [near_x,near_y]
        dat.data = near_list
        (self.pub).publish(dat)
        dist_from_node = math.sqrt((self.robot_x - int(self.coordinates[self.nearest]['x']))**2 + (self.robot_y - self.coordinates[self.nearest]['y'])**2)
        print dist_from_node
        if(dist_from_node < 0.05):
            if(self.index == len(self.final_path)):
                self.index = 0
                self.goal = '%'
            else:   
                self.nearest = self.final_path[self.index]
                self.index =self.index+1


class Vertex:
    def __init__(self, node):
        self.id = node
        self.adjacent = {}
        # Set distance to infinity for all nodes
        self.distance = sys.maxint
        # Mark all nodes unvisited        
        self.visited = False  
        # Predecessor
        self.previous = None

    def add_neighbor(self, neighbor, weight=0):
        self.adjacent[neighbor] = weight

    def get_connections(self):
        return self.adjacent.keys()  

    def get_id(self):
        return self.id

    def get_weight(self, neighbor):
        return self.adjacent[neighbor]

    def set_distance(self, dist):
        self.distance = dist

    def get_distance(self):
        return self.distance

    def set_previous(self, prev):
        self.previous = prev

    def set_visited(self):
        self.visited = True

    def __str__(self):
        return str(self.id) + ' adjacent: ' + str([x.id for x in self.adjacent])

class Graph:
    def __init__(self):
        self.vert_dict = {}
        self.num_vertices = 0

    def __iter__(self):
        return iter(self.vert_dict.values())

    def add_vertex(self, node):
        self.num_vertices = self.num_vertices + 1
        new_vertex = Vertex(node)
        self.vert_dict[node] = new_vertex
        return new_vertex

    def get_vertex(self, n):
        if n in self.vert_dict:
            return self.vert_dict[n]
        else:
            return None

    def add_edge(self, frm, to, cost = 0):
        if frm not in self.vert_dict:
            self.add_vertex(frm)
        if to not in self.vert_dict:
            self.add_vertex(to)

        self.vert_dict[frm].add_neighbor(self.vert_dict[to], cost)
        self.vert_dict[to].add_neighbor(self.vert_dict[frm], cost)

    def get_vertices(self):
        return self.vert_dict.keys()

    def set_previous(self, current):
        self.previous = current

    def get_previous(self, current):
        return self.previous

def shortest(v, path):
    ''' make shortest path from v.previous'''
    if v.previous:
        path.append(v.previous.get_id())
        shortest(v.previous, path)
    return

def dijkstra(aGraph, start, target):
    print '''Dijkstra's shortest path'''
    # Set the distance for the start node to zero 
    start.set_distance(0)

    # Put tuple pair into the priority queue
    unvisited_queue = [(v.get_distance(),v) for v in aGraph]
    heapq.heapify(unvisited_queue)

    while len(unvisited_queue):
        # Pops a vertex with the smallest distance 
        uv = heapq.heappop(unvisited_queue)
        current = uv[1]
        current.set_visited()

        #for next in v.adjacent:
        for next in current.adjacent:
            # if visited, skip
            if next.visited:
                continue
            new_dist = current.get_distance() + current.get_weight(next)
            
            if new_dist < next.get_distance():
                next.set_distance(new_dist)
                next.set_previous(current)
                print 'updated : current = %s next = %s new_dist = %s' \
                        %(current.get_id(), next.get_id(), next.get_distance())
            else:
                print 'not updated : current = %s next = %s new_dist = %s' \
                        %(current.get_id(), next.get_id(), next.get_distance())

        # Rebuild heap
        # 1. Pop every item
        while len(unvisited_queue):
            heapq.heappop(unvisited_queue)
        # 2. Put all vertices not visited into the queue
        unvisited_queue = [(v.get_distance(),v) for v in aGraph if not v.visited]
        heapq.heapify(unvisited_queue)
def path_plan(a,b):
    g = Graph()
    g.add_vertex('a')
    g.add_vertex('b')
    g.add_vertex('c')
    g.add_vertex('d')
    g.add_vertex('e')
    g.add_vertex('f')
    g.add_vertex('g')
    g.add_vertex('h')
    g.add_vertex('i')

    g.add_edge('a', 'b', 1)  
    g.add_edge('b', 'c', 1)
    g.add_edge('c', 'h', 1)
    g.add_edge('h', 'i', 1.4142135)
    g.add_edge('b', 'd', 1.4142135)
    g.add_edge('c', 'd', 1)
    g.add_edge('h', 'd', 1.4142135)
    g.add_edge('d', 'i', 2)
    g.add_edge('d', 'f', 1)
    g.add_edge('i', 'f', 2.236068)
    g.add_edge('f', 'g', 2)
    g.add_edge('d', 'e', 1)
    g.add_edge('f', 'e', 1.4142135)

    dijkstra(g, g.get_vertex(a), g.get_vertex(b)) 
    target = g.get_vertex(b)
    path = [target.get_id()]
    shortest(target, path)
    return path

if __name__ == "__main__":
    Trajectory()