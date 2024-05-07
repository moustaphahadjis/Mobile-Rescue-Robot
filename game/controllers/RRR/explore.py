import random
import numpy as np
from random import randrange
import math
import time
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.neighbors import KNeighborsClassifier

class Node():

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position



class Explore:
    def __init__(self):
        self.explored = []

    def get_surrounding_square(self, x, y, distance):
        surrounding_square = []
        for i in range(x - distance, x + distance + 1):
            for j in range(y - distance, y + distance + 1):
                if abs(i - x) == distance or abs(j - y) == distance:
                    surrounding_square.append((i, j,0))
        return surrounding_square
    
    def knn(self,x,y, map):
        X =  map.world[:1]
        X = map.world.reshape(X.shape[0], -1)
        Y = map.world[2]
        knn = KNeighborsClassifier(n_neighbors=5)


        knn.fit(X, Y)
        return  knn.predict(np.array((x,y))) 

        
    def getAvr(self):
        explored_points = self.explored

        total_x = 0
        total_y = 0
        for point in explored_points:
            total_x += point[0]
            total_y += point[1]

        average_x = total_x / len(explored_points)
        average_y = total_y / len(explored_points)
       # print(f'Average {average_x,average_y}')
        return (average_x, average_y)
    
    def getDist(self, start, end):
       a = -start[0] + end[0]
       b = -start[1] + end[1]

       return math.sqrt(a**2 + b**2)
    
    def getEnd(self, map):
        x = map.curr[0]
        y = map.curr[1]
        thr = 50
        cells =np.array([
            [x-thr, y,0], [x+thr, y,0],   
            [x, y-thr,0], [x, y+thr,0],    
            [x-thr, y-thr,0], [x+thr, y-thr,0],  
            [x-thr, y+thr,0], [x+thr, y+thr,0]],)
        
        cells = np.array(self.get_surrounding_square(x,y, 20))
        
        #print(cells.shape)
        if(len(self.explored)>0):
            avr = self.getAvr()
            for i in range(len(cells)):
                dist = self.getDist(avr,[cells[i][0],cells[i][1]])
                
                #path = self.pathFinder(map.world, (x,y), (cells[i][0],cells[i][1]))
                #if not map.securePath(path):
                #    dist = dist- dist
                #knn = self.knn(x,y,map)
                #print(f'Knn:{knn}')
                cells[i][2] = dist

            cells = cells[cells[:, 2].argsort()]
            cell = cells[-1]    
            #print(cells)
        else:
            cell = random.choice(cells)
            
            #path = self.pathFinder(map.world, (x,y), (cell[0],cell[1]))
            #print(cell)
            #paths.append(path)
                
        #print(f'path: {paths}')
        self.explored.append(cell)
        map.setExplored(cell[0],cell[1])
        
        return cell
    
        
    def randomEnd1(self):
        x = randrange(0,1000)
        y = randrange(0,1000)
        return (x,y)

    def randomEnd(self,maze, start, distance=300):
        
        rows, cols,_ = maze.shape
        start_row, start_col = start
        offset_row = random.randint(-distance, distance)
        offset_col = random.randint(-distance, distance)

        end_row = np.clip(start_row + offset_row, 0, rows - 1)
        end_col = np.clip(start_col + offset_col, 0, cols - 1)

        return (end_row, end_col)
    def pathFinder(self, maze, start, end):
        
        start_node = Node(None, start)
        start_node.g = start_node.h = start_node.f = 0
        end_node = Node(None, end)
        end_node.g = end_node.h = end_node.f = 0

        open_list = []
        closed_list = []

        open_list.append(start_node)

        t1 = time.time()

        while len(open_list) > 0:
            t2 = time.time()

            if t2 - t1>1:
                return []
            current_node = open_list[0]
            current_index = 0
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index
                    
            open_list.pop(current_index)
            closed_list.append(current_node)

            if current_node == end_node:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                return path[::-1] 

            children = []
            for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

                if node_position[0] > (maze.shape[0] - 1) or node_position[0] < 0 or node_position[1] > (maze.shape[1] -1) or node_position[1] < 0:
                    continue

                if maze[node_position[0]][node_position[1]] != 0:
                    continue

                new_node = Node(current_node, node_position)

                children.append(new_node)

            for child in children:

                for closed_child in closed_list:
                    if child == closed_child:
                        continue

                child.g = current_node.g + 1
                child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
                child.f = child.g + child.h

                for open_node in open_list:
                    if child == open_node and child.g > open_node.g:
                        continue
                open_list.append(child)

