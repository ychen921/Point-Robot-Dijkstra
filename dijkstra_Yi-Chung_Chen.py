import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import argparse
from heapq import heappush, heappop
import cv2
from tqdm import tqdm
import time
import os 
import math

class Node:
    def __init__(self, pos):
        self.x = pos[0]-1
        self.y = pos[1]-1

class PositionRobot:
    def __init__(self, InitStartNode, InitGoalNode):
        self.height = 500
        self.width = 1200
        self.clearance = 5
        self.init_image = np.zeros((self.height,self.width,3))
        self.MapSpace = np.copy(self.init_image)
        self.MapFlag = np.zeros((self.height,self.width))

        StartNode = [int(num) for num in InitStartNode.split('_')]
        GoalNode = [int(num) for num in InitGoalNode.split('_')]

        self.StartNode = Node(StartNode)
        self.GoalNode = Node(GoalNode)

        self.Actions = ['Up', 'Down', 'Left', 'Right', 'UpLeft', 'UpRight', 'DownLeft', 'DownRight']
        self.MapSpace, self.MapFlag = self.GenerateMap()


    def GenerateMap(self):

        for y in range(self.height):
            for x in range(self.width):
                
                # Obstacle with clearance
                h1 = 15*(x-self.clearance)+26*(y-self.clearance)-20150
                h2 = 780+self.clearance
                h3 = 15*(x-self.clearance)-26*(y+self.clearance)-7150
                h4 = 15*(x+self.clearance)+26*(y+self.clearance)-12350
                h5 = 520-self.clearance
                h6 = 15*(x+self.clearance)-26*(y-self.clearance)+650

                # Obstacle
                h_1 = 15*x+26*y-20150
                h_2 = 780
                h_3 = 15*x-26*y-7150
                h_4 = 15*x+26*y-12350
                h_5 = 520
                h_6 = 15*x-26*y+650

                if (y<500 and y>100-self.clearance and x<175+self.clearance and x>100-self.clearance) or \
                    (y<400+self.clearance and y>0 and x<350+self.clearance and x>275-self.clearance) or \
                        (h1<0 and x<h2 and h3<0 and h4>0 and x>h5 and h6>0) or \
                            (x>900-self.clearance and y<450+self.clearance and x<1020 and y>375-self.clearance) or \
                                (x>1020-self.clearance and y<450+self.clearance and x<1100+self.clearance and y>50-self.clearance) or \
                                    (x>900-self.clearance and y<125+self.clearance and x<1020 and y>50-self.clearance):
                    self.MapSpace[y,x] = [0,0,0] # set in green
                    self.MapFlag[y,x] = 1

                # Obstacle 1
                if (y<500 and y>100 and x<175 and x>100):
                    self.MapSpace[y,x] = [255,255,255] # set in red
                # Obstacle 2
                if (y<400 and y>0 and x<350 and x>275):
                    self.MapSpace[y,x] = [255,255,255] # set in greed

                # Obstacle 3
                if (h_1<0 and x<h_2 and h_3<0 and h_4>0 and x>h_5 and h_6>0):
                    self.MapSpace[y,x] = [255,255,255] # set in greed

                if (x>900 and y<450 and x<1020 and y>375) or \
                        (x>1020 and y<450 and x<1100 and y>50) or \
                            (x>900 and y<125 and x<1020 and y>50):
                    self.MapSpace[y,x] = [255,255,255] # set in greed

        # Define Boundary of the shape
        # Lower boundary 
        # self.MapSpace[0,0:1200] = [255,255,0]
        # self.MapFlag[0,0:1200] = 1

        # # Upper boundary
        # self.MapSpace[499,0:1200] = [255,255,0]
        # self.MapFlag[499,0:1200] = 1

        # # Left boundary
        # self.MapSpace[0:499,0] = [255,255,0]
        # self.MapFlag[0:499,0] = 1

        # # Right boundary
        # self.MapSpace[1199:499,0] = [255,255,0]
        # self.MapFlag[1199:499,0] = 1

        return self.MapSpace, self.MapFlag
    
    def MoveUp(self, x, y):
        y+=1
        cost=1
        return x, y, cost
    
    def MoveDown(self, x, y):
        y-=1
        cost=1
        return x, y, cost
    
    def MoveLeft(self, x, y):
        x-=1
        cost=1
        return x, y, cost
    
    def MoveRight(self, x, y):
        y+=1
        cost=1
        return x, y, cost
    
    def MoveUpperLeft(self, x, y):
        x-=1
        y+=1
        cost=1.4
        return x, y, cost
    
    def MoveUpperRight(self, x, y):
        x+=1
        y+=1
        cost=1.4
        return x, y, cost
    
    def MoveDownLeft(self, x, y):
        x-=1
        y-=1
        cost=1.4
        return x, y, cost
    
    def MoveDownRight(self, x, y):
        x+=1
        y-=1
        cost=1.4
        return x, y, cost
    
    def CheckValidMove(self, x, y):
        h, w = self.MapFlag.shape
        # Move to boundary
        if (x>w-1) or (x<0) or (y>h-1) or (y<0):
            return False
        # Move to obstacles
        elif self.MapFlag[y,x] == 1:
            return False
        # Valid Move
        else:
            return True
        
    def CheckAchieveGoal(self, x, y):
        if x == self.GoalNode.x and y == self.GoalNode.y:
            return True
        return False
    
    def Action(self, act, x, y):
        if act == "Up":
            return self.MoveUp(x, y)
        elif act == "Down":
            return self.MoveDown(x, y)
        elif act == "Left":
            return self.MoveLeft(x, y)
        elif act == "Right":
            return self.MoveRight(x, y)
        elif act == "UpLeft":
            return self.MoveUpperLeft(x, y)
        elif act == "UpRight":
            return self.MoveUpperRight(x, y)
        elif act == "DownLeft":
            return self.MoveDownLeft(x, y)
        elif act == "DownRight":
            return self.MoveDownRight(x, y)
        
    
    def Visualization(self, VisitedNodes, BestPath):
        out = cv2.VideoWriter('dijkstra.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 20.0, (960,400))
        print('\nGenerating Video...')
        for count, coord in enumerate(tqdm(VisitedNodes)):
            x, y = coord
            self.MapSpace[y,x] = [0,0,255]
            if count % 100 == 0:
                save_frame = cv2.resize(cv2.flip(self.MapSpace,0), (960,400))
                out.write(save_frame.astype(np.uint8))

        for coord in tqdm(BestPath):
            x, y = coord
            self.MapSpace[y,x] = [0,255,0]
            
            save_frame = cv2.resize(cv2.flip(self.MapSpace,0), (960,400))
            out.write(save_frame.astype(np.uint8))

        print('Done !')
        cv2.imwrite('path.png', cv2.resize(cv2.flip(self.MapSpace,0), (960,400)))
        out.release()

    def DistanceMap(self):
        map = {}
        VisitedNode = {}
        path = {}
        for i in range(self.height):
            for j in range(self.width):
                map[(j,i)] = float('inf')
                path[(j,i)] = -1
                VisitedNode[(j,i)] = False

        return VisitedNode, path, map

    def Solver(self):

        # Check Valid start and goal node
        if self.CheckValidMove(self.StartNode.x, self.StartNode.y) is False:
            raise Exception("Sorry, this is an invalid coordinate (cooridnate is out of map or in the obstacles)")
        elif self.CheckValidMove(self.GoalNode.x, self.GoalNode.y) is False:
            raise Exception("Sorry, this is an invalid coordinate (cooridnate is out of map or in the obstacles)") 
        else:
            print("Valid coordinate! Start finding the designed path...")

        VisitedNode, path , dist = self.DistanceMap()

        OpenList = []
        ClosedList = []

        # Sort with lowest cost
        heappush(OpenList, [0, (self.StartNode.x, self.StartNode.y)])
        dist[(self.StartNode.x, self.StartNode.y)] = 0

        while len(OpenList) > 0:
            _, CurrNode = heappop(OpenList)
            VisitedNode[CurrNode] = True
            ClosedList.append(CurrNode)

            # Check achieve goal
            if self.CheckAchieveGoal(CurrNode[0], CurrNode[1]):
                print("Find the Path!\n")
                break

            for act in self.Actions:
                x, y, cost = self.Action(act=act, x=int(CurrNode[0]), y=int(CurrNode[1]))

                # Check the move is valid
                if self.CheckValidMove(x, y) == True:
                    # Check the node is visited before
                    if VisitedNode[(x,y)] == False:
                        # Check Distance Map
                        if dist[(x,y)] > dist[CurrNode]+cost:
                            path[(x,y)] = (CurrNode[0], CurrNode[1])
                            dist[(x,y)] = dist[(CurrNode[0], CurrNode[1])] + cost
                            heappush(OpenList, (dist[(x,y)], (x,y)))
                            
        if dist[(self.GoalNode.x, self.GoalNode.y)] == float('inf'):
            return ClosedList, [], dist[(self.StartNode.x, self.StartNode.y)]
        
        backtrack_states = []
        node = (self.GoalNode.x, self.GoalNode.y)
        while path[node] != -1:
            backtrack_states.append(node)
            node = path[node]
        backtrack_states.append((self.StartNode.x, self.StartNode.y))
        backtrack_states = list(reversed(backtrack_states)) 
        
        return ClosedList, backtrack_states, dist[(self.GoalNode.x, self.GoalNode.y)]

                

def main():    
    # Parse Command Line arguments
    Parser = argparse.ArgumentParser()
    Parser.add_argument('--InitNode', type=str, default='10 20', help='Initial node, Default: 10 10')
    Parser.add_argument('--GoalNode', type=str, default='200 400', help='Goal node, Default: 200, 250')

    Args = Parser.parse_args()
    InitNode = Args.InitNode
    GoalNode = Args.GoalNode

    dijkstra = PositionRobot(InitNode, GoalNode)
    allNode , TrackList, DistMap = dijkstra.Solver()

    dijkstra.Visualization(allNode, TrackList)

    

if __name__ == "__main__":
    main()