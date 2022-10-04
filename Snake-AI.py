import numpy as np
import serial, time
import matplotlib.pyplot as plt
import copy
# import pprint
import pygame

# Obstacle Maker has currently been commented out at line 596

# SIZE OF GRID DECIDED HERE
#Better kept as a factor of 640(8,10,16,20,32...) as Pygame window is chosen as 640x640 pixel
global size
size = 16

x = np.zeros((size,size),dtype=np.int)

#Change to True if Obstacles are wanted
WantObstacles= True

#initial conditions
snakehead_row=1
snakehead_col=4

#History1=[[1,1],[1,2]]
snakespeed = 0.05

eaten =0
length=3
food_row=7
food_col=7
global LastNode
LastNode = [-1,-1]

## Convert matrix to string
def MtoS(grid):
    my_array = np.array(grid)
    matrix = my_array[:,:,1]
    #pprint.pprint(matrix)
    dis.fill(white)
    block = 640/size
    for i in range(size):
        for ii in range(size):
            if (matrix[i,ii] == 1):
                pygame.draw.rect(dis, green, [(i*block),(ii*block), block, block])
            if (matrix[i,ii] == 2):
                pygame.draw.rect(dis, red, [(i*block),(ii*block), block, block])
            if ([i,ii]==[snakehead_row,snakehead_col]):
                pygame.draw.rect(dis, d_green, [(i*block),(ii*block), block, block])
            if (matrix[i,ii] == 3):
                pygame.draw.rect(dis, brown, [(i*block),(ii*block), block, block])

## Makes the snake grid costs -1 and targets 2
def snakeysnake(snake_info):
    row = snake_info[0]
    col = snake_info[1]
    grid[row][col][0] = -1 # make the cost obstacle
    grid[row][col][1] = 1 # demonstration

## Grid creator
def grid_maker():
    first_el = np.zeros((size,size),dtype=np.int)
    second_el = np.zeros((size,size),dtype=np.int)
    grid = [[[ [] for u in range(2) ] for o in range(size)] for k in range(size)] # Create an empty grid in desired form
    for i in range(size):
        for j in range(size):
            grid[i][j][0] = first_el[i,j]
            grid[i][j][1] = second_el[i,j]
    return grid

## Heuristic function
def hrstc (a,b,tx,ty): #manhattan distance used for distance heuristic, the weight of this function is implemented during Astar function
    h= (abs(tx-a) + abs(ty-b))
    return h

## A* Search function
def ASTAR(grid,snakehead_row,snakehead_column,food_row,food_column):

    queue = [] # empty list for the queue
    visited = [] # empty list for the visited list
    cost_queue = [] # empty list for the costs
    mod_grid = copy.deepcopy(grid)
    #pprint.pprint(grid)
    for i in range(size):
        if mod_grid[0][i][0] ==0:
            mod_grid[0][i][0] = 1
        if mod_grid[i][0][0] ==0:
            mod_grid[i][0][0] = 1
        if mod_grid[size-1][i][0] ==0:
            mod_grid[size-1][i][0] = 1
        if mod_grid[i][size-1][0] ==0:
            mod_grid[i][size-1][0] = 1
    cost = copy.deepcopy(mod_grid) # the copy of the grid for storing the cumulative cost

    x = snakehead_column
    y = snakehead_row

    pthlst1 = [] # Parent list (1st element of this is the parent of the 1st element of pthlst2)
    pthlst2 = [] # Children list

    while True: # Loop for search
        up_index = y-1
        down_index = y+1
        right_index = x+1
        left_index = x-1

        # FOR UP MOTION

        if not (up_index<0):
            if not (grid[up_index][x][0]<0 or [up_index,x] in visited):
                h = hrstc(food_row,food_column,up_index,x)
                if [up_index,x] in queue:
                    total = h + cost[up_index][x][0]
                    if (total < cost_queue[queue.index([up_index,x])]):
                        cost[up_index][x][0] = mod_grid[up_index][x][0]+cost[y][x][0] # change the cost grid
                        total = h + cost[up_index][x][0]
                        cost_queue[queue.index([up_index,x])] = total
                        pthlst1.append([y,x]) # save parent
                        pthlst2.append([up_index,x]) # save the child
                else: # if we do not have it in the queue
                    queue.append([up_index,x])  # add it to the queue
                    cost[up_index][x][0] = cost[up_index][x][0]+cost[y][x][0] # add the cumulative cost
                    cost_queue.append(cost[up_index][x][0]+h) # add its cost to the cost queue
                    pthlst1.append([y,x]) # save parent
                    pthlst2.append([up_index,x]) # save the child
        # FOR DOWN MOTION
        if not ( down_index>size-1):
            if not (grid[down_index][x][0]<0 or [down_index,x] in visited):
                h = hrstc(food_row,food_column,down_index,x)
                if [down_index,x] in queue:
                    total = h + cost[down_index][x][0]
                    if (total < cost_queue[queue.index([down_index,x])]):
                        cost[down_index][x][0] = mod_grid[down_index][x][0]+cost[y][x][0]
                        total = h + cost[down_index][x][0]
                        cost_queue[queue.index([down_index,x])] = total
                        pthlst1.append([y,x])
                        pthlst2.append([down_index,x])
                else:
                    queue.append([down_index,x])
                    cost[down_index][x][0] = cost[down_index][x][0]+cost[y][x][0]
                    cost_queue.append(cost[down_index][x][0]+h)
                    pthlst1.append([y,x])
                    pthlst2.append([down_index,x])

        # FOR LEFT MOTION
        if not ( left_index<0 ):
            if not (grid[y][left_index][0]<0 or [y,left_index] in visited ):
                h = hrstc(food_row,food_column,y,left_index)
                if [y,left_index] in queue:
                    total = h+cost[y][left_index][0]
                    if (total < cost_queue[queue.index([y,left_index])]):
                        cost[y][left_index][0] = mod_grid[y][left_index][0]+cost[y][x][0]
                        total = h+cost[y][left_index][0]
                        cost_queue[queue.index([y,left_index])] = total
                        pthlst1.append([y,x])
                        pthlst2.append([y,left_index])
                else:
                    queue.append([y,left_index])
                    cost[y][left_index][0] = cost[y][left_index][0]+cost[y][x][0]
                    cost_queue.append(cost[y][left_index][0]+h)
                    pthlst1.append([y,x])
                    pthlst2.append([y,left_index])

        # FOR RIGHT MOTION
        if not ( right_index>size-1):
            if not (grid[y][right_index][0]<0  or [y,right_index] in visited ):
                h = hrstc(food_row,food_column,y,right_index)
                if [y,right_index] in queue:
                    total = h + cost[y][right_index][0]
                    if (total < cost_queue[queue.index([y,right_index])]):
                        cost[y][right_index][0] = mod_grid[y][right_index][0]+cost[y][x][0]
                        total = h + cost[y][right_index][0]
                        cost_queue[queue.index([y,right_index])] = total
                        pthlst1.append([y,x])
                        pthlst2.append([y,right_index])
                else:
                    queue.append([y,right_index])
                    cost[y][right_index][0] = cost[y][right_index][0]+cost[y][x][0]
                    cost_queue.append(cost[y][right_index][0]+h)
                    pthlst1.append([y,x])
                    pthlst2.append([y,right_index])

        visited.append([y,x])
        # Hit the target
        if y == food_row and x == food_column:
            flag = 1
            mypath = []
            if y == snakehead_row and x == snakehead_column:
                found_coord = [snakehead_row,snakehead_column]
                mypath.append([snakehead_row,snakehead_column])
            else:
                found_coord = [y,x]
                followpath = found_coord
                while flag:
                    if not followpath == [snakehead_row,snakehead_column]:
                        mypath.append(followpath)
                        inscnd = pthlst2.index(followpath)
                        followpath = pthlst1[inscnd]
                    else:
                        mypath.append([snakehead_row,snakehead_column])
                        break
            return mypath
        else:
            try: # This try-except statement checks if the cost_queue is empty and no solution returns
                y,x = queue[cost_queue.index(min(cost_queue))]
            except:
                return  []
            if len(queue) == 0:
                return []

        # Pop the minimum cost element from the cost_queue and the queue lists
            queue.pop(cost_queue.index(min(cost_queue)))
            #print("COST IS")
            #print(min(cost_queue))
            cost_queue.pop(cost_queue.index(min(cost_queue)))

## A* Search Approach hen Length is Longer
def ASTARlong(grid,snakehead_row,snakehead_column,food_row,food_column,History1):

    queue = [] # empty list for the queue
    visited = [] # empty list for the visited list
    cost_queue = [] # empty list for the costs
    mod_grid = copy.deepcopy(grid)
    edge_cost = 20

    #Make edge nodes of snake game have high cost
    for i in range(size):
        if mod_grid[0][i][0] ==0:
            mod_grid[0][i][0] = edge_cost
        if mod_grid[i][0][0] ==0:
            mod_grid[i][0][0] = edge_cost
        if mod_grid[size-1][i][0] ==0:
            mod_grid[size-1][i][0] = edge_cost
        if mod_grid[i][size-1][0] ==0:
            mod_grid[i][size-1][0] = edge_cost

    #Make nodes between rows and columns 1 to 6 be medium cost if they are not obstacles/snakebody
    for i in range(size-2):
        for j in range(size-2):
            if mod_grid[j+1][i+1][0] ==0:
               mod_grid[j+1][i+1][0] =14

    #Make nodes neighboring snake body be no cost
    count = 0
    for i in History1:
        row = i[0]
        col = i[1]
        #UP Check
        if (count != 0):
            if (row != 0) and (mod_grid[row-1][col][0] != -1) and (mod_grid[row-1][col][0] != edge_cost):
                mod_grid[row-1][col][0] = 0
            #DOWN Check
            if (row != size-1) and (mod_grid[row+1][col][0] != -1) and (mod_grid[row+1][col][0] != edge_cost):
                mod_grid[row+1][col][0] = 0
            #LEFT Check
            if (col != 0) and (mod_grid[row][col-1][0] != -1) and (mod_grid[row][col-1][0] != edge_cost):
                mod_grid[row][col-1][0] = 0
            #RIGHT Check
            if (col != size-1) and (mod_grid[row][col+1][0] != -1) and (mod_grid[row][col+1][0] != edge_cost):
                mod_grid[row][col+1][0] = 0
        count +=1

    #pprint.pprint(mod_grid)
    cost = copy.deepcopy(mod_grid) # the copy of the grid for storing the cumulative cost

    x = snakehead_column
    y = snakehead_row

    pthlst1 = [] # Parent list (1st element of this is the parent of the 1st element of pthlst2)
    pthlst2 = [] # Children list

    while True: # Loop for search
        up_index = y-1
        down_index = y+1
        right_index = x+1
        left_index = x-1

        # FOR UP MOTION

        if not (up_index<0):
            if not (grid[up_index][x][0]<0 or [up_index,x] in visited):
                h = hrstc(food_row,food_column,up_index,x)
                if [up_index,x] in queue:
                    total = h + cost[up_index][x][0]
                    if (total < cost_queue[queue.index([up_index,x])]):
                        cost[up_index][x][0] = mod_grid[up_index][x][0]+cost[y][x][0] # change the cost grid
                        total = h + cost[up_index][x][0]
                        cost_queue[queue.index([up_index,x])] = total
                        pthlst1.append([y,x]) # save parent
                        pthlst2.append([up_index,x]) # save the child
                else: # if we do not have it in the queue
                    queue.append([up_index,x])  # add it to the queue
                    cost[up_index][x][0] = cost[up_index][x][0]+cost[y][x][0] # add the cumulative cost
                    cost_queue.append(cost[up_index][x][0]+h) # add its cost to the cost queue
                    pthlst1.append([y,x]) # save parent
                    pthlst2.append([up_index,x]) # save the child
        # FOR DOWN MOTION
        if not ( down_index>size-1):
            if not (grid[down_index][x][0]<0 or [down_index,x] in visited):
                h = hrstc(food_row,food_column,down_index,x)
                if [down_index,x] in queue:
                    total = h + cost[down_index][x][0]
                    if (total < cost_queue[queue.index([down_index,x])]):
                        cost[down_index][x][0] = mod_grid[down_index][x][0]+cost[y][x][0]
                        total = h + cost[down_index][x][0]
                        cost_queue[queue.index([down_index,x])] = total
                        pthlst1.append([y,x])
                        pthlst2.append([down_index,x])
                else:
                    queue.append([down_index,x])
                    cost[down_index][x][0] = cost[down_index][x][0]+cost[y][x][0]
                    cost_queue.append(cost[down_index][x][0]+h)
                    pthlst1.append([y,x])
                    pthlst2.append([down_index,x])

        # FOR LEFT MOTION
        if not ( left_index<0 ):
            if not (grid[y][left_index][0]<0 or [y,left_index] in visited ):
                h = hrstc(food_row,food_column,y,left_index)
                if [y,left_index] in queue:
                    total = h+cost[y][left_index][0]
                    if (total < cost_queue[queue.index([y,left_index])]):
                        cost[y][left_index][0] = mod_grid[y][left_index][0]+cost[y][x][0]
                        total = h+cost[y][left_index][0]
                        cost_queue[queue.index([y,left_index])] = total
                        pthlst1.append([y,x])
                        pthlst2.append([y,left_index])
                else:
                    queue.append([y,left_index])
                    cost[y][left_index][0] = cost[y][left_index][0]+cost[y][x][0]
                    cost_queue.append(cost[y][left_index][0]+h)
                    pthlst1.append([y,x])
                    pthlst2.append([y,left_index])

        # FOR RIGHT MOTION
        if not ( right_index>size-1):
            if not (grid[y][right_index][0]<0  or [y,right_index] in visited ):
                h = hrstc(food_row,food_column,y,right_index)
                if [y,right_index] in queue:
                    total = h + cost[y][right_index][0]
                    if (total < cost_queue[queue.index([y,right_index])]):
                        cost[y][right_index][0] = mod_grid[y][right_index][0]+cost[y][x][0]
                        total = h + cost[y][right_index][0]
                        cost_queue[queue.index([y,right_index])] = total
                        pthlst1.append([y,x])
                        pthlst2.append([y,right_index])
                else:
                    queue.append([y,right_index])
                    cost[y][right_index][0] = cost[y][right_index][0]+cost[y][x][0]
                    cost_queue.append(cost[y][right_index][0]+h)
                    pthlst1.append([y,x])
                    pthlst2.append([y,right_index])

        visited.append([y,x])
        # Hit the target
        if y == food_row and x == food_column:
            flag = 1
            mypath = []
            if y == snakehead_row and x == snakehead_column:
                found_coord = [snakehead_row,snakehead_column]
                mypath.append([snakehead_row,snakehead_column])
            else:
                found_coord = [y,x]
                followpath = found_coord
                while flag:
                    if not followpath == [snakehead_row,snakehead_column]:
                        mypath.append(followpath)
                        inscnd = pthlst2.index(followpath)
                        followpath = pthlst1[inscnd]
                    else:
                        mypath.append([snakehead_row,snakehead_column])
                        break
            return mypath
        else:
            try: # This try-except statement checks if the cost_queue is empty and no solution returns
                y,x = queue[cost_queue.index(min(cost_queue))]
            except:
                return  []
            if len(queue) == 0:
                return []

        # Pop the minimum cost element from the cost_queue and the queue lists
            queue.pop(cost_queue.index(min(cost_queue)))
            #print("COST IS")
            #print(min(cost_queue))
            cost_queue.pop(cost_queue.index(min(cost_queue)))

## Food generator
def food_maker():
    food_row = np.random.randint(low = 0, high = size-1)
    food_column = np.random.randint(low = 0, high = size-1)
    print("FOOD IS")
    print(food_row,food_col)
    return food_row,food_column

## Obstacle generator
def obstacle_maker_5(grid):
    counter = 0
    while counter < 5:
        obstacle_row = np.random.randint(low = 1, high = size-2)
        obstacle_column =  np.random.randint(low = 1, high = size-2)
        if not grid[obstacle_row][obstacle_column][0] == -1:
            grid[obstacle_row][obstacle_column][0] = -1 # obstacle
            grid[obstacle_row][obstacle_column][1] = 3 # demonstration
            print(obstacle_row,obstacle_column)
            counter += 1


## Dance around if A* cant find the target
def dancemaxNEW(grid,current_node,food_node,History1):
    food_row = food_node[0]
    food_col = food_node[1]
    row = current_node[0]
    col = current_node[1]
    up_index = row-1
    down_index = row+1
    right_index = col+1
    left_index = col-1

    # give initial low values so that they wont remain undefined and cant be maximum
    h_up = -111
    h_down = -111
    h_left = -111
    h_right = -111

    edge_cost=1
    around_snake=2
    #Grid Changed to Weighted Grid
    mod_grid = copy.deepcopy(grid)

    #Change edges to high cost
    for i in range(size):
        if mod_grid[0][i][0] ==0:
            mod_grid[0][i][0] = edge_cost
        if mod_grid[i][0][0] ==0:
            mod_grid[i][0][0] = edge_cost
        if mod_grid[size-1][i][0] ==0:
            mod_grid[size-1][i][0] = edge_cost
        if mod_grid[i][size-1][0] ==0:
            mod_grid[i][size-1][0] = edge_cost

# Chang cost of nodes neighboing snake body to no cost
    count = 0
    for i in History1:
        rowA = i[0]
        colA = i[1]
        #UP Check
        if (count != 0): #Except for snake head or else heading direction will always be low cost
            if (rowA != 0) and (mod_grid[rowA-1][colA][0] != -1) and (mod_grid[rowA-1][colA][0] != edge_cost):
                mod_grid[rowA-1][colA][0] = around_snake
            #DOWN Check
            if (rowA != size-1) and (mod_grid[rowA+1][colA][0] != -1) and (mod_grid[rowA+1][colA][0] != edge_cost):
                mod_grid[rowA+1][colA][0] = around_snake
            #LEFT Check
            if (colA != 0) and (mod_grid[rowA][colA-1][0] != -1) and (mod_grid[rowA][colA-1][0] != edge_cost):
                mod_grid[rowA][colA-1][0] = around_snake
            #RIGHT Check
            if (colA != size-1) and (mod_grid[rowA][colA+1][0] != -1) and (mod_grid[rowA][colA+1][0] != edge_cost):
                mod_grid[rowA][colA+1][0] = around_snake
        count +=1

    #pprint.pprint(mod_grid)

    # Creating Weighted Cost grid based on maximum distance and minimum cost
    if not (up_index==-1):
        if not (grid[up_index][col][0]<0):
            h_up = hrstc(food_row,food_col,up_index,col)
            print(h_up)
            if (mod_grid[up_index][col][0]==edge_cost):
                h_up += -1
            if (mod_grid[up_index][col][0]==around_snake):
                h_up += 1
            # Check if the node is not a dead end
            if not SafetoGo(mod_grid,[row,col],[up_index,col]):
                h_up += -10
    if not (down_index==size):
        if not (grid[down_index][col][0]<0):
            h_down = hrstc(food_row,food_col,down_index,col)
            print(h_down)
            if (mod_grid[down_index][col][0]==edge_cost):
                h_down += -1
            if (mod_grid[down_index][col][0]==around_snake):
                h_down += 1
            if not SafetoGo(mod_grid,[row,col],[down_index,col]):
                h_down += -10
    if not (right_index==size):
        if not (grid[row][right_index][0]<0):
            h_right = hrstc(food_row,food_col,row,right_index)
            print(h_right)
            if (mod_grid[row][right_index][0]==edge_cost):
                h_right += -1
            if (mod_grid[row][right_index][0]==around_snake):
                h_right += 1
            if not SafetoGo(mod_grid,[row,col],[row,right_index]):
                h_right += -10
    if not (left_index==-1):
        if not (grid[row][left_index][0]<0):
            h_left = hrstc(food_row,food_col,row,left_index)
            print(h_left)
            if (mod_grid[row][left_index][0]==edge_cost):
                h_left += -1
            if (mod_grid[row][left_index][0]==around_snake):
                h_left += 1
            if not SafetoGo(mod_grid,[row,col],[row,left_index]):
                h_left += -10

    #There is no path to go to
    if (h_up == -111 and h_down == -111 and h_left == -111 and h_right == -111):
        return 0
    else:
        Costs = [h_up,h_down,h_right,h_left]
        print("Costs are -------------------")
        print(Costs)
        Ans=Costs.index(max(Costs))
        if Ans == 0:
            next_move = [up_index,col]
        if Ans == 1:
            next_move = [down_index,col]
        if Ans == 3:
            next_move = [row,left_index]
        if Ans == 2:
            next_move = [row,right_index]

        print(next_move)
        return next_move

# Check if food node is surrounded by 3 obstacle/snake body nodes and returns True or False
def SafetoGo(grid,current_node,food_node):
    row = food_node[0]
    col = food_node[1]
    SH_row = current_node[0]
    SH_col = current_node[1]
    global LastNode
    LastNode =[-1,-1]

    up_index = row-1
    down_index = row+1
    right_index = col+1
    left_index = col-1

    count = 0
    if not (up_index<0):
        if (grid[up_index][col][0]<0):
            if (current_node != [up_index,col]):
                count += 1
                if [up_index,col] in History1:
                    LastNode = [up_index,col]
    else:
        count += 1
    if not (down_index>size-1):
        if  (grid[down_index][col][0]<0):
            if (current_node != [down_index,col]):
                count += 1
                if [down_index,col] in History1:
                    LastNode = [down_index,col]
    else:
        count += 1
    if not (right_index>size-1):
        if  (grid[row][right_index][0]<0):
            if (current_node != [row,right_index]):
                count += 1
                if [right_index,col] in History1:
                    LastNode = [right_index,col]
    else:
        count += 1
    if not (left_index<0):
        if  (grid[row][left_index][0]<0):
            if (current_node != [row,left_index]):
                count += 1
                if [left_index,col] in History1:
                    LastNode = [left_index,col]
    else:
        count += 1

    if (count <3):
        return True
    else:
        return False

A = 1
# Initialize the grid
History1=[]
def snake_generator():
    counter = 0
    snake_row = np.random.randint(low = 0, high = size-1) ## get random numbers
    snake_col =  np.random.randint(low = 0, high = size-1)
    History1.append([snake_row,snake_col])
    while counter < 3:
        if (snake_row != size-1):
            snake_row += 1
            History1.append([snake_row,snake_col])
            counter += 1 ## Increase counter
        elif (snake_col != size-1):
            snake_col += 1
            History1.append([snake_row,snake_col])
            counter += 1 ## Increase counter
        elif (snake_col != 0):
            snake_col -= 1
            History1.append([snake_row,snake_col])
            counter += 1 ## Increase counter
        elif (snake_row != 0):
            snake_row -= 1
            History1.append([snake_row,snake_col])
            counter += 1 ## Increase counter

grid = grid_maker()

snake_generator()

snakehead_row = History1[0][0]
snakehead_col = History1[0][1]
discard = History1.pop(0)
print(History1)
print("lalala")
for i in range(length):
    snakeysnake(History1[i])
## Create obstacles
if WantObstacles == True:
    obstacle_maker_5(grid)

## Initialize the food
grid[food_row][food_col][1] = 2

pygame.init()

white = (255, 255, 255)
green = (86, 252, 3)
red = (255, 0, 0)
d_green = (60, 163, 11)
brown =(97, 68, 27)

dis = pygame.display.set_mode((640, 640))
pygame.display.set_caption('THEODORE!!')

game_over = False


clock = pygame.time.Clock()

## Main loop
while (A==1):

    ## Add new snake head to the history list
    History1=[[snakehead_row,snakehead_col]]+History1

    ## If an excess number of elements exist in History list, pop them out and delete the snake's tail
    while len(History1) > length:
        grid[History1[-1][0]][History1[-1][1]][0] = 0 # snake is not on this node anymore
        grid[History1[-1][0]][History1[-1][1]][1] = 0
        History1.pop(-1)

    #Slow down game when snake is longer to visualize decision making better
    #if length > 20:
    #    snakespeed=0.1

    # Only follow A* path if the snake will survive after eating food , or else stall until it is available
    #if SafetoGo(grid,[snakehead_row,snakehead_col],[food_row,food_col]):
    #    if length > 13:     #Astarlong is more safe approach but it more time conssuming so start it when snake is longer , use unweighted grid for start
    #        path = ASTARlong(grid,snakehead_row,snakehead_col,food_row,food_col,History1)
    #    else:
    #        path = ASTAR(grid,snakehead_row,snakehead_col,food_row,food_col)
    #else:
    #    print("NOT SAFE ABORT")
    #    path = []
    path = ASTARlong(grid,snakehead_row,snakehead_col,food_row,food_col,History1)
    tail_row=History1[-1][0]
    tail_col=History1[-1][1]

    if SafetoGo(grid,[snakehead_row,snakehead_col],[food_row,food_col]):
        if length > 13:     #Astarlong is more safe approach but it more time conssuming so start it when snake is longer , use unweighted grid for start
            path = ASTARlong(grid,snakehead_row,snakehead_col,food_row,food_col,History1)
        else:
            path = ASTAR(grid,snakehead_row,snakehead_col,food_row,food_col)
    else:
        if (LastNode != [-1,-1]):
            Index = History1.index(LastNode)
            print("Last node is " +str(LastNode) + " Index is " + str(Index))
            Tail2food = len(History1) - Index + 3
            if (len(path) > Tail2food) :
                path = ASTARlong(grid,snakehead_row,snakehead_col,food_row,food_col,History1)
                print("Risk it for the Biscuit")
                print("Length of path is " + str(len(path)) + " , Tail2food is " + str(Tail2food))
            if (len(path) <= Tail2food) :
                print("NOT SAFE ABORT")
                print("Length of path is " + str(len(path)) + " , Tail2food is " + str(Tail2food))
                path = []
        else:
            print("NOT SAFE ABORT")
            path = []

    ## Check if the path length is greater than zero, and if it is, make the snake head the next step, which is the last second element of path variable
    if len(path)>1:
        snakehead_row = path[-2][0]
        snakehead_col = path[-2][1]

    ## Create the snake in the grid by -1 costs and 2? targets
    for i in range(length):
        snakeysnake(History1[i]) # make the grid elements where snake is located -1

    ## Some sanity checks here
    print("History")
    print(History1)
    print("path")
    print(path)

    MtoS(grid)
    pygame.display.update()
    time.sleep(snakespeed)
    #pprint.pprint(grid)

    ## If 1 step remaining to the target
    if (len(path) == 2):
        eaten = eaten+1 ## Add 1 to the eaten variable
        length = length+1  ## Make the snake longer

        food_row,food_col = food_maker() ## Create new food

        while grid[food_row][food_col][0]<0: ## If the created food is on an obstacle/snake, recreate
            food_row,food_col = food_maker()

        grid[food_row][food_col][1] = 2 ## Make the food target 1
        ## SHOW IT
        snakeysnake([snakehead_row,snakehead_col])
        MtoS(grid)
        pygame.display.update()
        ## Some more sanity checks
        print('food')
        print(food_row)
        print(food_col)

    ## If the created food is just where the snake's next step is on
    elif (len(path) == 1):
        ## Recreate the food
        food_row,food_col = food_maker()
        while grid[food_row][food_col][0]<0:
            food_row,food_col = food_maker()
        grid[food_row][food_col][1] = 2

    ## If A* cant find the food anywhere
    elif path == []:
        breakflag = 0 # Should we break everything?
        while 1:
            ## Check what we get from the dance function
            print("DANCE MAX")
            coord = dancemaxNEW(grid,[snakehead_row,snakehead_col],[food_row,food_col],History1)




            if coord == 0: ## If dance function gives 0, snake is dead
                print('snake')
                print(snakehead_row)
                print(snakehead_col)
                print("NOWHERE TO GO")

                ## Demonstrate LAST step before dying
                snakeysnake([snakehead_row,snakehead_col])
                MtoS(grid)
                pygame.display.update()
                time.sleep(1)

                print("FINAL LENGTH IS: " + str(length))
                breakflag = 1 ## Yes, break everything. GAME IS OVER
                #bb = "<001111000100001010100101100000011001100110100101010000100011110088>"
                #serialcomm.write(bb.encode('utf-8'))
                #time.sleep(5)
                break
            else: ## If there are other places to dance around, do it
                snakehead_row = coord[0]
                snakehead_col = coord[1]
                break
        if breakflag == 1: ## I have to break then
            break
