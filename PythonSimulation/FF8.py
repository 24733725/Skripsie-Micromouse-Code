import API
import sys

def log(string):
    sys.stderr.write("{}\n".format(string))
    sys.stderr.flush()


MAZE_CELL_WIDTH = 16
MAZE_CELL_HEIGHT = 16
END_CELL_X = 8
END_CELL_Y = 8

class Cell:
    def __init__(self):
        self.walls = 0x0F  # binary - 0000 1111 last 1st bit: explored y/n and 4 bits:walls from top clockwise
        self.dist = 0

class Direction:
    NORTH = 0
    NE = 1
    EAST = 2
    SE = 3
    SOUTH = 4
    SW = 5
    WEST = 6
    NW = 7
    

class RelativeDirection:
    STRAIGHT = 0
    SR = 1
    RIGHT = 2
    RB = 3
    BACK = 4
    LB = 5
    LEFT = 6
    SL = 7

maze = [[Cell() for _ in range(MAZE_CELL_HEIGHT)] for _ in range(MAZE_CELL_WIDTH)]

class Mouse:
    heading = 0
    current_cell_x = 0
    current_cell_y = 0

def rel_to_fixed_dir(mouse_dir):
    return int((Mouse.heading  + mouse_dir) % 8)

def add_wall(x, y, dir, maze):
    if (maze[x][y].walls & 0xF0) == 0:
        
        maze[x][y].walls |= (0b01 << (dir//2))
        if dir == Direction.NORTH:
            API.setWall(x, y, "n")
            if y + 1 < MAZE_CELL_HEIGHT:
                maze[x][y + 1].walls |= (0b01 << (Direction.SOUTH//2))
        elif dir == Direction.EAST:
            API.setWall(x, y, "e")
            if x + 1 < MAZE_CELL_WIDTH:
                maze[x + 1][y].walls |= (0b01 << (Direction.WEST//2))
        elif dir == Direction.SOUTH:
            API.setWall(x, y, "s")
            if y > 0:
                maze[x][y - 1].walls |= (0b01 << (Direction.NORTH//2))
        elif dir == Direction.WEST:
            API.setWall(x, y, "w")
            if x > 0:
                maze[x - 1][y].walls |= (0b01 <<(Direction.EAST//2))

def read_wall(x, y, dir, maze):
    if (maze[x][y].walls & (0x01 << (dir//2))) == 0:
        return 0
    else:
        return 1

def set_explored(x, y, maze):
    if x>=0 and x<MAZE_CELL_WIDTH and y>=0 and y<MAZE_CELL_HEIGHT:
        maze[x][y].walls |= 0xF0

def get_explored(x, y, maze):
    if x>=0 and x<MAZE_CELL_WIDTH and y>=0 and y<MAZE_CELL_HEIGHT:
        if (maze[x][y].walls & 0xF0) == 0:
            return 0
        else:
            return 1
    else: return 1

def maze_init():
    for i in range(MAZE_CELL_WIDTH):
        for j in range(MAZE_CELL_HEIGHT):
            maze[i][j].dist = abs(END_CELL_X - i) + abs(END_CELL_Y - j)
            maze[i][j].walls = 0
            # API.setText(i, j, f'{maze[i][j].walls}')
            API.setText(i, j, f'{maze[i][j].dist}')
            # API.setColor(i,j,'w')

    for i in range(MAZE_CELL_WIDTH):
        API.setWall(i, 0, "s")
        add_wall(i, 0, Direction.SOUTH, maze)
        API.setWall(i, MAZE_CELL_HEIGHT-1, "n")
        add_wall(i, MAZE_CELL_HEIGHT-1, Direction.NORTH, maze)
        
    for i in range(MAZE_CELL_HEIGHT):
        API.setWall(0, i, "w")
        add_wall(0, i, Direction.WEST, maze)
        API.setWall(MAZE_CELL_WIDTH-1, i, "e")
        add_wall(MAZE_CELL_WIDTH-1, i, Direction.EAST, maze)
        
        
    add_wall(0, 0, Direction.EAST, maze)
    add_wall(0, 0, Direction.SOUTH, maze)
    add_wall(0, 0, Direction.WEST, maze)
    set_explored(0, 0, maze)

    # for i in range(MAZE_CELL_HEIGHT):
    #     print("|{:08b}|{:08b}|{:08b}|{:08b}|{:08b}|{:08b}|".format(
    #         maze[0][i].walls, maze[1][i].walls, maze[2][i].walls,
    #         maze[3][i].walls, maze[4][i].walls, maze[5][i].walls))
    
    # print(read_wall(0, 0, Direction.NORTH))
def moveForward(distance=None):
    API.moveForward(distance) #y axis flipped 
    if Mouse.heading == 0:
        Mouse.current_cell_y += 1
    elif Mouse.heading == 1:
        Mouse.current_cell_x +=  1
        Mouse.current_cell_y +=  1
    elif Mouse.heading == 2:
        Mouse.current_cell_x += 1
    elif Mouse.heading == 3:
        Mouse.current_cell_x += 1
        Mouse.current_cell_y -= 1
    elif Mouse.heading == 4:
        Mouse.current_cell_y -= 1
    elif Mouse.heading == 5:
        Mouse.current_cell_x -= 1
        Mouse.current_cell_y -= 1
    elif Mouse.heading == 6:
        Mouse.current_cell_x -= 1
    elif Mouse.heading == 7:
        Mouse.current_cell_x -= 1
        Mouse.current_cell_y += 1
    if API.wallLeft():
        if Mouse.heading == 0:
            add_wall(Mouse.current_cell_x, Mouse.current_cell_y, Direction.WEST, maze)
        elif Mouse.heading == 2:
            add_wall(Mouse.current_cell_x, Mouse.current_cell_y, Direction.NORTH, maze)
        elif Mouse.heading == 4:
            add_wall(Mouse.current_cell_x, Mouse.current_cell_y, Direction.EAST, maze)
        elif Mouse.heading == 6:
            add_wall(Mouse.current_cell_x, Mouse.current_cell_y, Direction.SOUTH, maze)       
    if API.wallFront():
        if Mouse.heading == 0:
            add_wall(Mouse.current_cell_x, Mouse.current_cell_y, Direction.NORTH, maze)
        elif Mouse.heading == 2:
            add_wall(Mouse.current_cell_x, Mouse.current_cell_y, Direction.EAST, maze)
        elif Mouse.heading == 4:
            add_wall(Mouse.current_cell_x, Mouse.current_cell_y, Direction.SOUTH, maze)
        elif Mouse.heading == 6:
            add_wall(Mouse.current_cell_x, Mouse.current_cell_y, Direction.WEST, maze)
    if API.wallRight():
        if Mouse.heading == 0:
            add_wall(Mouse.current_cell_x, Mouse.current_cell_y, Direction.EAST, maze)
        elif Mouse.heading == 2:
            add_wall(Mouse.current_cell_x, Mouse.current_cell_y, Direction.SOUTH, maze)
        elif Mouse.heading == 4:
            add_wall(Mouse.current_cell_x, Mouse.current_cell_y, Direction.WEST, maze)
        elif Mouse.heading == 6:
            add_wall(Mouse.current_cell_x, Mouse.current_cell_y, Direction.NORTH, maze)
    set_explored(Mouse.current_cell_x, Mouse.current_cell_y, maze)

def turnRight():
    API.turnRight()
    Mouse.heading = int(( Mouse.heading + 2)%8)
    
def turnLeft():
    API.turnLeft()
    Mouse.heading = int((Mouse.heading +6)%8)
    
def turnRight45():
    API.turnRight45()
    Mouse.heading = int(( Mouse.heading + 1)%8)
    
def turnLeft45():
    API.turnLeft45()
    Mouse.heading = int((Mouse.heading + 7)%8)   
    
def turn_to_direction(target_dir):
    # Calculate the difference between current heading and target direction in the range [-3, 3]
    diff = (target_dir - Mouse.heading ) % 8

    if diff == 1:  # Right turn
        turnRight45()

    elif diff == 2:  # Turn around (180 degrees)
        turnRight()

    elif diff == 3:  
        turnRight()
        turnRight45()
    elif diff == 4:  
        turnRight()
        turnRight()
    elif diff == 5:  
        turnLeft()
        turnLeft45()
        
    elif diff == 6:  
        turnLeft()
    elif diff == 7:  
        turnLeft45()
    elif diff == 0: 
        return   
 
def dir_of_lowest(x, y, maze):
    min = 255
    dir = Direction.NORTH
    if read_wall(x,y,Direction.NORTH, maze)==0:    
        if maze[x][y+1].dist < min: 
            min = maze[x][y+1].dist
            dir = Direction.NORTH
    if read_wall(x,y,Direction.EAST, maze)==0:    
        if maze[x+1][y].dist < min: 
            min = maze[x+1][y].dist
            dir = Direction.EAST
    if read_wall(x,y,Direction.SOUTH, maze)==0:    
        if maze[x][y-1].dist < min: 
            min = maze[x][y-1].dist
            dir = Direction.SOUTH
    if read_wall(x,y,Direction.WEST, maze)==0:    
        if maze[x-1][y].dist < min: 
            min = maze[x-1][y].dist
            dir = Direction.WEST
    return dir
                    
def flood(ex, ey, maze):
    nochange_flag = 0
    maze[ex][ey].dist = 0
    API.setText(ex, ey, f'{maze[ex][ey].dist}')
    while nochange_flag == 0:
        change_flag = 0     
        # maze[ex][ey].dist = 0
        for x in range(MAZE_CELL_WIDTH):
            for y in range(MAZE_CELL_HEIGHT): 
                # if not (x== ex and y== ey) and not ((maze[x][y].walls & 0x0F ) == 0x0F):                     
                if not (x== ex and y== ey):                     
                    min = MAZE_CELL_HEIGHT * MAZE_CELL_WIDTH -1
                    if read_wall(x,y,Direction.NORTH, maze)==0:    
                        if maze[x][y+1].dist < min: min = maze[x][y+1].dist
                    if read_wall(x,y,Direction.EAST, maze)==0:    
                        if maze[x+1][y].dist < min: min = maze[x+1][y].dist
                    if read_wall(x,y,Direction.SOUTH, maze)==0:    
                        if maze[x][y-1].dist < min: min = maze[x][y-1].dist
                    if read_wall(x,y,Direction.WEST, maze)==0:    
                        if maze[x-1][y].dist < min: min = maze[x-1][y].dist
                        
                    if (maze[x][y].dist != min+1) and (min !=MAZE_CELL_HEIGHT * MAZE_CELL_WIDTH -1 ): 
                        change_flag = 1
                        maze[x][y].dist = min + 1
                    
                    API.setText(x, y, f'{maze[x][y].dist}')
        if change_flag == 0: nochange_flag=1
            
   
def at_goal():
    return Mouse.current_cell_x == END_CELL_X and Mouse.current_cell_y == END_CELL_Y

def get_mid_direction(a , b):
    if (a==0 and b ==2) or (a==2 and b ==0): return 1
    if (a==2 and b ==4) or (a==4 and b ==2): return 3
    if (a==4 and b ==6) or (a==6 and b ==4): return 5
    if (a==6 and b ==0) or (a==0 and b ==6): return 7
dirs = []
dists = []
path = []

def get_path():
    race_maze = maze
    for i in range(MAZE_CELL_WIDTH):
        for j in range (MAZE_CELL_HEIGHT):
            if get_explored(i, j , race_maze) == 0:
                add_wall(i, j, Direction.EAST, race_maze)
                add_wall(i, j, Direction.SOUTH, race_maze)
                add_wall(i, j, Direction.WEST, race_maze)
                add_wall(i, j, Direction.NORTH, race_maze)
    flood(END_CELL_X, END_CELL_Y, race_maze)
    xt = 0
    yt = 0
    len = 0
    while not (xt == END_CELL_X and yt == END_CELL_Y):
        p = dir_of_lowest(xt,yt, race_maze)
        path.append(p) 
        len = len + 1
        if (p == Direction.NORTH):
            yt += 1
        elif (p == Direction.EAST):
            xt += 1
        elif (p == Direction.SOUTH):
            yt -= 1
        elif (p == Direction.WEST):
            xt -= 1  
    log(path)    
    j = 0
    while (j < len ):
        diag_flag = 0
        diag = []
        hcounts = 2
        diag.append(path[j])
        if j < len - 2  :
            if (path[j] == path[j+2]) and not (path[j]==path[j+1]):
                diag_flag = 1
                diag.append(path[j+1])
                diag.append(path[j+2])
                i = j+3
                while (i < len):
                    if (path[i] == path[i-2]):
                        hcounts += 1 
                        diag.append(path[i])
                        i += 1
                    else:
                        break
        if (diag_flag == 0):
            dirs.append(path[j])
            dists.append(2)
            j += 1
        else:
            dirs.append(diag[0])
            dists.append(1)
            dirs.append(get_mid_direction(int(diag[0]), int(diag[1])))
            dists.append(hcounts)
            dirs.append(diag[-1])
            dists.append(1)
            j += hcounts + 1
        log(diag)
   
    log(dirs) 
    log(dists) 

def race(dir, dist):
    log("in race")
    log(dir)
    log(dist)
    for i in range(len(dir)) :
        turn_to_direction(dir[i])
        for _ in range(int(dist[i])):
            API.moveForwardHalf()  
            
def find_all_paths(start, goal, maze):
    def dfs(current, path, all_paths):
        # Add the current cell to the path
        path.append(current)

        # If we've reached the goal, add the path to the list of all paths
        if current == goal:
            all_paths.append(list(path))  # Make a copy of the current path
        else:
            # Explore neighboring cells
            for neighbor in get_neighbors(current, maze):
                if neighbor not in path:  # Avoid revisiting cells in the current path
                    dfs(neighbor, path, all_paths)

        # Backtrack: remove the current cell from the path
        path.pop()

    all_paths = []
    dfs(start, [], all_paths)
    return all_paths

def get_neighbors(cell, maze):
    x, y = cell
    neighbors = []
    
    # Check if neighboring cells are open (no wall blocking the path)
    if not read_wall(x, y, Direction.NORTH, maze):
        neighbors.append((x, y + 1))
    if not read_wall(x, y, Direction.EAST, maze):
        neighbors.append((x + 1, y))
    if not read_wall(x, y, Direction.SOUTH, maze):
        neighbors.append((x, y - 1))
    if not read_wall(x, y, Direction.WEST, maze):
        neighbors.append((x - 1, y))
    # log(f'{neighbors}')

    return neighbors
def convert_to_dirs(paths):
    updated = []
    for path in paths:
        temp = []
        prev_x = 0
        prev_y = 0
        dir = 0
        for cell in path:
            x, y = cell
            if x==0 and y==0: continue
            
            if prev_y > y: dir = Direction.SOUTH
            elif prev_y < y: dir = Direction.NORTH
            elif prev_x < x: dir = Direction.EAST
            elif prev_x >  x: dir = Direction.WEST
                
            prev_x = x
            prev_y = y
            temp.append(dir)

        updated.append(temp)
    return updated

def detect_diagonals(paths):
    alldirs = []
    alldists = []
    for path in paths:
        tempdir = []
        tempdist = []
        le = len(path)
        j = 0
        while (j < le ):
            diag_flag = 0
            diag = []
            hcounts = 2
            diag.append(path[j])
            if j < le - 2  :
                if (path[j] == path[j+2]) and not (path[j]==path[j+1]):
                    diag_flag = 1
                    diag.append(path[j+1])
                    diag.append(path[j+2])
                    i = j+3
                    while (i < le):
                        if (path[i] == path[i-2]):
                            hcounts += 1 
                            diag.append(path[i])
                            i += 1
                        else:
                            break
            if (diag_flag == 0):
                tempdir.append(path[j])
                tempdist.append(2)
                j += 1
            else:
                tempdir.append(diag[0])
                tempdist.append(1)
                tempdir.append(get_mid_direction(int(diag[0]), int(diag[1])))
                tempdist.append(hcounts)
                tempdir.append(diag[-1])
                tempdist.append(1)
                j += hcounts + 1
            # log(diag)
        alldirs.append(tempdir)
        alldists.append(tempdist)
    return alldirs, alldists
def compress_path(dir, dist):
    outdir = []
    outdist = []
    l = len(dir)
    i = 0
    while i < l:
        sum = dist[i]
        j = i+1
        while  j < l:
            if dir[j]==dir[j-1]: 
                sum += dist[j]
            else: break 
            j += 1
        outdir.append(dir[i])
        outdist.append(sum)
        i+=j-i
    return outdir, outdist
def score_path(dir, dist):
    score = 0.5 * (len(dist) -1) #num turns
    for i ,d in enumerate(dist):
        if (dir[i] == 1) or (dir[i] == 3)  or (dir[i] == 5) or (dir[i] == 7) :
            score += 0.211*d
            # score += 0.111*d
        
        else: score += 0.15*d
        # else: score += 0.2*d
    return score
def len_path(dir, dist):
    score = 0
    for i, d in enumerate(dist):
        if (dir[i] == 1) or (dir[i] == 3)  or (dir[i] == 5) or (dir[i] == 7) :
            score += 0.705*d
        
        else: score += 0.5*d
    return score
if __name__ == "__main__":
    maze_init()

    while not at_goal():
        turn_to_direction(dir_of_lowest(Mouse.current_cell_x,Mouse.current_cell_y, maze))
        while (not API.wallFront()) and (dir_of_lowest(Mouse.current_cell_x,Mouse.current_cell_y, maze) == rel_to_fixed_dir(RelativeDirection.STRAIGHT)):
            moveForward()
            flood(END_CELL_X, END_CELL_Y, maze)
            
    log('goal')
    while not (Mouse.current_cell_x == 0 and Mouse.current_cell_y == 0):
        flood(0,0, maze)
        turn_to_direction(dir_of_lowest(Mouse.current_cell_x,Mouse.current_cell_y, maze))
        moveForward()
        
    log('home')
    flood(END_CELL_X, END_CELL_Y, maze)
    # get_path()  
    # race()  
    start = (0, 0)  # Starting cell coordinates
    goal = (END_CELL_X, END_CELL_Y)  # Goal cell coordinates

    # while not at_goal():
    #     turn_to_direction(dir_of_lowest(Mouse.current_cell_x,Mouse.current_cell_y, maze))
    #     while (not API.wallFront()) and (dir_of_lowest(Mouse.current_cell_x,Mouse.current_cell_y, maze) == rel_to_fixed_dir(RelativeDirection.STRAIGHT)):
    #         moveForward()
    #         flood(END_CELL_X, END_CELL_Y, maze)
            
    # log('goal')
    # while not (Mouse.current_cell_x == 0 and Mouse.current_cell_y == 0):
    #     flood(0,0, maze)
    #     turn_to_direction(dir_of_lowest(Mouse.current_cell_x,Mouse.current_cell_y, maze))
    #     moveForward()
        
    # log('home')
    # flood(END_CELL_X, END_CELL_Y, maze)
    # # get_path()  
    # # race()  
    # start = (0, 0)  # Starting cell coordinates
    # goal = (END_CELL_X, END_CELL_Y)  # Goal cell coordinates
    
    # while not at_goal():
    #     turn_to_direction(dir_of_lowest(Mouse.current_cell_x,Mouse.current_cell_y, maze))
    #     while (not API.wallFront()) and (dir_of_lowest(Mouse.current_cell_x,Mouse.current_cell_y, maze) == rel_to_fixed_dir(RelativeDirection.STRAIGHT)):
    #         moveForward()
    #         flood(END_CELL_X, END_CELL_Y, maze)
            
    # log('goal')
    # while not (Mouse.current_cell_x == 0 and Mouse.current_cell_y == 0):
    #     flood(0,0, maze)
    #     turn_to_direction(dir_of_lowest(Mouse.current_cell_x,Mouse.current_cell_y, maze))
    #     moveForward()
        
    # log('home')
    # flood(END_CELL_X, END_CELL_Y, maze)
    # # get_path()  
    # # race()  
    # start = (0, 0)  # Starting cell coordinates
    # goal = (END_CELL_X, END_CELL_Y)  # Goal cell coordinates
    
    race_maze = maze
    for i in range(MAZE_CELL_WIDTH):
        for j in range (MAZE_CELL_HEIGHT):
            if get_explored(i, j , race_maze) == 0:
                add_wall(i, j, Direction.EAST, race_maze)
                add_wall(i, j, Direction.SOUTH, race_maze)
                add_wall(i, j, Direction.WEST, race_maze)
                add_wall(i, j, Direction.NORTH, race_maze)
    all_possible_paths = find_all_paths(start, goal, race_maze)
    log(f"Number of possible paths: {len(all_possible_paths)}")
    # for path in all_possible_paths:
    #     log(path)
    all_possible_paths = convert_to_dirs(all_possible_paths)
    for path in all_possible_paths:
        log(path)
        
    dirs, dists = detect_diagonals(all_possible_paths)
    min = 9999
    min_index = 0
    for i in range(len(dirs)):
        log(dirs[i])
        log(dists[i])
        log("compressed:")
        dirs[i], dists[i] = compress_path(dirs[i], dists[i])
        log(dirs[i])
        log(dists[i])
        
        score = score_path(dirs[i], dists[i])
        if score < min:
            min = score
            min_index = i
        log(score)
    log("race path:") 
    log(f'length num cells: {len_path(dirs[min_index], dists[min_index])}')
    log(f'turns : {len(dists[min_index])-1}')
    log(f'score:{score_path(dirs[min_index], dists[min_index])}')
    race(dirs[min_index], dists[min_index])
