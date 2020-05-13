import numpy as np
class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)
    close_error=0
    open_error=0
    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node

        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)
        print(current_node.position)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        adj_list=[(0, -1,0), (0, 1,0), (-1, 0,0), (1, 0,0), (-1, -1,0), (-1, 1,0), (1, -1,0), (1, 1,0),(0, -1,1), (0, 1,1), (-1, 0,1), (1, 0,1), (-1, -1,1), (-1, 1,1), (1, -1,1), (1, 1,1),(0, -1,-1), (0, 1,-1), (-1, 0,-1), (1, 0,-1), (-1, -1,-1), (-1, 1,-1), (1, -1,-1), (1, 1,-1),(0,0,1),(0,0,-1)]
        
        for new_position in adj_list: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1],current_node.position[2] + new_position[2])

            # Make sure within range
            if node_position[0] > 9 or node_position[0] < 0 or node_position[1] > 9 or node_position[1] < 0 or node_position[2] > 9 or node_position[2] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]][node_position[2]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    close_error=1
            if close_error==1:
                close_error=0
                continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2) + ((child.position[2] - end_node.position[2]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g >=open_node.g:
                    open_error=1
            if open_error==1:
                open_error=0
                continue

            # Add the child to the open list
            open_list.append(child)


def main():

    maze = np.zeros((10,10,10))

    start = (0, 0,0)
    end = (7, 6, 5)

    path = astar(maze, start, end)
    print(path)
    #print(len(maze[len(maze)-1]))


if __name__ == '__main__':
    main()