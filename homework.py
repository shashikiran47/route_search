import Queue
import collections
from time import time



def process_input(l):
    path_info = Pathinfo(destination=l[1], path_cost=int(l[2]))
    if l[0] not in adjacency_list:
        adjacency_list[l[0]] = [path_info]
    else:
        if algorithm == 'DFS':
            adjacency_list[l[0]].insert(0,path_info)
        else:
            adjacency_list[l[0]].append(path_info)


def process_sunday_input(li):
    sunday_traffic_info[li[0]] = int(li[1])


def bfs():
    explored = set()
    frontier = Queue.Queue()
    frontier.put(start_state)
    node = Node(parent='no_parent', path_cost=0)
    Tree[start_state] = node
    while frontier.qsize() > 0:
        parent = frontier.get()
        explored.add(parent)
        if parent == goal_State:
            break
        children = adjacency_list.get(parent)
        if children is None:
            continue
        for child in children:
            child_name =  child.destination
            if child_name not in explored and child_name not in frontier.queue:
                node = Node(parent=parent, path_cost= Tree[parent].path_cost + 1)
                Tree[child_name] = node
                frontier.put(child.destination)


def dfs():
    explored = set()
    frontier = Queue.LifoQueue()
    frontier.put(start_state)
    node = Node(parent='no_parent', path_cost=0)
    Tree[start_state] = node
    while frontier.qsize() > 0:
        parent = frontier.get()
        explored.add(parent)
        if parent == goal_State:
            break
        children = adjacency_list.get(parent)
        if children is None:
            continue
        for child in children:
            child_name = child.destination
            if child_name not in explored and child_name not in frontier.queue:
                node = Node(parent=parent, path_cost=Tree[parent].path_cost + 1)
                Tree[child_name] = node
                frontier.put(child_name)


def ucs():
    explored = set()
    frontier = Queue.PriorityQueue()
    frontier.put((1, 1, start_state))
    node = Node(parent='no_parent', path_cost=0)
    Tree[start_state] = node
    j=1
    while frontier.qsize() > 0:
        _, _, parent = frontier.get()
        explored.add(parent)
        if parent == goal_State:
            break
        children = adjacency_list.get(parent)
        if children is None:
            continue
        for child in children:
            j+=1
            already_present = False
            child_name = child.destination
            current_path_cost = Tree[parent].path_cost + child.path_cost
            node = Node(parent=parent, path_cost=current_path_cost)
            if child_name not in explored:
                for i in range(0,frontier.qsize()):
                    pathcost, secondpriority, nodename = frontier.queue[i]
                    if child_name == nodename:
                        already_present = True
                        if current_path_cost < pathcost:
                            ll = []
                            while (z != child_name):
                                ll.append((x, y, z))
                                x, y, z = frontier.get()
                            for gg in ll:
                                frontier.put(gg)
                            frontier.put((current_path_cost, j, nodename))
                            #frontier.queue[i]=(current_path_cost, j, nodename)
                            Tree[child_name]= node
                            break

                if not already_present:
                    frontier.put((current_path_cost, j, child_name))
                    Tree[child_name] = node


def astar():
    explored = set()
    frontier = Queue.PriorityQueue()
    frontier.put((1, 1, start_state))
    node = Node(parent='no_parent', path_cost=0)
    Tree[start_state] = node
    j=1
    while frontier.qsize() > 0:
        _, _, parent = frontier.get()
        explored.add(parent)
        if parent == goal_State:
            break
        children = adjacency_list.get(parent)
        if children is None:
            continue
        for child in children:
            j+=1
            already_present = False
            child_name = child.destination
            current_path_cost = Tree[parent].path_cost + child.path_cost + sunday_traffic_info[child_name]
            path_cost_uptonow = Tree[parent].path_cost + child.path_cost
            node = Node(parent=parent, path_cost=path_cost_uptonow)
            if child_name not in explored:
                for i in range(0,frontier.qsize()):
                    pathcost, secondpriority, nodename = frontier.queue[i]
                    if child_name == nodename:
                        already_present = True
                        if current_path_cost < pathcost:
                            ll = []
                            x,y,z = frontier.get()
                            while (z != child_name):
                                ll.append((x,y,z))
                                x, y, z = frontier.get()
                            for gg in ll:
                                frontier.put(gg)
                            frontier.put((current_path_cost,j, nodename))
                            #frontier.queue[i]=(current_path_cost, j, nodename)
                            Tree[child_name]= node
                            break

                if not already_present:
                    frontier.put((current_path_cost, j, child_name))
                    Tree[child_name] = node


def write_output():
    path = []
    output = open("output.txt", "w")
    current_state = goal_State
    initial_node = start_state + " " + "0" + "\n"
    while current_state != start_state:
        strin = current_state + " " + str(Tree[current_state].path_cost) + "\n"
        path.append(strin)
        current_state = Tree[current_state].parent

    output.write(initial_node)
    for current_node in reversed(path):
        output.write(current_node)

ll = []
Pathinfo = collections.namedtuple('Pathinfo', 'destination path_cost')
Node = collections.namedtuple('Node', 'parent path_cost')
adjacency_list = {}
sunday_traffic_info = {}
Tree = {}
input_file = open('input.txt');
algorithm = input_file.readline().rstrip('\n')
start_state = input_file.readline().rstrip('\n')
goal_State = input_file.readline().rstrip('\n')
number_traffic_lines = int(input_file.readline().rstrip('\n'))
i = 0
while i < number_traffic_lines:
    line = input_file.readline().rstrip('\n').split()
    process_input(line)
    i+=1
number_sunday_traffic_lines = int(input_file.readline().rstrip('\n'))
i = 0
while i < number_sunday_traffic_lines:
    line = input_file.readline().rstrip('\n').split()
    process_sunday_input(line)
    i+=1

if algorithm == 'BFS':
    bfs()
elif algorithm == 'DFS':
    dfs()
elif algorithm == 'UCS':
    ucs()
else:
    astar()

write_output()









