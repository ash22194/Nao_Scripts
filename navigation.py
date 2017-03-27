#!/usr/bin/env python3

import heapdict as hp
import math
import pdb

global rotation_cost
rotation_cost = 1
global translation_cost
translation_cost = 1
global grid_size_x
global grid_size_y
grid_size_x = 37
grid_size_y = 23
global pi 
pi = math.pi
global target_state
target_state = (-31,5,0)
global sqrt
sqrt = math.sqrt
global ball_position
ball_position = (2,2,0)


def main():
    # This is a sample main function that shows examples and expected
    # outputs of calling the heuristic and astar functions.
    #
    # It is called in the conditional at the bottom if this file
    # is the main file your interpreter is running on and it is
    # not called when this file is used as a module for our grading.
    # You do not have to delete this function before submitting
    # the file for grading.

    # initState = (2, 3, 6, 1, 0, 5, 4, 7, 8)
    # initState = (0, 2, 3, 4, 5, 6, 7, 8, 1)
    # initState = (1, 2, 3, 4, 0, 8, 7, 6, 5)
    # initState = (1, 2, 3, 4, 5, 6, 8, 7, 0)
    # initState = (7, 5, 8, 2, 4, 6, 3, 1, 0)
    # initState = (2, 1, 3, 5, 4, 6, 7, 8, 0)
    global pi
    x = -30
    y = -23
    theta = 0
    initState = (x,y,theta)
    print('Neighbouring states : ')
    neighbour_states, h_, move = compute_neighbours(initState)
    # for i in range(len(neighbour_states)):
    #     print('   + {}'.format(neighbour_states[i]),'Heuristic : ',h_[i],'Move : ',move[i])
    path, states = astar(initState, heuristic_euclidian)
    for i in range(len(path)):
        print('Move : ',path[len(path) - 1 - i])

    # print('=== heuristic_misplaced')
    # print(' + Expected value: {}'.format(7))
    # print(' +     Your value: {}'.format(heuristic_misplaced(initState)))
    # print('=== heuristic_manhattan')
    # print(' + Expected value: {}'.format(8))
    # print(' +     Your value: {}'.format(heuristic_manhattan(initState)))
    # print('=== astar_misplaced')
    # expectedPath = 'ulddrurd'
    # expectedStates = [
    #     (2, 8, 3, 1, 0, 5, 4, 7, 6),
    #     (2, 8, 3, 1, 5, 0, 4, 7, 6),
    #     (2, 8, 3, 1, 5, 6, 4, 7, 0),
    #     (2, 0, 3, 1, 8, 5, 4, 7, 6),
    #     (0, 2, 3, 1, 8, 5, 4, 7, 6),
    #     (1, 2, 3, 0, 8, 5, 4, 7, 6),
    #     (1, 2, 3, 4, 8, 5, 0, 7, 6),
    #     (1, 2, 3, 4, 8, 5, 7, 0, 6),
    #     (1, 2, 3, 4, 0, 5, 7, 8, 6),
    #     (1, 2, 3, 4, 5, 0, 7, 8, 6),
    #     (1, 2, 3, 4, 5, 6, 7, 8, 0)
    # ]
    # path, states = astar(initState, heuristic_misplaced)
    # print(' + Expected path: {}'.format(expectedPath))
    # print(' +     Your path: {}'.format(path))
    # print(' + Expected states:')
    # for i in range(len(expectedStates)):
    #     print('   + {}'.format(expectedStates[i]))
    # print(' + Your states:')
    # print(len(states))
    # # for i in range(len(states)):
    # #     print('   + {}'.format(states[i]))

    # print('=== astar_manhattan')
    # expectedPath = 'ulddrurd'
    # expectedStates = [
    #     (2, 8, 3, 1, 0, 5, 4, 7, 6),
    #     (2, 0, 3, 1, 8, 5, 4, 7, 6),
    #     (0, 2, 3, 1, 8, 5, 4, 7, 6),
    #     (1, 2, 3, 0, 8, 5, 4, 7, 6),
    #     (1, 2, 3, 4, 8, 5, 0, 7, 6),
    #     (1, 2, 3, 4, 8, 5, 7, 0, 6),
    #     (1, 2, 3, 4, 0, 5, 7, 8, 6),
    #     (1, 2, 3, 4, 5, 0, 7, 8, 6),
    #     (1, 2, 3, 4, 5, 6, 7, 8, 0)
    # ]
    # path, states = astar(initState, heuristic_manhattan)
    # print(' + Expected path: {}'.format(expectedPath))
    # print(' +     Your path: {}'.format(path))
    # print(' + Expected states:')
    # for i in range(len(expectedStates)):
    #     print('   + {}'.format(expectedStates[i]))
    # print(' + Your states:')
    # print(len(states))
    # # for i in range(len(states)):
    # #     print('   + {}'.format(states[i]))

def heuristic_misplaced(state):
    """
    The number of misplaced tiles.
    The blank space is not tile and should
    not be included in your misplaced tile count.

    :param state: A tuple of the flattened board.
    :return: The number of misplaced tiles.
    """
    
    # [TODO: Your implementation here]
    a = 0
    for i in range(len(state)):
        if ((state[i]!=(i+1))&(state[i]!=0)):
            a = a + 1

    return a # TODO: Replace this with your value.

def heuristic_euclidian(state,target):

    global rotation_cost
    global translation_cost
    global sqrt

    t = (state[0] - target[0])**2 + (state[1] - target[1])**2
    t = sqrt(t)
    theta = state[2]*pi/4
    theta_target = target[2]*pi/4
    r = min(abs(theta - theta_target),2*pi - abs(theta - theta_target))
    h = t*translation_cost + r*rotation_cost

    return h

def heuristic_manhattan(state):
    """
    The sum of the Manhattan distances from the
    misplaced tiles to their correct positions.*

    :param state: A tuple of the flattened board.
    :return: The summed manhattan distances.
    """

    # [TODO: Your implementation here]
    a = 0
    for i in range(len(state)):
        if (state[i]!=0):
            x_d = (state[i] - 1)%3
            y_d = int((state[i] - x_d - 1)/3)
            x = (i)%3
            y = int((i - x)/3)
            a = a + abs(x_d - x) + abs(y_d - y)

    return a # TODO: Replace this with your value.

def compute_neighbours(state):
    """
    Given a state computes the neighbouring states in the graph
    :state: A tuple of the flattened board
    :neighbour_states: List of tuple of neighbouring states
    """
    global rotation_cost
    global translation_cost
    global grid_size
    global pi
    global sqrt

    front_translation_cost = translation_cost
    side_translation_cost = 2*translation_cost
    back_translation_cost = 3*translation_cost
    neighbour_states = []
    g_neighbours = dict()
    move = []

    x = state[0]
    y = state[1]
    theta = state[2]*pi/4
    if (theta==2*pi):
        theta = 0
        state[2] = theta

    theta_1 = theta + pi/4
    theta_2 = theta - pi/4
    if (theta_2 < 0):
        theta_2 = theta_2 + 2*pi
    if (theta_1 > 6.2):
        theta_1 = 0

    orientation = int(theta/(pi/4))
    if (orientation%2==1):
        front_translation_cost = front_translation_cost*sqrt(2)
        side_translation_cost = side_translation_cost*sqrt(2)
        back_translation_cost = back_translation_cost*sqrt(2)

        for i in [-1,0,1]:
            for j in [-1,0,1]:
                x_ = x + i
                y_ = y + j

                # pdb.set_trace();
                if (not((x_ < -grid_size_x) | (x_ > grid_size_x) | (y_ < -grid_size_y) | (y_ > grid_size_y) | ((x_==ball_position[0]) & (y_==ball_position[1])))):
                # if (not((x_ < 0) | (x_ > grid_size_x) | (y_ < 0) | (y_ > grid_size_y) | ((x_==ball_position[0]) & (y_==ball_position[1])))):

                    if ((i+j)%2==1):
                        continue
                    else:
                        if ((i==0) & (j==0)):
                            continue
                        # pdb.set_trace()
                        state_ = (x_,y_,orientation)
                        neighbour_states.append(state_)
                        move.append((x_ - x,y_ - y,0))
                        if (i*math.sin(theta) - j*math.cos(theta) > 0):
                            cost = round(front_translation_cost,3)
                        if (i*math.sin(theta) - j*math.cos(theta) < 0):                            
                            cost = round(back_translation_cost,3)
                        if (abs(i*math.sin(theta) - j*math.cos(theta)) < 0.1):  
                            cost = round(side_translation_cost,3)

                        g_neighbours[state_] = (cost)

    else:
        for i in [-1,0,1]:
            for j in [-1,0,1]:
                x_ = x + i
                y_ = y + j
                # pdb.set_trace()
                # if (not((x_ < 0) | (x_ > grid_size_x) | (y_ < 0) | (y_ > grid_size_y) | ((x_==ball_position[0]) & (y_==ball_position[1])))):
                if (not((x_ < -grid_size_x) | (x_ > grid_size_x) | (y_ < -grid_size_y) | (y_ > grid_size_y) | ((x_==ball_position[0]) & (y_==ball_position[1])))):

                    if ((i+j)%2==0):
                        continue
                    else:
                        state_ = (x_,y_,orientation)
                        neighbour_states.append(state_)
                        move.append((x_ - x,y_ - y,0))
                        if (i*math.sin(theta) - j*math.cos(theta) > 0):
                            cost = round(front_translation_cost,3)
                        if (i*math.sin(theta) - j*math.cos(theta) < 0):                            
                            cost = round(back_translation_cost,3)
                        if (abs(i*math.sin(theta) - j*math.cos(theta)) < 0.1):  
                            cost = round(side_translation_cost,3)

                        g_neighbours[state_] = (cost)

    neighbour_states.append((x,y,int(theta_1/(pi/4))))
    # g_neighbours.append(round(rotation_cost*pi/4,3))
    g_neighbours[(x,y,int(theta_1/(pi/4)))] = round(rotation_cost*pi/4,3)
    move.append((0,0,1))
    neighbour_states.append((x,y,int(theta_2/(pi/4))))
    # g_neighbours.append(round(rotation_cost*pi/4,3))
    g_neighbours[(x,y,int(theta_2/(pi/4)))] = round(rotation_cost*pi/4,3)
    move.append((0,0,-1))

    return neighbour_states, g_neighbours, move

def check_validity(state):
    """
    Given a state computes the number of inversions and determines if it is valid
    :state: A tuple of the flattened board
    :valid: A bool variable which is true if the state is a valid state 
    """
    inversion = 0
    for i in range(len(state)):
        if (state[i]==0):
            continue
        for j in range(i+1,len(state)):
            if (state[j]==0):
                continue
            if (state[j] < state[i]):
                inversion = inversion + 1

    if (inversion%2==0):
        return True
    else:
        return False

def astar(init_state, heuristic):
    """
    A^* implementation.

    :param init_state: A tuple of the flattened board.
    :param heuristic: The heuristic function
    :return: A tuple where:
        The first element is a string representing the optimal path.
            Use the characters 'r', 'l', 'u', and 'd' for
            'right', 'left', 'up', and 'down' directions, respectively.
        The second element is a list that contains states in the
            in order they were visited by your algorithm.
    """


    # [TODO: Your implementation here]
    global target_state
    states_visited = []
    mov = dict()    
    open_states = hp.heapdict()
    closed_states = hp.heapdict()
    moves = dict()
    next = init_state
    states_visited.append(init_state)
    g = dict()
    g[next] = 0
    no_path = False

    while (heuristic(next,target_state)!=0):
        g_ = g[next]
        f = g_ + heuristic(next,target_state)
        # closed_states[next] = int(str(f) + (''.join(map(str,next))))
        closed_states[next] = f
        neighbour_states, g__, moves_ = compute_neighbours(next)
        h = []
        for i in range(len(neighbour_states)):
            h.append(heuristic(neighbour_states[i],target_state))

        for i in range(len(neighbour_states)):
            if (closed_states.get(neighbour_states[i])==None):
                # pdb.set_trace()
                g[neighbour_states[i]] = g_ + g__[neighbour_states[i]]
                f = h[i] + g_ + g__[neighbour_states[i]]
                # open_states[neighbour_states[i]] = (f,neighbour_states[i])
                open_states[neighbour_states[i]] = (f)                
                moves[neighbour_states[i]] = (moves_[i],next)
                # open_states[neighbour_states[i]] = int(str(h[i] + g_ + 1) + (''.join(map(str,neighbour_states[i]))))

        if (len(open_states)==0):
            no_path = True
            print("No path found!")
            break
        else:
            next = open_states.popitem()
            next = next[0]
            states_visited.append(next)    
            mov[next] = moves[next]

    if (not(no_path)):
        m = []
        while(next!=init_state):
            state = mov[next]
            m.append(state[0])
            next = state[1]
    else:
        m = None

    return m, states_visited # TODO: Replace this with your values.

if __name__=='__main__':
    main()
