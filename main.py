###=================================================
# This file is where you need to create a plan to reach the goal state form the initial state
# This file must accept any combination of with the given blocks: A, B, C, D, E
# This file should also reach the final state of any combination with the blocks above
# It must also display all intermediate states
###=================================================
import copy
from state import State


class PriorityQueue:
    def __init__(self):
        """
        Priority Queue Class: used in the A* Implementation
        """
        self.list = []  # a list of [value, key] where value is a node representing a state and key is its heuristic

    def insert(self, value, key):
        """
        :param value: the state to inject, injected as a node
        :param key: the heuristic of the state being injected
        :return: none
        """

        self.list.append([value, key])  # appends an element to a priority queue since the plain implementation is used

    def delete_min(self):
        """
        :return: the node of the state with the smallest priority
        """
        minimum = 1000000  # a large enough minimum that no key will be greater than it
        index = None  # the smallest element to delete
        for item in range(len(self.list)):  # looping over each item in the queue, where item is an index
            if self.list[item][1] < minimum:  # checks if the key is smaller than the current smallest key
                minimum = self.list[item][1]  # updates the new smallest key
                index = item  # updates the index of the element with the smallest key

        toReturn = self.list.pop(index)  # removes the element with the smallest key and stores it in toReturn
        return toReturn[0]  # returns the element with the smallest key


class Node:
    def __init__(self, state, parent=None, action=None, path_cost=0):
        """
        Tree Node Object
        :param state: the state of the block world problem (list of block objects)
        :param parent: the state of the block world problem before the action taken to create this state (Node)
        :param action: the action taken to create this state (string)
        :param path_cost: the cost/heuristic of moving to this state
        """
        self.state = state  # a list of block objects representing the state
        self.parent = parent  # the node of representing previous state (undoing the action to reach this state)
        self.action = action  # the action taken to reach this state, a string
        self.path_cost = path_cost  # the total cost of the Node (for priority queue, determined by depth + heuristic)
        self.depth = 0  # the depth in the tree
        if self.parent:  # sets the depth of the node in the tree
            self.depth = self.parent.depth + 1  # if the node has a parent, the depth is 1 + the parent's depth

    def __str__(self):
        """
        string representation of the node
        :return: the action string
        """

        return self.action

    def path_back(self, node):
        """
        Used to find the solution to the block world problem (list of nodes)
        :param node: the node of the goal state
        :return: returns a list of all the states represented as nodes taken to reach the goal state
        """

        solution_list = []  # list to store nodes of the states in the solution

        while node:  # while the node exists (i.e. is not the parent of the root which is None)
            solution_list.append(node)  # add the node to the solution list
            node = node.parent  # the node becomes the parent of the node

        path = reversed(solution_list)  # reverse the list, so it is root to end
        return path  # return the list of nodes (representing states) in the solution

    def solution(self):
        """
        Determines the actions taken to get to the goal state
        :param path: a list generated from the path_back method
        :return: a list of the states as a list of block objects and a list with the actions (string form) taken to get to the goal state
        """
        path = self.path_back(self)  # gets the list of nodes in the solution using the path_back method

        state_list_solution = []  # a list of the lists of block objects representing states (state element of node class)
        string_solution = []  # a list of strings describing the actions taken to reach each state
        for node in path:  # for each node in the solution
            state_list_solution.append(node.state)  # add node.state to get the list of block objects
            string_solution.append(node.action)  # add node.action to get the string representation of the action taken

        return state_list_solution, string_solution  # return both lists to use with State.display

    def expand(self, plan):
        """
        Used to determine all potential actions that could be taken from this state
        :param plan: generally self when called in a method of the Plan class; used to access Plan method functions
        :return: A list of new potential states based on allowed actions from this state
        """
        potential_moves = []  # list to store new potential states
        table = State.find(self.state, "table")  # finds the table

        in_air = False  # bool defining if any block is in the air in a given state
        for block in self.state:  # for each block in the state
            if block.air:  # if any block is in the air
                in_air = True  # in_air is true, meaning the block must be stacked or put down

        for block in self.state:  # for each block in the state

            if block.clear and not in_air:  # if the block does not have anything on it and no block is in the air
                if block.on == table:  # if the block is on the table, a solution is picking up the block
                    new_state = copy.deepcopy(self.state)  # create a new_state to save this modified block object
                    block_m = State.find(new_state, block.id)  # block_m is a part of new_state
                    Plan.pickup(plan, block_m)  # change block_m to have the attributes of being picked up
                    new_action = Node(new_state, self, f"pickup({block_m})")  # convert this new state into a Node
                    potential_moves.append(new_action)  # append the Node of this new state to the potential moves list


                elif block.on and block.on != table and not in_air:  # if the block is on another block and no block is in the air
                    new_state = copy.deepcopy(self.state)  # create a new_state to save this modified block object
                    block_m = State.find(new_state, block.id)  # block_m is part of the new state
                    block_s = State.find(new_state,
                                         block.on.id)  # block_s is part of new state, block_m is about to be removed from being on top of it
                    Plan.unstack(plan, block_m, block_s)  # change block_m to no longer be stacked on block_s
                    new_action = Node(new_state, self,
                                      f"unstack({block_m}, {block_s})")  # put this new state into a Node
                    potential_moves.append(new_action)  # append the Node of this new state to the potential moves list

            elif block.air:  # if the block is in the air (picked up or unstacked)
                new_state = copy.deepcopy(self.state)  # create a new state to save the modified block object
                block_m = State.find(new_state, block.id)  # block_m is from new state; to be modified
                Plan.putdown(plan, block_m)  # put block_m on the table (in the new state)
                new_action = Node(new_state, self, f"putdown({block_m})")  # create a Node for the new state
                potential_moves.append(new_action)  # append the Node of the new state to the potential moves list

                for b in self.state:  # for each block in the current state
                    if b.clear and block != b:  # if that block is clear (i.e. a block can be placed on top of it)
                        new_state = copy.deepcopy(self.state)  # create a new state to save the modified block object
                        block_m = State.find(new_state, block.id)  # block_m from new state, to be modified
                        block_s = State.find(new_state, b.id)  # block_s from new state, to be modified
                        Plan.stack(plan, block_m, block_s)  # block_m is placed on block_s in the new state
                        new_action = Node(new_state, self, f"stack({block_m}, {block_s})")  # Node for new state created
                        potential_moves.append(new_action)  # add Node for new state to potential_moves

        return potential_moves  # return potential moves


class Plan:

    def __init__(self, initial_state, goal_state):
        """
        Initialize initial state and goal state
        :param initial_state: list of blocks in the initial state
        :type initial_state: list of block.Block objects
        :param goal_state: list of blocks in the goal state
        :type initial_state: list of block.Block objects
        """
        self.initial_state = initial_state  # assigning the initial state
        self.goal_state = goal_state  # assigning the goal state

    # ***=========================================
    # First implement all the operators
    # I implemented two operators to give you guys an example
    # Please implement the remainder of the operators
    # ***=========================================

    def putdown(self, block1):
        """
        Operator to put the block on the table
        :param block1: block1 to put on the table
        :type block1: Object of block.Block
        :return: None
        """

        # get table object from initial state
        table = State.find(self.initial_state, "table")

        if block1.air:
            block1.on = table
            block1.clear = True
            block1.air = False

    def unstack(self, block1, block2):
        """
        Operator to unstack block1 from block 2

        :param block1: block1 to unstack from block2
        :type block1: Object of block.Block
        :type block2: Object of block.Block
        :return: None
        """

        # if block1 is clear safe to unstack
        if block1.clear:
            # block1 should be in air
            # block1 should not be on block2
            # set block2 to clear (because block1 is in air)
            block1.clear = False
            block1.air = True
            block1.on = None

            block2.clear = True

    def pickup(self, block):
        """
        Operator to pick block up from table
        :param block: block to pick up from table
        :type block: Object of block.Block
        :return: None
        """

        table = State.find(self.initial_state, "table")

        # if block is on table and able to be picked up (nothing stacked on top)
        if block.on == table and block.clear:
            # block1 should not be clear as it is in the air
            # block1 should not be on anything
            block.clear = False
            block.air = True
            block.on = None

    def stack(self, block1, block2):
        """
        Operator to stack block on top of another block
        :param block1: block1 to stack on top of block2
        :type block1: An object of block.Block
        :param block2: block2 will have block1 stacked on top of it
        :type block2: An object of block.Block
        :return: None
        """
        # if block1 is in the air and block2 is clear:
        if block1.air and block2.clear:
            # block1 is no longer in the air and is on block2
            block1.air = False
            block1.on = block2
            block1.clear = True

            # block2 is no longer clear
            block2.clear = False

    def move(self, block1, block2, block3):
        """
        Operator to move block1 from being stacked on block2 to being stacked on block3
        :param block1: the block object to move
        :param block2: the block object that block1 is initially on top of
        :param block3: the block object that block1 should end up on top of
        :type block1, block2, block3: block.Block objects
        :return: None
        """

        if block1.clear and block3.clear:
            # block1 is removed from block2
            block2.clear = True

            # block1 is moved to block3
            block3.clear = False
            block1.on = block3

    # ***=========================================
    # After you implement all the operators
    # The next step is to implement the actual plan.
    # Please fill in the sample plan to output the appropriate steps to reach the goal
    # ***=========================================

    def heuristic(self, state, goal):  # used to compare how similar the current state is with the goal state
        """
        Used to calculate distance from goal
        :param state: (list of block objects) a list of blocks in the current state
        :param goal: (list of block objects) a list of blocks in the goal state
        :return: (int) the cost/heuristic of each state
        """

        cost = 0  # if state is goal state, cost = 0
        for block in state:  # for each block in the current state

            state_block = State.find(state, block.id)  # find the ID of the block in the current state
            goal_block = State.find(goal, block.id)  # find the ID of the block in the goal state

            ### The heuristic is determined by comparing which objects the blocks are on; the more blocks that are on
            ### (in) the correct places, the closer to the goal state we are.

            if state_block.on != goal_block.on:  # if the blocks are not on the same block/table
                cost += 10  # add 10 to the heuristic
            elif state_block.air:
                cost -= 2  # subtract 2 from heuristic

        return cost  # return the cost

    ## A* SEARCH IMPLEMENTATION ##

    def a_star(self, initial, goal):
        """
        a_star search to find the goal state (solution to problem)
        :param initial: the initial state
        :type initial: list of block objects
        :param goal: the goal state
        :type goal: list of block objects
        :return: a list containing the states to get from the initial state to the goal state
        """

        initial_cost = self.heuristic(initial.state, goal.state)  # get the cost of the initial state
        if initial_cost == 0:  # checks if initial state is goal state
            return initial.state, ["Initial State == Goal State"]  # returns the states and actions taken to reach the solution

        explored = []  # a list to store explored states

        frontier = PriorityQueue()  # the frontier for A*, uses a plain priority queue

        initial_node = Node(initial.state, None, action="Initial State", path_cost=initial_cost)  # represent initial state as a node
        frontier.insert(initial_node, initial_node.path_cost)  # adds the initial state (as a node) to the frontier

        while frontier.list:  # while the frontier is not empty
            current_state = frontier.delete_min()  # gets the state (node) with the highest priority (smallest key)
            explored.append(current_state)  # adds that state to the explored list

            if self.heuristic(current_state.state, goal.state) == 0:  # checks if the current state is the goal state
                return current_state.solution()  # returns the states and actions taken to reach the solution

            neighbor_list = current_state.expand(self)  # identifies all possible moves from the current state (finds all neighbors/children)
            for new_state in neighbor_list:  # for each neighbor/child
                if new_state not in explored:  # if the neighbor/child has not already been explored
                    new_state.path_cost = self.heuristic(new_state.state, goal.state) + new_state.depth
                    frontier.insert(new_state, new_state.path_cost)  # add neighbor/child to frontier
                    ## NOTE: the key/priority is determined by the value of heuristic + value of depth for the child

        return [], ["No solution"]  # return no solution if the frontier is empty and no solution has been reached
                                    # should not happen since there is infinite table space

    def sample_plan(self):

        # get the specific block objects
        # Then, write code to understand the block i.e., if it is clear (or) on table, etc.
        # Then, write code to perform actions using the operators (pick-up, stack, unstack).

        # Below I manually hardcoded the plan for the current initial and goal state
        # You must automate this code such that it would produce a plan for any initial and goal states.

        '''block_c = State.find(self.initial_state, "C")
        block_d = State.find(self.initial_state, "D")

        # Unstack the block
        self.unstack(block_d, block_c)

        # print the state
        action = f"unstack{block_d, block_c}"
        State.display(self.initial_state, message=action)

        # put the block on the table
        self.putdown(block_d)

        # print the state
        action = f"Putdown({block_d}, table)"
        State.display(self.initial_state, message=action)'''

        ###############################

        ## USED TO TEST EXPAND FUNCTION
        '''expansion = test.expand(self)
        for i in expansion:
            print(self.heuristic(i.state, my_goal.state), i)

        expansion2 = expansion[1].expand(self)
        for i in expansion2:
            print(self.heuristic(i.state, my_goal.state), i)'''

        ##############################
        """
        sample_plan: executes the plan to solve the blocks problem
        return: none
        state change: prints out ascii blocks and steps to reach the solution
        """

        init = Node(self.initial_state, action="Initial State")  # initial state as node
        goal = Node(self.goal_state, action="Goal")  # goal state as node
        solution_state, solution_string = self.a_star(init, goal)   # executes the A* Search algorithm

        for i in range(len(solution_state)):  # displays each step
            State.display(solution_state[i], message=solution_string[i])



if __name__ == "__main__":
    # get the initial state
    initial_state = State()
    initial_state_blocks = initial_state.create_state_from_file("input2.txt")

    # display initial state
    State.display(initial_state_blocks, message="Initial State")

    # get the goal state
    goal_state = State()
    goal_state_blocks = goal_state.create_state_from_file("goal2.txt")

    # display goal state
    State.display(goal_state_blocks, message="Goal State")

    """
    Sample Plan
    """

    p = Plan(initial_state_blocks, goal_state_blocks)
    p.sample_plan()
