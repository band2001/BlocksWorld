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

        self.list.append([value, key])

    def delete_min(self):
        """
        :return: the node of the state with the smallest priority
        """
        min = 10000000
        index = -1
        # the smallest element to delete
        for item in range(len(self.list)):
            if self.list[item][1] < min:
                min = self.list[item][1]
                index = item

        toReturn = self.list.pop(index)
        return toReturn[0]



class Node:
    def __init__(self, state, parent=None, action=None, path_cost=0):
        """
        Tree Node Object
        :param state: the state of the block world problem
        :param parent: the state of the block world problem before the action taken to create this state
        :param action: the action taken to create this state
        :param path_cost: the cost/heuristic of moving to this state
        """
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0
        if self.parent:
            self.depth = self.parent.depth + 1


    def __str__(self):
        """
        string representation of the node
        :return: the action string
        """

        return self.action


    def path_back(self, node):
        """
        Used to find the solution to the block world problem
        :param node: the node of the goal state
        :return: returns a list of all the states taken to reach the goal state
        """

        solution_list = []

        while node:
            solution_list.append(node)
            node = node.parent

        path = reversed(solution_list)
        return path

    def solution(self, path):
        """
        Determines the actions taken to get to the goal state
        :param path: a list generated from the path_back method
        :return: a list with the actions taken to get to the goal state
        """
        solution = []
        for node in path:
            solution.append(node.action)

        return solution

    def expand(self, plan):
        """
        Used to determine all potential actions that could be taken from this state
        :param plan: generally self when called in a method of the Plan class; used to access Plan method functions
        :return: A list of new potential states based on allowed actions from this state
        """

        potential_moves = []  # list to store new potential states
        table = State.find(self.state, "table")  # finds the table

        for block in self.state:  # for each block in the state

            if block.clear:  # if the block does not have anything on it
                if block.on == table:  # if the block is on the table, a solution is picking up the block
                    new_state = copy.deepcopy(self.state)  # create a new_state to save this modified block object
                    block_m = State.find(new_state, block.id)  # block_m is a part of new_state
                    Plan.pickup(plan, block_m)  # change block_m to have the attributes of being picked up
                    new_action = Node(new_state, self, f"pickup({block_m})")  # convert this new state into a Node
                    potential_moves.append(new_action)  # append the Node of this new state to the potential moves list


                elif block.on and block.on != table:  # if the block is on another block
                    new_state = copy.deepcopy(self.state)  # create a new_state to save this modified block object
                    block_m = State.find(new_state, block.id)  # block_m is part of the new state
                    block_s = State.find(new_state, block.on.id)  # block_s is part of new state, block_m is about to be removed from being on top of it
                    Plan.unstack(plan, block_m, block_s)  # change block_m to no longer be stacked on block_s
                    new_action = Node(new_state, self, f"unstack({block_m}, {block_s})")  # put this new state into a Node
                    potential_moves.append(new_action)  # append the Node of this new state to the potential moves list

            elif block.air:  # if the block is in the air (picked up or unstacked)
                #if table.clear:  # if it can be put on the table
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
        self.initial_state = initial_state
        self.goal_state = goal_state

    #***=========================================
    # First implement all the operators
    # I implemented two operators to give you guys an example
    # Please implement the remainder of the operators
    #***=========================================

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

    def heuristic(self, state, goal):
        """
        Used to calculate distance from goal
        :param state: (list of block objects) a list of blocks in the current state
        :param goal: (list of block objects) a list of blocks in the goal state
        :return: (int) the cost/heuristic of each state
        """

        cost = 0  # if state is goal state, cost = 0
        for block in state:

            state_block = State.find(state, block.id)
            goal_block = State.find(goal, block.id)

            if state_block.on != goal_block.on:
                cost += 1

        return cost

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

        ##############################

        initial = Node(self.initial_state, action="Initial State")
        goal = Node(self.goal_state, action="Goal")
        pqueue = PriorityQueue()
        def plan(current_state_node):
            moves = current_state_node.expand(self)
            for move in moves:
                cost = self.heuristic(move.state, goal.state)
                if cost == 0:
                    return move
                pqueue.insert(move, cost)
            lowest_cost = pqueue.delete_min()
            State.display(lowest_cost.state)
            return plan(lowest_cost)
        solution_node = plan(initial)
        print("Solution: ")
        State.display(solution_node.state)
        path = Node.path_back(self, solution_node)
        solution = Node.solution(self, path)
        print(solution)
            



if __name__ == "__main__":

    # get the initial state
    initial_state = State()
    initial_state_blocks = initial_state.create_state_from_file("input2.txt")

    # display initial state
    State.display(initial_state_blocks, message="Initial State")

    # get the goal state
    goal_state = State()
    goal_state_blocks = goal_state.create_state_from_file("goal2.txt")

    #display goal state
    State.display(goal_state_blocks, message="Goal State")

    """
    Sample Plan
    """

    p = Plan(initial_state_blocks, goal_state_blocks)
    p.sample_plan()
    






