###=================================================
# This file is where you need to create a plan to reach the goal state form the initial state
# This file must accept any combination of with the given blocks: A, B, C, D, E
# This file should also reach the final state of any combination with the blocks above
# It must also display all intermediate states
###=================================================

from state import State

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

    def sample_plan(self):

        # get the specific block objects
        # Then, write code to understand the block i.e., if it is clear (or) on table, etc.
        # Then, write code to perform actions using the operators (pick-up, stack, unstack).

        # Below I manually hardcoded the plan for the current initial and goal state
        # You must automate this code such that it would produce a plan for any initial and goal states.

        block_c = State.find(self.initial_state, "C")
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
        State.display(self.initial_state, message=action)



if __name__ == "__main__":

    # get the initial state
    initial_state = State()
    initial_state_blocks = initial_state.create_state_from_file("input.txt")

    #display initial state
    State.display(initial_state_blocks, message="Initial State")

    # get the goal state
    goal_state = State()
    goal_state_blocks = goal_state.create_state_from_file("goal.txt")

    #display goal state
    State.display(goal_state_blocks, message="Goal State")

    """
    Sample Plan
    """

    p = Plan(initial_state_blocks, goal_state_blocks)
    p.sample_plan()
    






