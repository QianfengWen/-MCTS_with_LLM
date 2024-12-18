# Description file for MCTS, all the codes follow sequential order

# ============================
# 1. Initialize parameters before MCTS
startEgoState = [] # [x y theta kappa speed acc] for the start state

# ============================
# 2. Ego vehicle has a predifined path, which is a list of waypoints
waypoints = [] # nx2 matrix [x y] for the waypoints

# ============================
# 3. Create a reference path using waypoints
def referencePathFrenet(waypoints):
    """
    Matlab API: referencePathFrenet
    Create a reference path using waypoints
    """
    pass

refPath = referencePathFrenet(waypoints)
# refPath = [x y theta kappa dkappa s]

# ============================
# 4. Convert the start state to frenet coordinates
def globalToFrenet(startEgoState, refPath):
    """
    Matlab API: globalToFrenet
    Convert the start state to frenet coordinates
    """
    pass

egoFrenetState = globalToFrenet(startEgoState, refPath) # [s ds dss l dl dll] for the start state

# ============================
# 5. Compute predicted positions for detected cars
# TODO: need to get from CARLA
predictedActPositions = [] # mx3 matrix [x y, z] for the predicted positions


# ============================
# Helper functions
def getDisplacement(node, jerkS, deltaL, TimeResolution):
    """
    Compute the displacement, speed, and lane change for a given node
    """
    pass

def checkCollision(node, newNode, predictedActPositions, egoVehicle, profiles, TimeResolution, scenario, refPath):
    """
    Check if a collision occurs between the ego vehicle and the predicted positions
    """
    pass

# ============================
# 6. Node class for MCTS
class Node:
    """
    Node class for MCTS
    parameters:
    - state: the state of the node in global coordinates [x y theta kappa speed acc]
    - time: the time of the node
    - children: the number of children of the node
    - visits: the number of visits to the node
    - score: the score of the node
    - index: the index of the node in the MCTS tree
    - parent: the parent of the node
    - UCB: the UCB value of the node
    - egoFrenetState: the state of the node in frenet coordinates [s ds dss l dl dll]
    - avgScore: the average score of the node
    - laneChangingProperties: the properties of the node, initialized to {'LeftChange': 0, 'RightChange': 0, 'Change': False}
    """
    def __init__(self):
        pass

# ============================
# 7. MCTS class
class MCTS:
    """
    MCTS class, currently in matlab we use a linear array to store the nodes, but in python we could use a tree structure
    """
    def __init__(self):
        pass

    def selectNode(self, currentNode):
        """
        Select the node with the highest UCB value
        """
        pass

    def expandNode(self, currentNode):
        """
        Expand the node by adding children nodes, including acceleration, deceleration, keeping speed and lane change
        """
        """
        For acceleration and deceleration:
        1. We sample a range of acceleration and deceleration
        2. Then we compute the corresponding jerk, longitudinal displacement
        """

        """
        For keeping speed:
        1. We let acceleration be 0, and compute the corresponding jerk, longitudinal displacement
        """

        """
        For lane change:
        1. We first have to know which lane we are in, also getting the lane width and sample legal lateral displacement
        2. Then we compute the corresponding jerk, deltaL, and TimeResolution
        """
        pass

    def rollout(self, currentNode):
        """
        Rollout the node by simulating the future, and use costFunction to compute the cost of the simulated node at the end of the rollout
        """
        pass

    def backpropagate(self, currentNode, score):
        """
        Backpropagate the cost to the root node
        """
        pass

    def runMCTS(self):
        """
        Run the MCTS algorithm
        """
        pass



def costFunction(node, newNode, checkPoint, predicted, MaxTimeHorizon, TimeResolution, egoVehicle, speedlimit, profiles, scenario):
    """
    Compute the cost of the simulated node at the end of the rollout
    parameters:
    - node: the node at the start of the rollout
    - newNode: the node at the end of the rollout
    - checkPoint: the check point for the total destination
    - predicted: the predicted positions of the detected cars
    - MaxTimeHorizon: the maximum time horizon of the rollout
    - TimeResolution: the time resolution of the rollout
    - egoVehicle: the ego vehicle
    - speedlimit: the speed limit of the road
    - profiles: the profiles of the detected cars (used for getting the size of the car to detect collision)
    - scenario: the scenario of the simulation in matlab, only used to get current timestamp
    """
    pass