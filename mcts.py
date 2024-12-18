import numpy as np

# set parameters
TIME = 0
max_iter = 50
accMax = 5
limitJerk = 15
speedlimit = 20
MaxTimeHorizon = 2.0
MaxRolloutHorizon = 2.0
TimeResolution = 1.0
refPath = [] # TODO: set refPath


# TODO: set start state, make sure it aligns with carla
# start state for ego vehicle
startEgoState = np.array([0, 0, 0, 0, 0]) # [x y theta kappa speed acc]
# start state for ego vehicle in frenet frame
egoFrenetState = np.array([0, 0, 0, 0, 0]) # [s ds dss l dl dll]

# TODO: compute predicted positions for detected cars
predictedActPositions = [] # list of predicted positions for detected cars

# Matlab API
def frenet2global(refPath, egoFrenetState):
    # TODO: implement frenet2global
    return np.array([0, 0, 0, 0, 0]) # [x y theta kappa speed acc]

def getDisplacement(node, jerkS, deltaL, TimeResolution):
    deltaT = TimeResolution;
    deltaSpeedS = node.egoFrenetState[3] * TimeResolution + 1/2 * jerkS * TimeResolution**2;
    displacementS = node.egoFrenetState[2] * deltaT + 1/2 * node.egoFrenetState[3] * TimeResolution**2 + 1/6 * jerkS * TimeResolution**3;
    deltaSpeedL = 0;
    displacementL = deltaL;
    return displacementS, deltaSpeedS, displacementL, deltaSpeedL

def checkCollision(node, newNode, predictedActPositions, egoVehicle, profiles, TimeResolution, scenario, refPath):
    # TODO: implement collision check
    return False

class Node:
    """
    Node class for MCTS
    """
    def __init__(self):
        self.visits = 1
        self.time = TIME
        self.state = startEgoState # [x y theta kappa speed acc]
        self.children = 0
        self.index = 0
        self.score = 0
        self.parent = 0
        self.UCB = np.inf
        self.egoFrenetState = egoFrenetState # [s ds dss l dl dll]
        self.laneChangingProperties = {'LeftChange': 0, 'RightChange': 0, 'Change': False}
        self.avgScore = 0


class MCTS:
    """
    MCTS class
    """
    def __init__(self):
        self.root = Node()
        self.iter = 0
        self.bestNode = self.root
        self.tree = {1: self.root}

    def selectNode(self, currentNode):
        """
        Select the node with the highest UCB value
        """
        while len(currentNode.children) != 1:
            bestChild = self.tree[currentNode.children[1]]  # First child after index 0
            for i in range(1, len(currentNode.children)):
                child = self.tree[currentNode.children[i]]
                if child.UCB >= bestChild.UCB:
                    bestChild = child
            currentNode = bestChild
        return currentNode

    def expandNode(self, currentNode, refPath):
        """
        Expand the node with actions
        """
        if currentNode.time >= MaxTimeHorizon:
            return self.tree
            
        # Emergency brake node
        emergencyBrakeNode = Node()
        emergencyBrakeNode.state = currentNode.state.copy()
        emergencyBrakeNode.time = currentNode.time + TimeResolution
        emergencyBrakeNode.index = len(self.tree) + 1
        emergencyBrakeNode.parent = currentNode.index
        
        # # Calculate emergency brake trajectory
        emergencyAcc = -(2 * currentNode.egoFrenetState[1] / TimeResolution) - currentNode.egoFrenetState[2]
        emergencyAcc = max(emergencyAcc, -8)  # Limit deceleration
        emergencyJerkS = (emergencyAcc - currentNode.egoFrenetState[2]) / TimeResolution
        
        # # Update state with emergency brake
        displacementS, deltaSpeedS, displacementL, deltaSpeedL = getDisplacement(
            currentNode, emergencyJerkS, 0, self.TimeResolution)
        
        emergencyBrakeNode.egoFrenetState = currentNode.egoFrenetState + np.array([
            displacementS, deltaSpeedS, emergencyJerkS * TimeResolution,
            displacementL, deltaSpeedL, 0])
            
        # # Add node if no collision
        if not self.checkCollision(currentNode, emergencyBrakeNode):
            currentNode.children.append(emergencyBrakeNode.index)
            self.tree[emergencyBrakeNode.index] = emergencyBrakeNode 

        # accelerate node
        for acc in range(1, self.accMax + 1):
            # Slowing down, we ignore the situation of going backward, so we only
            # expand a node with negative acc if and only if the speed is bigger
            # than 1m/s.
            if currentNode.egoFrenetState[2] > 1:
                jerk1 = (-acc - currentNode.egoFrenetState[3]) / TimeResolution;
                
                decelerationNode = Node()
                decelerationNode.state = currentNode.state.copy()
                decelerationNode.time = currentNode.time + self.TimeResolution
                decelerationNode.index = len(self.tree) + 1
                decelerationNode.parent = currentNode.index
                [displacementS1, deltaSpeedS1, displacementL1, deltaSpeedL1] = getDisplacement(currentNode, jerk1, 0, TimeResolution)
                decelerationNode.laneChangingProperties.Change = False;
                decelerationNode.egoFrenetState = decelerationNode.egoFrenetState + [displacementS1, deltaSpeedS1, jerk1 * TimeResolution, displacementL1, deltaSpeedL1, 0];
                decelerationNode.state = frenet2global(refPath, decelerationNode.egoFrenetState);
                feasible = decelerationNode.egoFrenetState[2] + (acc + 3) * TimeResolution * 0.5;
                if not checkCollision(currentNode, decelerationNode, predictedActPositions, TimeResolution, refPath) and decelerationNode.egoFrenetState[2] > 0 and feasible >= 0:
                    currentNode.children.append(decelerationNode.index)
                    self.tree[decelerationNode.index] = decelerationNode

                # acceleration section
                jerk2 = (acc - currentNode.egoFrenetState[3]) / TimeResolution;
                accelerationNode = Node()
                accelerationNode.state = currentNode.state.copy()
                accelerationNode.time = currentNode.time + TimeResolution
                accelerationNode.index = len(self.tree) + 1
                accelerationNode.parent = currentNode.index
                [displacementS2, deltaSpeedS2, displacementL2, deltaSpeedL2] = getDisplacement(currentNode, jerk2, 0, TimeResolution)
                accelerationNode.laneChangingProperties.Change = False;
                accelerationNode.egoFrenetState = accelerationNode.egoFrenetState + [displacementS2, deltaSpeedS2, jerk2 * TimeResolution, displacementL2, deltaSpeedL2, 0];
                accelerationNode.state = frenet2global(refPath, accelerationNode.egoFrenetState);
                feasible = accelerationNode.egoFrenetState[2] + (acc + 3) * TimeResolution * 0.5;
                if not checkCollision(currentNode, accelerationNode, predictedActPositions, TimeResolution, refPath) and accelerationNode.egoFrenetState[2] > 0 and feasible >= 0:
                    currentNode.children.append(accelerationNode.index)
                    self.tree[accelerationNode.index] = accelerationNode
                
        # TODO: implement lane changing


        return self.tree

    def rollout(self, node):
        """
        Rollout simulation from node
        """
        cost = 0
        currNode = node
        
        while currNode.time < MaxTimeHorizon:
            # Random action selection
            rand_num = np.random.random()
            rand_acc = np.random.randint(1, accMax + 1)
            
            newNode = Node()
            newNode.time = currNode.time + TimeResolution
            
            if rand_num <= 0.1:
                # Deceleration
                deltaAcc = -rand_acc - currNode.egoFrenetState[2]
                jerk = deltaAcc / TimeResolution
                
                # Update state
                displacementS, deltaSpeedS, displacementL, deltaSpeedL = getDisplacement(
                    currNode, jerk, 0, TimeResolution)
                    
                newNode.egoFrenetState = currNode.egoFrenetState + np.array([
                    displacementS, deltaSpeedS, jerk * TimeResolution,
                    displacementL, deltaSpeedL, 0])
                
                newNode.state = frenet2global(refPath, newNode.egoFrenetState)
                if checkCollision(currNode, newNode):
                    cost = -10000
                    break                 
            elif rand_num >= 0.9:
                # speed up
                deltaAcc = rand_acc - currNode.egoFrenetState[3];
                jerk = deltaAcc / TimeResolution;

                newNode.egoFrenetState = currNode.egoFrenetState + np.array([displacementS3, deltaSpeedS3, jerk * TimeResolution, displacementL3, deltaSpeedL3, 0])
                newNode.state = frenet2global(refPath, newNode.egoFrenetState)
                if checkCollision(currNode, newNode):
                    cost = -10000
                    break
            else:
                # keep speed
                deltaAcc = -currNode.egoFrenetState[3];
                jerk = deltaAcc / TimeResolution;

                [displacementS3, deltaSpeedS3, displacementL3, deltaSpeedL3] = getDisplacement(currNode, jerk, 0, TimeResolution)

                newNode.egoFrenetState = currNode.egoFrenetState + np.array([displacementS3, deltaSpeedS3, jerk * TimeResolution, displacementL3, deltaSpeedL3, 0])
                newNode.state = frenet2global(refPath, newNode.egoFrenetState)
                if checkCollision(currNode, newNode):
                    cost = -10000
                    break

            currNode = newNode
            
        return cost + self.calculateCost(node, currNode)

    def backpropagate(self, node, score):
        """
        Backpropagate score through tree
        """
        while node.parent != 0:
            self.tree[node.index].score += score
            self.tree[node.index].visits += 1
            node = self.tree[node.parent]
            
        self.tree[node.index].score += score 
        self.tree[node.index].visits += 1


    def updateUCB(self, node):
        # this function updates the ucb of all nodes in the tree, using a bfs
        # starting from the root
        queue = [node]
        while queue:
            currNode = queue[0]
            queue.pop(0)
            if currNode.visits == 0:
                self.tree[currNode.index].UCB = np.inf
                self.tree[currNode.index].avgScore = np.inf
            else:
                self.tree[currNode.index].UCB = currNode.score / currNode.visits + 5 * np.sqrt(np.log(self.tree[1].visits) / currNode.visits)
                self.tree[currNode.index].avgScore = self.tree[currNode.index].score / self.tree[currNode.index].visits
            if len(currNode.children) != 1:
                for i in range(1, len(currNode.children) - 1):
                    queue.append(self.tree[currNode.children[i + 1]])

    
    def run(self):
        while self.iter < max_iter:
            self.iter += 1
            self.expandNode(self.root, refPath)
            self.updateUCB(self.root)
            self.selectNode(self.root)
            self.rollout(self.root)
            self.backpropagate(self.root, self.rollout(self.root))


def costFunction(node, newNode, checkPoint, predicted, MaxTimeHorizon, TimeResolution, egoVehicle, speedlimit, profiles, scenario):
    # TODO: implement cost function
    return 0    


if __name__ == "__main__":
    mcts = MCTS()
    mcts.run()


