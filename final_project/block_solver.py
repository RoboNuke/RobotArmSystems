from lang_parser import Problem, parseCmd
from math import sqrt

class BlockFinder():
    def __init__(self) -> None:
        self.prob = None

    def setProblem(self, prob):
        self.prob = prob

    def setScene(self, sg):
        self.sceneGraph = sg

    def solve(self):
        if prob.type == 'GLOBAL':
            return self.solveGlobal()
        elif prob.type == 'RELATIVE':
            return self.solveRelative()
        elif prob.type == 'GOAL':
            return self.solveGoal()

    def solveGlobal(self):
        # get extream point
        expt = [0,0]
        if self.prob.dir_constraint[0] > 0:  # rightmost
            expt[0] = self.sceneGraph.maxes[1]
        elif self.prob.dir_constraint[0] < 0: #leftmost
            expt[0] = self.sceneGraph.maxes[0]
        
        if self.prob.dir_constraint[1] > 0: # highest
            expt[1] = self.sceneGraph.maxes[3]
        elif self.prob.dir_constraint[1] < 0: # lowest
            expt[1] = self.sceneGraph.maxes[2]

        exDists = self.sceneGraph.getDistTo(expt)
        exDists = [sqrt(d[0]*d[0] + d[1]*d[1]) for d in exDists]
        blockIdx = exDists.index(min(exDists))
        return self.sceneGraph.getBlockByIdx(blockIdx)

    def solveRelative(self):
        # get reference 
        refBlock = self.getBlockByRef()
        # get dist to ref block
        dists = self.sceneGraph.getDistTo(refBlock[0])
        dists = [[-d[0],-d[1]] for d in dists] # invert so positive is (right, above) 
        # cut out by constraint
        lrc = self.prob.dir_constraint[0]
        udc = self.prob.dir_constraint[1]

        possIdxs = []
        for i in range(len(dists)):
            if (lrc > 0 and dists[i][0] > 0) or (lrc < 0 and dists[i][0] < 0) or lrc == 0:
                if( udc > 0 and dists[i][1] > 0) or (udc < 0 and dists[i][1] < 0) or udc == 0:
                    possIdxs.append(i)
        
        # get best solution
        best = 0
        bestD = 100000
        for idx in possIdxs:
            d = 0
            if not lrc == 0:
                d += dists[idx][0] ** 2
            if not udc == 0:
                d += dists[idx][1] ** 2
            d = sqrt(d)
            if d < bestD:
                best = idx
                bestD = d
        return self.sceneGraph.getBlockByIdx(best)

    def solveGoal(self):
        return self.getBlockByRef()

    def getBlockByRef(self):
        ref = prob.ref_block
        for block in self.sceneGraph.blocks:
            if block[1] == ref:
                outblock = block
                self.sceneGraph.blocks.remove(block)
                return outblock
        
class SceneGraph():
    def __init__(self) -> None:
        self.blocks = []
        self.maxes = [100000,-100000,100000,-100000]

    def addBlock(self, color, loc, idx):
        self.blocks.append([loc, color, idx])
        self.maxes[0] = min(self.maxes[0],loc[0]) # most left
        self.maxes[1] = max(self.maxes[1], loc[0]) # most right
        self.maxes[2] = min(self.maxes[2], loc[1]) # lowest
        self.maxes[3] = max(self.maxes[3], loc[1]) # highest

    def getDistTo(self, loc):
        dists = []
        for block in self.blocks:
            dlr = loc[0] - block[0][0]
            dud = loc[1] - block[0][1]
            dists.append([dlr,dud])
        return dists
    
    def getBlockByIdx(self, idx):
        return self.blocks[idx]

def getDefaultScene():
    # Test Scene Definition
    sg = SceneGraph()
    sg.addBlock('red', (5,5,0), 0)
    sg.addBlock('blue', (7,7,0), 1)
    sg.addBlock('green', (2,0,0), 2)
    sg.addBlock('yellow', (0,10,0), 3)
    sg.addBlock('purple', (10,2,0), 4)
    """ Map Key 
    Y-------  3
    -----B--  1
    ---R----  0
    ------P-  4
    -G------  2
    """
    return sg

if __name__=="__main__":
    test_cases = [
        "Grab the red block",
        "Grab the rightmost block",
        "Grab the lowest block",
        "Grab the block on the left of the red block",
        "Grab the block on the left and above the red block",
        "Grab the highest and leftmost block",
        "Grab the block on the right and below the red block",
        "Grab the block below the red block"
    ]

    test_answers = [
        0, 4, 2, 2, 3, 3, 4, 4
    ]
    bf = BlockFinder()
    for i in range(len(test_cases)):
        sg = getDefaultScene()
        prob = parseCmd(test_cases[i])
        bf.setProblem(prob)
        bf.setScene(sg)
        ans = bf.solve()
        print("Command:", test_cases[i])
        print("\tGot:", ans)
        print("\tCorrect Answer:", test_answers[i])
        print("\tSucceeded:", ans[2] == test_answers[i])