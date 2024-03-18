


"""
Example Statements:

Grab the red block
Grab the block on the left
Grab the lowest block
Grab the block to the left of the red block
Grab the block highest and to the left
Grab the block right and below the red block
"""

class Problem():
    
    def __init__(self):
        self.TYPES = {"GLOBAL":0, "RELATIVE":1, "GOAL":2}
        self.type = None 

        self.ref_block = None

        # 0 - no constrain
        # 1 - greater (right, above)
        # -1 - lower (left, below)
        self.dir_constraint = [0,0]

        self.CONIDXLR = {'right':1, 'left':-1, 'leftmost':-1, 'rightmost':1}
        self.CONIDXUD = {'above':1, 'below':-1, 'highest':1, 'lowest':-1}

def parseCmd(cmd):
    words = cmd.split()[2:]
    #print(words)
    # check next work for 
    colors = ['red', 'blue', 'green']
    relativeLocs = ['right','left','above','below']
    extreamLocs = ['rightmost', 'leftmost', 'highest', 'lowest']
    problemOut = Problem()
    if words[0] in extreamLocs: # looking for block on extream
        extreams = []
        for word in words:
            if word in extreamLocs:
                extreams.append(word)
        #print("Init Cmd:", cmd)
        #for con in extreams:
            #print("\tExtream Constraint:", con)

        problemOut.type = "GLOBAL"
        for i in range(len(extreams)):
            if extreams[i] in problemOut.CONIDXLR:
                problemOut.dir_constraint[0] = problemOut.CONIDXLR[extreams[i]]      
            elif extreams[i] in problemOut.CONIDXUD:
                problemOut.dir_constraint[1] = problemOut.CONIDXUD[extreams[i]]
        return problemOut
    
    elif words[0] == 'block': # relative corelativemmand
        # get ref block
        words = words[1:]
        for i in range(len(words)-1, 0, -1):
            if words[i] == 'the':
                ref = words[i+1]
                words = words[:i]
                break
        # get other cmd lists
        conTokens = []
        for word in words:
            if word in relativeLocs:
                conTokens.append(word)
        # proof of success       
        #print("Init Cmd: "+ cmd)
        #print('\tref:', ref)
        #for conTon in conTokens:
        #    print('\tRelative Constraint:', conTon)

        problemOut.type = 'RELATIVE'
        problemOut.ref_block = ref
        for i in range(len(conTokens)):
            if conTokens[i] in problemOut.CONIDXLR:
                problemOut.dir_constraint[0] = problemOut.CONIDXLR[conTokens[i]]
            elif conTokens[i] in problemOut.CONIDXUD:
                problemOut.dir_constraint[1] = problemOut.CONIDXUD[conTokens[i]]
        return problemOut

        

    elif words[0] in colors: # grab specific color
        goal = words[0]
        problemOut.type = 'GOAL'
        problemOut.ref_block = goal
        #print("Init Cmd:", cmd)
        #print("\tGoal:", goal)
        return problemOut
    else:
        print(cmd, " not supported" )
    



if __name__=="__main__":
    test_cases = [
        "Grab the red block",
        "Grab the rightmost block",
        "Grab the lowest block",
        "Grab the block on the left of the red block",
        "Grab the highest and leftmost block",
        "Grab the block on the right and below the red block",
        "Grab the block below the red block"
    ]

    for case in test_cases:
        outy = parseCmd(case)
        print("\t",outy.type)
        print("\t", outy.ref_block)
        print("\t",outy.dir_constraint)

