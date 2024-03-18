from block_solver import BlockFinder, SceneGraph
from scene_graph_perception import SceneGraphPerception
from lang_parser import parseCmd, Problem
from robotControlRework import RobotControl


if __name__ == "__main__":
    SGP = SceneGraphPerception()
    RC = RobotControl()
    SGP.startCamera()
    #for i in range(10):
    while True:
        bf = BlockFinder()
        print ("Problem Set")
        # make the scene graph
        for i in range(3):
            raw_img = SGP.getImg()
            while raw_img is None:
                raw_img = SGP.getImg()
            print("Got Image")
            if raw_img is not None:
                scene_graph, post_img = SGP.getGraph(raw_img)
                for block in scene_graph.blocks:
                    print(block)
                if not SGP.displayImg(post_img):
                    break
        bf.setScene(scene_graph)
        print("Scene Graph Map")
        raw_cmd = input("Which block should I grab?   ")
        #raw_cmd = "Grab the red block"
        if raw_cmd == "q":
            break
        if raw_cmd == "":
            continue
        # set problem
        prob = parseCmd(raw_cmd)
        bf.setProblem(prob)
        ans = bf.solve()
        print("Picking the block")
        RC.pick(ans[0][0], ans[0][1], ans[0][2])
        RC.placeInBin(ans[1])


    SGP.closeCamera()
