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
        raw_cmd = input("Which block should I grab?   ")
        if raw_cmd == "q":
            break

        # set problem and finder
        prob = parseCmd(raw_cmd)
        bf = BlockFinder()
        bf.setProblem(prob)

        # make the scene graph
        raw_img = SGP.getImg()
        while raw_img is not None:
            raw_img = SGP.getImg()
        if raw_img is not None:
            scene_graph, post_img = SGP.getGraph(raw_img)
            for block in scene_graph.blocks:
                print(block)
            if not SGP.displayImg(post_img):
                break
        
        bf.setScene(scene_graph)
        ans = bf.solve()

        RC.pick(ans[0][0], ans[0][1], ans[0][2])


    SGP.closeCamera()
