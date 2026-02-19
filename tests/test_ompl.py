from ompl import base as ob, geometric as og

def test0_ompl():
    def isStateValid(state):
        return True

    space = ob.RealVectorStateSpace(2)

    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
    si.setup()

    start = ob.State(space)
    start[0] = -0.5
    start[1] = 0.0

    goal = ob.State(space)
    goal[0] = 0.5
    goal[1] = 0.0

    pdef = ob.ProblemDefinition(si)
    pdef.setStartAndGoalStates(start, goal)

    planner = og.RRT(si)
    planner.setProblemDefinition(pdef)
    planner.setup()

    solved = planner.solve(1.0)

    if solved:
        print("✅ OMPL works! Found a solution.")
        print(pdef.getSolutionPath())
    else:
        print("❌ No solution found.")


def test2D_ompl():
    # Define a simple 2D state space

    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1); bounds.setHigh(1)
    space.setBounds(bounds)

    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(ob.StateValidityCheckerFn(lambda s: True))
    si.setup()

    start, goal = ob.State(space), ob.State(space)
    start[0], start[1] = -0.5, 0.0
    goal[0], goal[1]  =  0.5, 0.0

    pdef = ob.ProblemDefinition(si)
    pdef.setStartAndGoalStates(start, goal)

    planner = og.RRT(si)
    planner.setProblemDefinition(pdef)
    planner.setup()

    # Should print "Solved: True"
    print("Solved:", planner.solve(1.0))

test0_ompl()
test2D_ompl()