import fcl
import numpy as np

# Create two box shapes (1×1×1)
box1 = fcl.Box(1.0, 1.0, 1.0)
box2 = fcl.Box(1.0, 1.0, 1.0)

# Identity rotation
rot = np.eye(3)

# Positions (overlapping)
pos1 = np.array([0.0, 0.0, 0.0])
pos2 = np.array([0.5, 0.0, 0.0])  # overlaps box1

# Transforms
tf1 = fcl.Transform(rot, pos1)
tf2 = fcl.Transform(rot, pos2)

# Collision objects
obj1 = fcl.CollisionObject(box1, tf1)
obj2 = fcl.CollisionObject(box2, tf2)

# Request & result
req = fcl.CollisionRequest()
res = fcl.CollisionResult()

# Run collision test
ret = fcl.collide(obj1, obj2, req, res)

print("Collision detected:", res.is_collision)
