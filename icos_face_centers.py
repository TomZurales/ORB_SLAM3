import numpy as np
from itertools import combinations
from scipy.spatial import ConvexHull

# Golden ratio
phi = (1 + 5**0.5) / 2

# Create 12 vertices of an icosahedron
verts = []
for i in [-1, 1]:
    for j in [-1, 1]:
        verts += [
            (0, i, j*phi),
            (i, j*phi, 0),
            (i*phi, 0, j),
        ]
verts = np.array(verts)

# Get the faces via convex hull
hull = ConvexHull(verts)
faces = hull.simplices

# Compute face centers and normalize them
face_centers = []
for tri in faces:
    pts = verts[tri]
    center = np.mean(pts, axis=0)
    unit_vector = center / np.linalg.norm(center)
    face_centers.append(unit_vector)

face_centers = np.array(face_centers)

print(face_centers)