import math
import numpy as np
import matplotlib.pyplot as plt
def simplify_points (pts, tolerance): 
    anchor  = 0
    floater = len(pts) - 1
    stack   = []
    keep    = set()

    stack.append((anchor, floater))  
    while stack:
        anchor, floater = stack.pop()
      
        # инициализация отрезка
        if pts[floater] != pts[anchor]:
            anchorX = float(pts[floater][0] - pts[anchor][0])
            anchorY = float(pts[floater][1] - pts[anchor][1])
            seg_len = math.sqrt(anchorX ** 2 + anchorY ** 2)
            # get the unit vector
            anchorX /= seg_len
            anchorY /= seg_len
        else:
            anchorX = anchorY = seg_len = 0.0
    
        # внутренний цикл:
        max_dist = 0.0
        farthest = anchor + 1
        for i in range(anchor + 1, floater):
            dist_to_seg = 0.0
            # compare to anchor
            vecX = float(pts[i][0] - pts[anchor][0])
            vecY = float(pts[i][1] - pts[anchor][1])
            seg_len = math.sqrt( vecX ** 2 + vecY ** 2 )
            # dot product:
            proj = vecX * anchorX + vecY * anchorY
            if proj < 0.0:
                dist_to_seg = seg_len
            else: 
                # compare to floater
                vecX = float(pts[i][0] - pts[floater][0])
                vecY = float(pts[i][1] - pts[floater][1])
                seg_len = math.sqrt( vecX ** 2 + vecY ** 2 )
                # dot product:
                proj = vecX * (-anchorX) + vecY * (-anchorY)
                if proj < 0.0:
                    dist_to_seg = seg_len
                else:  # расстояние от до прямой по теореме Пифагора:
                    dist_to_seg = math.sqrt(abs(seg_len ** 2 - proj ** 2))
                if max_dist < dist_to_seg:
                    max_dist = dist_to_seg
                    farthest = i

        if max_dist <= tolerance: # использование отрезка
            keep.add(anchor)
            keep.add(floater)
        else:
            stack.append((anchor, farthest))
            stack.append((farthest, floater))

    keep = list(keep)
    keep.sort()
    return [pts[i] for i in keep]

odometria = []
lidar = []
with open('examp11.txt', 'r') as f:
    for line in f:
        odometria_temp = (line[:line.find(";")].split(", "))
        lidar_temp = (line[line.find(";") + 1:].split(", "))
        odometria.append(odometria_temp)
        lidar.append(lidar_temp)
i = 0
cloudX = []
cloudY = []
for x, y, fi in odometria:
  fi0 = float(fi) + 2.0944  # вычисляем самую левую точку сектора
  for l in lidar[i]:
    if float(l) != 5.6 and float(l) >= 0.4:
      cloudX.append((float(l) * np.cos(float(fi0))) + float(x) + 0.3 * np.cos(float(fi)))
      cloudY.append((float(l) * np.sin(float(fi0))) + float(y) + 0.3 * np.sin(float(fi)))
    fi0 -= 0.0063
  i += 1
plt.scatter(cloudX, cloudY, s = 0.2)
plt.show
mass = []
x = []
y = []
for i in range(len(cloudX)):
  mass.append([cloudX[i], cloudY[i]])
new_points = simplify_points(mass, 0.05)
for point in new_points:
  x.append(point[0])
  y.append(point[1])
plt.scatter(x, y, s = 1)
plt.show()
