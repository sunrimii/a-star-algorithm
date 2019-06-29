import matplotlib.pyplot as plt
import numpy as np

class Map:
    def __init__(self):
        self.w = 10
        self.h = 10
        self.map_ = [Node(x, y) for y in range(self.h) for x in range(self.w)]

    def __call__(self, x, y):
        within_boundary = (0 <= x < self.w) and (0 <= y < self.h)
        return self.map_[y*self.w + x] if within_boundary else None

class Node:
    def __init__(self, x, y):
        self.is_obstacle = False
        self.pos = (x, y)
        self.parent = None

        self.g = 0
        self.h = None
        self.f = 0

    def get_reachalbe_nbr(self):
        """
        回傳: 此節點的可到達的鄰居節點 即過濾掉 超出邊界 或 是障礙物 或 在關閉列表 或 因邊點阻擋而不能過 的節點
        """
        nbr = []
        x, y = self.pos

        up = map_(x, y+1)
        down = map_(x, y-1)
        left = map_(x-1, y)
        right = map_(x+1, y)
        upper_left = map_(x-1, y+1)
        upper_right = map_(x+1, y+1)
        bottom_left = map_(x-1, y-1)
        bottom_right = map_(x+1, y-1)

        nbr = [up, down, left, right]

        if not(up and left and up.is_obstacle and left.is_obstacle):
            nbr.append(upper_left)
        if not(up and right and up.is_obstacle and right.is_obstacle):
            nbr.append(upper_right)
        if not(down and left and down.is_obstacle and left.is_obstacle):
            nbr.append(bottom_left)
        if not(down and right and down.is_obstacle and right.is_obstacle):
            nbr.append(bottom_right)
        
        nbr = [node for node in nbr if node and not(node.is_obstacle) and not(node in close)]

        return nbr
    
    def calc_h(self):
        """
        回傳: 此節點到終點之間的曼哈頓距離
        """
        self.h = abs(dest.pos[0] - self.pos[0]) + abs(dest.pos[1] - self.pos[1])

    def calc_g_of_step(self, node):
        """
        輸入: 任一鄰居節點
        回傳: 此節點到鄰居節點間的移動代價
        """
        dx = node.pos[0] - self.pos[0]
        dy = node.pos[1] - self.pos[1]
        in_corner = False if (dx is 0) or (dy is 0) else True
        return 14 if in_corner else 10

    def get_route(self, route):
        """
        輸入: 從終點開始遞迴的追蹤父節點直到起點 將路線放進傳入的空陣列
        """
        route.append(self.pos)
        if self.parent:
            self.parent.get_route(route)


# 初始化地圖
map_ = Map()
# 選定終點
dest = map_(9, 0)
# 選定起點
start = map_(0, 9)
# 放置障礙物
obstacles = [(1,9), (1,8), (1,7), (6,0), (6,1), (6,2), (6,3), (4,4), (4,5), (4,6), (5,4), (5,5), (5,6), (6,4), (6,5), (6,6)]
for x, y in obstacles:
    map_(x, y).is_obstacle = True

# 初始化開啟列表 關閉列表
open_ = [start]
close = []

while open_:
    # 選出f最小的節點為當前節點
    main_node = min(open_, key=lambda node: node.f)

    # 將當前節點 移出開啟列表 加入關閉列表
    open_.remove(main_node)
    close.append(main_node)

    # 找出當前節點的可到達的鄰居節點
    nbr = main_node.get_reachalbe_nbr()

    # 若鄰居節點是終點則結束循環
    if dest in nbr:
        dest.parent = main_node
        route = []
        dest.get_route(route)
        break

    # 遍歷每個可到達的鄰居節點
    for nbr_node in nbr:
        # 若沒計算過h則計算
        if nbr_node.h is None:
            nbr_node.calc_h()
        
        # 若鄰居節點在開啟列表裡
        if nbr_node in open_:
            tentative_g = main_node.g + main_node.calc_g_of_step(nbr_node)
            is_better_parent = tentative_g < nbr_node.g
            # 若當前節點對鄰居節點來說是更好的父節點
            if is_better_parent:
                nbr_node.parent = main_node
                nbr_node.g = tentative_g
                nbr_node.f = nbr_node.g + nbr_node.h
        # 若鄰居節點不在開啟列表裡
        else:
            open_.append(nbr_node)
            nbr_node.parent = main_node
            nbr_node.g = main_node.g + main_node.calc_g_of_step(nbr_node)
            nbr_node.f = nbr_node.g + nbr_node.h
else:
    route = None

# 顯示路線
img = np.ones((map_.h, map_.w))
for x, y in obstacles:
    img[y, x] = 2
for x, y in route:
    img[y, x] = 0
plt.imshow(img, interpolation="nearest", cmap="RdGy")
plt.show()