y1 = 10
y2 = 20

x1 = 5
x2 = 8

eps = 0

for x in range(int(y1) - eps, int(y2) + eps):
    for y in range(int(x1) - eps, int(x2) + eps):
        print(x, y)