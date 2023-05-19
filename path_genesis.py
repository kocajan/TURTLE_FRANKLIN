path1 = [] #rovne 90 doprava 90 doleva 90 doleva 90 do prava
for i in range(500):
    if i <=100:
        path1.append((250,i)) #rovne
    if i > 100 and i <= 200:
        k = i -100
        path1.append((250+k,100))
    if i > 200 and i <= 300:
        k = i -200
        path1.append((350,100+k))
    if i > 300 and i <= 400:
        k = i -300
        path1.append((350-k,200))
    if i > 400 and i <= 500:
        k = i -400
        path1.append((250,200+k))
print(path1)

path2 = [] #rovne sikmo 45 sikm -45 (90) rovne
for i in range(500):
    if i <=100:
        path2.append((250,i)) #rovne
    if i > 100 and i <= 200:
        k = i -100
        path2.append((250+k,100+k))
    if i > 200 and i <= 300:
        k = i -200
        path2.append((350,200+k))
    if i > 300 and i <= 400:
        k = i -300
        path2.append((350-k,200-k))
    if i > 400 and i <= 500:
        k = i -400
        path2.append((250,100+k))
print(path2)