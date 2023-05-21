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
#print(path1)

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
        path2.append((350-k,300-k))
    if i > 400 and i <= 500:
        k = i -400
        path2.append((250,200+k))
#print(path2)
path2 = [] #rovne sikmo 45 sikm -45 (90) rovne //shorter
for i in range(50):
    if i <=10:
        path2.append((250,i)) #rovne
    if i > 10 and i <= 20:
        k = i -10
        path2.append((250+k,10+k))
    if i > 20 and i <= 30:
        k = i -20
        path2.append((260,20+k))
    if i > 30 and i <= 40:
        k = i -30
        path2.append((260-k,30-k))
    if i > 40 and i <= 50:
        k = i -40
        path2.append((250,20+k))
print(path2)