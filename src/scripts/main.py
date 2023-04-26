"""
callback je potreba dodelat
while in_garage = false
    picture
    if 2f
        if 2f < 20
            goal = find_goal(PARK)
        else
            goal = find_goal(2f)
    elseif 1f
        goal = find_goal(1f)
    elseif yellow
        if yellow < 50
            fotka
            fitovani
            goal = find_goal(yellow_close)
        else
            goal = find_goal(yellow_far)
    else 
        goal = find_goal(ROTACE)
    path = find_path(goal)
    move(path)
"""