def motorLogic(currentPos, newPos):
    if currentPos == 0 and newPos == 2:
        #rotate ccw 3 revs
    elif currentPos == 2 and newPos == 0:
        #rotate cw 3 revs
    else:
        if currentPos < newPos:
            #rotate cw 3 revs
        else:
            #rotate ccw 3 revs