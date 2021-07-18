# -*- coding: utf-8 -*-
"""
Created on Mon Jul  5 21:51:51 2021

@author: dhruv
"""

# Note: I have used spyder as the environment. Steps to run. Please input the 
# sequence or comment out while running the file
def moving_back(move):
 
    l = len(move)
    countUp, countDown = 0, 0
    countLeft, countRight = 0, 0
 
    # traverse the instruction string
    # 'move'
    for i in range(l):
 
        # for each movement increment
        # its respective counter
        if (move[i] == 'U'):
            countUp += 1
 
        elif(move[i] == 'D'):
            countDown += 1
 
        elif(move[i] == 'L'):
            countLeft += 1
 
        elif(move[i] == 'R'):
            countRight += 1
 
    # required final position of robot
    print("Final Position: (", (countRight - countLeft),
          ", ", (countUp - countDown), ")")
    if countRight - countLeft == 0 and countUp - countDown == 0:
        print("True")
    else:
        print("False")
 
 
# Driver code
if __name__ == '__main__':
    move = input("")
    moving_back(move)
    