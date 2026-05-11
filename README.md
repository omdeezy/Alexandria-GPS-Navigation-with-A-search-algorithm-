In the heuristic file you can see the (X,Y) columns , those are the real coordinates of the Alexandria places (obtained from Google maps).
The coordinates are used in the Euclidean distance formula [math.sqrt((x2 - x1)**2 + (y2 - y1)**2)*111] to give us the final heuristic value.
In the Euclidean distance formula you can see i multiplied the equation by 111 , this is because when coordinates are calculated it give you
a tiny heuristic value for example 0.043 , and when the heuristic value is this tiny the code ignores it and treats it like its nothing ,
basically it treats it like the BFS with no heuristic . thats why i multiplied it by 111.
In the comparison table you can clearly see that the nodes expanded by the A* is less than the BFS , thats mean the A* is better at searching
for the goal than the BFS.
You can run the Astar_with_gui file and choose a start point and a goal to see the A* road map and the estimated travel time and the traveled distance
in kilometers.
