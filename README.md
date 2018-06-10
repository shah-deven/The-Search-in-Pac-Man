# The-Searchin-Pac-Man
## Depth-First Search (DFS)
Implemented the depth-first search (DFS) algorithm in the depthFirstSearch function in `search.py`. <br />
```
python pacman.py -l tinyMaze -p SearchAgent <br />
python pacman.py -l mediumMaze -p SearchAgent <br />
python pacman.py -l bigMaze -z .5 -p SearchAgent<br />
```

## Breadth-First Search (BFS)
Implemented the breadth-first search (BFS) algorithm in the breadthFirstSearch function in `search.py`.<br />
```
python pacman.py -l mediumMaze -p SearchAgent -a fn=bfs<br />
python pacman.py -l bigMaze -p SearchAgent -a fn=bfs -z .5<br />
```
The above solution is a generic solution which also works on eight puzzle<br />
`python eightpuzzle.py`

## Uniform-cost Search (UCS)
Implemented the uniform-cost search (UCS) algorithm in the uniformCostSearch function in `search.py` <br />
```
python pacman.py -l mediumMaze -p SearchAgent -a fn=ucs<br />
python pacman.py -l mediumDottedMaze -p StayEastSearchAgent<br />
python pacman.py -l mediumScaryMaze -p StayWestSearchAgent<br />
```

## A* Search
Implemented A* graph search in the function aStarSearch in search.py. A* takes a heuristic function as an argument. Heuristics take two arguments: a state in the search problem (the main argument), and the problem itself (for reference information). The nullHeuristic heuristic function in `search.py` is a trivial example.<br />
`python pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic `

## Finding all the Corners
Implemented the CornersProblem search problem in `searchAgents.py`. <br /><br />

In corner mazes, there are four dots, one in each corner. The new search problem is to find the shortest path through the maze that touches all four corners (whether the maze actually has food there or not). Note that for some mazes like tinyCorners, the shortest path does not always go to the closest food first! <br />

```
python pacman.py -l tinyCorners -p SearchAgent -a fn=bfs,prob=CornersProblem<br />
python pacman.py -l mediumCorners -p SearchAgent -a fn=bfs,prob=CornersProblem<br />
```

Implemented a heuristic for the CornersProblem in cornersHeuristic.<br />
`python pacman.py -l mediumCorners -p AStarCornersAgent -z 0.5<br />`

## Eating All The Dots
`python pacman.py -l testSearch -p AStarFoodSearchAgent<br />`

Implemented foodHeuristic in `searchAgents.py`<br />
`python pacman.py -l trickySearch -p AStarFoodSearchAgent`
