# An-Group-01-TW
# COS30019 Assignment 2A - Tree Based Search

## Overview
This project implements several search algorithms for the Route-Finding Problem from **COS30019 Introduction to AI**.

The program reads a graph problem from a text file and searches for a path from the **origin** node to **one of the destination nodes**.

The current implementation supports:
- **DFS** - Depth-First Search
- **BFS** - Breadth-First Search
- **GBFS** - Greedy Best-First Search
- **AS** - A* Search
- **CUS2** - Custom informed search

## Files
- `search.py` - main program containing file parsing and search algorithm implementations
- `utils.py` - helper utilities used by the program
- `PathFinder-test.txt` - sample input file for testing

## Requirements
- Python 3.x
- NumPy

Install NumPy if needed:

```bash
pip install numpy
```

## How to Run
Run the program from the command line using:

```bash
python search.py <filename> <method>
```

### Example

```bash
python search.py PathFinder-test.txt GBFS
```

## Supported Methods
Use one of the following method codes:

- `DFS`
- `BFS`
- `GBFS`
- `AS`
- `CUS2`

## Input File Format
The input file must follow this structure:

```text
Nodes:
1: (4,1)
2: (2,2)
3: (4,4)

Edges:
(2,1): 4
(2,3): 4
(3,1): 5

Origin:
2

Destinations:
5; 4
```

### Sections
- **Nodes:** each node ID and its `(x, y)` coordinate
- **Edges:** directed weighted edges in the form `(from,to): cost`
- **Origin:** the starting node
- **Destinations:** one or more goal nodes separated by semicolons

## Output Format
If a path is found, the program prints:

```text
filename method
goal number_of_nodes
path
```

### Example Output

```text
PathFinder-test.txt GBFS
4 4
2 1 4
```

This means:
- the input file was `PathFinder-test.txt`
- the method used was `GBFS`
- the goal reached was node `4`
- `4` nodes were created during the search
- the path found was `2 -> 1 -> 4`

If no path is found, the program prints:

```text
No path found.
```

## Implementation Notes

### Graph Representation
- Nodes are stored with their coordinates in a dictionary.
- Edges are stored as an adjacency list.
- Every node is added to the edge dictionary, even if it has no outgoing edges.

### Heuristic
For informed methods such as **GBFS** and **AS**, the heuristic is based on the Euclidean distance from the current node to the **nearest destination**.

### Tie-Breaking
Where possible, the implementation follows the assignment tie-breaking rules:
- smaller numbered nodes are expanded first when all else is equal
- earlier-added nodes are expanded first if a further tie remains

## Brief Algorithm Summary

### DFS
Explores one branch as far as possible before backtracking.

### BFS
Explores nodes level by level using a queue.

### GBFS
Chooses the next node using only the heuristic value.

### AS
Uses both path cost so far and heuristic value.

### CUS2
Custom bidirectional informed search using forward and backward exploration.

## Notes
- This project was developed for assignment use and command-line testing.
- The program expects the input file to be correctly formatted.
- `search.py` and `utils.py` should be kept in the same folder.

## Team
An-Group-01

## Acknowledgements
This implementation was developed using course materials, assignment requirements, weekly tutorial code patterns, and additional background resources on graph search algorithms.
