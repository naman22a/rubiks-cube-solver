# 🧩 Rubik's Cube Solver

> A high-performance Rubik's Cube solver implementing multiple search algorithms to find optimal or near-optimal solutions for 3x3x3 cube states. The solver supports standard cube moves and provides benchmark comparisons across different search strategies.

## 🚀 Features

-   Supports standard 3x3x3 Rubik's Cube notation and state representation
-   Implements multiple search algorithms:
    -   Breadth-First Search (BFS)
    -   Depth-First Search (DFS)
    -   Iterative Deepening Depth-First Search (IDDFS)
    -   A\* Search with heuristic
    -   Bidirectional Search
-   Accurate cube state transformation with orientation and permutation tracking
-   Benchmark framework for timing and performance comparison

## 🧠 Example

Applying a fixed scramble:  
`R U F`

### Scrambled State

**Corners Permutation:**  
`0 7 1 2 5 3 6 4`  
**Corners Orientation:**  
`2 2 0 1 1 1 0 1`

**Edges Permutation:**  
`7 0 1 2 8 4 6 11 5 9 10 3`  
**Edges Orientation:**  
`1 0 0 0 1 1 0 0 1 0 0 0`

---

### ✅ Solutions Found

-   **BFS solution:** `F' U' R'`
-   **DFS solution:** `U D U D U D U D F' U' R'`
-   **IDDFS solution:** `F' U' R'`
-   **A\* Search solution:** `F' U' R'`
-   **Bidirectional Search solution:** `F' U R`

---

## 📊 Benchmark Results

| Algorithm            | Time (s) | Solution Length | Success |
| -------------------- | -------- | --------------- | ------- |
| Breadth-First Search | 0.113478 | 3               | ✅      |
| Depth-First Search   | 0.040101 | 11              | ✅      |
| IDDFS                | 0.006324 | 3               | ✅      |
| A\* Search           | 0.002618 | 3               | ✅      |
| Bidirectional Search | 0.003743 | 3               | ✅      |

---

## 🛠️ Installation

Clone this repository:

```bash
git clone https://github.com/naman22a/rubiks-cube-solver.git
cd rubiks-cube-solver
```

## 🔧 Build Instructions

🔹 Using g++

```bash
g++ main.cpp -o rubiks_solver
./rubiks_solver
```

## 🙌 Contributing

Pull requests are welcome! For major changes, please open an issue first to discuss what you would like to change.

## 📫 Stay in touch

-   Author - [Naman Arora](https://namanarora.xyz)
-   Twitter - [@naman_22a](https://twitter.com/naman_22a)

## 📄 License

This project is licensed under the GPL V3 License. See the [LICENSE](./LICENSE) file for details.
