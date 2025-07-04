#include <iostream>
#include <vector>
#include <random>
#include <ctime>
#include <queue>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <functional>
#include <chrono>
#include <iomanip>
using namespace std;

#define CORNERS 8
#define EDGES 12
#define SCRAMBLE_LENGTH 2

enum moves
{
    U,
    Udash,
    D,
    Ddash,
    R,
    Rdash,
    L,
    Ldash,
    F,
    Fdash,
    B,
    Bdash,
};

class CubeState
{
public:
    vector<int> corners;
    vector<int> corners_orientation;
    vector<int> edges;
    vector<int> edges_orientation;

    bool operator==(const CubeState &other) const
    {
        return corners == other.corners &&
               corners_orientation == other.corners_orientation &&
               edges == other.edges &&
               edges_orientation == other.edges_orientation;
    }
};

namespace std
{
    template <>
    struct hash<CubeState>
    {
        size_t operator()(const CubeState &state) const
        {
            size_t h = 0;
            for (int c : state.corners)
                h = h * 31 + c;
            for (int co : state.corners_orientation)
                h = h * 31 + co;
            for (int e : state.edges)
                h = h * 31 + e;
            for (int eo : state.edges_orientation)
                h = h * 31 + eo;
            return h;
        }
    };
}

class RubiksCube
{
public:
    CubeState state;
    RubiksCube()
    {
        for (int i = 0; i < CORNERS; i++)
        {
            state.corners.push_back(i);
            state.corners_orientation.push_back(0);
        }
        for (int i = 0; i < EDGES; i++)
        {
            state.edges.push_back(i);
            state.edges_orientation.push_back(0);
        }
    }

    void applyMove(moves move)
    {
        // Temporary arrays to store current state
        int temp_corners[CORNERS];
        int temp_corners_orientation[CORNERS];
        int temp_edges[EDGES];
        int temp_edges_orientation[EDGES];

        // Copy current state to temporary arrays
        for (int i = 0; i < CORNERS; i++)
        {
            temp_corners[i] = state.corners[i];
            temp_corners_orientation[i] = state.corners_orientation[i];
        }
        for (int i = 0; i < EDGES; i++)
        {
            temp_edges[i] = state.edges[i];
            temp_edges_orientation[i] = state.edges_orientation[i];
        }

        switch (move)
        {
        case U:
            // Corners: 0->1, 1->2, 2->3, 3->0
            state.corners[1] = temp_corners[0];
            state.corners[2] = temp_corners[1];
            state.corners[3] = temp_corners[2];
            state.corners[0] = temp_corners[3];
            // Corner orientations: unchanged
            // Edges: 0->1, 1->2, 2->3, 3->0
            state.edges[1] = temp_edges[0];
            state.edges[2] = temp_edges[1];
            state.edges[3] = temp_edges[2];
            state.edges[0] = temp_edges[3];
            // Edge orientations: unchanged
            break;

        case Udash:
            // Corners: 0->3, 3->2, 2->1, 1->0
            state.corners[3] = temp_corners[0];
            state.corners[2] = temp_corners[3];
            state.corners[1] = temp_corners[2];
            state.corners[0] = temp_corners[1];
            // Corner orientations: unchanged
            // Edges: 0->3, 3->2, 2->1, 1->0
            state.edges[3] = temp_edges[0];
            state.edges[2] = temp_edges[3];
            state.edges[1] = temp_edges[2];
            state.edges[0] = temp_edges[1];
            // Edge orientations: unchanged
            break;

        case D:
            // Corners: 4->5, 5->6, 6->7, 7->4
            state.corners[5] = temp_corners[4];
            state.corners[6] = temp_corners[5];
            state.corners[7] = temp_corners[6];
            state.corners[4] = temp_corners[7];
            // Corner orientations: unchanged
            // Edges: 8->9, 9->10, 10->11, 11->8
            state.edges[9] = temp_edges[8];
            state.edges[10] = temp_edges[9];
            state.edges[11] = temp_edges[10];
            state.edges[8] = temp_edges[11];
            // Edge orientations: unchanged
            break;

        case Ddash:
            // Corners: 4->7, 7->6, 6->5, 5->4
            state.corners[7] = temp_corners[4];
            state.corners[6] = temp_corners[7];
            state.corners[5] = temp_corners[6];
            state.corners[4] = temp_corners[5];
            // Corner orientations: unchanged
            // Edges: 8->11, 11->10, 10->9, 9->8
            state.edges[11] = temp_edges[8];
            state.edges[10] = temp_edges[11];
            state.edges[9] = temp_edges[10];
            state.edges[8] = temp_edges[9];
            // Edge orientations: unchanged
            break;

        case R:
            // Corners: 0->4, 4->7, 7->3, 3->0
            state.corners[4] = temp_corners[0];
            state.corners[7] = temp_corners[4];
            state.corners[3] = temp_corners[7];
            state.corners[0] = temp_corners[3];
            // Corner orientations: +1 mod 3 for affected corners
            state.corners_orientation[4] = (temp_corners_orientation[0] + 1) % 3;
            state.corners_orientation[7] = (temp_corners_orientation[4] + 1) % 3;
            state.corners_orientation[3] = (temp_corners_orientation[7] + 1) % 3;
            state.corners_orientation[0] = (temp_corners_orientation[3] + 1) % 3;
            // Edges: 3->11, 11->7, 7->4, 4->3
            state.edges[11] = temp_edges[3];
            state.edges[7] = temp_edges[11];
            state.edges[4] = temp_edges[7];
            state.edges[3] = temp_edges[4];
            // Edge orientations: unchanged
            break;

        case Rdash:
            // Corners: 0->3, 3->7, 7->4, 4->0
            state.corners[3] = temp_corners[0];
            state.corners[7] = temp_corners[3];
            state.corners[4] = temp_corners[7];
            state.corners[0] = temp_corners[4];
            // Corner orientations: -1 mod 3 for affected corners
            state.corners_orientation[3] = (temp_corners_orientation[0] + 2) % 3; // +2 is equivalent to -1 mod 3
            state.corners_orientation[7] = (temp_corners_orientation[3] + 2) % 3;
            state.corners_orientation[4] = (temp_corners_orientation[7] + 2) % 3;
            state.corners_orientation[0] = (temp_corners_orientation[4] + 2) % 3;
            // Edges: 3->4, 4->7, 7->11, 11->3
            state.edges[4] = temp_edges[3];
            state.edges[7] = temp_edges[4];
            state.edges[11] = temp_edges[7];
            state.edges[3] = temp_edges[11];
            // Edge orientations: unchanged
            break;

        case L:
            // Corners: 1->5, 5->6, 6->2, 2->1
            state.corners[5] = temp_corners[1];
            state.corners[6] = temp_corners[5];
            state.corners[2] = temp_corners[6];
            state.corners[1] = temp_corners[2];
            // Corner orientations: +1 mod 3 for affected corners
            state.corners_orientation[5] = (temp_corners_orientation[1] + 1) % 3;
            state.corners_orientation[6] = (temp_corners_orientation[5] + 1) % 3;
            state.corners_orientation[2] = (temp_corners_orientation[6] + 1) % 3;
            state.corners_orientation[1] = (temp_corners_orientation[2] + 1) % 3;
            // Edges: 1->9, 9->6, 6->5, 5->1
            state.edges[9] = temp_edges[1];
            state.edges[6] = temp_edges[9];
            state.edges[5] = temp_edges[6];
            state.edges[1] = temp_edges[5];
            // Edge orientations: unchanged
            break;

        case Ldash:
            // Corners: 1->2, 2->6, 6->5, 5->1
            state.corners[2] = temp_corners[1];
            state.corners[6] = temp_corners[2];
            state.corners[5] = temp_corners[6];
            state.corners[1] = temp_corners[5];
            // Corner orientations: -1 mod 3 for affected corners
            state.corners_orientation[2] = (temp_corners_orientation[1] + 2) % 3;
            state.corners_orientation[6] = (temp_corners_orientation[2] + 2) % 3;
            state.corners_orientation[5] = (temp_corners_orientation[6] + 2) % 3;
            state.corners_orientation[1] = (temp_corners_orientation[5] + 2) % 3;
            // Edges: 1->5, 5->6, 6->9, 9->1
            state.edges[5] = temp_edges[1];
            state.edges[6] = temp_edges[5];
            state.edges[9] = temp_edges[6];
            state.edges[1] = temp_edges[9];
            // Edge orientations: unchanged
            break;

        case F:
            // Corners: 0->1, 1->5, 5->4, 4->0
            state.corners[1] = temp_corners[0];
            state.corners[5] = temp_corners[1];
            state.corners[4] = temp_corners[5];
            state.corners[0] = temp_corners[4];
            // Corner orientations: +1 mod 3 for affected corners
            state.corners_orientation[1] = (temp_corners_orientation[0] + 1) % 3;
            state.corners_orientation[5] = (temp_corners_orientation[1] + 1) % 3;
            state.corners_orientation[4] = (temp_corners_orientation[5] + 1) % 3;
            state.corners_orientation[0] = (temp_corners_orientation[4] + 1) % 3;
            // Edges: 0->5, 5->8, 8->4, 4->0
            state.edges[5] = temp_edges[0];
            state.edges[8] = temp_edges[5];
            state.edges[4] = temp_edges[8];
            state.edges[0] = temp_edges[4];
            // Edge orientations: flip each edge (0->1, 1->0)
            state.edges_orientation[5] = 1 - temp_edges_orientation[0];
            state.edges_orientation[8] = 1 - temp_edges_orientation[5];
            state.edges_orientation[4] = 1 - temp_edges_orientation[8];
            state.edges_orientation[0] = 1 - temp_edges_orientation[4];
            break;

        case Fdash:
            // Corners: 0->4, 4->5, 5->1, 1->0
            state.corners[4] = temp_corners[0];
            state.corners[5] = temp_corners[4];
            state.corners[1] = temp_corners[5];
            state.corners[0] = temp_corners[1];
            // Corner orientations: -1 mod 3 for affected corners
            state.corners_orientation[4] = (temp_corners_orientation[0] + 2) % 3;
            state.corners_orientation[5] = (temp_corners_orientation[4] + 2) % 3;
            state.corners_orientation[1] = (temp_corners_orientation[5] + 2) % 3;
            state.corners_orientation[0] = (temp_corners_orientation[1] + 2) % 3;
            // Edges: 0->4, 4->8, 8->5, 5->0
            state.edges[4] = temp_edges[0];
            state.edges[8] = temp_edges[4];
            state.edges[5] = temp_edges[8];
            state.edges[0] = temp_edges[5];
            // Edge orientations: flip each edge
            state.edges_orientation[4] = 1 - temp_edges_orientation[0];
            state.edges_orientation[8] = 1 - temp_edges_orientation[4];
            state.edges_orientation[5] = 1 - temp_edges_orientation[8];
            state.edges_orientation[0] = 1 - temp_edges_orientation[5];
            break;

        case B:
            // Corners: 2->3, 3->7, 7->6, 6->2
            state.corners[3] = temp_corners[2];
            state.corners[7] = temp_corners[3];
            state.corners[6] = temp_corners[7];
            state.corners[2] = temp_corners[6];
            // Corner orientations: +1 mod 3 for affected corners
            state.corners_orientation[3] = (temp_corners_orientation[2] + 1) % 3;
            state.corners_orientation[7] = (temp_corners_orientation[3] + 1) % 3;
            state.corners_orientation[6] = (temp_corners_orientation[7] + 1) % 3;
            state.corners_orientation[2] = (temp_corners_orientation[6] + 1) % 3;
            // Edges: 2->7, 7->10, 10->6, 6->2
            state.edges[7] = temp_edges[2];
            state.edges[10] = temp_edges[7];
            state.edges[6] = temp_edges[10];
            state.edges[2] = temp_edges[6];
            // Edge orientations: flip each edge
            state.edges_orientation[7] = 1 - temp_edges_orientation[2];
            state.edges_orientation[10] = 1 - temp_edges_orientation[7];
            state.edges_orientation[6] = 1 - temp_edges_orientation[10];
            state.edges_orientation[2] = 1 - temp_edges_orientation[6];
            break;

        case Bdash:
            // Corners: 2->6, 6->7, 7->3, 3->2
            state.corners[6] = temp_corners[2];
            state.corners[7] = temp_corners[6];
            state.corners[3] = temp_corners[7];
            state.corners[2] = temp_corners[3];
            // Corner orientations: -1 mod 3 for affected corners
            state.corners_orientation[6] = (temp_corners_orientation[2] + 2) % 3;
            state.corners_orientation[7] = (temp_corners_orientation[6] + 2) % 3;
            state.corners_orientation[3] = (temp_corners_orientation[7] + 2) % 3;
            state.corners_orientation[2] = (temp_corners_orientation[3] + 2) % 3;
            // Edges: 2->6, 6->10, 10->7, 7->2
            state.edges[6] = temp_edges[2];
            state.edges[10] = temp_edges[6];
            state.edges[7] = temp_edges[10];
            state.edges[2] = temp_edges[7];
            // Edge orientations: flip each edge
            state.edges_orientation[6] = 1 - temp_edges_orientation[2];
            state.edges_orientation[10] = 1 - temp_edges_orientation[6];
            state.edges_orientation[7] = 1 - temp_edges_orientation[10];
            state.edges_orientation[2] = 1 - temp_edges_orientation[7];
            break;

        default:
            break;
        }
    }

    void scramble()
    {
        // Initialize random number generator
        static mt19937 rng(static_cast<unsigned>(time(nullptr)));
        uniform_int_distribution<int> dist(0, 11); // 0 to 11 for 12 moves

        const int num_moves = SCRAMBLE_LENGTH; // Standard scramble length
        int last_face = -1;                    // Track last face to avoid consecutive same-face moves

        // Array to map moves to their face (0=U, 1=D, 2=R, 3=L, 4=F, 5=B)
        int move_to_face[12] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5};

        cout << "Scramble sequence: ";
        for (int i = 0; i < num_moves; i++)
        {
            int move;
            int face;
            do
            {
                move = dist(rng); // Generate random move (0 to 11)
                face = move_to_face[move];
            } while (face == last_face); // Avoid same face as previous move

            last_face = face; // Update last face

            // Apply the move
            applyMove(static_cast<moves>(move));

            // Print the move for user reference
            const char *move_names[] = {"U", "U'", "D", "D'", "R", "R'", "L", "L'", "F", "F'", "B", "B'"};
            cout << move_names[move] << " ";
        }
        cout << endl;
    }

    void print()
    {
        cout << "Corners: " << endl;
        for (int i = 0; i < CORNERS; i++)
            cout << state.corners[i] << " ";
        cout << endl;

        cout << "Corners Orientation: " << endl;
        for (int i = 0; i < CORNERS; i++)
            cout << state.corners_orientation[i] << " ";
        cout << endl;

        cout << "Edges: " << endl;
        for (int i = 0; i < EDGES; i++)
            cout << state.edges[i] << " ";
        cout << endl;

        cout << "Edges Orientation: " << endl;
        for (int i = 0; i < EDGES; i++)
            cout << state.edges_orientation[i] << " ";
        cout << endl;
    }

    bool isSolved() const
    {
        for (int i = 0; i < CORNERS; i++)
        {
            if (state.corners[i] != i || state.corners_orientation[i] != 0)
                return false;
        }
        for (int i = 0; i < EDGES; i++)
        {
            if (state.edges[i] != i || state.edges_orientation[i] != 0)
                return false;
        }
        return true;
    }

    vector<moves> solveBFS()
    {
        struct Node
        {
            CubeState state;
            vector<moves> Moves;
            int last_face;
            int depth;
        };

        queue<Node> q;
        unordered_set<CubeState> visited;
        int move_to_face[12] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5};
        const long long max_iterations = 1000000; // Prevent memory overflow
        const int max_depth = 8;                  // Limit depth to control memory usage

        q.push({state, {}, -1, 0});
        visited.insert(state);
        long long iterations = 0;

        while (!q.empty() && iterations < max_iterations)
        {
            Node current = q.front();
            q.pop();
            iterations++;

            // Progress feedback every 100,000 iterations
            if (iterations % 100000 == 0)
            {
                cout << "Processed " << iterations << " states, queue size: " << q.size() << ", depth: " << current.depth << endl;
            }

            // Check if current state is solved
            RubiksCube temp;
            temp.state = current.state;
            if (temp.isSolved())
            {
                cout << "Solution found at depth: " << current.depth << endl;
                return current.Moves;
            }

            // Skip if depth limit reached
            if (current.depth >= max_depth)
            {
                continue;
            }

            // Try all possible moves
            for (int m = 0; m < 12; m++)
            {
                if (move_to_face[m] == current.last_face)
                    continue;

                RubiksCube next;
                next.state = current.state;
                next.applyMove(static_cast<moves>(m));

                if (visited.find(next.state) == visited.end())
                {
                    visited.insert(next.state);
                    vector<moves> new_moves = current.Moves;
                    new_moves.push_back(static_cast<moves>(m));
                    q.push({next.state, new_moves, move_to_face[m], current.depth + 1});
                }
            }
        }

        cout << "No solution found within depth " << max_depth << " or iteration limit " << max_iterations << endl;
        return {};
    }

    vector<moves> solveDFS()
    {
        vector<moves> solution;
        unordered_set<CubeState> visited;
        int move_to_face[12] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5}; // U, U', D, D', R, R', L, L', F, F', B, B'
        const int max_depth = 12;                                    // Limit depth to prevent stack overflow
        long long iteration_count = 0;
        const long long max_iterations = 1000000; // Prevent excessive recursion

        if (dfs(0, max_depth, -1, solution, visited, iteration_count, max_iterations))
        {
            cout << "Solution found at depth: " << solution.size() << " after " << iteration_count << " iterations" << endl;
            return solution;
        }

        cout << "No solution found within depth " << max_depth << " or iteration limit " << max_iterations << endl;
        return {};
    }

private:
    bool dfs(int depth, int max_depth, int last_face, vector<moves> &solution, unordered_set<CubeState> &visited, long long &iteration_count, const long long max_iterations)
    {
        iteration_count++;
        if (iteration_count >= max_iterations)
        {
            return false;
        }

        if (isSolved())
        {
            return true;
        }

        if (depth >= max_depth)
        {
            return false;
        }

        CubeState original_state = state;
        int move_to_face[12] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5};

        for (int m = 0; m < 12; m++)
        {
            if (move_to_face[m] == last_face)
                continue; // Skip same face moves

            applyMove(static_cast<moves>(m));
            CubeState current_state = state;

            if (visited.find(current_state) == visited.end())
            {
                visited.insert(current_state);
                solution.push_back(static_cast<moves>(m));

                if (dfs(depth + 1, max_depth, move_to_face[m], solution, visited, iteration_count, max_iterations))
                {
                    return true;
                }

                solution.pop_back();
                visited.erase(current_state); // Optional: Remove to reduce memory
            }

            state = original_state; // Undo move
        }
        return false;
    }

public:
    vector<moves> solveIDDFS()
    {
        vector<moves> solution;
        unordered_set<CubeState> visited;
        const int max_depth = 12; // Maximum depth to search
        long long iteration_count = 0;
        const long long max_iterations = 1000000; // Prevent excessive recursion

        // Iterate over increasing depths
        for (int depth_limit = 1; depth_limit <= max_depth; depth_limit++)
        {
            visited.clear(); // Reset visited states for each depth
            iteration_count = 0;
            cout << "Trying depth limit: " << depth_limit << endl;
            if (dfs2(depth_limit, -1, solution, visited, iteration_count, max_iterations))
            {
                cout << "Solution found at depth: " << solution.size() << " after " << iteration_count << " iterations" << endl;
                return solution;
            }
            if (iteration_count >= max_iterations)
            {
                cout << "Stopped at depth " << depth_limit << " due to iteration limit " << max_iterations << endl;
            }
        }

        cout << "No solution found within depth " << max_depth << endl;
        return {};
    }

private:
    bool dfs2(int depth_limit, int last_face, vector<moves> &solution, unordered_set<CubeState> &visited, long long &iteration_count, const long long max_iterations)
    {
        iteration_count++;
        if (iteration_count >= max_iterations)
        {
            return false;
        }

        if (isSolved())
        {
            return true;
        }

        if (depth_limit <= 0)
        {
            return false;
        }

        CubeState original_state = state;
        int move_to_face[12] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5};

        for (int m = 0; m < 12; m++)
        {
            if (move_to_face[m] == last_face)
                continue; // Skip same face moves

            applyMove(static_cast<moves>(m));
            CubeState current_state = state;

            if (visited.find(current_state) == visited.end())
            {
                visited.insert(current_state);
                solution.push_back(static_cast<moves>(m));

                if (dfs2(depth_limit - 1, move_to_face[m], solution, visited, iteration_count, max_iterations))
                {
                    return true;
                }

                solution.pop_back();
                // Note: Not removing visited state to maintain cycle prevention
            }

            state = original_state; // Undo move
        }
        return false;
    }

public:
    vector<moves> solveAStar()
    {
        struct Node
        {
            CubeState state;
            vector<moves> Moves;
            int g; // Cost so far (number of moves)
            int f; // g + h (total estimated cost)
            int last_face;
            bool operator>(const Node &other) const
            {
                return f > other.f; // Min-heap: prefer lower f
            }
        };

        priority_queue<Node, vector<Node>, greater<Node>> pq;
        unordered_map<CubeState, int> best_g; // Track best g(n) for each state
        int move_to_face[12] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5};
        const long long max_iterations = 1000000; // Prevent memory overflow
        const int max_depth = 20;
        26; // Limit depth for feasibility

        RubiksCube initial_cube;
        initial_cube.state = state;
        int h = initial_cube.heuristic();
        pq.push({state, {}, 0, h, -1});
        best_g[state] = 0;
        long long iterations = 0;

        while (!pq.empty() && iterations < max_iterations)
        {
            iterations++;
            Node current = pq.top();
            pq.pop();

            if (iterations % 100000 == 0)
            {
                cout << "Processed " << iterations << " states, queue size: " << pq.size() << ", depth: " << current.g << endl;
            }

            RubiksCube temp;
            temp.state = current.state;
            if (temp.isSolved())
            {
                cout << "Solution found at depth: " << current.g << " after " << iterations << " iterations" << endl;
                return current.Moves;
            }

            if (current.g >= max_depth)
            {
                continue;
            }

            for (int m = 0; m < 12; m++)
            {
                if (move_to_face[m] == current.last_face)
                    continue;

                RubiksCube next;
                next.state = current.state;
                next.applyMove(static_cast<moves>(m));

                int new_g = current.g + 1;
                auto it = best_g.find(next.state);
                if (it == best_g.end() || new_g < it->second)
                {
                    best_g[next.state] = new_g;
                    int h = next.heuristic();
                    vector<moves> new_moves = current.Moves;
                    new_moves.push_back(static_cast<moves>(m));
                    pq.push({next.state, new_moves, new_g, new_g + h, move_to_face[m]});
                }
            }
        }

        cout << "No solution found within depth " << max_depth << " or iteration limit " << max_iterations << endl;
        return {};
    }

private:
    int heuristic() const
    {
        int misplaced = 0;
        for (int i = 0; i < CORNERS; i++)
        {
            if (state.corners[i] != i)
                misplaced++; // Count misplaced corners
        }
        for (int i = 0; i < EDGES; i++)
        {
            if (state.edges[i] != i)
                misplaced++; // Count misplaced edges
        }
        return misplaced; // Admissible: underestimates moves needed
    }

public:
    vector<moves> solveBidirectional()
    {
        struct Node
        {
            CubeState state;
            vector<moves> Moves;
            int g; // Depth (moves taken)
            int last_face;
        };

        // Initialize forward and backward queues
        queue<Node> forward_queue;
        queue<Node> backward_queue;
        unordered_map<CubeState, pair<vector<moves>, int>> forward_visited;  // state -> (moves, g)
        unordered_map<CubeState, pair<vector<moves>, int>> backward_visited; // state -> (moves, g)
        int move_to_face[12] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5};
        const long long max_iterations = 500000; // Per direction
        const int max_depth = 10;                // Per direction

        // Initialize forward search from scrambled state
        forward_queue.push({state, {}, 0, -1});
        forward_visited[state] = {{}, 0};

        // Initialize backward search from solved state
        RubiksCube solved_cube;
        backward_queue.push({solved_cube.state, {}, 0, -1});
        backward_visited[solved_cube.state] = {{}, 0};

        long long forward_iterations = 0, backward_iterations = 0;

        while (!forward_queue.empty() && !backward_queue.empty() &&
               forward_iterations < max_iterations && backward_iterations < max_iterations)
        {

            // Forward search step
            if (!forward_queue.empty() && forward_iterations < max_iterations)
            {
                forward_iterations++;
                Node current = forward_queue.front();
                forward_queue.pop();

                if (current.g >= max_depth)
                    continue;

                // Check for intersection with backward visited states
                auto it = backward_visited.find(current.state);
                if (it != backward_visited.end())
                {
                    // Solution found: combine forward and backward moves
                    vector<moves> solution = current.Moves;
                    vector<moves> backward_moves = it->second.first;
                    // Append inverse of backward moves in reverse order
                    for (auto it = backward_moves.rbegin(); it != backward_moves.rend(); ++it)
                    {
                        solution.push_back(static_cast<moves>(*it ^ 1)); // Inverse move (e.g., U -> U')
                    }
                    cout << "Solution found at total depth: " << current.g + it->second.second
                         << " (forward: " << current.g << ", backward: " << it->second.second << ")"
                         << " after " << (forward_iterations + backward_iterations) << " iterations" << endl;
                    return solution;
                }

                // Explore forward moves
                for (int m = 0; m < 12; m++)
                {
                    if (move_to_face[m] == current.last_face)
                        continue;

                    RubiksCube next;
                    next.state = current.state;
                    next.applyMove(static_cast<moves>(m));

                    auto visited_it = forward_visited.find(next.state);
                    int new_g = current.g + 1;
                    if (visited_it == forward_visited.end() || new_g < visited_it->second.second)
                    {
                        vector<moves> new_moves = current.Moves;
                        new_moves.push_back(static_cast<moves>(m));
                        forward_visited[next.state] = {new_moves, new_g};
                        forward_queue.push({next.state, new_moves, new_g, move_to_face[m]});
                    }
                }
            }

            // Backward search step
            if (!backward_queue.empty() && backward_iterations < max_iterations)
            {
                backward_iterations++;
                Node current = backward_queue.front();
                backward_queue.pop();

                if (current.g >= max_depth)
                    continue;

                // Check for intersection with forward visited states
                auto it = forward_visited.find(current.state);
                if (it != forward_visited.end())
                {
                    // Solution found: combine forward and backward moves
                    vector<moves> solution = it->second.first;
                    vector<moves> backward_moves = current.Moves;
                    // Append inverse of backward moves in reverse order
                    for (auto it = backward_moves.rbegin(); it != backward_moves.rend(); ++it)
                    {
                        solution.push_back(static_cast<moves>(*it ^ 1)); // Inverse move
                    }
                    cout << "Solution found at total depth: " << it->second.second + current.g
                         << " (forward: " << it->second.second << ", backward: " << current.g << ")"
                         << " after " << (forward_iterations + backward_iterations) << " iterations" << endl;
                    return solution;
                }

                // Explore backward moves (apply inverse moves)
                for (int m = 0; m < 12; m++)
                {
                    if (move_to_face[m] == current.last_face)
                        continue;

                    RubiksCube next;
                    next.state = current.state;
                    next.applyMove(static_cast<moves>(m ^ 1)); // Apply inverse move

                    auto visited_it = backward_visited.find(next.state);
                    int new_g = current.g + 1;
                    if (visited_it == backward_visited.end() || new_g < visited_it->second.second)
                    {
                        vector<moves> new_moves = current.Moves;
                        new_moves.push_back(static_cast<moves>(m)); // Store forward move for path reconstruction
                        backward_visited[next.state] = {new_moves, new_g};
                        backward_queue.push({next.state, new_moves, new_g, move_to_face[m]});
                    }
                }
            }

            if (forward_iterations % 100000 == 0 || backward_iterations % 100000 == 0)
            {
                cout << "Forward iterations: " << forward_iterations << ", queue size: " << forward_queue.size()
                     << ", Backward iterations: " << backward_iterations << ", queue size: " << backward_queue.size() << endl;
            }
        }

        cout << "No solution found within depth " << max_depth << " or iteration limit " << max_iterations << " per direction" << endl;
        return {};
    }
};

int main()
{
    RubiksCube cube;
    const char *move_names[] = {"U", "U'", "D", "D'", "R", "R'", "L", "L'", "F", "F'", "B", "B'"};

    // Fixed 2-move scramble for consistent testing
    vector<moves> fixed_scramble = {R, U, F}; // Short scramble to avoid memory issues
    cout << "Applying fixed scramble: ";
    for (size_t i = 0; i < fixed_scramble.size(); ++i)
    {
        cout << move_names[fixed_scramble[i]] << " ";
        cube.applyMove(fixed_scramble[i]);
    }
    cout << "\n\nScrambled state:\n";
    cube.print();

    // List of algorithms to benchmark
    vector<pair<string, function<vector<moves>(RubiksCube &)>>> algorithms;
    algorithms.push_back(make_pair("BFS", [](RubiksCube &c)
                                   { return c.solveBFS(); }));
    algorithms.push_back(make_pair("DFS", [](RubiksCube &c)
                                   { return c.solveDFS(); }));
    algorithms.push_back(make_pair("IDDFS", [](RubiksCube &c)
                                   { return c.solveIDDFS(); }));
    algorithms.push_back(make_pair("A* Search", [](RubiksCube &c)
                                   { return c.solveAStar(); }));
    algorithms.push_back(make_pair("Bidirectional Search", [](RubiksCube &c)
                                   { return c.solveBidirectional(); }));

    // Store results: algorithm name, time (seconds), solution length, success
    vector<tuple<string, double, int, bool>> results;

    // Benchmark each algorithm
    for (size_t i = 0; i < algorithms.size(); ++i)
    {
        // Reset cube to scrambled state
        RubiksCube test_cube;
        test_cube.state = cube.state;

        // Measure time
        auto start = chrono::high_resolution_clock::now();
        vector<moves> solution = algorithms[i].second(test_cube);
        auto end = chrono::high_resolution_clock::now();
        double duration = chrono::duration_cast<chrono::microseconds>(end - start).count() / 1e6; // Seconds

        // Record results
        bool success = !solution.empty();
        int solution_length = success ? solution.size() : 0;
        results.push_back(make_tuple(algorithms[i].first, duration, solution_length, success));

        // Print solution if found
        if (success)
        {
            cout << "\n"
                 << algorithms[i].first << " solution: ";
            for (size_t j = 0; j < solution.size(); ++j)
            {
                cout << move_names[solution[j]] << " ";
            }
            cout << endl;
        }
    }

    // Print benchmark results
    cout << "\n=== Benchmark Results ===\n";
    cout << left << setw(25) << "Algorithm" << setw(15) << "Time (s)"
         << setw(15) << "Solution Length" << setw(10) << "Success" << endl;
    cout << string(60, '-') << endl;
    for (size_t i = 0; i < results.size(); ++i)
    {
        string name = get<0>(results[i]);
        double time = get<1>(results[i]);
        int length = get<2>(results[i]);
        bool success = get<3>(results[i]);
        cout << left << setw(25) << name
             << fixed << setprecision(6) << setw(15) << time
             << setw(15) << (success ? to_string(length) : "N/A")
             << setw(10) << (success ? "Yes" : "No") << endl;
    }

    // Verify one solution (e.g., A* Search) to ensure correctness
    cout << "\nVerifying A* Search solution:\n";
    RubiksCube verify_cube;
    verify_cube.state = cube.state;
    vector<moves> solution = verify_cube.solveAStar();
    if (!solution.empty())
    {
        for (size_t i = 0; i < solution.size(); ++i)
        {
            verify_cube.applyMove(solution[i]);
        }
        cout << "Final state after applying A* solution:\n";
        verify_cube.print();
    }
    else
    {
        cout << "No solution to verify.\n";
    }

    return 0;

    // RubiksCube cube;

    // cout << "Initial state:\n";
    // cube.print();

    // cout << "\nScrambling cube...\n";
    // cube.scramble();

    // cout << "\nScrambled state:\n";
    // cube.print();

    // cout << "\nSolving cube...\n";
    // vector<moves> solution = cube.solveBidirectional();
    // if (!solution.empty())
    // {
    //     cout << "Solution found: ";
    //     const char *move_names[] = {"U", "U'", "D", "D'", "R", "R'", "L", "L'", "F", "F'", "B", "B'"};
    //     for (moves m : solution)
    //     {
    //         cout << move_names[m] << " ";
    //         cube.applyMove(m); // Apply solution moves to verify
    //     }
    //     cout << endl;

    //     cout << "\nFinal state after applying solution:\n";
    //     cube.print();
    // }
    // else
    // {
    //     cout << "No solution found within iteration limit.\n";
    // }

    // return 0;
}