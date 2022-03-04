#ifndef HUNGARIAN_H
#define HUNGARIAN_H

#include <vector>

struct WeightedBipartiteEdge {
    int left;
    int right;
    float cost;

    WeightedBipartiteEdge() : left(), right(), cost() {}
    WeightedBipartiteEdge(int left, int right, float cost) : left(left), right(right), cost(cost) {}
};

// Given the number of nodes on each side of the bipartite graph and a list of edges, returns a minimum-weight perfect matching.
// If a matching is found, returns a length-n vector, giving the nodes on the right that the left nodes are matched to.
// If no matching exists, returns an empty vector.
// (Note: Edges with endpoints out of the range [0, n) are ignored.)
const std::vector<int> hungarianMinimumWeightPerfectMatching(int n, const std::vector<WeightedBipartiteEdge> allEdges);

static const std::pair<int, std::vector<int> > bruteForceInternal(const int n, const std::vector<WeightedBipartiteEdge> edges, std::vector<bool>& leftMatched, std::vector<bool>& rightMatched, const int edgeUpTo = 0, const int matchCount = 0) {
    if (matchCount == n) {
        return std::make_pair(0, std::vector<int>());
    }

    int bestCost = 1 << 20;
    std::vector<int> bestEdges;
    for (int edgeIndex = edgeUpTo; edgeIndex < edges.size(); ++edgeIndex) {
        const WeightedBipartiteEdge& edge = edges[edgeIndex];
        if (!leftMatched[edge.left] && !rightMatched[edge.right]) {
            leftMatched[edge.left] = true;
            rightMatched[edge.right] = true;
            std::pair<int, std::vector<int> > remainder = bruteForceInternal(n, edges, leftMatched, rightMatched, edgeIndex + 1, matchCount + 1);
            leftMatched[edge.left] = false;
            rightMatched[edge.right] = false;

            if (remainder.first + edge.cost < bestCost) {
                bestCost = remainder.first + edge.cost;
                bestEdges = remainder.second;
                bestEdges.push_back(edgeIndex);
            }
        }
    }

    return std::make_pair(bestCost, bestEdges);
}

const std::vector<int> bruteForce(const int n, const std::vector<WeightedBipartiteEdge> edges) {
    std::vector<bool> leftMatched(n), rightMatched(n);
    std::vector<int> edgeIndices = bruteForceInternal(n, edges, leftMatched, rightMatched).second;
    std::vector<int> matching(n);
    for (int i = 0; i < edgeIndices.size(); ++i) {
        const WeightedBipartiteEdge& edge = edges[edgeIndices[i]];
        matching[edge.left] = edge.right;
    }
    return matching;
}
#endif // HUNGARIAN_H
