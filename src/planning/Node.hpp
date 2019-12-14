#ifndef NODE_HPP
#define NODE_HPP

class Node
{
public:
  Point<int> n;
  mutable const Node *p;
  mutable float f_score;
  mutable float g_score;

  Node(Point<int> n,  const Node *p, float g_score):
  n(Point<int>(n.x, n.y)), p(p), f_score(std::numeric_limits<float>::infinity()), g_score(g_score)
  {};
};

inline bool operator==(const Node& lhs, const Node& rhs)
{
    return (lhs.n.x == rhs.n.x) && (lhs.n.y == rhs.n.y);
}

inline bool operator<(const Node& lhs, const Node& rhs)
{
    // Go from bottom left to top right
    return lhs.f_score < rhs.f_score;
}


#endif
