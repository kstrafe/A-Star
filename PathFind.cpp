#include "PathFind.hpp"
#include <queue> // Using std::priority_queue
#include <map> // Using std::map
#include <memory> // Using std::unique_ptr
#include <cmath> // Using std::abs and std::sqrt



std::string pathFind
(
    const int xStart, const int yStart,
    const int xFinish, const int yFinish,
    std::function<bool(const int, const int)> collision
)
{
    enum {dir = 8}; // number of possible directions to go at any position

    class node
    {
        // current position
        int xPos;
        int yPos;
        // total distance already travelled to reach the node
        int level;
        // priority=level+remaining distance estimate
        int priority;  // smaller: higher priority

    public:

        node(int xp, int yp, int d, int p)
        :
        xPos(xp),
        yPos(yp),
        level(d),
        priority(p)
        {}

        inline int getxPos() const {return xPos;}
        inline int getyPos() const {return yPos;}
        inline int getLevel() const {return level;}
        inline int getPriority() const {return priority;}

        inline void updatePriority(const int xDest, const int yDest)
        {
             priority = level + estimate(xDest, yDest) * 10; //A*
        }

        // give better priority to going strait instead of diagonally
        inline void nextLevel(const int i) // i: direction
        {
             level += (dir == 8?(i % 2 == 0 ? 10 : 14) : 10);
        }

        // Estimation function for the remaining distance to the goal.
        inline const int estimate(const int xDest, const int yDest) const
        {
            static int xd, yd, d;
            xd = xDest - xPos;
            yd = yDest - yPos;

            // Euclidian Distance
            //d = static_cast<int>(std::sqrt(xd * xd + yd * yd));

            // Manhattan distance
            d = std::abs(xd) + std::abs(yd);

            // Chebyshev distance
            //d = std::max(std::abs(xd), std::abs(yd));

            return d;
        }

        inline bool operator<(const node &rhs) const
        {
            return this->getPriority() > rhs.getPriority();
        }
    };

    const static int dx[dir] = {1, 1, 0, -1, -1, -1, 0, 1};
    const static int dy[dir] = {0, 1, 1, 1, 0, -1, -1, -1};

    std::priority_queue<node> pq[2]; // pqueue of open (not-yet-tried) nodes
    int pqi = 0; // pq index
    std::unique_ptr<node> n0(new node(xStart, yStart, 0, 0)); // mapp
    std::unique_ptr<node> m0;
    int i, j, x, y, xdx, ydy;
    char c;

    std::map<int, std::map<int, int>> closed_nodes_map; // map of closed (tried-out) nodes
    std::map<int, std::map<int, int>> open_nodes_map; // map of open (not-yet-tried) nodes
    std::map<int, std::map<int, int>> dir_map; // map of directions

    // create the start node and push into list of open nodes
    n0->updatePriority(xFinish, yFinish);
    pq[pqi].push(*n0);
    open_nodes_map[x][y] = n0->getPriority(); // mark it on the open nodes map

    // A* search
    while(!pq[pqi].empty())
    {
        // get the current node w/ the highest priority
        // from the list of open nodes
        n0.reset(new node(pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
                     pq[pqi].top().getLevel(), pq[pqi].top().getPriority()));

        x = n0->getxPos(); y = n0->getyPos();

        pq[pqi].pop(); // remove the node from the open list
        open_nodes_map[x][y] = 0;
        // mark it on the closed nodes map
        closed_nodes_map[x][y] = 1;

        // quit searching when the goal state is reached
        //if((*n0).estimate(xFinish, yFinish) == 0)
        if(x == xFinish && y == yFinish)
        {
            // generate the path from finish to start
            // by following the directions
            std::string path = "";
            while(!(x == xStart && y == yStart))
            {
                j = dir_map[x][y];
                c = '0' + (j + dir / 2) % dir;
                path = c + path;
                x += dx[j];
                y += dy[j];
            }

            // empty the leftover nodes
            //while(!pq[pqi].empty()) pq[pqi].pop();
            /// No need to, object is local. Let STD handle it.
            return path;
        }

        // generate moves (child nodes) in all possible directions
        for(i = 0; i < dir; ++i)
        {
            xdx = x + dx[i]; ydy = y + dy[i];

            if(!(/*xdx < lower_x_bound *//*|| xdx > upper_x_bound - 1 *//*We have unlimited size*//*|| ydy < lower_y_bound *//*|| ydy > upper_y_bound - 1*//*Unlimited size ||*/
             collision(xdx, ydy) == true
                || closed_nodes_map[xdx][ydy] == 1))
            {
                // generate a child node
                m0.reset(new node(xdx, ydy, n0->getLevel(),
                             n0->getPriority()));
                m0->nextLevel(i);
                m0->updatePriority(xFinish, yFinish);

                // if it is not in the open list then add into that
                if(open_nodes_map[xdx][ydy] == 0)
                {
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    pq[pqi].push(*m0);
                    // mark its parent node direction
                    dir_map[xdx][ydy] = (i + dir / 2 )%dir;
                }
                else if(open_nodes_map[xdx][ydy]>m0->getPriority())
                {
                    // update the priority info
                    open_nodes_map[xdx][ydy] = m0->getPriority();
                    // update the parent direction info
                    dir_map[xdx][ydy] = (i + dir / 2) % dir;

                    // replace the node
                    // by emptying one pq to the other one
                    // except the node to be replaced will be ignored
                    // and the new node will be pushed in instead
                    while(!(pq[pqi].top().getxPos() == xdx &&
                           pq[pqi].top().getyPos() == ydy))
                    {
                        pq[1 - pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pq[pqi].pop(); // remove the wanted node

                    // empty the larger size pq to the smaller one
                    if(pq[pqi].size() > pq[1 - pqi].size())
                        pqi = 1 - pqi;
                    while(!pq[pqi].empty())
                    {
                        pq[1 - pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pqi = 1 - pqi;
                    pq[pqi].push(*m0); // add the better node instead
                }
            }
        } // Possible child nodes in all directions
    } // while pq is not empty.
    return ""; // no route found
}
