#include <bits/stdc++.h>
using namespace std;

const int N = 100005;        // N为边的数量
int n, m, start;             // n个点m条边，start起始节点编号
int dist[N];                 // dist[i] i结点到起始点(1号结点)的距离
bool st[N];                  // st[i]用于标记i结点的最短路是否确定，若确定st[i]=true;
vector<pair<int, int>> g[N]; // 表示存在一条从点x到点y的有向边，边长为z

int Dijkstra()
{
    memset(dist, 0x3f, sizeof dist); //除1号结点外，其他均初始为无穷大
    memset(st, 0x00, sizeof st);
    dist[start] = 0;

    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> heap; // first存储距离，second存储节点编号
    heap.push({0, start});
    while (!heap.empty())
    {
        int node = heap.top().second;
        heap.pop();
        if (st[node])
        {
            continue;
        }
        st[node] = true;        //如果该点已经确定了最短距则跳过
        for (auto &e : g[node]) // 存储从点x到点y的有向边，边长为z
        {
            int v = e.first;
            int w = e.second;
            if (dist[v] > dist[node] + w)
            {
                dist[v] = dist[node] + w;
                heap.push({dist[v], v});
            }
        }
    }

    if (dist[n] == 0x3f3f3f3f)
        return -1;
    else
        return dist[n];
}

int main()
{

    cin >> n >> m >> start; // n个点m条边，start起始节点编号
    int x, y, z;            // 表示存在一条从点x到点y的有向边，边长为z
    while (m--)             // m条边
    {
        cin >> x >> y >> z;
        g[x].push_back({y, z}); // 存储从点x到点y的有向边，边长为z
    }

    cout << Dijkstra() << endl;

    //求dist[i] i结点到起始点(1号结点)的距离
    for (int i = 1; i <= n; ++i)
    {
        cout << dist[i] << " ";
    }

    return 0;
}
