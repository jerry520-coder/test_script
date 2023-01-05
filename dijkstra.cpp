#include <iostream>
#include <cstdio>
#include <algorithm>
#include <cstring>
using namespace std;
const int N = 510; // N为点的最大数量限制
int g[N][N];       // 存储每条边的cost，稠密图用邻接矩阵存储比较节省空间
int dist[N];       // dist[i] i结点到起始点(1号结点)的距离，中间有其他节点
bool st[N];        // st[i] 用于标记i结点的最短路是否确定，若确定st[i]=true;
int n, m;          // n个点m条边
int Dijkstra()
{
    memset(dist, 0x3f, sizeof dist); //除1号结点外，其他均初始为无穷大
    dist[1] = 0;
    for (int i = 0; i < n; i++) // n次迭代，每次寻找不在s集合中距离最近的点t
    {
        int t = -1; // 便于更新第一个点

        for (int j = 1; j <= n; j++) //将距离起点最近的节点加入S集合
        {
            if (!st[j] && (t == -1 || dist[j] < dist[t])) //! st[j] 节点j不在s集合中，且dist要小
            {
                t = j;
            }
        }

        st[t] = true; //将t加到s集合中

        for (int j = 1; j <= n; j++) //用t更新其他点的距离
        {
            dist[j] = min(dist[j], dist[t] + g[t][j]);
        }
    }

    if (dist[n] == 0x3f3f3f3f)
        return -1; //路径不存在
    else
        return dist[n];
}
int main()
{
    scanf("%d%d", &n, &m);
    memset(g, 0x3f, sizeof g); //邻接矩阵的初始化，由于求的是最小值，因此初始为无穷大
    while (m--)
    {
        int x, y, z; // 存储从点x到点y的有向边，边长为z
        scanf("%d%d%d", &x, &y, &z);
        g[x][y] = min(g[x][y], z); //重边中去取最小值
    }
    printf("%d\n", Dijkstra());
    return 0;
}



