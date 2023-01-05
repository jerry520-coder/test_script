#include <bits/stdc++.h>

using namespace std;

const int x_sampling_sum_ = 5;   // 行数
const int y_sampling_sum_ = 7;   // 列数
const float x_resolution_ = 0.2; // 行分辨率
const float y_resolution_ = 0.2; // 列分辨率

struct VecXY
{
    float x = 0;
    float y = 0;
    VecXY(){};
    VecXY(const float &x_in, const float &y_in) : x(x_in), y(y_in){};
};

struct VecXYW : public VecXY
{
    float w = 0;
    VecXYW(){};
    VecXYW(const float &x_in, const float &y_in, const float &w_in) : VecXY(x_in, y_in), w(w_in){};
};

VecXYW pose_(0, 0, 0);

float UnifyAngle(const float &angle)
{
    return angle < -M_PI ? (angle + 2 * M_PI) : (angle > M_PI ? (angle - 2 * M_PI) : angle);
}

VecXYW G2L(const VecXYW &robot_pose, const VecXYW &in)
{
    VecXYW out;
    VecXYW temp;
    temp.x = in.x - robot_pose.x;
    temp.y = in.y - robot_pose.y;
    temp.w = in.w;

    out.x = temp.x * cosf(robot_pose.w) + temp.y * sinf(robot_pose.w);
    out.y = -temp.x * sinf(robot_pose.w) + temp.y * cosf(robot_pose.w);
    out.w = UnifyAngle(temp.w - robot_pose.w);
    return out;
}

VecXYW L2G(const VecXYW &robot_pose, const VecXYW &in)
{
    VecXYW out;
    out.x = in.x * cosf(robot_pose.w) - in.y * sinf(robot_pose.w) + robot_pose.x;
    out.y = in.x * sinf(robot_pose.w) + in.y * cosf(robot_pose.w) + robot_pose.y;
    out.w = UnifyAngle(in.w + robot_pose.w);
    return out;
}

std::vector<VecXYW> HorizontalSampling(const VecXYW &sampling_path,
                                       const VecXYW &start,
                                       const VecXYW &start_with_theta,
                                       const float &y_resolution) // 路径坐标系下
{
    /**
     * @brief y轴负方向到正方向进行采样,输出全局坐标系下的采样点
     *
     */

    VecXYW sampling_path_temp = sampling_path;
    VecXYW y_sampling_point; // y方向采样局部的,相对于小车的

    std::vector<VecXYW> y_sampling_list;

    for (int y_index = y_sampling_sum_ / 2; y_index > 0; --y_index) // y负方向
    {
        sampling_path_temp.y = -y_index * y_resolution;
        y_sampling_point = L2G(start_with_theta, sampling_path_temp); // 全局坐标系
        y_sampling_point.w = start.w;

        // y_sampling_point = G2L(pose_, sampling_global_temp); //小车坐标系下

        y_sampling_list.emplace_back(y_sampling_point);
    }

    for (int y_index = 0; y_index < y_sampling_sum_ / 2 + 1; ++y_index) // y正方向
    {
        sampling_path_temp.y = y_index * y_resolution;

        // RNS_ERRO("sampling_path_temp.y :%.2f", sampling_path_temp.y);
        // RNS_DBUG("sampling_path(xyw) %.2f,%.2f,%.2f", sampling_path.x, sampling_path.y, sampling_path.w);

        y_sampling_point = L2G(start_with_theta, sampling_path_temp); // 全局坐标系
        y_sampling_point.w = start.w;

        // y_sampling_point = G2L(pose_, sampling_global_temp); //小车坐标系下

        y_sampling_list.emplace_back(y_sampling_point);
    }

    return y_sampling_list;
}

std::vector<std::vector<VecXYW>> GenerateSamplePoints(const std::vector<VecXYW> &path_, const int &index_)
{
    /**
     * @brief 输出全局坐标系下的横纵向采样点
     *
     */

    VecXYW start;
    VecXYW end;
    float x_sampling_dis; // 路径坐标系下 ： x方向采样距离

    std::vector<VecXYW> y_sampling_list;
    std::vector<std::vector<VecXYW>> sampling_list;

    // 先找纵向
    int x_index = 0;                  // x方向采样点索引
    int x_path_index = 0;             // x方向路径索引
    while (x_index < x_sampling_sum_) // 纵向采5个点
    {
        int len = path_.size();
        float x_rest = x_resolution_; // 其余的
        bool x_path_index_Change = false;

        while (index_ + x_path_index < len - 1)
        {
            start = path_[index_ + x_path_index];
            end = path_[index_ + x_path_index + 1];

            VecXYW start_with_theta(start.x, start.y, atan2f(end.y - start.y, end.x - start.x));
            VecXYW end_rel_start = G2L(start_with_theta, VecXYW(end.x, end.y, 0)); // 路径坐标系下 ： 段路径终点

            // RNS_DBUG("start_with_theta(xyw) %.2f,%.2f,%.2f", start_with_theta.x, start_with_theta.y, start_with_theta.w);
            // RNS_DBUG("end_rel_start(xyw) %.2f,%.2f,%.2f", end_rel_start.x, end_rel_start.y, end_rel_start.w);

            if (x_path_index_Change)
            {
                x_sampling_dis = 0.0;
            }

            if (x_index == 0 && x_path_index == 0)
            {
                VecXYW point_rel_start = G2L(start_with_theta, VecXYW(pose_.x, pose_.y, 0)); // 路径坐标系下 ： 小车
                x_sampling_dis = point_rel_start.x;
            }

            // RNS_ERRO("x_sampling_dis: %.2f", x_sampling_dis);
            // RNS_ERRO("end_rel_start: %.2f", end_rel_start.x);
            // RNS_ERRO("x_rest: %.2f", x_rest);
            // RNS_ERRO("end_rel_start.x - x_sampling_dis: %.2f", end_rel_start.x - x_sampling_dis);

            if (end_rel_start.x - x_sampling_dis > x_rest) // 在当前路径线段内
            {
                // RNS_ERRO("x_rest: %.2f", x_rest);
                // RNS_ERRO("end_rel_start.x - x_sampling_dis: %.2f", end_rel_start.x - x_sampling_dis);
                // RNS_ERRO("x_sampling_dis: %.2f", x_sampling_dis);

                x_sampling_dis += x_rest;

                // RNS_ERRO("x_sampling_dis: %.2f", x_sampling_dis);
                VecXYW sampling_path(x_sampling_dis, 0, 0); // 路径坐标系

                // RNS_DBUG("start_with_theta(xyw) %.2f,%.2f,%.2f", start_with_theta.x, start_with_theta.y, start_with_theta.w);

                y_sampling_list = HorizontalSampling(sampling_path, start, start_with_theta, y_resolution_);

                sampling_list.emplace_back(y_sampling_list);
                break;
            }
            else
            {
                x_rest = x_rest - (end_rel_start.x - x_sampling_dis);
                x_path_index_Change = true;
                ++x_path_index;
            }
        }

        if (index_ + x_path_index == len - 1)
        {
            start = path_[len - 2];
            end = path_[len - 1];

            VecXYW start_with_theta(start.x, start.y, atan2f(end.y - start.y, end.x - start.x));
            VecXYW end_rel_start = G2L(start_with_theta, VecXYW(end.x, end.y, 0)); // 路径坐标系下 ： 路径终点

            VecXYW sampling_path(end_rel_start);
            y_sampling_list = HorizontalSampling(sampling_path, start, start_with_theta, 0.0);

            sampling_list.emplace_back(y_sampling_list);
            break;
        }

        ++x_index;
    }

    return sampling_list;
}

std::vector<std::vector<float>> CalcEdgeCost(const int &N,
                                             const std::vector<std::vector<VecXYW>> &sampling_list)
{
    /**
     * @brief 计算边权问题。包括障碍物的cost、和原路径的cost、以及与小车当前距离的cost
     *
     */

    std::vector<std::vector<float>> side_cost(N, std::vector<float>(N, 1e6)); // 邻接矩阵的初始化，由于求的是最小值，因此初始为无穷大

    float self_distance_cost = 0.0;
    float path_distance_cost = 0.0;
    float obstacle_cost = 0.0;

    int i = 0; // 遍历每一行
    int k = 0; // k为图（邻接矩阵）的顶点

    while (k < N - y_sampling_sum_) // 遍历每个邻接矩阵的顶点（不含最后一行）
    {
        for (int j = 0; j < sampling_list[0].size(); ++j) // j遍历每列
        {
            if (k > y_sampling_sum_ * i)
            {
                ++i; // 换下一行
            }

            int division = y_sampling_sum_ / 2; // 对横向采样进行先递减和递增划分

            if (j < division) // 右划分
            {
                path_distance_cost = y_resolution_ * (division - j);

                if (k == 0) // 小车为邻接矩阵的顶点
                {
                    self_distance_cost = hypotf(pose_.x - sampling_list[i][j].x, pose_.y - sampling_list[i][j].y);
                }
                else
                {
                    self_distance_cost = hypotf(sampling_list[i - 1][k - y_sampling_sum_ * (i - 1) - 1].x - sampling_list[i][j].x, sampling_list[i - 1][k - y_sampling_sum_ * (i - 1) - 1].y - sampling_list[i][j].y);
                }

                side_cost[k][i * y_sampling_sum_ + j + 1] = path_distance_cost + self_distance_cost;
            }
            else // 左划分（包含j=0）
            {
                path_distance_cost = y_resolution_ * (j - division);

                if (k == 0) // 小车为邻接矩阵的顶点
                {
                    self_distance_cost = hypotf(pose_.x - sampling_list[i][j].x, pose_.y - sampling_list[i][j].y);
                }
                else
                {
                    self_distance_cost = hypotf(sampling_list[i - 1][k - y_sampling_sum_ * (i - 1) - 1].x - sampling_list[i][j].x, sampling_list[i - 1][k - y_sampling_sum_ * (i - 1) - 1].y - sampling_list[i][j].y);
                }

                side_cost[k][i * y_sampling_sum_ + j + 1] = path_distance_cost + self_distance_cost;
            }
        }
        ++k;
    }

    return side_cost;
}

std::vector<VecXYW> TrajectoryGeneration(const std::vector<VecXYW> &path_, const int &index_)
{
    /**
     * @brief 基于采样点，用Dijkstra的方案求出最优轨迹（全局坐标系下）
     *
     */
    std::vector<VecXYW> trajectory;
    std::vector<std::vector<VecXYW>> sampling_list = GenerateSamplePoints(path_, index_);

    //****************************************test : sampling_list*************************************************************//
    // for (int i = 0; i < sampling_list.size(); ++i)
    // {
    //     for (int j = 0; j < sampling_list[0].size(); ++j)
    //     {
    //         RNS_DBUG("ij(%d %d)  sampling_list x:%.2f y:%.2f w:%.2f", i, j, sampling_list[i][j].x, sampling_list[i][j].y, sampling_list[i][j].w);
    //     }
    // }

    cout << endl;
    cout << "sampling_list : " << endl;
    for (int i = 0; i < sampling_list.size(); ++i)
    {
        for (int j = 0; j < sampling_list[0].size(); ++j)
        {
            printf("ij(%d %d)  x:%.2f y:%.2f w:%.2f", i, j, sampling_list[i][j].x, sampling_list[i][j].y, sampling_list[i][j].w);
            cout << endl;
        }
    }
    //****************************************test : sampling_list*************************************************************//

    int N = x_sampling_sum_ * y_sampling_sum_ + 1; // N为点的最大数量限制

    std::vector<std::vector<float>> side_cost = CalcEdgeCost(N, sampling_list); // 存储每条边的cost，计算邻接矩阵
    std::vector<float> node_cost(N, 1e6);                                       // node_cost[i] i结点到起始点的cost，中间有其他节点
    std::vector<bool> visited(N, false);                                        // visited[i] 用于标记i结点的最短路是否确定
    std::vector<int> path_index(N, 0);                                          // 记录最短路的路径，默认等于源点

    node_cost[0] = 0.0;
    for (int i = 0; i < N; ++i) // n次迭代，每次寻找不在s集合中距离最近的点min_cost_index
    {
        int min_cost_index = -1; // 权值最小的下标，-1 便于更新第一个点

        for (int j = 0; j < N; ++j) // 从剩下未放入S集中所有节点中找一个cost最小
        {
            if (!visited[j] && (min_cost_index == -1 || node_cost[j] < node_cost[min_cost_index]))
            {
                min_cost_index = j;
            }
        }

        visited[min_cost_index] = true;

        for (int j = 0; j < N; ++j) // 用min_cost_index更新其他点的距离
        {
            if (!visited[j] &&
                node_cost[min_cost_index] + side_cost[min_cost_index][j] < node_cost[j])
            {
                node_cost[j] = node_cost[min_cost_index] + side_cost[min_cost_index][j];
                path_index[j] = min_cost_index;
            }
        }
    }

    int end_index = 0; // 搜索结束后的路径终点索引
    float min_cost = 1e6;
    // 寻找最后一行的cost最小的索引
    for (int i = 1; i <= y_sampling_sum_; ++i)
    {
        if (node_cost[N - i] < min_cost)
        {
            end_index = N - i;
            min_cost = node_cost[N - i];
        }
    }

    std::vector<int> path_temp;
    path_temp.emplace_back(end_index);

    int t = path_index[end_index]; // 下一个路径索引
    while (t != 0)
    {
        path_temp.emplace_back(t);
        t = path_index[t];
    }

    //****************************************test: path_temp*************************************************************//
    // for (int i = 0; i < path_temp.size(); ++i)
    // {
    //     RNS_DBUG("i(%d)  path_temp %.2f", i, path_temp[i]);
    // }

    cout << endl;
    cout << "path_temp : " << endl;
    for (int i = 0; i < path_temp.size(); ++i)
    {
        printf("i(%d) %d", i, path_temp[i]);
        cout << endl;
    }

    // cout << endl;
    // cout << "node_cost : " << endl;
    // for (int i = 0; i < node_cost.size(); ++i)
    // {
    //     printf("i(%d) %.2f", i, node_cost[i]);
    //     cout << endl;
    // }
    //****************************************test: path_temp*************************************************************//

    // 将邻接矩阵坐标转化为全局坐标
    for (int i = 0; i < path_temp.size(); ++i)
    {
        for (int j = 1; j <= x_sampling_sum_; ++j) // 定位行的位置
        {
            if (path_temp[i] <= y_sampling_sum_ * j)
            {
                int c_index = y_sampling_sum_ * j - path_temp[i];
                c_index = y_sampling_sum_ - c_index; // 定位列的位置

                trajectory.emplace_back(sampling_list[j - 1][c_index - 1]);
                break;
            }
        }
    }

    // 反转
    reverse(trajectory.begin(), trajectory.end());

    //****************************************test: trajectory*************************************************************//
    // for (int i = 0; i < trajectory.size(); ++i)
    // {
    //     RNS_DBUG("i(%d)  trajectory x:%.2f y:%.2f w:%.2f", i, trajectory[i].x, trajectory[i].y, trajectory[i].w);
    // }

    cout << endl;
    cout << "trajectory : " << endl;
    for (int i = 0; i < trajectory.size(); ++i)
    {
        printf("i(%d)  x:%.2f y:%.2f w:%.2f", i, trajectory[i].x, trajectory[i].y, trajectory[i].w);
        cout << endl;
    }
    //****************************************test: trajectory*************************************************************//
    return trajectory;
}

int main()
{
    int N; // N个路径点
    // int index = 0; // 路径索引
    std::vector<VecXYW> path;
    cout << "路径点个数：";
    cin >> N;

    VecXYW point;
    cout << "输入路径点：" << endl;

    while (N--)
    {
        cin >> point.x >> point.y >> point.w;
        path.push_back(point);
    }

    //==============================================test : CalcEdgeCost===============================================//

    // std::vector<std::vector<VecXYW>> sampling_list = GenerateSamplePoints(path, 0);
    // for (int i = 0; i < sampling_list.size(); ++i)
    // {
    //     for (int j = 0; j < sampling_list[0].size(); ++j)
    //     {
    //         printf("ij(%d %d)  x:%.2f y:%.2f w:%.2f", i, j, sampling_list[i][j].x, sampling_list[i][j].y, sampling_list[i][j].w);
    //         cout << endl;
    //     }
    // }

    // int r_len = sampling_list.size();
    // int c_len = sampling_list[0].size();

    // int N1 = r_len * c_len + 1; // N为点的最大数量限制

    // std::vector<std::vector<float>> side_cost = CalcEdgeCost(N1, sampling_list);

    // for (int i = 0; i < side_cost.size(); ++i)
    // {
    //     for (int j = 0; j < side_cost[0].size(); ++j)
    //     {
    //         printf("ij(%d %d)  %.2f", i, j, side_cost[i][j]);
    //         cout << endl;
    //     }
    // }

    //==============================================test : TrajectoryGeneration===============================================//
    std::vector<VecXYW> trajectory = TrajectoryGeneration(path, 0);

    return 0;
}
