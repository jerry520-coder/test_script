#include <bits/stdc++.h>

using namespace std;

// 贝塞尔曲线函数
void bezier(int n, double *x, double *y)
{
    int i, j;
    double t;
    for (i = 0; i <= 100; i++)
    {
        t = i / 100.0;
        x[i] = 0;
        y[i] = 0;
        for (j = 0; j <= n; j++)
        {
            x[i] += pow(1 - t, n - j) * pow(t, j) * x[j];
            y[i] += pow(1 - t, n - j) * pow(t, j) * y[j];
        }
    }
}
int main()
{
    int n;
    double x[100], y[100];
    printf("请输入控制点的个数：");
    scanf("%d", &n);
    printf("请输入控制点的坐标：\n");
    for (int i = 0; i <= n; i++)
    {
        scanf("%lf %lf", &x[i], &y[i]);
    }
    bezier(n, x, y);
    printf("贝塞尔曲线的轨迹为：\n");
    for (int i = 0; i <= 100; i++)
    {
        printf("(%.2lf,%.2lf)\n", x[i], y[i]);
    }
    return 0;
}

// 计算贝塞尔曲线的函数
double bezier(double t, double p0, double p1, double p2, double p3)
{
    double x = pow((1 - t), 3) * p0 + 3 * t * pow((1 - t), 2) * p1 + 3 * pow(t, 2) * (1 - t) * p2 + pow(t, 3) * p3;
    return x;
}
// 主函数
int main()
{
    double p0, p1, p2, p3;
    double t;
    double x;
    // 输入贝塞尔曲线的4个控制点
    printf("Please input the four control points of the Bezier curve: \n");
    scanf("%lf %lf %lf %lf", &p0, &p1, &p2, &p3);
    // 输入t的值
    printf("Please input the value of t: \n");
    scanf("%lf", &t);
    // 计算贝塞尔曲线的值
    x = bezier(t, p0, p1, p2, p3);
    printf("The value of the Bezier curve is: %lf\n", x);
    return 0;
}
