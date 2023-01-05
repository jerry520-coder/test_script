% clear all;close all;clc;

% ts = 0; te = 10; %起始结束时间

% xs = [ts^5 ts^4 ts^3 ts^2 ts 1]; %起始状态x v a
% vs = [5 * ts^4 4 * ts^3 3 * ts^2 2 * ts 1 0];
% as = [20 * ts^3 12 * ts^2 6 * ts 2 0 0];

% xe = [te^5 te^4 te^3 te^2 te 1]; %结束状态x v a
% ve = [5 * te^4 4 * te^3 3 * te^2 2 * te 1 0];
% ae = [20 * te^3 12 * te^2 6 * te 2 0 0];

% A = [xs; vs; as; xe; ve; ae];
% t = ts:0.01:te;

% for i = -5:5
%     B = [0 0 0 i 0 0]'; %[xs vs as xe ve ae] %都是横向的状态

%     X = inv(A) * B;

%     x = X(1) * t.^5 + X(2) * t.^4 + X(3) * t.^3 + X(4) * t.^2 + X(5) * t + X(6);
%     v = 5 * X(1) * t.^4 + 4 * X(2) * t.^3 + 3 * X(3) * t.^2 + 2 * X(4) * t + X(5);
%     a = 20 * X(1) * t.^3 + 12 * X(2) * t.^2 + 6 * X(3) * t + 2 * X(4);

%     subplot(3, 1, 1);
%     plot(t, x) %位置
%     hold on;

%     subplot(3, 1, 2);
%     plot(t, v) %速度
%     hold on;

%     subplot(3, 1, 3);
%     plot(t, a) %加速度
%     hold on;

% end

dt_ = 0.1;
i = 2;
yaw_(1) = 0;
x_(1) = 0;
y_(1) = 0;
v = 0.2;
y = 0.6;

for t = 0:0.1:3

    yaw_(i) = yaw_(i - 1) + y * dt_;
    x_(i) = v * cos(yaw_(i - 1)) * dt_;
    y_(i) = v * sin(yaw_(i - 1)) * dt_;
    i = i + 1;

end

plot(x_, y_);
