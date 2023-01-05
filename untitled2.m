clc, clear

N = 50;
step = 0.02;
x_up = zeros(N, 1);
y_up = zeros(N, 1);
w_up = zeros(N, 1);
x_down = zeros(N - 1, 1);
y_down = zeros(N - 1, 1);
w_down = zeros(N - 1, 1);

delete test.txt
find = fopen('test.txt', 'w');

for i = 2:1:N

    % x_up(i, 1) = 0 * i;
    % y_up(i, 1) = 0.05 * i;
    % w_up(i, 1) = 0 * i;
    %
    % x_up(i, 1) = 0.01 * i;
    % y_up(i, 1) = 0.05 * i;
    % w_up(i, 1) = 0 * i;

    % w_up(i, 1) = atan(y_up(i, 1) / x_up(i, 1));

    % t = 0.1 * i;
    % x_up(i, 1) = ye / te^5 * (10 * te^2 * t^3 - 15 * te * t^4 + 10 * t^5);
    % y_up(i, 1) = t;
    % w_up(i, 1) = atan(y_up(i, 1) / x_up(i, 1));

    %%======================================sin曲线=========================================================%%
    y_up(i, 1) = 0.05 * i;
    x_up(i, 1) = 0.8 * sin(5 * y_up(i, 1));

    w_up(i, 1) = atan2(y_up(i) - y_up(i - 1), x_up(i) - x_up(i - 1));
    %%======================================sin曲线=========================================================%%

    %%======================================直线=========================================================%%
    % y_up(i, 1) = 0;
    % x_up(i, 1) = 0.1 * i;

    % w_up(i, 1) = atan2(y_up(i) - y_up(i - 1), x_up(i) - x_up(i - 1));
    %%======================================直线=========================================================%%

    %     x_up(i, 1) = step * i;
    %     y_up(i, 1) = sqrt((N * step / 2)^2 - (x_up(i, 1) - N * step / 2)^2);

    %     if i == 1
    %         y1 = y_up(i, 1) - 0;
    %         x1 = x_up(i, 1) - 0;
    %         w_up(i, 1) = atan2(y1, x1);
    %     else
    %         y1 = y_up(i, 1) - y_up(i - 1, 1);
    %         x1 = x_up(i, 1) - x_up(i - 1, 1);
    %         w_up(i, 1) = atan2(y1, x1);
    %         % w_up(i, 1) = atan2(y_up(i, 1), x_up(i, 1));
    %     end

    % fprintf(find, '{"x":%.2f, "y":%.2f, "w":%.2f}', x_up(i, 1), y_up(i, 1), w_up(i, 1));
    % fprintf(find, ',\n');

    fprintf(find, '%.2f %.2f %.2f', x_up(i, 1), y_up(i, 1), w_up(i, 1));
    fprintf(find, '\n');

end

% for i = N - 1:-1:1
%     x_down(i, 1) = step * i;
%     y_down(i, 1) = -sqrt((N * step / 2)^2 - (x_down(i, 1) - N * step / 2)^2);

%     if i == N - 1
%         y1 = y_down(i, 1) - y_up(N, 1);
%         x1 = x_down(i, 1) - x_up(N, 1);
%         w_down(i, 1) = atan2(y1, x1);
%     else
%         y1 = y_down(i, 1) - y_down(i + 1, 1);
%         x1 = x_down(i, 1) - x_down(i + 1, 1);
%         w_down(i, 1) = atan2(y1, x1);
%         % w_down(i, 1) = atan2(y_down(i, 1), x_down(i, 1));

%     end

%     fprintf(find, '{"x":%.2f, "y":%.2f, "w":%.2f}', x_down(i, 1), y_down(i, 1), w_down(i, 1));
%     fprintf(find, ',\n');

% end

% fprintf('{"x":%.2f, "y":%.2f, "w":%.2f}', x(i, 1), y(i, 1), w(i, 1));
% fprintf(',\n');
plot(x_up, y_up);

% plot(x_up, y_up, x_down, y_down);

% x1 = [1.5, 2, -1.3, 0.1, 1.7];
% y1 = [3.5, 0, -2.5, 0.5, -5.1];

% a = atan2(y1, x1);
