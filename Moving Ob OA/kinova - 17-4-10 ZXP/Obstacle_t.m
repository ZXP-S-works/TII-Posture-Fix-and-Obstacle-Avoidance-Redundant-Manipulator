function [xob_t,yob_t,zob_t] = Obstacle_t(t)
% Input:
%   t    time

% Output:
%   xob
%   yob
%   zob     obstacle position in x, y, z -axis in time t, accroding to this
%           function's definition of the obstacle trajactory.
%
% Author: Xupeng Zhu
% Email: zxpzzz521@gmail.com
% 2019.4.23 v1

global T;

xob_t = -0.05; yob_t = -0.4; zob_t = 0.24;
%vect = [sqrt(1/2), -sqrt(1/2), 0];
vect = [1, 0, 0];
lenth = 0.2;
ob_t = vect .* lenth * (t/T);

xob_t = xob_t + ob_t(1);
yob_t = yob_t + ob_t(2);
zob_t = zob_t + ob_t(3);

end

