function tau3 = thruster_map_2az(u, lx, ly)
% u = [u1x u1y u2x u2y]'  (N)
% lx, ly are 2x1 thruster positions in BODY (m)
% tau3 = [X Y N]' (N, N, N*m)

u1x = u(1); u1y = u(2);
u2x = u(3); u2y = u(4);

X = u1x + u2x;
Y = u1y + u2y;

% yaw moment: N = lx*Fy - ly*Fx (moment arms) [2][5]
N = lx(1)*u1y - ly(1)*u1x + lx(2)*u2y - ly(2)*u2x;

tau3 = [X; Y; N];
end
