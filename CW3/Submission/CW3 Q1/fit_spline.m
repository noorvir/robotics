close all; clear all;
% p = [0 2*pi pi/2 pi];
% x = [0,2,3,5];
% cs = spline(x,p);
% xx = linspace(0,5,101);
% 
% figure;
% plot(x,p,'o',xx,ppval(cs,xx),'-');
% 
% c3 = cs.coefs(1,:);
% c2 = cs.coefs(2,:);
% c1 = cs.coefs(3,:);
% 
% 
% xx = linspace(0,3,101);
% y = polyval(c3,xx);
% figure;
% plot (xx,y)
% 
% 



xx = linspace(2,5,303);
xx1 = linspace(1,2,101);
xx2 = linspace(5,6,101);
% y = 1./[xx1,xx,xx2] .* sin([xx1,xx,xx2]);
y = sin([xx1,xx,xx2]);

points_x = [1,2,3,4,5,6];
% points_y = 1./points_x .* sin(points_x);
points_y = sin(points_x);
% number of points to interpolate between
N = numel(points_x);

% Built in function
cs = spline(points_x,[0 points_y 0]);

% number of equations
n = N-2;

% Implementation
D = sparse(1:n,1:n,4*ones(1,n),n,n);
E = sparse(2:n,1:n-1,ones(1,n-1),n,n);
S = E+D+E';
A = full(S)
A(1,1:2) = [5,1];
% A(end,end) = 2

b = zeros(N-2,1);

% Coefficient matrix
a = zeros(N-2,4);

for i = 2:N-3
    b(i) = 3* (points_y(i+2) - points_y(i));
end

b(1) = 3* (points_y(3) + points_y(2)) - 0 - 6*points_y(1) - 0;
b(N-2) = 3* (points_y(N) - points_y(N-2)) - 0;

v = linsolve(A,b);
v = [0;v;0];

x_eval = linspace(0.1,1,101);
% z = zeros(N-1,numel(x_eval));

% go over each piece of the spline and calculate the coeffiecients for the 
% cubic polynomial
for i = 1:N-2
    a(i,4) = points_y(i+1);
    a(i,3) = v(i+1);
    a(i,2) = 3*(points_y(i+2) - points_y(i+1)) - 2 * v(i+1) - v(i+2);
    a(i,1) = 2*(points_y(i+1) - points_y(i+2)) + v(i+1) + v(i+2);
end


% Quadratic polynomial for first point
p4(1,5) = points_y(1);
p4(1,4) = v(1); % initial velocity
p4(1,3) = 0; % initial acceleration
p4(1,2) = 4*(points_y(2)- points_y(1)) - v(2) - 0 - 0;
p4(1,1) = v(2) + 0 + 0 - 3*(points_y(2) - points_y(1));


pf_A = [1,1,1;2,3,4;2,6,12];
pf_b = [points_y(6)-points_y(5)-v(5);...
        v(6)-v(5);...
        0];


pf_coefs = linsolve(pf_A,pf_b);

% Quadratic polynomial for last points
p4(2,5) = points_y(5);
p4(2,4) = v(5); % initial velocity
p4(2,3) = pf_coefs(1); % initial acceleration
p4(2,2) = pf_coefs(2);
p4(2,1) = pf_coefs(3);


% Use inbuilt MATLAB format to evaluate spline
z.breaks = points_x(2:end);
z.form = 'pp';
z.coefs = a;
z.pieces = N-2;
z.order = 4;
z.dim = 1;


% Evaluate first polynomial
z1.breaks = points_x(1:2);
z1.form = 'pp';
z1.coefs = p4(1,:);
z1.pieces = 1;
z1.order = 5;
z1.dim = 1;


% Evaluate first polynomial
z2.breaks = points_x(5:6);
z2.form = 'pp';
z2.coefs = p4(2,:);
z2.pieces = 1;
z2.order = 5;
z2.dim = 1;

z_eval = ppval(z,xx);
z1_eval = ppval(z1,xx1);
z2_eval = ppval(z2,xx2);

figure;
plot(points_x, points_y,'o', [xx1,xx,xx2], y, 'r-', xx, z_eval,'k-');
hold on;
plot(xx1, z1_eval,'m-');
hold on;
plot(xx2, z2_eval,'m-');
hold off;
figure;
plot(points_x, points_y,'o', [xx1,xx,xx2], y, 'r-', [xx1,xx,xx2], ppval(cs,[xx1,xx,xx2]),'b-');





% Recursively calculate the nth derivate of a polynomial and return it in a
% format compatible with evaulation using the MATLAB ppval function
function [coefs, pol] = polynomial(coefs, n)

    N = numel(coefs);
    
    if n > 0
        coefs = polyder(coefs);
        n = n - 1;
        coefs = ploynomial(coefs, n);
    end
    
    
    pol = coefs;
    
    pol.breaks = N;
    pol.form = 'pp';
    pol.coefs = coefs;
    pol.pieces = N-1;
    pol.order = numel(coefs);
    pol.dim = 1;
end

