clc, clear, close all

%% PART ONE - FIT TO FIGURE 8 DATA





%% PART TWO - FIT TO BOX DATA






%% PART THREE - FIT TO SINE WAVE







%% example
% define global variables
global dt; global t; global u; global xhat;

% initialize data and time parameters
data = readmatrix("HW2p3data.txt");
u = data(:, 1:2);
xhat = data(:, 3:5);
N = size(xhat, 1);
dt = 1/100;
T = dt*N;
t = linspace(0, T, N);

% Set up optimization parameters
A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];
xpi0 = ones(N+1, 3);
xpi0(N+1,1:3) = [3.5 13 0];
xpi0(1,1:3) = xhat(1,:);
for j = 2:N
    xupvec = [xpi0(j-1,1) xpi0(j-1,2) xpi0(j-1,3) u(j-1,1) u(j-1,2) xpi0(N+1,1) xpi0(N+1,2)];
    xpi0(j,1:3) = xpi0(j-1,1:3) + (dt.*modeldd(xupvec));
end
fun = @costfunction;
nonlcon = @dynamicsfunction;
options = optimoptions(@fmincon, 'Algorithm', 'sqp', 'MaxIterations', 1500);

% Call fmincon
xpistar = fmincon(fun, xpi0, A, b, Aeq, beq, lb, ub, nonlcon, options);

% Plot results
plot(xhat(1:end-1, 1), xhat(1:end-1,2), xpi0(1:end-1,1), xpi0(1:end-1,2), xpistar(1:end-1,1), xpistar(1:end-1,2));
legend('xhat', 'first trajectory', 'last trajectory')

%% define functions
% model function
function xdot = modeldd(xup)
    B = [cos(xup(3)) cos(xup(3)); sin(xup(3)) sin(xup(3)); (1/xup(7)) (-1/xup(7))];
    xcol = 0.5*xup(6)*B*xup(4:5)';
    xdot = xcol';
end

% cost function
function J = costfunction(xi)
    N = length(xi);
    global dt; global t; global u; global xhat;
    Q = diag([2, 2, 1]);
    J = 0;
    for i = 1:N
        J = J + dt*(xi(i,1:3)-xhat(i,1:3))*Q*(xi(i,1:3)-xhat(i,1:3))';
    end
end

% dynamics function
function [c, ceq] = dynamicsfunction(xp)
    c = [];
    N = length(xp)-1;
    xsim = ones(N, 3);
    for j = 2:N
        xup = [xsim(j-1,1) xsim(j-1,2) xsim(j-1,3) u(j-1,1) u(j-1,2) xp(N+1,1) xp(N+1,2)];
        xsim(j,1:3) = xsim(j-1,1:3) + (dt.*modeldd(xup));
    end
    ceq = xp(2:N,1:3) - xsim(1:N-1,1:3);
    ceq = [xp(1,1:3) - [0 0 0]; ceq];
end