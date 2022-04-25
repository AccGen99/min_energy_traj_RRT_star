function val = run(tf, shifted_x, phi_f, plots, i_o)

phi = [0, phi_f]; % phi(1) is initial and phi(2) is final
xf = shifted_x; % Final x-coordinate

robot_parameters;
vals = merv_ode_calcs(tf, tau_w);
% Lagragian for x = [x, y, phi] and x_dot = [x_dot, y_dot, phi_dot]
alpha = [0, 0, 0]; %[alpha_x, alpha_y, alpha_phi];   x
% alpha_phi
alpha(3) = (-1*(C3^1.5)*phi_f/(C4*(C3^0.5)*tf - 2*C4*tanh((C3^0.5)*tf/2)));

lambda = [0, 0, 0]; %[lambda_xdot, lambda_ydot, lambda_phidot];  x_dot

% Vector of phi_dot values
t_span = linspace(0, tf, 500);
phi_dot = 0*t_span;  % Array vector, phi_dot(i) corresponds to time(i)
range = 1:length(t_span);
h2 = vals(1);
h1 = vals(2);
% Rotational Velocity
for i=range
    phi_dot(i) = MERV(C3, C4, alpha(3), t_span(i), tf);
    %phi_dot(i) = -MERV_old(phi_f, t_span(i), tf, tau_w, h2, h1, alpha(3));%/(2.25*pi);
end

t_step = t_span(2)-t_span(1);
% Rotational Acceleration
ang_acc = 0*t_span;
for j = range
    if j == length(t_span)
        ang_acc(j) = -phi_dot(j)/t_step;
    else
        ang_acc(j) = (phi_dot(j+1) - phi_dot(j)) / t_step;%calcAacc(phi_f, t_span(j), tf, tau_w, h1);%/(2.25*pi);
    end
end

HA = 0;
for j = range
    phi_d = phi_dot(j);
    phi_dd = ang_acc(j);
    HA = HA + phi_d*t_step + 0.5*phi_dd*t_step^2;
end

if i_o
    fprintf('Heading Angle = %.2f\n', HA)
end
    
if plots
    figure
    plot(t_span, phi_dot)
    title('Angular Velocity vs Time')
    xlabel('Time')
    ylabel('Angular Velocity')
end

% Translational velocity in x-direction requires appropriate values of 
% initial acceleration and alpha_x
% For positive xf, alpha_x < 0 and a_x(0) > 0
u_max = [0, -1, 1]';
ax_max = abs(maxAcc(k2, R_calc(0), A, B, u_max)); % Max possible initial acceleration
if i_o
    fprintf('Max Possible Acc = %.2f\n', ax_max)
end
    
tspan = t_span;%[0, tf];
ax_0 = 0; % Initial calculation for zero initial-acceleration condition
ic = [0, ax_0]; % Ini_vel & ini_acc
opts = odeset('RelTol',1e-2,'AbsTol',1e-4);
[~,vx_1]=ode45(@(t,y) vx_ode(t, y, t_span, phi_dot, C1, C2, alpha_x),...
    tspan,ic,opts);
fL = vx_1(end,1);

ic2 = [0, ax_max];
[~,vx_2] = ode45(@(t,y) vx_ode(t, y, t_span, phi_dot, C1, C2,...
    alpha_x), tspan, ic2, opts);
fU = vx_2(end,1);

ax_0_t = ax_max*fL/(fL-fU);

% Finding final position candidate for x coordinate
ic3 = [0, ax_0_t];
[t_3,vx_3] = ode45(@(t,y) vx_ode(t, y, t_span, phi_dot, C1, C2,...
    alpha_x), tspan, ic3, opts);
t_step = t_3(2) - t_3(1);
vx_vector = vx_3(:,1);
ax_vector = vx_3(:,2);
% Integrating velocity & acceleration assuming constant acceleration in
% each time interval
rng = 1:length(t_3);
c_xf = 0;
for j = rng
    if j~= length(t_3)
        ini_vel = vx_vector(j);
        accln = ax_vector(j);
        fin_vel = vx_vector(j+1);
        c_xf = c_xf + (fin_vel^2 - ini_vel^2)/(2*accln);
    end
        %    c_xf = c_xf + ini_vel*t_step + 0.5*accln*t_step^2;
end
s_factor = c_xf/xf;
alphax_scaled = alpha_x/(s_factor);
ax_0_t_scaled = ax_0_t/s_factor;

if ax_0_t_scaled > ax_max
    ax_0_t_scaled = ax_max;
end

if i_o
    fprintf('Required Acc = %.2f\n', ax_0_t_scaled)
end

alpha(1) = alphax_scaled;

%v_vec = vx_3(:,1)/s_factor;

% Validating
ic4 = [0, ax_0_t_scaled];
[t_4,vx_4] = ode45(@(t,y) vx_ode(t, y, t_span, phi_dot, C1, C2,...
    alphax_scaled), tspan, ic4, opts);
t_step = t_4(2) - t_4(1);

vx_vector1 = vx_4(:,1);
ax_vector1 = vx_4(:,2);

if plots
    figure
    plot(t_span, vx_4(:,1))
    title('X Velocity vs Time')
    xlabel('Time')
    ylabel('X Velocity')
end

rng = 1:length(t_4);
displ = 0;
for j = rng
    ini_vel = vx_vector1(j);
    accln = ax_vector1(j);
    displ = displ + ini_vel*t_step + 0.5*accln*t_step^2;
end

if i_o
    fprintf('Final x-coordinate = %.2f\n', displ(end))
end

% Velocity for x, y and phi
vx_vectOr = interp1(t_4, vx_vector1, t_span);
vy_vector = 0*vx_vectOr;

% Acceleration for x, y and phi
ax_vectOr = interp1(t_4, ax_vector1, t_span);
ay_vector = 0*ax_vectOr;
range1 = 1:length(phi_dot);

x_dot_dot = [ax_vectOr', ay_vector', ang_acc'];
x_dot = [vx_vectOr', vy_vector', phi_dot'];

% Find heading angle
t_step0 = t_span(2) - t_span(1);
phi = phiCalc(phi_dot, ang_acc, t_step0);

% Finding lambda
lamda = 0*x_dot;
for k = range1
    xdot_vec = x_dot(k,:);
    xdotdot_vec = x_dot_dot(k,:);
    omega = phi_dot(k);
    R_dot = omega*R_dot_calc(phi(k));
    R_ = R_calc(phi(k));
    lamda(k,:) = FindLambda(w2, A, xdot_vec', k2, w1, Q, xdotdot_vec',...
        C, omega, R_, R_dot);
end

% Finding lamda dot
%lamda_dot = DerLamda(lamda, t_span);%tf);

% Find control vector
u = 0*lamda;%_dot;
for n = range1
    xdotvec = x_dot(n,:);
    phi_t = phi(n);
    R_now = R_calc(phi_t);
    lamda_val = lamda(n,:);
    u(n,:) = calcContVec(w2, B, R_now, xdotvec', D, A, lamda_val', w1);
end

E = 0*range1;
for n=range1
    u_ = u(n,:);
    x_dot_ = x_dot(n,:);
    p = phi(n);
    R_0 = R_calc(p);
    E(n) = energyCalc(w1,u_',w2,x_dot_',R_0,B);
end
Energy = t_step*cumtrapz(E);
if i_o
    fprintf('Energy = %.2f\n', Energy(end));
end

if plots
    figure
    plot(t_span, Energy)
    title('Energy Drawn vs Time')
    xlabel('Time')
    ylabel('Energy')
end

power = 0*Energy;
for o=range1
    if o==length(range1)
        power(o)=0;%Energy(o)/t_step;
    else
        power(o) = (Energy(o+1)-Energy(o))/(t_step);        
    end
end

if plots
    figure
    plot(t_span, power)
    title('Power vs Time')
    xlabel('Time')
    ylabel('Power (W)')
end


u_max = 1.0;
u_max_obtd = max(u);
u_max_obtd = max(u_max_obtd);
u_max_obtd = abs(u_max_obtd);

u_min_obtd = min(u);
u_min_obtd = min(u_min_obtd);
u_min_obtd = abs(u_min_obtd);

if u_min_obtd > u_max_obtd
    u_max_obtd = u_min_obtd;
end

if i_o
    fprintf('U max val = %.2f\n', u_max_obtd);
    fprintf('\n')
end

tol_angle = 1e-1;
if HA <= phi_f + tol_angle || HA >= phi_f - tol_angle
    cond1 = 1;
else
    cond1 = 0;
end

tol_pos = 1e-2;
if displ(end) <= xf + tol_pos || displ(end) >= xf - tol_pos
    cond2 = 1;
else
    cond2 = 0;
end

tol_vel = 1e-2;
if vx_4(end,1) <= 0 + tol_vel || vx_4(end,1) >= 0 - tol_vel
    cond3 = 1;
else
    cond3 = 0;
end

if cond1 && cond2 && cond3
    ret_val = 1;
else
    ret_val = 0;
end

diff = u_max_obtd - u_max;
val = [Energy(end), ret_val, diff];
end