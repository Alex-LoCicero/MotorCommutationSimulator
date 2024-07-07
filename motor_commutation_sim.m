

% This script simulates a BLDC motor with P pole pairs and S slots.
% Here we are visualizing trapezoidal control.

clear all
clc
close 

%% Parameters
P = 1; % pole pairs (poles / 2) 
S = 6; % slots (number of windings), must be multiple of 3
if mod(S,3)
    error('Number of slots must be a multiple of 3')
end

%% Initialize 
simulate = true;
r = 2; % radius of motor for plotting
C = []; % charge matrix time x slots 
E = []; % for plotting electrical cycles
theta_s = []; % fixed angular position of each slot
for s =1:S
    theta_s(s) = 2*pi/S*(s-1);
end
theta_p = []; % fixed angular position of each pole in rotating reference frame
for p =1:2*P
    theta_p(p) = 2*pi/(2*P)*(p-1);
end
theta_loc = 0; % angular position of rotor with respect to slot 1 (phase A) on the positive x axis
theta_delta = []; % rotation between pole and the nearest slot 
theta_least = 0; % min(theta_delta)
tol = 0.0001;
for p = 1:P
    elec_cycle(p) = 2*pi/P*(p-1);
end
elec_cycle(P+1) = 2*pi;
pos = [];
rot_counter = -1;
% NOTES
% - pole 1 is implicitly South (negative)
% - motor rotates CCW

%% Simulate 
valid = false;
while ~valid
    sim_type = input(['Type s to step through the program manually ' ...
        'OR type the number of steps you would like to simulate ' ...
        '(i.e. type 10 for 10 steps): '], 's');
    valid = true;
    if ~strcmp(sim_type,'s')
        Time = round(str2num(sim_type)) + 1;
        if ~isnumeric(Time) || isempty(Time)
            valid = false;
        end
    end
end
t = 1;
while(simulate)
    % determine theta_delta
    theta_delta = ones(3,S)*inf;
    theta_least = inf;
    for s = 1:S
        for p = 1:2*P
            delta = theta_s(s) - theta_p(p);
            if abs(delta) < tol
                delta = 0;
            end
            if abs(delta) < theta_delta(1,s) || (2*pi - abs(delta)) < theta_delta(1,s)
                if (abs(delta) >= theta_delta(1,s) || delta < 0) && (2*pi - abs(delta)) < theta_delta(1,s) 
                    delta = 2*pi - abs(delta); % handle wrapping (closest slot leads/lags pole across 2pi)
                end
                theta_delta(1,s) = abs(delta);
                theta_delta(2,s) = mod(p,2); % north pole (0) or south pole (1)
                if delta == 0 
                    theta_delta(3,s) = 0;
                else
                    theta_delta(3,s) = delta/abs(delta); % pole lags slot (+) or pole leads slot (-)
                end
                if (delta > 0) && (delta < theta_least) 
                    theta_least = delta; % smallest nonzero distance that slot leads a pole (CCW)
                end
            end
        end
    end
    
    % update C matrix
    c = [];
    for s = 1:S
        if ~theta_delta(1,s) % neutral charge for slot with pole on it 
            c(s) = 0;
        elseif ~theta_delta(2,s) % north
            if theta_delta(3,s) > 0 % north pole lags slot
                c(s) = -1;
            else
                c(s) = 1;
            end
        else % south
            if theta_delta(3,s) > 0 % south pole lags slot
                c(s) = 1;
            else
                c(s) = -1;
            end
        end
    end
    C(t,:) = c;
    if any(abs(elec_cycle - theta_loc)<tol)
        E(t) = 1;
    else
        E(t) = 0;
    end

    pos(t) = theta_loc + 2*pi*rot_counter;
    if theta_loc < tol 
        rot_counter = rot_counter + 1;
        pos(t) = theta_loc + 2*pi*rot_counter;
    elseif abs(2*pi - theta_loc) < tol
        pos(t) = theta_loc + 2*pi*rot_counter;
        rot_counter = rot_counter + 1;
    end
    
    % plot 
    figure(1)
    clf(1)
    movegui('northwest')
    plot_motor(r, theta_s) % slots
    plot_rotor(r, theta_p) % poles
    scatter(cos(theta_loc), sin(theta_loc), 200,'red', 'filled', 'o') % tracker
    pause(0.25)
    figure(2)
    movegui('southwest')
    clf(2)
    plot_phases(C, P, E, S, pos)
    pause(0.25)

     % continue?
      if strcmp(sim_type,'s')
          rsp = input('Hit enter to continue, type q to quit...', 's');
          if strcmp(rsp, 'q')
              simulate = false;
          end
      else
          if t >= Time
              simulate = false;
          end
      end

    % compute new positions
    theta_loc = theta_loc+theta_least;
    if theta_loc > 2*pi
        theta_loc = theta_loc - 2*pi;
    end
    for p = 1:2*P
        theta_p(p) = theta_p(p) + theta_least;
        if theta_p(p) >= 2*pi
            theta_p(p) = theta_p(p) - 2*pi;
        end
    end
    t = t+1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Helper Functions 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function plot_motor(r, theta_s)
    scatter(0,0,50000, 'filled','o','MarkerFaceColor','b')
    box on 
    axis square 
    grid on
    hold on
    phases = ['A', 'B', 'C'];
    slots = [];
    S = length(theta_s);
    for s = 1:S
        i = mod(s,3);
        if ~i
            i = 3;
        end
        slots(s) = phases(i);
        line([0 r*cos(theta_s(s))], [0 r*sin(theta_s(s))],...
            'Color', 'r', 'LineStyle', '-', 'LineWidth',5);
        text(1.2*r*cos(theta_s(s)), 1.2*r*sin(theta_s(s)), phases(i),...
            'FontSize', 20, 'Color','r', 'FontWeight','bold')
    end
    scatter(0,0,15000, 'filled','o','MarkerFaceColor','k')
    xlim([-1.4*r 1.4*r])
    ylim([-1.4*r 1.4*r])
    title('Motor Simulation')
end

function plot_rotor(r, theta_p)
    P = length(theta_p); % number of poles
    r = r*.25;% rotor radius
    for p = 1:P
        if ~mod(p,2) % north
            text(1*r*cos(theta_p(p)), 1*r*sin(theta_p(p)), 'N', 'FontSize', 10, 'Color','w')
        else
            text(1*r*cos(theta_p(p)), 1*r*sin(theta_p(p)), 'S', 'FontSize', 10, 'Color','w')
        end
    end
    elec_cycle = 2*pi/(P/2);
    for p = 1:P/2
        line([2*r*cos(p*elec_cycle),4*r*cos(p*elec_cycle)],...
            [2*r*sin(p*elec_cycle), 4*r*sin(p*elec_cycle)], 'linestyle', ':','linewidth', 2, 'color', 'g');
    end
end

function plot_phases(C, P, E, slots, pos)
    P = 2*P; % number of poles
    S = 3;
    phases = ["A", "B", "C"];
    for s = 1:S
        subplot(S,1,s);
        plot(pos,C(:,s), 'LineWidth',3)
        if mod(s,3) == 0
            phase = phases(3);
        else
            phase = phases(mod(s,3));
        end
        title("Phase " + string(phase) + ": " + string(P) + " Poles and " + string(slots) + " slots")
        xlabel('position (rad)')
        ylabel('charge (relative)')
        for t = 1:length(E)
            if E(t)
                line([pos(t), pos(t)],...
                    [-1, 1], 'linestyle', ':','linewidth', 2, 'color', 'g');
            end
        end
    end
end

    
