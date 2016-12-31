close all;
clear;

% original in C:/user/mike/documents/MATLAB/assignment_week3

addpath('utils');



%%% pre-calculated trajectories

trajopt = 5;

switch trajopt
    case 1
      trajhandle = @traj_line;
      rootname = 'line';
      waypoints = [];
      max_time = 20;
    case 2
      trajhandle = @traj_helix;
      rootname = 'helix';
      waypoints = [];
      max_time = 20;
    case 3
      trajhandle = @traj_generator;
      rootname = 'wapt5';
      max_time = 20;
      waypoints = [0    0   0;
              1    1   1;
              2    0   2;
              3    -1  1;
              4    0   0]';
    case 4
      trajhandle = @traj_generator;
      rootname = 'waptcirc';
      n = 30;
      npcirc = 10;
      rad = 5;
      cen = [0 0 0];
      wpt = zeros(n+1,3);
      zmax = 5;
      for i = 1:n
          ang = 2*pi*(i-1)/npcirc
          z = zmax*(i-1)/(n-1);
          npt = cen + [ rad*cos(ang) rad*sin(ang) z ]
          wpt(i,:) = npt;
      end
      waypoints = wpt';
      max_time = 300;

   case 5
      trajhandle = @traj_generator;
      rootname = 'waptspiral';
      n = 42;
      npcirc = 12;
      cen = [0 0 0];
      wpt = zeros(n+1,3);
      zmax = 2.0;
      rmax = 1.0;
      highcen = [0 0 zmax];
      for i = 1:n
          ang = 2*pi*(i-1)/npcirc;
          z = zmax*(i-1)/(n-1);
          r = rmax*((i-1)/(n-1))^2;
          npt = cen + [ r*cos(ang) r*sin(ang) z ];
          wpt(i,:) = npt;
      end      
      wpt(n+1,:) = highcen;
      wpt(n+2,:) =     cen;
      waypoints = wpt';
      max_time = 300;      
    otherwise
      error('Error bad trajopt specified')
end

stfname = sprintf('tstate_%s.csv',rootname)
% 
% trajhandle = @traj_helix;

%% Trajectory generation with waypoints
%% You need to implement this
% trajhandle = @traj_generator;

trajhandle([],[],waypoints);


%% controller
controlhandle = @controller;


% Run simulation with given trajectory generator and controller
% state - n x 13, with each row having format [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]
[t, state, QP] = simulation_3d(trajhandle, controlhandle,max_time);

cdata = [t state];
headers = {'t','x', 'y', 'z', 'xd', 'yd', 'zd', 'qw', 'qx', 'qy', 'qz', 'p', 'q', 'r'};
csvwrite_with_headers(stfname,cdata,headers)
