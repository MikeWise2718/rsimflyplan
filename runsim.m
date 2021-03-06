close all;
clear;

% original in C:/user/mike/documents/MATLAB/assignment_week3

addpath('utils');



%%% pre-calculated trajectories

trajopt = 5;

waypoints = [];
max_time = 20;

switch trajopt
    case 1
      trajhandle = @traj_line;
      rootname = 'line';
    case 2
      trajhandle = @traj_helix;
      rootname = 'helix';
    case 3
      trajhandle = @traj_generator0;
      rootname = 'wapt5';
      waypoints = [0   0   0;
                   1   1   1;
                   2   0   2;
                   3  -1   1;
                   4   0   0]'; % note the transpose
    case 4
      trajhandle = @traj_generator_spline;
      rootname = 'waptcirc';
      n = 30;
      npcirc = 10;
      rad = 5;
      cen = [0 0 0];
      wpt = zeros(n+1,3);
      zmax = 5;
      for i = 1:n
          ang = 2*pi*(i-1)/npcirc;
          z = zmax*(i-1)/(n-1);
          npt = cen + [ rad*cos(ang) rad*sin(ang) z ];
          wpt(i,:) = npt;
      end
      waypoints = wpt';
      max_time = 300;
   case 5
      trajhandle = @traj_generator_spline;
      rootname = 'waptspiral';
      n = 42;
      npcirc = 12;
      cen = [0 0 0];
      cenh = [0 0 0.1];
      wpt = zeros(n+1,3);
      zmax = 2.0;
      rmax = 1.0;
      highcen = [0 0 zmax];
      highcenl = [0 0 zmax-0.2];
      for i = 1:n
          ang = 2*pi*(i-1)/npcirc;
          z = zmax*(i-1)/(n-1);
          r = rmax*((i-1)/(n-1))^2;
          npt = cen + [ r*cos(ang) r*sin(ang) z ];
          wpt(i,:) = npt;
      end      
      wpt(n+1,:) = highcen;
      wpt(n+2,:) = highcenl;
      wpt(n+3,:) =     cen;
      waypoints = wpt';
      max_time = 30;      
    otherwise
      error('Error bad trajopt specified')
end


% 
% trajhandle = @traj_helix;

%% Trajectory generation with waypoints
%% You need to implement this
% trajhandle = @traj_generator;

if ~isempty(waypoints)
   trajhandle([],[],waypoints);
end


%% controller
controlhandle = @controller;


% Run simulation with given trajectory generator and controller
% state - n x 13, with each row having format [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]
[t, state, QP] = simulation_3d(trajhandle, controlhandle,max_time);

cdata = [t state];
stfname = sprintf('tstate_%s.csv',rootname);
headers = {'t','x', 'y', 'z', 'xdot', 'ydot', 'zdot', 'qw', 'qx', 'qy', 'qz', 'p', 'q', 'r'};
csvwrite_with_headers(stfname,cdata,headers)

qdata = [QP.time_hist' QP.state_hist' QP.state_des_hist'];
qfname = sprintf('tstate_quad_%s.csv',rootname);
qheaders = {'t','x', 'y', 'z', 'xdot', 'ydot', 'zdot', 'des_x', 'des_y', 'des_z', 'des_xdot', 'des_ydot', 'des_zdot'};
csvwrite_with_headers(qfname,qdata,qheaders)

% calc error
p = QP.state_hist(1:3,:)';
des_p = QP.state_des_hist(1:3,:)';
n = size(p,1);
diffsq = sum((des_p-p) .* (des_p-p),2);
maxerr = max(diffsq);
avgerr = sum(diffsq)/n;
fprintf(1,'Run:%s - steps:%d maxerr:%.3g avgerr:%.3g\n',rootname,n,maxerr,avgerr) 