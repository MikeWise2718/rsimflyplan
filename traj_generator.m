function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

persistent waypoints0 traj_time d0 d pp ppd ppdd
if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
    
    % Note this o3 stuff is to get the spline to use end conditions
    % that set the starting and endinf velocity to zero
    %
    o3 = [0;0;0];
    pp = spline(traj_time,[o3,waypoints0,o3]);
    ppd = pp;
    M = [0 3 0 0;0 0 2 0;0 0 0 1;0 0 0 0];
    ppd.coefs = ppd.coefs*M;
    
    ppdd = ppd;
    ppdd.coefs = ppdd.coefs*M;
    

else
    desired_state.acc = zeros(3,1);
    desired_state.vel = zeros(3,1);
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
    
    if(t == 0)
        desired_state.pos = waypoints0(:,1);    
    elseif (t>=traj_time(end))
        desired_state.pos = waypoints0(:,end);    
    else
        desired_state.pos = ppval(pp,t);
        desired_state.vel = ppval(ppd,t);    
        desired_state.acc = ppval(ppdd,t);    
    end
end
%


%% Fill in your code here

% desired_state.pos = zeros(3,1);
% desired_state.vel = zeros(3,1);
% desired_state.acc = zeros(3,1);
% desired_state.yaw = 0;
end

