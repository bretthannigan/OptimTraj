% MAIN.m  --  OpenSim Model Dynamics Interface
%
% This script interfaces with an OpenSim model's dynamics.
%

clc; clear; 
addpath ../../

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Set up parameters and options                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

import org.opensim.modeling.*
geometryPath = 'C:\Users\bchannig\Documents\OpenSim\4.0\Geometry';
modelPath = 'C:\Users\bchannig\Documents\MATLAB\Lee2016\OsimModels\LowerLimb_v4.osim';
ModelVisualizer.addDirToGeometrySearchPaths(geometryPath);
osimModel = Model(modelPath);

param = getOsimParameters(osimModel);
param.stateCst = readStoFile();
if ~all(ismember(param.stateCst.stateNames, param.xNames))
    % Error - states in stateNames that are not in model.
end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Set up function handles                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.func.dynamics =  @(t,x,u)( dynamics_OpenSim(t,x,u,param) );

problem.func.pathObj = @(t,x,u)( obj_actSquared_OpenSim(u) );

%problem.func.bndCst = @(t0,x0,tF,xF)( stepConstraint(x0,xF,param) );

%problem.func.pathCst = @(t,x,u)( pathConstraint(x) );


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%               Set up bounds on time, state, and control                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

eps = 1e-3;

problem.bounds.initialTime.low = param.stateCst.time(1) - eps;
problem.bounds.initialTime.upp = param.stateCst.time(1) + eps;
problem.bounds.finalTime.low = param.stateCst.time(end) - eps;
problem.bounds.finalTime.upp = param.stateCst.time(end) + eps;

problem.bounds.state.low = param.xMin - eps;
problem.bounds.state.upp = param.xMax + eps;
problem.bounds.initialState.low = [-0.02467413-eps; -eps; -0.74652253-eps; -eps; 0.00000053-eps; -eps; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf];%param.stateCst.states(:,1) - eps;
problem.bounds.initialState.upp = [-0.02467413+eps; eps; -0.74652253+eps; eps; 0.00000053+eps; eps; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf];%param.stateCst.states(:,1) + eps;
problem.bounds.finalState.low = [-0.02467413-eps; -eps; -0.74652253-eps; -eps; 0.00000053-eps; -eps; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf];%[1.48352986-eps; -eps; -1.39626340-eps; -eps; -eps; -eps; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -Inf];%param.stateCst.states(:,end) - eps;
problem.bounds.finalState.upp = [-0.02467413+eps; eps; -0.74652253+eps; eps; 0.00000053+eps; eps; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf];%[1.48352986+eps; eps; -1.39626340+eps; eps; eps; eps; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf; Inf];%param.stateCst.states(:,end) + eps;

problem.bounds.control.low = param.uMin;
problem.bounds.control.upp = param.uMax;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              Create an initial guess for the trajectory                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.guess.time = param.stateCst.time;%[param.stateCst.time(1) param.stateCst.time(end)];
problem.guess.state = param.stateCst.states;%[param.stateCst.states(:,1) param.stateCst.states(:,end)];
problem.guess.control = 0.02*(ones(param.nu, length(param.stateCst.time)) + eps);%param.uMin*(ones(1, length(param.stateCst.time)) + eps);  %Start with passive trajectory


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Options:                                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


%NOTE:  Here I choose to run the optimization twice, mostly to demonstrate
%   functionality, although this can be important on harder problems. I've
%   explicitly written out many options below, but the solver will fill in
%   almost all defaults for you if they are ommitted.

% method = 'trapezoid';
% method = 'trapGrad';   % This one is also good
% method = 'hermiteSimpson';
% method = 'hermiteSimpsonGrad';   % Suggested method
method = 'chebyshev';   
% method = 'rungeKutta';  %slow!
% method = 'rungeKuttaGrad';
% method = 'gpops';

%%%% Method-independent options:
problem.options(1).nlpOpt = optimset(...
    'Display','iter',...   % {'iter','final','off'}
    'TolFun',1e-3,...
    'MaxFunEvals',1e6);   %options for fmincon
% problem.options(2).nlpOpt = optimset(...
%     'Display','iter',...   % {'iter','final','off'}
%     'TolFun',1e-6,...
%     'MaxFunEvals',5e5);   %options for fmincon

switch method
    
    case 'trapezoid'
        problem.options(1).method = 'trapezoid'; % Select the transcription method
        problem.options(1).trapezoid.nGrid = 10;  %method-specific options
        
        %problem.options(2).method = 'trapezoid'; % Select the transcription method
        %problem.options(2).trapezoid.nGrid = 25;  %method-specific options
        
    case 'trapGrad'  %trapezoid with analytic gradients
        
        problem.options(1).method = 'trapezoid'; % Select the transcription method
        problem.options(1).trapezoid.nGrid = 10;  %method-specific options
        problem.options(1).nlpOpt.GradConstr = 'on';
        problem.options(1).nlpOpt.GradObj = 'on';
        problem.options(1).nlpOpt.DerivativeCheck = 'off';
        
        problem.options(2).method = 'trapezoid'; % Select the transcription method
        problem.options(2).trapezoid.nGrid = 45;  %method-specific options
        problem.options(2).nlpOpt.GradConstr = 'on';
        problem.options(2).nlpOpt.GradObj = 'on';
        
    case 'hermiteSimpson'
        
        % First iteration: get a more reasonable guess
        problem.options(1).method = 'hermiteSimpson'; % Select the transcription method
        problem.options(1).hermiteSimpson.nSegment = 6;  %method-specific options
        
        % Second iteration: refine guess to get precise soln
        problem.options(2).method = 'hermiteSimpson'; % Select the transcription method
        problem.options(2).hermiteSimpson.nSegment = 15;  %method-specific options
        
    case 'hermiteSimpsonGrad'  %hermite simpson with analytic gradients
        
        problem.options(1).method = 'hermiteSimpson'; % Select the transcription method
        problem.options(1).hermiteSimpson.nSegment = 6;  %method-specific options
        problem.options(1).nlpOpt.GradConstr = 'on';
        problem.options(1).nlpOpt.GradObj = 'on';
        problem.options(1).nlpOpt.DerivativeCheck = 'off';
        
        problem.options(2).method = 'hermiteSimpson'; % Select the transcription method
        problem.options(2).hermiteSimpson.nSegment = 15;  %method-specific options
        problem.options(2).nlpOpt.GradConstr = 'on';
        problem.options(2).nlpOpt.GradObj = 'on';
        
        
    case 'chebyshev'
        
        % First iteration: get a more reasonable guess
        problem.options(1).method = 'chebyshev'; % Select the transcription method
        problem.options(1).chebyshev.nColPts = 9;  %method-specific options
        
%         % Second iteration: refine guess to get precise soln
%         problem.options(2).method = 'chebyshev'; % Select the transcription method
%         problem.options(2).chebyshev.nColPts = 50;  %method-specific options
        
    case 'multiCheb'
        
        % First iteration: get a more reasonable guess
        problem.options(1).method = 'multiCheb'; % Select the transcription method
        problem.options(1).multiCheb.nColPts = 6;  %method-specific options
        problem.options(1).multiCheb.nSegment = 4;  %method-specific options
        
        % Second iteration: refine guess to get precise soln
        problem.options(2).method = 'multiCheb'; % Select the transcription method
        problem.options(2).multiCheb.nColPts = 9;  %method-specific options
        problem.options(2).multiCheb.nSegment = 4;  %method-specific options
        
    case 'rungeKutta'
        problem.options(1).method = 'rungeKutta'; % Select the transcription method
        problem.options(1).defaultAccuracy = 'low';
        problem.options(2).method = 'rungeKutta'; % Select the transcription method
        problem.options(2).defaultAccuracy = 'medium';
    
    case 'rungeKuttaGrad'
      
        problem.options(1).method = 'rungeKutta'; % Select the transcription method
        problem.options(1).defaultAccuracy = 'low';
        problem.options(1).nlpOpt.GradConstr = 'on';
        problem.options(1).nlpOpt.GradObj = 'on';
        problem.options(1).nlpOpt.DerivativeCheck = 'off';
        
        problem.options(2).method = 'rungeKutta'; % Select the transcription method
        problem.options(2).defaultAccuracy = 'medium';
        problem.options(2).nlpOpt.GradConstr = 'on';
        problem.options(2).nlpOpt.GradObj = 'on';
        
    case 'gpops'
        problem.options = [];
        problem.options.method = 'gpops';
        problem.options.defaultAccuracy = 'high';
        problem.options.gpops.nlp.solver = 'snopt';  %Set to 'ipopt' if you have GPOPS but not SNOPT
        
    otherwise
        error('Invalid method!');
end



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Solve!                                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%%%%% THE KEY LINE:
soln = optimTraj(problem);

% Transcription Grid points:
t = soln(end).grid.time;
q = soln(end).grid.state;
u = soln(end).grid.control;

save('block.mat', 't', 'q', 'u', 'soln');

% %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% %                     Plot the solution                                   %
% %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% 
% %Anim.figNum = 1; clf(Anim.figNum);
% Anim.speed = 0.25;
% Anim.plotFunc = @(t,q)( drawRobot(q,param) );
% Anim.verbose = true;
% animate(t,q,Anim);
% 
% figure(2); clf;
% subplot(1,2,1);
% plot(t,q);
% legend('q1','q2','q3','q4','q5');
% xlabel('time')
% ylabel('link angles')
% subplot(1,2,2);
% plot(t,u);
% legend('u1','u2','u3','u4','u5');
% xlabel('time')
% ylabel('joint torques')
% 
% if isfield(soln(1).info,'sparsityPattern')
%    figure(3); clf;
%    spy(soln(1).info.sparsityPattern.equalityConstraint);
%    axis equal
%    title('Sparsity pattern in equality constraints')
% end