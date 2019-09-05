function dx = dynamics_OpenSim(t, x, u, p)
%DYNAMICS_OPENSIM Run OpenSim Model Dynamics
%   Interfaces with the OpenSim API to update a model's state, load
%   controls, and return the resulting state variable deriviatives.
%
%   Inputs:
%       t: scalar or vector of time values.
%       x: matrix or column vector of states to load (optional).
%       u: matrix or column vector of control values (optional).
%       osimModel: handle to the OpenSim model object.
%       osimState: handle to the OpenSim state object.
%       p: structure with model information.
%
%   Outputs:
%       dx: matrix of state derivatives at each time step.

%   See also:
%       [1] DYNAMICS function from: OptimTraj by Matthew Kelly, 
%           https://github.com/MatthewPeterKelly/OptimTraj
%       [2] COMPUTEOPENSIMMODELXDOT function from: L.-F. Lee and B. R. 
%           Umberger, “Generating optimal control simulations of 
%           musculoskeletal movement using OpenSim and MATLAB,” PeerJ, vol. 
%           4, p. e1638, 2016.
%
%   $Author: BH $    $Date: 2019-08-16 $    $Revision: 0 $

    %% Error Checking
    osimModel = p.model;
    osimState = p.state;
    if(~isa(osimModel, 'org.opensim.modeling.Model'))
        error('OpenSimPlantFunction:InvalidArgument', [...
            '\tError in OpenSimPlantFunction\n',...
            '\topensimModel is not type (org.opensim.modeling.Model).']);
    end
    if(~isa(osimState, 'org.opensim.modeling.State'))
        error('OpenSimPlantFunction:InvalidArgument', [...
            '\tError in OpenSimPlantFunction\n',...
            '\topensimState is not type (org.opensim.modeling.State).']);
    end
    
    nt = length(t);
    dx = zeros(p.nx, nt);
    
    for it=1:nt

        %% Load Initial States
        osimState.setTime(t(it));
        updateOsimState(x(:,it), p);

        %% Compute State Derivatives
        % Not sure if this is necessary --- need to check.
        osimModel.computeStateVariableDerivatives(osimState);

        %% Update Controls
        if(~isempty(u)) % There are controls.
            if isa(u, 'function_handle')
                controlVector = u(osimModel, osimState);
            else
                controlVector = u(:,it);
            end
            updateOsimControl(controlVector, p);
        end

        %% Compute State Derivatives (Again)
        osimModel.computeStateVariableDerivatives(osimState);
        dx(:,it) = getOsimStateDerivative(p);
    end
    
    % --------------------------------------------------------------------
    % Helper Functions
    % --------------------------------------------------------------------
    
    function updateOsimState(x, p)
        for ix = 0:(p.nx-1)
            osimState.updY().set(ix, x(ix+1));
        end
    end

    function updateOsimControl(u, p)
        controlReference = osimModel.updControls(osimState);
        controlUpdate = org.opensim.modeling.Vector(1, 0.0);
        for i=0:(p.nu-1)
            controlUpdate.set(0, u(i+1));
            osimModel.updActuators().get(i).addInControls(controlUpdate, controlReference);
        end
        osimModel.setControls(osimState, controlReference);
    end

    function dx = getOsimStateDerivative(p)
        dx = zeros(p.nx, 1);
        for ix=0:(p.nx-1)
            dx(ix+1,1) = osimState.getYDot().get(ix);
        end
    end
    
end