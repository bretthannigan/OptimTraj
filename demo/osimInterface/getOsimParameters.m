function p = getOsimParameters(model)
% p = getOsimParameters()
%
% This function returns the parameters struct for the five link biped, with
% parameters that correspond to the robot RABBIT, taken from the 2003 paper
% by Westervelt, Grizzle, and Koditschek: "Hybrid Zero Dynamics of Planar
% Biped Walkers"
%

    MIN_COORD_VELOCITY = 0; % Lower bound on generalized coordinate velocities (rad/s).
    MAX_COORD_VELOCITY = 10; % Upper bound on generalized coordinate velocities (rad/s).
    MIN_ACTIVATION = 0.01; % Lower bound on muscle activation.
    MAX_ACTIVATION = 1.0; % Upper bound on muscle activation.
    MIN_FIBER_LENGTH = 0.01; % Lower bound on muscle fiber length.
    MAX_FIBER_LENGTH = 1.0; % Upper  bound on muscle fiber length.

    p.model = model;
    p.state = model.initSystem();
    p.name = model.getName();
    p.nx = model.getNumStateVariables();
    p.nu = model.getNumControls();
    p.coords = model.getCoordinateSet();
    p.nCoord = model.getNumCoordinates();
    p.actuators = model.getMuscles();
    p.nActuator = p.actuators.getSize();

    % Get the names of the states from the model
    p.xNames = cell(p.nx, 1);
    p.xMin = zeros(p.nx, 1);
    p.xMax = zeros(p.nx, 1);
    for i = 1:p.nx
        stateVariableName = model.getStateVariableNames().getitem(i-1);
        p.xNames{i,1} = char(stateVariableName);
        % Find minimum and maximum values permitted by model for each state
        % variable.
        stateVariableParts = split(char(stateVariableName), '/');
        switch stateVariableParts{2}
            case 'jointset' % This state variable is a generalized coordinate.
                if strcmp(stateVariableParts{end}, 'speed')
                    % OpenSim does not have bounds on generalized coordinate
                    % velocities, so use the constants from this file.
                    p.xMin(i) = MIN_COORD_VELOCITY;
                    p.xMax(i) = MAX_COORD_VELOCITY;
                elseif strcmp(stateVariableParts{end}, 'value')
                    % OpenSim has bounds on generalized coordinate positions,
                    % so read them in.
                    setI = p.coords.getIndex(stateVariableParts{end-1});
                    if setI>=0
                        p.xMin(i) = p.coords.get(setI).get_range(0); % Minimum range.
                        p.xMax(i) = p.coords.get(setI).get_range(1); % Maximum range.
                    else
                        error(['jointset state variable not found: ' stateVariableParts{end-1}]);
                    end
                else
                    error(['Unrecognized jointset state variable: ' stateVariableParts{end}]);
                end
            case 'forceset' % This is an actuator state variable. 
                setI = p.actuators.getIndex(stateVariableParts{end-1});
                if strcmp(stateVariableParts{end}, 'activation')
                    % OpenSim muscle classes have minimum activation 
                    % properties, so read this (in a roundabout way). There is 
                    % no corresponding maximum activation, so use the constant 
                    % from this file.
                    if setI>=0
                        muscleType = p.actuators.get(setI).getConcreteClassName();
                        try
                            minActivation = eval(['org.opensim.modeling.' char(muscleType) '().getMinimumActivation()']);
                            if isnan(minActivation)
                                p.xMin(i) = MIN_ACTIVATION;
                            else
                                p.xMin(i) = minActivation;
                            end
                        catch
                            p.xMin(i) = MIN_ACTIVATION;
                        end
                        p.xMax(i) = MAX_ACTIVATION;
                    else
                        error(['forceset state variable not found: ' stateVariableParts{end-1}]);
                    end
                elseif strcmp(stateVariableParts{end}, 'fiber_length')
                    % OpenSim muscle classes have minimum fiber length
                    % properties, so read this in the same way as previous.
                    % There is no corresponding maximum fiber length, so use
                    % the constant from this file.
                    if setI>=0
                        muscleType = p.actuators.get(setI).getConcreteClassName();
                        try
                            minFiberLength = eval(['org.opensim.modeling.' char(muscleType) '().getMinimumFiberLength()']);
                            if isnan(minFiberLength)
                                p.xMin(i) = MIN_FIBER_LENGTH;
                            else
                                p.xMin(i) = minFiberLength;
                            end
                        catch
                            p.xMin(i) = MIN_FIBER_LENGTH;
                        end
                        p.xMax(i) = MAX_FIBER_LENGTH;
                    else
                        error(['forceset state variable not found: ' stateVariableParts{end-1}]);
                    end
                else
                    error(['Unrecognized forceset state variable: ' stateVariableParts{end}]);
                end
        end
    end

    % Get the names of the controls/muscles from the model (same in this case)
    p.uNames = cell(p.nu, 1);
    p.uMin = zeros(p.nu, 1);
    p.uMax = zeros(p.nu, 1);
    for i = 1:p.nu
       thisActuator = p.actuators.get(i-1);
       p.uNames{i,1} = char(thisActuator.getName());
       p.uMin(i,1) = p.actuators.get(i-1).get_min_control();
       p.uMax(i,1) = p.actuators.get(i-1).get_max_control();
    end

end