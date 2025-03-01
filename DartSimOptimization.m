%All angles in radians, units in mkgs
clearvars; close all; 
format long;

PitchAngle=deg2rad(45);

InitialBaseVelocityGuess=17;
BaseVelocityBounds=[15,20];
BaseYawGuess=0.12;
BaseYawBounds=[0.005;0.2];

InitialOutpostVelocityGuess=14;
OutpostVelocityBounds=[12.5,16];
OutpostYawGuess=-0.09;
OutpostYawBounds=[-0.17;-0.03];

for k=1:5
    if k==1 %Base Yaw Angle
        f=@(BaseYaw)SimulationError(InitialBaseVelocityGuess,InitialOutpostVelocityGuess,BaseYaw,OutpostYawGuess,PitchAngle,true,false);
    
        % Set nondefault solver options
        options = optimoptions("lsqnonlin","OptimalityTolerance",0.001,"Display",...
            "off");
        % Solve
        [solution,objectiveValue] = lsqnonlin(f,BaseYawGuess,...
            repmat(BaseYawBounds(1),size(BaseYawGuess)),repmat(BaseYawBounds(2),...
            size(BaseYawGuess)),options);
        % Clear variables
        clearvars options
        disp('Base Yaw Angle:')
        disp(solution)
        BaseYawGuess=solution;
    end
    if k==2 %Base Initial Velocity
        f=@(InitialBaseVelocity)SimulationError(InitialBaseVelocity,InitialOutpostVelocityGuess,BaseYawGuess,OutpostYawGuess,PitchAngle,true,true);

        % Set nondefault solver options
        options = optimoptions("lsqnonlin","OptimalityTolerance",0.001,"Display",...
            "off");
        % Solve
        [solution,objectiveValue] = lsqnonlin(f,InitialBaseVelocityGuess,...
            repmat(BaseVelocityBounds(1),size(InitialBaseVelocityGuess)),repmat(BaseVelocityBounds(2),...
            size(InitialBaseVelocityGuess)),options);
        % Clear variables
        clearvars options
        disp('Base Initial Velocity:')
        disp(solution)
    end
    if k==3 %Outpost Yaw Angle
        f=@(OutpostYaw)SimulationError(InitialBaseVelocityGuess,InitialOutpostVelocityGuess,BaseYawGuess,OutpostYaw,PitchAngle,false,false);
    
        % Set nondefault solver options
        options = optimoptions("lsqnonlin","OptimalityTolerance",0.001,"Display",...
            "off");
        % Solve
        [solution,objectiveValue] = lsqnonlin(f,OutpostYawGuess,...
            repmat(OutpostYawBounds(1),size(OutpostYawGuess)),repmat(OutpostYawBounds(2),...
            size(OutpostYawGuess)),options);
        % Clear variables
        clearvars options
        disp('Outpost Yaw Angle:')
        disp(solution)
        OutpostYawGuess=solution;
    end
    if k==4 %Outpost Initial Velocity
        f=@(InitialOutpostVelocity)SimulationError(InitialBaseVelocityGuess,InitialOutpostVelocity,BaseYawGuess,OutpostYawGuess,PitchAngle,false,true);

        % Set nondefault solver options
        options = optimoptions("lsqnonlin","OptimalityTolerance",0.001,"Display",...
            "off");
        % Solve
        [solution,objectiveValue] = lsqnonlin(f,InitialOutpostVelocityGuess,...
            repmat(OutpostVelocityBounds(1),size(InitialOutpostVelocityGuess)),repmat(OutpostVelocityBounds(2),...
            size(InitialOutpostVelocityGuess)),options);
        % Clear variables
        clearvars options
        disp('Outpost Initial Velocity:')
        disp(solution)
    end
end

function f=SimulationError(InitialBaseVelocity,InitialOutpostVelocity,BaseYawAngle,OutpostYawAngle,PitchAngle,TargetingBase,OptimizingVelocity)
sim = 

end
