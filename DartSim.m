%All angles in radians, units in mkgs
classdef DartSim < handle
    properties (AbortSet)
        Timestep;
        InitialAngle;
        Mass;
        InertiaMatrix;
        AirVelocity;
        AngularVelocity;
        Position;
        DBVelocity; %Dart Basis velocity-> Z is along longitudinal axis of dart
        DisplayBasisVectors;
        RunningOptimization;
        TargetingBase;
    end
    properties (Constant)
        ForceXInterpolant = load('Interpolants.mat').ForceXInterpolant;
        ForceYInterpolant = load('Interpolants.mat').ForceYInterpolant;
        ForceZInterpolant = load('Interpolants.mat').ForceZInterpolant;
        TorqueXInterpolant = load('Interpolants.mat').TorqueXInterpolant;
        TorqueYInterpolant = load('Interpolants.mat').TorqueYInterpolant;
        TorqueZInterpolant = load('Interpolants.mat').TorqueZInterpolant;
    end
    properties (SetAccess = private)
        PlotPitchAngularAcceleration;
        PlotMisalignment;
        PlotPosition;
        PlotHeadPosition;
        PlotDBVelocity;
        PlotDartBasisX;
        PlotDartBasisY;
        PlotDartBasisZ;
        PlotTime;
        PlotAngularVelocity;
        TargetHit = false;
        Error;
    end

    methods
        %Class Constructor:
        function obj = DartSim(InitialAngle, DBVelocity, Position, Mass, InertiaMatrix, varargin)
                if nargin < 5
                    error('Insufficient arguments');
                end
                DefaultTimestep = 0.001;
                DefaultAirVelocity = [0;0;0];
                DefaultAngularVelocity = [0;0;0];
                DefaultDisplayBasisVectors = false;
                DefaultRunningOptimization = false;
                DefaultTargetingBase = true;
                p = inputParser;
                addParameter(p,'Timestep',DefaultTimestep);
                addParameter(p,'AirVelocity',DefaultAirVelocity);
                addParameter(p,'AngularVelocity',DefaultAngularVelocity);
                addParameter(p,'DisplayBasisVectors',DefaultDisplayBasisVectors);
                addParameter(p,'RunningOptimization',DefaultRunningOptimization);
                addParameter(p,'TargetingBase',DefaultTargetingBase);
                parse(p, varargin{:});

                obj.InitialAngle=InitialAngle;
                obj.DBVelocity = DBVelocity;
                obj.Mass=Mass;
                obj.InertiaMatrix=InertiaMatrix;
                obj.Position = Position;
                obj.Timestep=p.Results.Timestep;
                obj.AirVelocity=p.Results.AirVelocity;
                obj.AngularVelocity=p.Results.AngularVelocity;
                obj.DisplayBasisVectors=p.Results.DisplayBasisVectors;
                obj.RunningOptimization=p.Results.RunningOptimization;
                obj.TargetingBase=p.Results.TargetingBase;
        end

        %Runs the simulation for a given setup
        function run(obj)        
            %Starting conditions
            DartBasis=AngleToDartBasis(obj.InitialAngle);
            Gravity=[0;-9.8083;0]; %Local gravitational acceleration for Seattle, WA
            HeadToCoMLength = 0.04;
            %Gravity=[0;-9.7877;0]; %Local gravitational acceleration for Shenzhen, China
            Velocity=DartBasisToWorldBasis(obj.DBVelocity,DartBasis);
            DecomposedInertiaMatrix = decomposition(obj.InertiaMatrix,'lu'); %L-U decomposition
            Time=0;
            
            j=1;
            while obj.Position(2)>0   
                %Plotting vectors
                Misalignment=DartBasis(:,3)-Velocity./norm(Velocity);
                obj.PlotMisalignment(j,:)=Misalignment;
                obj.PlotPosition(j,:)=obj.Position;
                obj.PlotDBVelocity(j,:)=obj.DBVelocity;
                obj.PlotTime(j)=Time;
                obj.PlotAngularVelocity(j,:)=obj.AngularVelocity;
            
                %Interpolate query point into reference data
                DBAeroForceX=obj.ForceXInterpolant(obj.DBVelocity(1),obj.DBVelocity(2),obj.DBVelocity(3));
                DBAeroForceY=obj.ForceYInterpolant(obj.DBVelocity(1),obj.DBVelocity(2),obj.DBVelocity(3));
                DBAeroForceZ=obj.ForceZInterpolant(obj.DBVelocity(1),obj.DBVelocity(2),obj.DBVelocity(3));
                PitchTorque=obj.TorqueYInterpolant(obj.DBVelocity(1),obj.DBVelocity(2),obj.DBVelocity(3));
                YawTorque=obj.TorqueXInterpolant(obj.DBVelocity(1),obj.DBVelocity(2),obj.DBVelocity(3));
                RollTorque=obj.TorqueZInterpolant(obj.DBVelocity(1),obj.DBVelocity(2),obj.DBVelocity(3));
            
                %Simulation
                %Translational Dynamics
                DBAeroForce=[DBAeroForceX;DBAeroForceY;DBAeroForceZ];
                AeroForce=DartBasisToWorldBasis(DBAeroForce,DartBasis);
                Acceleration=Gravity+(AeroForce./obj.Mass);
                obj.Position=obj.Position+Velocity*obj.Timestep+0.5*Acceleration*(obj.Timestep^2);
                HeadPosition=obj.Position+HeadToCoMLength*DartBasis(:,3);
                Velocity=Velocity+Acceleration*obj.Timestep;
                DBAirVelocity=WorldBasisToDartBasis(obj.AirVelocity,DartBasis);
                obj.DBVelocity=WorldBasisToDartBasis(Velocity,DartBasis)-DBAirVelocity;
                
                %Rotational Dynamics
                DBTorque=[PitchTorque;YawTorque;RollTorque];
                DBAngularAcceleration=DecomposedInertiaMatrix\DBTorque;
                AngularAcceleration=DartBasisToWorldBasis(DBAngularAcceleration,DartBasis);
                DartBasis(:,1)=UpdateVector(obj.AngularVelocity,DartBasis(:,1),obj.Timestep);
                DartBasis(:,2)=UpdateVector(obj.AngularVelocity,DartBasis(:,2),obj.Timestep);
                DartBasis(:,3)=UpdateVector(obj.AngularVelocity,DartBasis(:,3),obj.Timestep);
                obj.AngularVelocity=obj.AngularVelocity+AngularAcceleration*obj.Timestep;
                
                Time=Time+obj.Timestep;
                %Stop when dart hits the target and record hit
                %Outpost
                if 2.354053<obj.Position(1) && obj.Position(1)<2.489053 && 1.4989477<obj.Position(2) && obj.Position(2)<1.5566663 && 16.6838448<obj.Position(3) && obj.Position(3)<16.7946805
                    obj.TargetHit=true;
                    break
                end
                %Base
                if 7.4325335<obj.Position(1) && obj.Position(1)<7.5675337 && 1.0484971<obj.Position(2) && obj.Position(2)<1.1062019 && 26.0548107<obj.Position(3) && obj.Position(3)<26.1656942
                    obj.TargetHit=true;
                    break
                end     
                
                %Plot dart basis vectors along path
                k=j/500;
                if mod(k,1)==1
                    obj.PlotDartBasisX{k}={[obj.Position(1), obj.Position(1)+DartBasis(1,1)],[obj.Position(3), obj.Position(3)+DartBasis(3,1)],[obj.Position(2), obj.Position(2)+DartBasis(2,1)]};
                    obj.PlotDartBasisY{k}={[obj.Position(1), obj.Position(1)+DartBasis(1,2)],[obj.Position(3), obj.Position(3)+DartBasis(3,2)],[obj.Position(2), obj.Position(2)+DartBasis(2,2)]};
                    obj.PlotDartBasisZ{k}={[obj.Position(1), obj.Position(1)+DartBasis(1,3)],[obj.Position(3), obj.Position(3)+DartBasis(3,3)],[obj.Position(2), obj.Position(2)+DartBasis(2,3)]};
                end
                 
                if obj.RunningOptimization && obj.TargetingBase
                        if abs(obj.Position(3)-26.11025245) < 0.009
                            IdealIntersectionCoordinates=[7.5000336,1.0773495];
                            IntersectionCoordinates=[obj.Position(1),obj.Position(2)];
                            obj.Error=IntersectionCoordinates-IdealIntersectionCoordinates;
                            break
                        end
                elseif obj.RunningOptimization && ~obj.TargetingBase
                        if abs(obj.Position(3)-16.73926265) < 0.007
                            IdealIntersectionCoordinates=[2.421553,1.527807];
                            IntersectionCoordinates=[obj.Position(1),obj.Position(2)];
                            obj.Error=IntersectionCoordinates-IdealIntersectionCoordinates;
                            break
                        end  
                end

                obj.PlotPitchAngularAcceleration(j,:)=DBAngularAcceleration(1);
                obj.PlotHeadPosition(j,:)=HeadPosition;
                j=j+1; %increment iteration var
            end
        end
        
    end
end

%Convert world basis vector to dart basis
function f = WorldBasisToDartBasis(WorldBasisVector,DartBasis)
    f=DartBasis\WorldBasisVector;
end

%Convert dart basis vector to world basis
function p = DartBasisToWorldBasis(DartBasisVector,DartBasis)
    p=DartBasis*DartBasisVector;
end

%Generate a Rodrigues rotation matrix using a fixed axis to
%rotate about, then multiply the vector by it's Rodrigues matrix.
function d = UpdateVector(FixedAxisVector,Vector,Timestep)
    if all(FixedAxisVector==[0;0;0])
        RodriguesMatrix=eye(3);
    else
        Theta=Timestep*norm(FixedAxisVector);
        FixedAxisVector=FixedAxisVector./norm(FixedAxisVector);
    
        FixedAxisMatrix=[0, -FixedAxisVector(3), FixedAxisVector(2)
                     FixedAxisVector(3), 0, -FixedAxisVector(1)
                     -FixedAxisVector(2), FixedAxisVector(1), 0];
        RodriguesMatrix=eye(3)+FixedAxisMatrix*sin(Theta)+(FixedAxisMatrix^2)*(1-cos(Theta));
    end
    d=RodriguesMatrix*Vector;
end

%Convert initial pitch and yaw angle to a dart basis
function t = AngleToDartBasis(Angle)
    RotationMatrix=[cos(Angle(2)),0,sin(Angle(2));0,1,0;-sin(Angle(2)),0,cos(Angle(2))]*[1,0,0;0,cos(-Angle(1)),-sin(-Angle(1));0,sin(-Angle(1)),cos(-Angle(1))];
    DartBasis(:,1)=-RotationMatrix*[1;0;0];
    DartBasis(:,2)=RotationMatrix*[0;1;0];
    DartBasis(:,3)=RotationMatrix*[0;0;1];
    t=DartBasis;
end
