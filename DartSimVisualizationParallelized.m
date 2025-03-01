%All angles in radians, units in mkgs
clear vars
close all
clc
format long

figure

iterNum = 2;
HitTracker=zeros(1,iterNum);
PositionPlots=cell(1,iterNum);
HeadPositionPlots=cell(1,iterNum);
DartBasisXPlots=cell(1,iterNum);
DartBasisYPlots=cell(1,iterNum);
DartBasisZPlots=cell(1,iterNum);
RandomError=true;
rng('shuffle');
disp("Running...")
tic
parfor k = 1:iterNum
    %Starting conditions

    DisplayBasisVectors=true;
    TargetingBase=true;
    Pitch=deg2rad(45);
    if RandomError==true
        PitchAngleAlignmentError=normrnd(0,0.003);
        YawAngleAlignmentError=normrnd(0,0.003);
        YawPositionError=0.0003835*rand-0.0003835*rand;
        InitialVelocityError=normrnd(0,0.03);
        AirVelocity = [normrnd(0,0.1); 0; normrnd(0,0.1)];
    else
        PitchAngleAlignmentError=0;
        YawAngleAlignmentError=0;
        YawPositionError=0;
        InitialVelocityError=0;
        AirVelocity = [0; 0; 0];
    end
    
    if TargetingBase
        InitialAngle=[Pitch+PitchAngleAlignmentError;0.126689635808396+YawPositionError+YawAngleAlignmentError;0]; %pitch,yaw,roll
        DBVelocity=[0;0; 16.333260875976510+InitialVelocityError]; %Dart Basis velocity-> Z is along longitudinal axis of dart
    else 
        InitialAngle=[Pitch+PitchAngleAlignmentError;-0.10992110217290+YawPositionError+YawAngleAlignmentError;0]; %pitch,yaw,roll
        DBVelocity=[0;0;13.075933425262566+InitialVelocityError]; %Dart Basis velocity-> Z is along longitudinal axis of dart
    end
    
    Position=[4.225;0.94;0.5852483];
    Mass=.218;
    InertiaMatrix=[0.00106138,-0.00000049,0.0
                   -0.00000049,0.00106249,0.00000031
                   0.0,0.00000031,0.00008259]; %From SolidWorks
    
    %Initializes the simulation and runs it:
    CurrentSim = DartSim(InitialAngle, DBVelocity, Position, Mass, InertiaMatrix, AirVelocity = AirVelocity);
    CurrentSim.run();
    if CurrentSim.TargetHit 
        HitTracker(k)=1;
    end
    PositionPlots{k}=CurrentSim.PlotPosition;
    HeadPositionPlots{k}=CurrentSim.PlotHeadPosition;
    DartBasisXPlots{k}=CurrentSim.PlotDartBasisZ;
    disp(CurrentSim.PlotDartBasisZ);
    DartBasisYPlots{k}=CurrentSim.PlotDartBasisZ;
    DartBasisZPlots{k}=CurrentSim.PlotDartBasisZ;
end

for k=1:iterNum
    %Plot current simulation run:
    %plot3(PositionPlots{k}(:,1),PositionPlots{k}(:,3),PositionPlots{k}(:,2), 'blue')
    plot3(HeadPositionPlots{k}(:,1),HeadPositionPlots{k}(:,3),HeadPositionPlots{k}(:,2), 'blue')
    hold on
    for d=1:length(DartBasisXPlots)
        plot3(DartBasisXPlots{k,d,1},DartBasisXPlots{k,d,2},DartBasisXPlots{k,d,3},'yellow');
        plot3(DartBasisYPlots{k}{d}{1},DartBasisYPlots{k}{d}{2},DartBasisYPlots{k}{d}{3},'blue');
        plot3(DartBasisZPlots{k}{d}{1},DartBasisZPlots{k}{d}{2},DartBasisZPlots{k}{d}{3},'red');
    end
    %plot(CurrentSim.PlotTime,CurrentSim.PlotMisalignment(:,2))
end

toc
fprintf("Hit Percentage = %d \n",(nnz(HitTracker)/iterNum)*100);
%Plot Targets
X=[2.354053,2.354053,2.489053,2.489053];
Z=[16.6838448,16.7946805,16.7946805,16.6838448];
Y=[1.4989477,1.5566663,1.5566663,1.4989477];
fill3(X,Z,Y,'r')

X=[7.4325335,7.4325335,7.5675337,7.5675337];
Z=[26.0548107,26.1656942,26.1656942,26.0548107];
Y=[1.0484971,1.1062019,1.1062019,1.0484971];
fill3(X,Z,Y,'r')

hold off
xlabel('X Position(m)') 
ylabel('Z Position(m)') 
zlabel('Y Position(m)') 
axis equal
grid on
xlim([0,15])
ylim([0,28])
zlim([0,8])


%hold off
%xlabel('Time')
%ylabel('X misalignment')
%{
figure
plot(PlotTime,PlotMisalignment(:,2))
xlabel('Time')
ylabel('Y misalignment')
%}

