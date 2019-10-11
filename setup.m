% Setup for the juggling robot. Should be run first
%   *
% *  *
% \[T]/     By Dennis Lioubartsev
% Contains all motor parameters to set up statespace representations of
% shoulder, upper arm and elbow motors
% Runs the simulation
% Plots the graphs
%% Reset workspace
clc;
clear all;
close all;

%% Choose controller

%% Motor 1 datasheet
% km ? 0.65ke
L1 = 3.5*10^(-3); %motor winding, inductance [H]
R1 = 1.9; %[Ohm]
Cv1 = 1.3*10^(-6)*60/(2*pi); %[s*Nm/rad] 
J1 = 1.3*10^(-4); %[kg m2]
Ke1 = 0.15; %[sV/rad]
Km1 = 0.15; %[Nm/A]
Tl1 = 0; %[Nm] Load torque
Un1 = 24; %[V], nominal voltage
N1 = 36; %[] Gear Ratio

%% Motor 2 datasheet
L2 = 5.2*10^(-3); %motor winding, inductance [H]
R2 = 2.45; %[Ohm]
Cv2 = 1.3*10^(-6)*60/(2*pi); %[s*Nm/rad] 
J2 = 0.79*10^(-4); %[kg m2]
Ke2 = 0.12; %[sV/rad]
Km2 = 0.12; %[Nm/A]
Tl2 = 0; %[Nm] Load torque
Un2 = 24; %[V], nominal voltage
N2 = 36; %[] Gear Ratio

%% Motor 3 datasheet
L3 = 3.5*10^(-3); %motor winding, inductance [H]
R3 = 1.9; %[Ohm]
Cv3 = 1.3*10^(-6)*60/(2*pi); %[s*Nm/rad] 
J3 = 0.98*10^(-4); %[kg m2]
Ke3 = 0.12; %[sV/rad]
Km3 = 0.12; %[Nm/A]
Tl3 = 0; %[Nm] Load torque
Un3 = 24; %[V], nominal voltage
N3 = 36; %[] Gear Ratio

%% Other variables
effWorm = 0.8; %http://gearsolutions.com/features/investigations-on-the-efficiency-of-worm-gear-drives/
effPlanet = 0.98;

%% Linearize model 
linSys = linearize('armModelIO_Redux'); 

[a,b,c,d] = ssdata(linSys); 
[A,B,C,D] = ssdata(linSys); 



%% Run the model
%start the collision library
addpath("SM_Contact_Forces_Lib_R18b_v4p1");
startup_Contact_Forces;
%start simulation
sim('armModel.slx'); %run simulink of LQR
%sim('armModelLQG.slx'); %run simscape of LQG

%% Plots etc
if isempty(ser_catchFlag) || isempty(ser_q_ref) || isempty(ser_ee_ref) || isempty(ser_q) % Make sure that the files are generated.
    disp("Some timeseries from the simulation were not generated...");
    return
end

%plot the EE trajectory
figure(1)
plot(ser_ee.time, ser_ee.data);
title("EE path")
xlabel('time [s]')
ylabel('position [m]')
legend('X','Y', "Z");
hold on;

flagTimeStamps = []; %generate a vector where all catches start.
for ii = 1:length(ser_catchFlag.data)
    if ser_catchFlag.data(ii) == 1
        flagTimeStamps = [flagTimeStamps ser_catchFlag.time(ii)];
    end
end
%Find where all catches end to get the execution time.

%Define tolerance
noiseTol = 0; %tolerance coming from noise.
tolerance = 0.04;
catchTol = tolerance + noiseTol; %total tolerance conisdering the noise

err1 = abs(ser_ee.data(:,1) - ser_ee_ref.data(:,1)); %Actual error of the three joints
err2 = abs(ser_ee.data(:,2) - ser_ee_ref.data(:,2));


errTot = sqrt(err1.^2+err2.^2);
%Iteration starts

figure(2);
for jj = 1:length(flagTimeStamps)-1
    %idxVec = (ser_q.time == flagTimeStamps(jj)); %Vector containing 1 on the timestamp of flagTimeStamps(jj)
    idxVec = ((ser_q.time >= flagTimeStamps(jj)) & (ser_q.time <= (flagTimeStamps(jj) + 0.00009)));
    idx = find(idxVec, 1); %target index where catch happens

    condVec = errTot < catchTol; % Vector cointaining 1s where tolerances are met
    endPoint = find(condVec((idx+20):end), 1) + (idx+20); %Add back the trimmed part of the vector.
    
    %DEBUG
    %disp("Iteration " + jj + " idx = " + idx + " endPoint = " + endPoint);
    %DEBUG
    
    disp("Control cycle time (ticks): " + (endPoint - idx));
    
    %figure(jj+1)
    subplot(3, 2, jj)
    plot(ser_ee.time(idx:endPoint), errTot(idx:endPoint));
    title("Catch " + jj)
    xlabel('time [s]')
    ylabel('error [m]')
    %legend('T1 error');
    hold on;
    
end