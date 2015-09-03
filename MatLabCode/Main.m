function pressuresensor()

clc;
close all;
clear all;

cal_file_name = [];
applied_press =[];
p = [];
r =0;


%% Specify the COM port that the RFduino is connected to
comPort='com5';

%% initialize serial communication
if(~exist('serialFlag','var'))
    [capacitance.s, serialFlag]=setupSerial(comPort);
end

UserInterface

