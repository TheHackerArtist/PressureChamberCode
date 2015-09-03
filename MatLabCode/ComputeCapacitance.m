function [ Capacitance ] = ComputeCapacitance( CDC_Data_Reg, Input_Range )
%This Funciton Computes the Capacitance
%   C(pF) = (Data Register / 0xFFF0) x Input_Range
%   The output and input are in Int, Reg_Size is defined as 0XFFF0 in the main fcn
Reg_Size = hex2dec('FFF0');
Capacitance = (CDC_Data_Reg/Reg_Size)*Input_Range;

end

