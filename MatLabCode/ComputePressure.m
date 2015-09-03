function [ Pressure] = ComputePressure(CalibrationBytes,Pressure_Data_Reg,Temp_Data_Reg)
%This function computes the Pressure based on the calibration bytes and raw
%register data
%    
%   Pressure sensitivity | SENS
%   Pressure offset | OFF
%   Temperature coefficient of pressure sensitivity | TCS
%   Temperature coefficient of pressure offset | TCO
%   Reference temperature | Tref
%   Temperature coefficient of the temperature | TEMPSENS
%
%   Detailed explanation:
%   *First we compute the difference between actual and reference
%   temperature
%   dT = Temp_Data_Reg - Tref*(2^8) // The Tref is shifted by 8 bits for
%   substraction...I beleive... ( 24 bit - 16bit data) The result is a
%   signed long (Int32bit)
%   *Actual temperature computation
%   TEMP = 20°C + dT*TEMPSENS = 2000+dT*C6/(2^23) //Range -4000 to 85000
%   The result is a signed long
%   *Then we compute the pressure. 1) Offset at actual temperature
%   OFFSET = OFF + TCO*dT // C2*(2^16) + (C4*dT)/(2^7) Result is signed 64
%   bit integer
%   2) Sensitivity at actualy temperarture 
%   SENSITIVITY = SENS + TCS*dt // C1*2^15 + (C3*dT)/2^8 
%   Finally the pressure is computed
%   P = Pressure_Reg_Data*SENSITIVITY - OFFSET // (Pressure_Reg_Data*SENSITIVITY/(2^21) -
%   OFF)/(2^15)
SENS = CalibrationBytes(1);
OFF = CalibrationBytes(2);
TCS = CalibrationBytes(3);
TCO = CalibrationBytes(4);
Tref = CalibrationBytes(5);
TEMPSENS = CalibrationBytes(6);

dT = Temp_Data_Reg - Tref*(2^8);
Temp = 2000 + dT*TEMPSENS/(2^23);
OFFSET = OFF*(2^16) + (TCO*dT)/(2^7);
SENSITIVITY = SENS*(2^15) + (TCS*dT)/(2^8);
Pressure = (Pressure_Data_Reg*SENSITIVITY/(2^21) - OFFSET)/(2^15);

end

