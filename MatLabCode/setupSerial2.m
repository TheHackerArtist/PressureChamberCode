function [ s, flag] = setupSerial(comPort)
%Initialize serial port communication between Arduino and Matlab
%Ensure that the arduino is also communicating with Matlab at this time. 
%if setup is complete then the value of setup is returned as 1 else 0.

flag =1;
s = serial(comPort);
set(s,'DataBits',8);
set(s,'StopBits',1);
set(s,'BaudRate',115200);
set(s,'Parity','none');
fopen(s);
counter = 0;

flushinput(s);
disp( 'I am alive...just waiting for some info!');
while (counter <3)
if ( s.BytesAvailable  ~= 0 && counter < 25)
    disp(fscanf(s,'%c',1))
    counter = counter + 1;
    disp (counter);
end
end
disp( 'Hurray I am free');
disp('must hit endstop!');
fprintf( s, '%c', 'a');
end

