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
input='b';

while (s.BytesAvailable == 0) 
    disp('waiting for command from arduino')
end
while (input~='a')
    input = fscanf(s, '%c', 1); % scans for one character
end
if (input=='a')
    disp('serial read');
end
fprintf(s,'%c','a');
disp( 'a was sent');
mbox = msgbox('Serial Communication setup.'); uiwait(mbox);
%fscanf(s,'%u');
while (s.BytesAvailable == 0) 
    disp('waiting for command from arduino')
end
while (input ~= 'b');
    input = fscanf(s, '%c', 1); % scans for one character
      
end
disp('b was read! WOOOOT');
while (s.BytesAvailable == 0) 
    disp('waiting for command from arduino')
end
while (input ~= 'c');
   
input = fscanf(s, '%c', 1); % scans for one character
end
disp('OHHHH my GAWD it is a c');
end

