function varargout = UserInterface(varargin)
% USERINTERFACE MATLAB code for UserInterface.fig
%      USERINTERFACE, by itself, creates a new USERINTERFACE or raises the existing
%      singleton*.
%
%      H = USERINTERFACE returns the handle to a new USERINTERFACE or the handle to
%      the existing singleton*.
%
%      USERINTERFACE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in USERINTERFACE.M with the given input arguments.
%
%      USERINTERFACE('Property','Value',...) creates a new USERINTERFACE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before UserInterface_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to UserInterface_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help UserInterface

% Last Modified by GUIDE v2.5 03-Sep-2015 16:08:08

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @UserInterface_OpeningFcn, ...
                   'gui_OutputFcn',  @UserInterface_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
end
% End initialization code - DO NOT EDIT


% --- Executes just before UserInterface is made visible.
function UserInterface_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to UserInterface (see VARARGIN)

% Choose default command line output for UserInterface
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

handles.sheet = 1;
guidata(hObject, handles);

handles.Input_Range = 2;

guidata(hObject, handles);
% UIWAIT makes UserInterface wait for user response (see UIRESUME)
% uiwait(handles.figure1);
end

% --- Outputs from this function are returned to the command line.
function varargout = UserInterface_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
end


function edit9_Callback(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit9 as text
%        str2double(get(hObject,'String')) returns contents of edit9 as a double
end

% --- Executes during object creation, after setting all properties.
function edit9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function DynUserFrequencyTxt_Callback(hObject, eventdata, handles)
% hObject    handle to DynUserFrequencyTxt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of DynUserFrequencyTxt as text
%        str2double(get(hObject,'String')) returns contents of DynUserFrequencyTxt as a double
end

% --- Executes during object creation, after setting all properties.
function DynUserFrequencyTxt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to DynUserFrequencyTxt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end



function DynUserToTxt_Callback(hObject, eventdata, handles)
% hObject    handle to DynUserToTxt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of DynUserToTxt as text
%        str2double(get(hObject,'String')) returns contents of DynUserToTxt as a double
end

% --- Executes during object creation, after setting all properties.
function DynUserToTxt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to DynUserToTxt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function DynStartSt_Callback(hObject, eventdata, handles)
% hObject    handle to DynStartSt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of DynStartSt as text
%        str2double(get(hObject,'String')) returns contents of DynStartSt as a double
end

% --- Executes during object creation, after setting all properties.
function DynStartSt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to DynStartSt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

% --- Executes on button press in DynamicCalibrationBtn.
function DynamicCalibrationBtn_Callback(hObject, eventdata, handles)
% hObject    handle to DynamicCalibrationBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end

% --- Executes on button press in StaticCalibrationBtn.
function StaticCalibrationBtn_Callback(hObject, eventdata, handles)
% hObject    handle to StaticCalibrationBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
disp('calibration button hit');
Start = str2double(get(handles.CalFromUserTxt, 'string')); % Fetches the user inputed start location in mm and converts to double
disp(Start);
End = str2double(get(handles.CalToUserTxt, 'string')); % same for End position
disp(End);
Increment = get(handles.CalUserIncrementTxt, 'string'); % fetches the increment user inputed data as a string
disp(Increment);
Wait = get(handles.CalUserSpeedTxt, 'string'); % fetches the wait inputed data as a string
disp(Wait);
StartSteps = round(Start/0.00793750000); % computes the starting step position,double division
disp(StartSteps);
handles.StartSteps = StartSteps; % creats a place for the start steps inside the handles structure, to be fetched by anythingelsest be saved with guidata(hObject,handles)
EndSteps = round(End/0.00793750000); % computes the end step position
disp(EndSteps);
handles.EndSteps = EndSteps; % stores the end steps to be accessed by anything else must be saved with guidata(hObject,handles)
StartStepsStr = num2str(StartSteps); % converts the StartSteps double into a string so it can be sent over serial as a string
disp(StartStepsStr);
EndStepsStr = num2str(EndSteps); % converts the EndSteps double into a string so it can be sent over serial as a string
disp(EndStepsStr);
% space = ' ';
% OutputString = strcat('C0' , space , StartStepsStr , space , EndStepsStr , space , Increment , space , Wait);
% disp(OutputString);
flushinput(handles.s);
disp('Buffer flushed');
OutputString = sprintf('C0 %s %s %s %s',StartStepsStr,EndStepsStr,Increment,Wait);
disp(OutputString);
fprintf(handles.s,'%s', OutputString);

% Looping the serial read commandand storing the info in an array that will
% later be sent to an excel file. The data in the array will be ploted
% every iteration
for j = StartSteps+1 : str2double(Increment) : EndSteps %Issue here is that the user might input something that gives a decimal position...need to be an integer...must truncate)
disp(handles.s.BytesAvailable);    
while (handles.s.BytesAvailable == 0) 
    disp('waiting for command from arduino');
    pause(0.5);
end
input = fscanf(handles.s, '%c', 1); % scans for one character
handles.ArrayPosition(j) = j; % This will be used to plot
guidata(hObject,handles);

if input == 'P'
        while (handles.s.BytesAvailable <= 4)
        end
        rawPressure = fread(handles.s,1,'uint32');% reads four bytes and stores it as an integer
        disp('P');
        disp(dec2hex(rawPressure));
        handles.rawPressure = rawPressure;
        guidata(hObject,handles);
else
    disp('Error communicating with arduino');% in case somthing goes wrong...
end
while (handles.s.BytesAvailable == 0) 
    disp('waiting for command from arduino');
    pause(0.25);
end
input = fscanf(handles.s, '%c', 1);% scans for one character
if input == 'T'
    while (handles.s.BytesAvailable <= 4)
        end    
    rawTemperature = fread(handles.s,1,'uint32');% reads four bytes and stores it as an integer
    disp('T');    
    disp(dec2hex(rawTemperature));
        handles.rawTemperature = rawTemperature;
        guidata(hObject,handles);
else
    disp('Error communicating with arduino');
end
handles.PressureArray(j) = ComputePressure(handles.coefficients,handles.rawPressure,handles.rawTemperature); %handles.coefficients is an array that stores the 6 calibration data bytes
guidata(hObject,handles);
while (handles.s.BytesAvailable == 0) 
    disp('waiting for command from arduino');
    pause(0.2);
end
input = fscanf(handles.s, '%c', 1);
if input == 'C'
        
    while (handles.s.BytesAvailable <= 4)
        disp('hELPPP iM stuck here!')
        pause(0.2)
    end 
    rawCapacitance = fread(handles.s,1,'uint32');% reads four bytes and stores it as an integer
    disp('C');
    disp(dec2hex(rawCapacitance));
    handles.rawCapacitance = rawCapacitance;
    guidata(hObject,handles);
else
    disp('Error communicating with arduino');
end
handles.CapacitanceArray(j) = ComputeCapacitance( handles.rawCapacitance, handles.Input_Range );
guidata(hObject,handles);
disp('Pressure :');
disp(handles.PressureArray(j));
disp('Capacitance:');
disp(handles.CapacitanceArray(j));
%axes(handles.axes_pressure); 
plot(handles.axes_pressure,handles.ArrayPosition,handles.PressureArray);
%axes(handles.axes_capacitance); 
plot(handles.axes_capacitance,handles.ArrayPosition,handles.CapacitanceArray);
set(handles.CurrentPositionTxt, 'String', j); 
end
xlswrite('calibrationData.xlxs',handles.ArrayPosition,handles.sheet,'A1');

xlswrite('calibrationData.xlxs',handles.PressureArray,handles.sheet,'B1');

xlswrite('calibrationData.xlxs',handles.CapacitanceArray,handles.sheet,'C1');

end
% Further imporvements would be to ask the user how he would like to name
% the file he is writing to...
% also, updating the position in the position static text would help know
% where the progession is
% Could I have to check that the serial buffer has something in it before
% going to the next command



function CalFromUserTxt_Callback(hObject, eventdata, handles)
% hObject    handle to CalFromUserTxt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of CalFromUserTxt as text
%        str2double(get(hObject,'String')) returns contents of CalFromUserTxt as a double
end

% --- Executes during object creation, after setting all properties.
function CalFromUserTxt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to CalFromUserTxt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end



function CalToUserTxt_Callback(hObject, eventdata, handles)
% hObject    handle to CalToUserTxt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of CalToUserTxt as text
%        str2double(get(hObject,'String')) returns contents of CalToUserTxt as a double
end

% --- Executes during object creation, after setting all properties.
function CalToUserTxt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to CalToUserTxt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end



function CalUserIncrementTxt_Callback(hObject, eventdata, handles)
% hObject    handle to CalUserIncrementTxt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of CalUserIncrementTxt as text
%        str2double(get(hObject,'String')) returns contents of CalUserIncrementTxt as a double
end

% --- Executes during object creation, after setting all properties.
function CalUserIncrementTxt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to CalUserIncrementTxt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function CalUserSpeedTxt_Callback(hObject, eventdata, handles)
% hObject    handle to CalUserSpeedTxt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of CalUserSpeedTxt as text
%        str2double(get(hObject,'String')) returns contents of CalUserSpeedTxt as a double
end

% --- Executes during object creation, after setting all properties.
function CalUserSpeedTxt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to CalUserSpeedTxt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


% --- Executes on button press in HomeButton.
function HomeButton_Callback(hObject, eventdata, handles)
% hObject    handle to HomeButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
disp('home button pressed');
fprintf(handles.s,'%s', 'G2');
set(handles.CurrentPositionTxt, 'String', '0');
disp('G2 Command sent') % This is for monitoring purposes
end

% --- Executes on button press in SlowMoveBtn.
function SlowMoveBtn_Callback(hObject, eventdata, handles)
% hObject    handle to SlowMoveBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end

% --- Executes on button press in FastMoveBtn.
function FastMoveBtn_Callback(hObject, eventdata, handles)
% hObject    handle to FastMoveBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end


function DesiredPositionTxt_Callback(hObject, eventdata, handles)
% hObject    handle to DesiredPositionTxt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of DesiredPositionTxt as text
%        str2double(get(hObject,'String')) returns contents of DesiredPositionTxt as a double
end

% --- Executes during object creation, after setting all properties.
function DesiredPositionTxt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to DesiredPositionTxt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

% --- Executes on button press in SerialBtn.
function SerialBtn_Callback(hObject, eventdata, handles)
% hObject    handle to SerialBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA
comPort = get(handles.COMportTxt,'String');
if(~exist('serialFlag','var'))
    [handles.s, handles.serialFlag] = setupSerial2(comPort); % was modified
end
guidata(hObject,handles);
flushinput(handles.s);
end



function COMportTxt_Callback(hObject, eventdata, handles)
% hObject    handle to COMportTxt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of COMportTxt as text
%        str2double(get(hObject,'String')) returns contents of COMportTxt as a double
end

% --- Executes during object creation, after setting all properties.
function COMportTxt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to COMportTxt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
closeSerial(hObject);
delete(hObject);
end

% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end

% --- Executes on button press in PressureDataBtn.
function PressureDataBtn_Callback(hObject, eventdata, handles)
% hObject    handle to PressureDataBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   C1 | Pressure sensitivity | SENS
%   C2 | Pressure offset | OFF
%   C3 | Temperature coefficient of pressure sensitivity | TCS
%   C4 | Temperature coefficient of pressure offset | TCO
%   C5 | Reference temperature | Tref
%   C6 | Temperature coefficient of the temperature | TEMPSENS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
flushinput(handles.s);
disp('Buffer was flushed');
fprintf(handles.s,'%s','M0');
disp('M0 was sent');

for j = 1:6
    while (handles.s.BytesAvailable == 0)
    end
    disp('pausing');
    pause(0.1);
    disp('stopped pausing');
    CurrentInput = fscanf(handles.s,'%c', 1 );
if CurrentInput == ' ' 
    disp('pausing');
    pause(0.1);
    disp('stopped pausing');
    databit = fscanf(handles.s,'%i',5);% reads four bytes and stores it as an integer
    disp(databit);
    disp(handles.s.BytesAvailable);
else
    disp('Error communicating with arduino');

end
handles.coefficients(j) = databit; % stores the calibration bytes inside the handles object
guidata(hObject,handles);
disp(handles.coefficients(j));
end
end


% --- Executes on button press in CloseSerialBtn.
function CloseSerialBtn_Callback(hObject, eventdata, handles)
% hObject    handle to CloseSerialBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
closeSerial(hObject);
end



function edit11_Callback(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit11 as text
%        str2double(get(hObject,'String')) returns contents of edit11 as a double
end

% --- Executes during object creation, after setting all properties.
function edit11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit12_Callback(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit12 as text
%        str2double(get(hObject,'String')) returns contents of edit12 as a double
end

% --- Executes during object creation, after setting all properties.
function edit12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function edit13_Callback(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit13 as text
%        str2double(get(hObject,'String')) returns contents of edit13 as a double
end
end
% --- Executes during object creation, after setting all properties.
function edit13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function edit14_Callback(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit14 as text
%        str2double(get(hObject,'String')) returns contents of edit14 as a double
end

% --- Executes during object creation, after setting all properties.
function edit14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function edit15_Callback(hObject, eventdata, handles)
% hObject    handle to edit15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit15 as text
%        str2double(get(hObject,'String')) returns contents of edit15 as a double
end

% --- Executes during object creation, after setting all properties.
function edit15_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function edit16_Callback(hObject, eventdata, handles)
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit16 as text
%        str2double(get(hObject,'String')) returns contents of edit16 as a double
end

% --- Executes during object creation, after setting all properties.
function edit16_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end