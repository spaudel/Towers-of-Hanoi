function varargout = Hanoi_GUI(varargin)
% HANOI_GUI MATLAB code for Hanoi_GUI.fig
%      HANOI_GUI, by itself, creates a new HANOI_GUI or raises the existing
%      singleton*.
%
%      H = HANOI_GUI returns the handle to a new HANOI_GUI or the handle to
%      the existing singleton*.
%
%      HANOI_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in HANOI_GUI.M with the given input arguments.
%
%      HANOI_GUI('Property','Value',...) creates a new HANOI_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Hanoi_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Hanoi_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Hanoi_GUI

% Last Modified by GUIDE v2.5 05-May-2014 16:45:09

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Hanoi_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @Hanoi_GUI_OutputFcn, ...
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
% End initialization code - DO NOT EDIT


% --- Executes just before Hanoi_GUI is made visible.
function Hanoi_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Hanoi_GUI (see VARARGIN)

% Choose default command line output for Hanoi_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Hanoi_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Hanoi_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function b2_box_Callback(hObject, eventdata, handles)
% hObject    handle to b2_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of b2_box as text
%        str2double(get(hObject,'String')) returns contents of b2_box as a double
box_string = get(hObject, 'String');
set(handles.b2_slider, 'Value', str2double(box_string));


% --- Executes during object creation, after setting all properties.
function b2_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to b2_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function b1_box_Callback(hObject, eventdata, handles)
% hObject    handle to b1_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of b1_box as text
%        str2double(get(hObject,'String')) returns contents of b1_box as a double
box_string = get(hObject, 'String');
set(handles.b1_slider, 'Value', str2double(box_string));


% --- Executes during object creation, after setting all properties.
function b1_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to b1_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tf_box_Callback(hObject, eventdata, handles)
% hObject    handle to tf_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tf_box as text
%        str2double(get(hObject,'String')) returns contents of tf_box as a double
box_string = get(hObject, 'String');
set(handles.tf_slider, 'Value', str2double(box_string));


% --- Executes during object creation, after setting all properties.
function tf_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tf_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function mL_box_Callback(hObject, eventdata, handles)
% hObject    handle to mL_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of mL_box as text
%        str2double(get(hObject,'String')) returns contents of mL_box as a double
box_string = get(hObject, 'String');
set(handles.mL_slider, 'Value', str2double(box_string));


% --- Executes during object creation, after setting all properties.
function mL_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mL_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function b1_slider_Callback(hObject, eventdata, handles)
% hObject    handle to b1_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
slider_string = get(hObject, 'Value');
set(handles.b1_box, 'String', slider_string);

% --- Executes during object creation, after setting all properties.
function b1_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to b1_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function b2_slider_Callback(hObject, eventdata, handles)
% hObject    handle to b2_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
slider_string = get(hObject, 'Value');
set(handles.b2_box, 'String', slider_string);

% --- Executes during object creation, after setting all properties.
function b2_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to b2_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function tf_slider_Callback(hObject, eventdata, handles)
% hObject    handle to tf_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
slider_string = get(hObject, 'Value');
set(handles.tf_box, 'String', slider_string);

% --- Executes during object creation, after setting all properties.
function tf_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tf_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in Run.
function Run_Callback(hObject, eventdata, handles)
% hObject    handle to Run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global m mL mLuser a Iz1 Iz2 Iz l b1 b2 b g; %system parameters
global tf ts omega_n zeta ; % model parameters
global Kp Kd;

m = 0.25; %link mass
a = 0.2; %link length
l = a/2;
g= 9.81; %gravity

%Get values from the GUI
mL = str2double(get(handles.mL_box, 'String')); %end effector mass
mLuser = mL;
b1 = str2double(get(handles.b1_box, 'String')); %joint/actuator friction
b2 = str2double(get(handles.b2_box, 'String'));
Iz1 = str2double(get(handles.Iz1_box, 'String'));
Iz2 = str2double(get(handles.Iz2_box, 'String'));

Iz = Iz1;
b = b1;

tf = str2double(get(handles.tf_box, 'String')); %move time
ts = str2double(get(handles.ts_box, 'String')); %settling time

%Calculate omega, zeta and gain values
zeta = 1; % No overshoot
omega_n = 4.6/(ts * zeta);

Kp = omega_n^2.*eye(2);
Kd = 2*zeta*omega_n.*eye(2);
K = [Kp Kd];

%Sliding control calculations
F = [zeros(2) eye(2); -Kp -Kd];
G = [zeros(2); eye(2)];
G_T = G';
P = lyap(F,eye(4));
epsilon = 0.1; %play with epsilon value
U = 0; 

%Parameter control calculations
pihat0 = [Iz; m; mL; b];

%Assign values to base
assignin('base','K', K);    % Feedback gain
assignin('base','m', m); 
assignin('base','a', a);
assignin('base','Iz', Iz);
assignin('base','Iz1', Iz1);
assignin('base','Iz2', Iz2);
assignin('base','l', l); 
assignin('base','g', g); 
assignin('base','mL', mL);
assignin('base','b1', b1);
assignin('base','b2', b2); 
assignin('base','tf', tf); 
assignin('base','ts', ts); 
assignin('base','epsilon',epsilon);
assignin('base','U',U);
assignin('base','P',P);
assignin('base','G_T',G_T);
assignin('base', 'pihat0', pihat0);

tspan = 6 * tf;
selected = get(get(handles.loopControl, 'SelectedObject'),'Tag');
if (strcmp(selected, 'slidingControl') == 1)
    [tout,~,yout]=sim('sliding_control', tspan);
end

if (strcmp(selected, 'parameterAdaptive') == 1)
    [tout,~,yout]=sim('ParameterAdaptive_control', tspan);
end


%Calculate aida
% theta2 = yout(:,4);
% qv = [yout(:,9) yout(:,10)];
% m11 = m*(a^2 + 2*l^2 + 2*a*l*cos(theta2)) + 2* Iz + 2*mL*a^2*(1+cos(theta2));
% m12 = m*l*(l+a*cos(theta2)) + Iz + mL*a^2*(1+cos(theta2));
% m21 = m*l*(l+a*cos(theta2))+Iz+mL*a^2*(1+cos(theta2));
% m22 = m*l^2+Iz+mL*a^2;
% M_theta = [m11 m12; m21 m22];

assignin('base', 'tout', tout);

%plot Trajectory
axes(handles.trajPlot)
plot(yout(:,3), yout(:,4));
title('Trajectory of end effector in operational space');
xlabel('X position [m]');
ylabel('Y position [m]');
hold on
plot(yout(1,3), yout(1,4), '*r');
legend('trajectory','starting position','location','south')
hold off

%plot Torque
axes(handles.torquePlot)
plot(tout, yout(:,7));
title('Torque vs Time');
xlabel('Time (s)');
ylabel('Torque [Nm]');
hold on
plot(tout, yout(:,8),'r');
legend('Link 1', 'Link 2','location','south');
hold off

%plot Theta1
axes(handles.thetaXPlot)
plot(tout, yout(:,5),'r');
hold on
plot(tout, yout(:,11));
%title('Theta 1 vs time');
xlabel('Time (s)');
ylabel('Theta1 [rad]');
legend('out', 'ref','location','northoutside');
hold off

%plot Theta2
axes(handles.thetaYPlot);
plot(tout, yout(:,6),'r');
hold on
plot(tout, yout(:,12));
%title('Theta2 vs time');
xlabel('Time (s)');
ylabel('Theta2 [rad]');

hold off

%plot PosX
axes(handles.PosTimeXPlot)
plot(tout, yout(:,3),'r');
hold on
plot(tout, yout(:,13));
%title('X position vs time');
xlabel('Time (s)');
ylabel('X position [m]');
hold off

%plot PosY
axes(handles.PosTimeYPlot)
plot(tout, yout(:,4), 'r');
hold on
plot(tout, yout(:,14));
%title('Y position vs time');
xlabel('Time (s)');
ylabel('Y position [m]');
hold off

%Calculate performance measure
completionTime = 5* tf;
set(handles.tCompletion, 'String', completionTime);

diffPosX = yout(:,13)-yout(:,3);
iseX = sum(diffPosX.^2);
set(handles.ISEPosX, 'String', iseX);

diffPosY = yout(:,14)-yout(:,4);
iseY = sum(diffPosY.^2);
set(handles.ISEPosY, 'String', iseY);

istorque1 = sum(yout(:,7).^2);
set(handles.ISTorque1, 'String', istorque1);

istorque2= sum(yout(:,8).^2);
set(handles.ISTorque2, 'String', istorque2);

%move 2 deviation calc
x = yout(:,3);
move2start = 100*tf+1;
move2stop = 200*tf - 1;
move2Deviation = abs(0.1 - x(move2start:move2stop));
move4start = 300*tf + 1;
move4stop = 400 * tf - 1;
move4Deviation = abs(0.3 - x(move4start:move4stop));
maxdeviation2 = max(move2Deviation);
maxdeviation4 = max(move4Deviation);
set(handles.maxDev2, 'String', maxdeviation2);
set(handles.maxDev4, 'String', maxdeviation4);

function ts_box_Callback(hObject, eventdata, handles)
% hObject    handle to ts_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ts_box as text
%        str2double(get(hObject,'String')) returns contents of ts_box as a double
box_string = get(hObject, 'String');
set(handles.ts_slider, 'Value', str2double(box_string));


% --- Executes during object creation, after setting all properties.
function ts_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ts_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function ts_slider_Callback(hObject, eventdata, handles)
% hObject    handle to ts_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
slider_string = get(hObject, 'Value');
set(handles.ts_box, 'String', slider_string);


% --- Executes during object creation, after setting all properties.
function ts_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ts_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function mL_slider_Callback(hObject, eventdata, handles)
% hObject    handle to mL_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
slider_string = get(hObject, 'Value');
set(handles.mL_box, 'String', slider_string);

% --- Executes during object creation, after setting all properties.
function mL_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mL_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function Iz1_box_Callback(hObject, eventdata, handles)
% hObject    handle to Iz1_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Iz1_box as text
%        str2double(get(hObject,'String')) returns contents of Iz1_box as a double
box_string = get(hObject, 'String');
set(handles.Iz1_slider, 'Value', str2double(box_string));


% --- Executes during object creation, after setting all properties.
function Iz1_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Iz1_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Iz2_box_Callback(hObject, eventdata, handles)
% hObject    handle to Iz2_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Iz2_box as text
%        str2double(get(hObject,'String')) returns contents of Iz2_box as a double
box_string = get(hObject, 'String');
set(handles.Iz2_slider, 'Value', str2double(box_string));



% --- Executes during object creation, after setting all properties.
function Iz2_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Iz2_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function Iz1_slider_Callback(hObject, eventdata, handles)
% hObject    handle to Iz1_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
slider_string = get(hObject, 'Value');
set(handles.Iz1_box, 'String', slider_string);

% --- Executes during object creation, after setting all properties.
function Iz1_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Iz1_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function Iz2_slider_Callback(hObject, eventdata, handles)
% hObject    handle to Iz2_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
slider_string = get(hObject, 'Value');
set(handles.Iz2_box, 'String', slider_string);

% --- Executes during object creation, after setting all properties.
function Iz2_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Iz2_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function ISTorque1_Callback(hObject, eventdata, handles)
% hObject    handle to ISTorque1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ISTorque1 as text
%        str2double(get(hObject,'String')) returns contents of ISTorque1 as a double


% --- Executes during object creation, after setting all properties.
function ISTorque1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ISTorque1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ISEPosX_Callback(hObject, eventdata, handles)
% hObject    handle to ISEPosX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ISEPosX as text
%        str2double(get(hObject,'String')) returns contents of ISEPosX as a double


% --- Executes during object creation, after setting all properties.
function ISEPosX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ISEPosX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function maxDev2_Callback(hObject, eventdata, handles)
% hObject    handle to maxDev2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of maxDev2 as text
%        str2double(get(hObject,'String')) returns contents of maxDev2 as a double


% --- Executes during object creation, after setting all properties.
function maxDev2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to maxDev2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tCompletion_Callback(hObject, eventdata, handles)
% hObject    handle to tCompletion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tCompletion as text
%        str2double(get(hObject,'String')) returns contents of tCompletion as a double


% --- Executes during object creation, after setting all properties.
function tCompletion_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tCompletion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ISTorque2_Callback(hObject, eventdata, handles)
% hObject    handle to ISTorque2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ISTorque2 as text
%        str2double(get(hObject,'String')) returns contents of ISTorque2 as a double


% --- Executes during object creation, after setting all properties.
function ISTorque2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ISTorque2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ISEPosY_Callback(hObject, eventdata, handles)
% hObject    handle to ISEPosY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ISEPosY as text
%        str2double(get(hObject,'String')) returns contents of ISEPosY as a double


% --- Executes during object creation, after setting all properties.
function ISEPosY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ISEPosY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function maxDev4_Callback(hObject, eventdata, handles)
% hObject    handle to maxDev4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of maxDev4 as text
%        str2double(get(hObject,'String')) returns contents of maxDev4 as a double


% --- Executes during object creation, after setting all properties.
function maxDev4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to maxDev4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
