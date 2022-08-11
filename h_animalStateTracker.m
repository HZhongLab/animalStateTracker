function varargout = h_animalStateTracker(varargin)
% H_ANIMALSTATETRACKER MATLAB code for h_animalStateTracker.fig
%      H_ANIMALSTATETRACKER, by itself, creates a new H_ANIMALSTATETRACKER or raises the existing
%      singleton*.
%
%      H = H_ANIMALSTATETRACKER returns the handle to a new H_ANIMALSTATETRACKER or the handle to
%      the existing singleton*.
%
%      H_ANIMALSTATETRACKER('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in H_ANIMALSTATETRACKER.M with the given input arguments.
%
%      H_ANIMALSTATETRACKER('Property','Value',...) creates a new H_ANIMALSTATETRACKER or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before h_animalStateTracker_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to h_animalStateTracker_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help h_animalStateTracker

% Last Modified by GUIDE v2.5 19-Oct-2018 20:44:59

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @h_animalStateTracker_OpeningFcn, ...
                   'gui_OutputFcn',  @h_animalStateTracker_OutputFcn, ...
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


% --- Executes just before h_animalStateTracker is made visible.
function h_animalStateTracker_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to h_animalStateTracker (see VARARGIN)

% Choose default command line output for h_animalStateTracker
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes h_animalStateTracker wait for user response (see UIRESUME)
% uiwait(handles.h_animalStateTracker);

h_AST_setup(hObject);


% --- Outputs from this function are returned to the command line.
function varargout = h_animalStateTracker_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in startTracking.
function startTracking_Callback(hObject, eventdata, handles)
% hObject    handle to startTracking (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global h_AST

if strcmpi(get(hObject, 'string'), 'start')
    set(hObject, 'String', 'Stop')
    h_AST.listenerObj = addlistener(h_AST.init.sessionObj, 'DataAvailable', @h_AST_listenerFcn);
    if ~isfield(h_AST.running, 'currentIdx') % this allow repeately start and stop
        h_AST.running.time = zeros(1, 50000);
        h_AST.running.relativeTimeInMin = zeros(1, 50000);        
        h_AST.running.speed = zeros(1, 50000);
        h_AST.running.startVol = zeros(1, 50000);
        h_AST.running.endVol = zeros(1, 50000);
        h_AST.running.duration = zeros(1, 50000);
        h_AST.running.currentIdx = 1;
%         h_AST.running.initialTime = []; %this is day number.
        pathName = get(handles.pathName, 'string');
        if ~exist(pathName, 'dir')
            pathName = pwd;
            set(handles.pathName, 'string', pwd);
        end
        fileName = get(handles.fileName, 'string');
        [pname, fname, fExt] = fileparts(fileName);
        if isempty(fExt)
            fExt = '.dat2';
        end
        j = 1;
        fullFileName = fullfile(pathName, [fname, '_running_', num2str(j),fExt]);
        while exist(fullFileName, 'file')
            j = j + 1;
            fullFileName = fullfile(pathName, [fname, '_running_', num2str(j),fExt]);
        end
        h_AST.running.fileName = fullFileName;
    end
    h_AST.running.FID = fopen(h_AST.running.fileName, 'a');
    h_AST.init.sessionObj.startBackground();
else
    h_AST.init.sessionObj.stop();
    fclose(h_AST.running.FID);
    delete(h_AST.listenerObj);
    set(hObject, 'String', 'Start')
end


% --- Executes on button press in enableRunningTracking.
function enableRunningTracking_Callback(hObject, eventdata, handles)
% hObject    handle to enableRunningTracking (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of enableRunningTracking

global h_AST

if get(hObject, 'value')
    [h_AST.running.chObj, h_AST.running.chIdxInSession] = addAnalogInputChannel(h_AST.init.sessionObj,h_AST.init.device,h_AST.init.runningChID,'Voltage');
    h_AST.running.chObj.Range = h_AST.init.runningInputRange;

%     h_AST.running.chObj.DurationInSeconds = 5;% this is a session property
%     h_AST.running.chObj.IsContinuous = true; % this is a session property
%     h_AST.init.sessionObj.DurationInSeconds = 5;
    h_AST.init.sessionObj.IsContinuous = true;
%     h_AST.init.sessionObj.NotifyWhenDataAvailableExceeds = xx % try setting this in the setup.
elseif ~isempty(h_AST.running)
    removeChannel(h_AST.init.sessionObj, h_AST.running.chIdxInSession);
    h_AST.running = [];
end


% --- Executes on button press in checkbox2.
function checkbox2_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox2


% --- Executes on button press in checkbox3.
function checkbox3_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox3


% --- Executes on button press in checkbox4.
function checkbox4_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox4



function h_AST_setup(hObject)

global h_AST

h_AST.currentHandles = guihandles(hObject);
h_AST.init.inputRate = 2000;%2kHz Note: only one input rate for all channels. There is only one input clock.
h_AST.init.recordRate = 20; % 20 points per second. this better to be able to be full divided by the inputrate.
h_AST.init.device = 'Dev4'; % currently assuming all on one device.
h_AST.init.sessionObj = daq.createSession('ni'); % one session for all channels.
h_AST.init.sessionObj.Rate = h_AST.init.inputRate;
h_AST.init.sessionObj.NotifyWhenDataAvailableExceeds = round(h_AST.init.inputRate/h_AST.init.recordRate);
h_AST.init.runningChID = 7;
h_AST.init.runningInputRange = [-10 10];
h_AST.running = [];
if exist('h_ASTSettings.mat', 'file')
    var = load('h_ASTSettings.mat', 'runningDevicePara');
    h_AST.running.devicePara = var.runningDevicePara;
else
    h_AST.running.devicePara.voltageRange = [0 5];
end
% h_AST.pupil = [];
% h_AST.EEG = [];
% h_AST.ERG = [];

% save below to "enable"
% h_AST
% h_AST.running.chObj = addAnalogInputChannel(h_AST.init.session,'Dev2',6,'Voltage');
% h_AST.running.chRange = h_AST.init.runningInputRange;

function h_AST_listenerFcn(src, event)

global h_AST
handles = h_AST.currentHandles;
% plot(handles.runningPlotAxes, event.TimeStamps, event.Data);
% disp('-1')
% try
    deltaVoltage = diff(h_AST.running.devicePara.voltageRange);
    thresh = 3/5*deltaVoltage;
    idx = h_AST.running.currentIdx;
%     if idx == 1
%         h_AST.running.initialTime = now - event.TimeStamps(end)/60/60/24;%this is in day number Set it here is more accurate.
%     end
%     disp('0')
    h_AST.running.time(idx) = now - diff(event.TimeStamps([1, end]))/2/60/60/24; %TimeStamps are in seconds time is in day number.
    h_AST.running.relativeTimeInMin(idx) = (h_AST.running.time(idx) - h_AST.running.time(1))*24*60;
    data = event.Data;
%     disp('1')
    diffData = diff(data);
    ind = find(abs(diffData)>3); %the the sensor go pass 360 degree. under normal circumstances, there should be only one. 
%     disp('1')
    for i = 1:length(ind) %Just in case it goes back and forth
        if diffData(ind(i))<thresh
            data(ind(i)+1:end) = data(ind(i)+1:end)+deltaVoltage;
        elseif diffData(ind(i))>thresh
            data(ind(i)+1:end) = data(ind(i)+1:end)-deltaVoltage;
        end
    end
    dataInDegree = data *360 / 5;
    h_AST.running.duration(idx) = mean(event.TimeStamps(end-10:end)) - mean(event.TimeStamps(1:11));
    h_AST.running.startVol(idx) = mean(event.Data(1:11));
    h_AST.running.endVol(idx) = mean(event.Data(end-10:end));
    
    % add a second level of suppression here for going from 359.999 to 0
    % degree. The algorithm above get rid of most but sometimes if there is
    % intermediate point during transision, it can escape.
    deltaDegree = mean(dataInDegree(end-10:end)) - mean(dataInDegree(1:11));
    if deltaDegree>200
        deltaDegree = deltaDegree - 360;
    elseif deltaDegree<-200
        deltaDegree = deltaDegree + 360;
    end
    % end of additional suppression
    
    h_AST.running.speed(idx) = deltaDegree/h_AST.running.duration(idx);
    h_AST.running.currentIdx = h_AST.running.currentIdx + 1;
%     disp('2')
    if h_AST.running.currentIdx > length(h_AST.running.time)
        h_AST.running.time = cat(2, h_AST.running.time, zeros(1, 50000));
        h_AST.running.relativeTimeInMin = cat(2, h_AST.running.relativeTimeInMin, zeros(1, 50000));
        h_AST.running.speed = cat(2, h_AST.running.speed, zeros(1, 50000));
        h_AST.running.startVol = cat(2, h_AST.running.startVol, zeros(1, 50000));
        h_AST.running.endVol = cat(2, h_AST.running.endVol, zeros(1, 50000));
        h_AST.running.duration = cat(2, h_AST.running.duration, zeros(1, 50000));
    end
    % disp('3')
    
    if idx == 1
        h_AST.running.plotObj = plot(handles.runningPlotAxes, h_AST.running.relativeTimeInMin(1:idx), h_AST.running.speed(1:idx));
%         axes(handles.runningPlotAxes);
        ylabel(handles.runningPlotAxes, 'Ang. vel. (d/s)'); xlabel(handles.runningPlotAxes, 'Time (min)');
        h_AST_setLimit;
    else
        set(h_AST.running.plotObj, 'xdata', h_AST.running.relativeTimeInMin(1:idx), 'ydata', h_AST.running.speed(1:idx));
        xLim = get(handles.runningPlotAxes, 'XLim');
        if h_AST.running.relativeTimeInMin(idx) > xLim(2) 
            h_AST_setLimit;
        end
    end
    
    saveEvery = 100;
    if mod(idx,saveEvery)==0 %save every 100 points
%         tic
        fwrite(h_AST.running.FID, cat(1, h_AST.running.time(idx-saveEvery+1:idx), h_AST.running.speed(idx-saveEvery+1:idx), ...
            h_AST.running.startVol(idx-saveEvery+1:idx),h_AST.running.endVol(idx-saveEvery+1:idx),h_AST.running.duration(idx-saveEvery+1:idx)), 'double');
%         disp(size(cat(1, h_AST.running.time(idx-100+1:idx), h_AST.running.speed(idx-100+1:idx))))
% toc
    end
% disp('4')
% catch
%     disp('error')
% end
    


% --- Executes on button press in setPath.
function setPath_Callback(hObject, eventdata, handles)
% hObject    handle to setPath (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

pname = uigetdir('', 'Please select a save path');
if pname~=0 %not cancel
    set(handles.pathName, 'String', pname);
end




function fileName_Callback(hObject, eventdata, handles)
% hObject    handle to fileName (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of fileName as text
%        str2double(get(hObject,'String')) returns contents of fileName as a double


% --- Executes during object creation, after setting all properties.
function fileName_CreateFcn(hObject, eventdata, handles)
% hObject    handle to fileName (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yLimStrRunning_Callback(hObject, eventdata, handles)
% hObject    handle to yLimStrRunning (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yLimStrRunning as text
%        str2double(get(hObject,'String')) returns contents of yLimStrRunning as a double

h_AST_setLimit;


% --- Executes during object creation, after setting all properties.
function yLimStrRunning_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yLimStrRunning (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function xLimStrRunning_Callback(hObject, eventdata, handles)
% hObject    handle to xLimStrRunning (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xLimStrRunning as text
%        str2double(get(hObject,'String')) returns contents of xLimStrRunning as a double

h_AST_setLimit;


% --- Executes during object creation, after setting all properties.
function xLimStrRunning_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xLimStrRunning (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in autoYScaleRunning.
function autoYScaleRunning_Callback(hObject, eventdata, handles)
% hObject    handle to autoYScaleRunning (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of autoYScaleRunning


h_AST_setLimit;


function h_AST_setLimit

global h_AST;

handles = h_AST.currentHandles;

if get(handles.autoYScaleRunning, 'value')
    set(handles.runningPlotAxes, 'YLimMode', 'auto');
else
    range = str2num(get(handles.yLimStrRunning, 'String'));
    set(handles.runningPlotAxes, 'YLim', range);
end

xString = get(handles.xLimStrRunning, 'String');
if strcmpi(xString, 'auto')
    set(handles.runningPlotAxes, 'XLimMode', 'auto');
else
    range = str2num(xString);
    if numel(range) == 1
        xlim = [-range, range]/2 + h_AST.running.relativeTimeInMin(h_AST.running.currentIdx-1);
    else
        xlim = range(1:2);
    end
    set(handles.runningPlotAxes, 'XLim', xlim);
end
% 
% if get(handles.autoYPowerSpectrum, 'value')
%     set(handles.powerAxes, 'YLimMode', 'auto');
% else
%     range = str2num(get(handles.setYLimPowerSpectrum, 'String'));
%     set(handles.powerAxes, 'YLim', range);
% end
% 
% xString = get(handles.setXLimPowerSpectrum, 'String');
% if strcmpi(xString, 'auto')
%     set(handles.powerAxes, 'XLim', [0 600]);
% else
%     range = str2num(xString);
%     set(handles.powerAxes, 'XLim', range);
% end


% --- Executes on button press in calibrate.
function calibrate_Callback(hObject, eventdata, handles)
% hObject    handle to calibrate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global h_AST;

if ~strcmpi(get(handles.startTracking, 'string'), 'start')
    return;
else
    disp('Rotary encoder calibration: Please rotate the ball by three full turns in 10s.');
    h_AST.init.sessionObj.IsContinuous = false;% it is set to true when enabled
    h_AST.init.sessionObj.DurationInSeconds = 11;
    data = h_AST.init.sessionObj.startForeground();
    h_AST.init.sessionObj.IsContinuous = true;% reset it back to continuous.
    
    [peakValue, peakPos] = h_AST_findPeaks(data);
    [valleyValue, valleyPos] = h_AST_findPeaks(-data);
    valleyValue = -valleyValue;
%     if ~err && ~err2
       h_AST.running.devicePara.voltageRange = [mean(valleyValue), mean(peakValue)];
       if diff(h_AST.running.devicePara.voltageRange)>3 % add error checking here. Other para checking can be add later.
           disp(['Range is ', num2str(h_AST.running.devicePara.voltageRange)]);
           pname = h_findFilePath('h_animalStateTracker.m');
           fileName = fullfile(pname, 'h_ASTSettings.mat');
           runningDevicePara = h_AST.running.devicePara;
           save(fileName, 'runningDevicePara');
       else
           disp('Calibration error. Range too small')
       end
%     else
%         disp('error in calibration')
%     end

end


function [peakValue, peakPos] = h_AST_findPeaks(data)

% smoothFactor = 5;
% data2 = smooth(data, smoothFactor);
data2 = data;% don't smooth

avg = mean(data2);
maxData = max(data2);
minData = min(data2);
I = data>avg;

labeledI = bwlabel(I);

peakValue = nan(1, max(labeledI));
peakPos = nan(1, max(labeledI));

for i = 1:max(labeledI)
    I2 = labeledI==i;
    if sum(I2)>10
        [peakValue(i), ind] = max(data2(I2));
        peakPos(i) = find(I2, 1)+ind-1;
    end
end

II1 = peakValue > avg + (maxData-avg)*0.8; % this also get rid of all nan.
II2 = peakPos < 10 | peakPos > length(data)-9;% cannot be at the very edge.

peakValue = peakValue(II1&~II2);
peakPos = peakPos(II1&~II2);

