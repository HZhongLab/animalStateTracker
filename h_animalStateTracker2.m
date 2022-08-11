function varargout = h_animalStateTracker2(varargin)
% H_ANIMALSTATETRACKER2 MATLAB code for h_animalStateTracker2.fig
%      H_ANIMALSTATETRACKER2, by itself, creates a new H_ANIMALSTATETRACKER2 or raises the existing
%      singleton*.
%
%      H = H_ANIMALSTATETRACKER2 returns the handle to a new H_ANIMALSTATETRACKER2 or the handle to
%      the existing singleton*.
%
%      H_ANIMALSTATETRACKER2('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in H_ANIMALSTATETRACKER2.M with the given input arguments.
%
%      H_ANIMALSTATETRACKER2('Property','Value',...) creates a new H_ANIMALSTATETRACKER2 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before h_animalStateTracker2_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to h_animalStateTracker2_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help h_animalStateTracker2

% Last Modified by GUIDE v2.5 29-Jan-2020 10:19:01

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @h_animalStateTracker2_OpeningFcn, ...
                   'gui_OutputFcn',  @h_animalStateTracker2_OutputFcn, ...
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


% --- Executes just before h_animalStateTracker2 is made visible.
function h_animalStateTracker2_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to h_animalStateTracker2 (see VARARGIN)

% Choose default command line output for h_animalStateTracker2
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes h_animalStateTracker2 wait for user response (see UIRESUME)
% uiwait(handles.h_animalStateTracker2);

h_AST2_setup(hObject);


% --- Outputs from this function are returned to the command line.
function varargout = h_animalStateTracker2_OutputFcn(hObject, eventdata, handles) 
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

global h_AST2

if strcmpi(get(hObject, 'string'), 'start')
    set(hObject, 'String', 'Stop')
    h_updateActiveChannels;
    [h_AST2.acq.chObj, h_AST2.acq.chIdxInSession] = addAnalogInputChannel(h_AST2.init.sessionObj,h_AST2.init.device,h_AST2.acq.activeChIDs,'Voltage');
    h_AST2.acq.listenerObj = addlistener(h_AST2.init.sessionObj, 'DataAvailable', @h_AST2_listenerFcn);

    % a good practice is probably to save repeated start and stop in different files.
    
    h_AST2.acq.time = zeros(1, 500000);
    numOfCh = length(h_AST2.acq.activeChIDs);
    h_AST2.acq.data = zeros(numOfCh, 50000);
    h_AST2.acq.currentIdx = 1;
    h_AST2.running.currentIdx = ones(1, numOfCh); % current position to cal speed data. This may not be correlated with acq.currentIdx.
    h_AST2.running.numOfTransition = zeros(1, numOfCh); % counting this to know when to calibrate the voltage.
    
    pathName = get(handles.pathName, 'string');
    if ~exist(pathName, 'dir')
        pathName = pwd;
        set(handles.pathName, 'string', pwd);
    end
    fileName = get(handles.fileName, 'string');
    [pname, fname, fExt] = fileparts(fileName);
    if isempty(fExt)
        fExt = '.ast2';
    end
    j = 1;
    fullFileName = fullfile(pathName, [fname, '_AST2_', num2str(j),fExt]);
    while exist(fullFileName, 'file')
        j = j + 1;
        fullFileName = fullfile(pathName, [fname, '_AST2_', num2str(j),fExt]);
    end
    h_AST2.acq.fileName = fullFileName;

    h_AST2.acq.FID = fopen(h_AST2.acq.fileName, 'a');
    h_writeAST2Header;
    h_AST2.init.sessionObj.startBackground();
else % stop
    h_AST2.init.sessionObj.stop();
    
    % save the unsaved data before quitting.
    saveEvery = h_AST2.init.saveEvery;
    listnerSessionLength = round(h_AST2.init.inputRate/h_AST2.init.recordRate);
    idx = h_AST2.acq.currentIdx;
    previousListenerNum = (idx-1)/listnerSessionLength;
    lastSavedListenerNum = floor(previousListenerNum / saveEvery) * saveEvery;
    idxToSave = lastSavedListenerNum*listnerSessionLength+1 : idx-1;
    scale = h_AST2.init.scale;
%     fwrite(h_AST2.acq.FID, cat(1, h_AST2.acq.time(idxToSave), h_AST2.acq.data(:,idxToSave)), 'double');
    fwrite(h_AST2.acq.FID, int16(h_AST2.acq.data(:,idxToSave)*scale), 'int16');
    
    % close everything.
    fclose(h_AST2.acq.FID);
    delete(h_AST2.acq.listenerObj);
    removeChannel(h_AST2.init.sessionObj,h_AST2.acq.chIdxInSession);    
    set(hObject, 'String', 'Start')
end


% % --- Executes on button press in enableRunningTracking.
% function enableRunningTracking_Callback(hObject, eventdata, handles)
% % hObject    handle to enableRunningTracking (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% 
% % Hint: get(hObject,'Value') returns toggle state of enableRunningTracking
% 
% % this part is no long used. 
% 
% global h_AST2
% 
% if get(hObject, 'value')
%     [h_AST2.acq.chObj, h_AST2.acq.chIdxInSession] = addAnalogInputChannel(h_AST2.init.sessionObj,h_AST2.init.device,h_AST2.init.acqChIDs,'Voltage');
%     h_AST2.acq.chObj.Range = h_AST2.init.runningInputRange;
% 
% %     h_AST2.acq.chObj.DurationInSeconds = 5;% this is a session property
% %     h_AST2.acq.chObj.IsContinuous = true; % this is a session property
% %     h_AST2.init.sessionObj.DurationInSeconds = 5;
%     h_AST2.init.sessionObj.IsContinuous = true;
% %     h_AST2.init.sessionObj.NotifyWhenDataAvailableExceeds = xx % try setting this in the setup.
% elseif ~isempty(h_AST2.running)
%     removeChannel(h_AST2.init.sessionObj, h_AST2.acq.chIdxInSession);
%     h_AST2.running = [];
% end




function h_AST2_setup(hObject)

% this is the initial parameters.

global h_AST2

h_AST2.currentHandles = guihandles(hObject);
h_AST2.init.inputRate = 1000;%2kHz Note: only one input rate for all channels. There is only one input clock.
h_AST2.init.recordRate = 4; % We will now only record the raw data. But this can serve as teh listening rate.
h_AST2.init.device = 'Dev1'; % currently assuming all on one device.
h_AST2.init.sessionObj = daq.createSession('ni'); % one session for all channels.
h_AST2.init.sessionObj.Rate = h_AST2.init.inputRate;
h_AST2.init.sessionObj.NotifyWhenDataAvailableExceeds = round(h_AST2.init.inputRate/h_AST2.init.recordRate);
h_AST2.init.acqChIDs = [0 1 2 3];
h_AST2.init.acqInputRange = [-10 10]; % this is default for M series.
h_AST2.init.sessionObj.IsContinuous = true;
h_AST2.init.saveEvery = 50; % save every 100 listner events.
h_AST2.init.scale = 2^15 / 10; % this is the factor to convert -10 to 10V to int16 to save space.
h_AST2.init.voltageRange = repmat([0 5], [length(h_AST2.init.acqChIDs), 1]); % start at 5 and calibrate as data is available.

h_AST2.disp.downSampleFactor = 50; % if the traces have > 500k points, it becomes very slow. Simple downsample for now. may be able to go fancy in the future.
h_AST2.running.speedDownSampleFactor = 50; % in other words, it will calculate a speed point every 50 raw data points.
h_AST2.running.currentIdx = 1;
err = h_AST2_loadSettings(1);
if err
    h_AST2.setting = [];
end



% this was for calibration. Should be no longer needed in v2. still keep
% for now.
% if exist('h_AST2Settings.mat', 'file')
%     var = load('h_AST2Settings.mat', 'runningDevicePara');
%     h_AST2.acq.devicePara = var.runningDevicePara;
% else
%     h_AST2.acq.devicePara.voltageRange = [0 5];
% end

% h_AST2.pupil = [];
% h_AST2.EEG = [];
% h_AST2.ERG = [];

% save below to "enable"
% h_AST2
% h_AST2.acq.chObj = addAnalogInputChannel(h_AST2.init.session,'Dev2',6,'Voltage');
% h_AST2.acq.chRange = h_AST2.init.runningInputRange;



function h_AST2_listenerFcn(src, event)

global h_AST2

currentData = event.Data;
currentTime = now + (event.TimeStamps-event.TimeStamps(end))/60/60/24; %TimeStamps are in seconds time is in day number.
currentLength = length(event.TimeStamps);
idx = h_AST2.acq.currentIdx;
h_AST2.acq.data(:, idx:idx+currentLength-1) = currentData';
h_AST2.acq.time(idx:idx+currentLength-1) = currentTime';
h_AST2.acq.currentIdx = idx + currentLength;

saveEvery = h_AST2.init.saveEvery;
currentListenerNum = (idx-1)/currentLength + 1;
% disp(num2str(mod(currentListenerNum,saveEvery)))
if mod(currentListenerNum,saveEvery)==0 %save every 100 sessions - 10s
    idxToSave = (currentListenerNum-saveEvery)*currentLength+(1:currentLength*saveEvery);
    % try save as int16 format to save space
    scale = h_AST2.init.scale;
%     dataToSave = int16(cat(1, h_AST2.acq.time(idxToSave), h_AST2.acq.data(:,idxToSave))*scale);
    dataToSave = int16(h_AST2.acq.data(:,idxToSave)*scale);
    fwrite(h_AST2.acq.FID, dataToSave, 'int16');
%     disp('saved')
end

h_AST2_displayData; % separate this into a different function to make it modular and easy to understand and modify.


function h_AST2_displayData

global h_AST2

% disp(['# ', num2str((h_AST2.acq.currentIdx-1)/round(h_AST2.init.inputRate/h_AST2.init.recordRate))])
% return;

handles = h_AST2.currentHandles;
endIdx = h_AST2.acq.currentIdx - 1; % this is the current data end.


% listnerSessionLength = round(h_AST2.init.inputRate/h_AST2.init.recordRate);
% tic
for i = 1:length(h_AST2.acq.activeChIDs)
    currentAxesID = find(h_AST2.init.acqChIDs == h_AST2.acq.activeChIDs(i), 1);
    currentAxes = ['plotAxes', num2str(currentAxesID)];
    currentPlotTag = ['CH', num2str(currentAxesID), '_Data'];
    currentDispOptTag = ['dispOpt', num2str(currentAxesID)];
    currentDispOpt = get(handles.(currentDispOptTag), 'value');
    if currentDispOpt == 1 % raw data
        dispDownSampleFactor = h_AST2.disp.downSampleFactor; % display will be come increasingly slow when data is > 500k samples.

        % note: plot greatly slow down when there is >600k samples per
        % channel. will need to downsize before display...
        dataToBePlot = h_AST2.acq.data(i, 1:dispDownSampleFactor:endIdx);
        
        yLabelText = 'Voltage (V)';
        color = 'blue';
        
        % relTimeInMin = (h_AST2.acq.time(1:endIdx) - h_AST2.acq.time(1))*24*60; % time was in day number
        
        % the above time is computer time and not accurate. Since there is no longer the option to stop, use this:
        relTimeInMin = (1:endIdx)/h_AST2.init.inputRate/60;
        relTimeInMin = relTimeInMin(1:dispDownSampleFactor:end);

    else % running data
        previousNumOfTransition = h_AST2.running.numOfTransition(i);
        h_computeRunningData(i);
        currentNumOfTransition = h_AST2.running.numOfTransition(i);
        
        if currentNumOfTransition>=3 && previousNumOfTransition<3
            previousVoltageRange = h_AST2.init.voltageRange(currentAxesID,:);
            h_calibrateVoltageRange(i);
            calibratedVoltageRange = h_AST2.init.voltageRange(currentAxesID,:);
            updateThresh = 0.05; % range should be ~ 5V, so if error is > 0.1%.
            if any(abs(calibratedVoltageRange-previousVoltageRange)>updateThresh)
                h_AST2.running.currentIdx(i) = 1;
                h_computeRunningData(i);
            end
        end
        
        dataToBePlot = h_AST2.running.speedData{i}(1:h_AST2.running.currentIdx(i)-1);
        downSampleFactor = h_AST2.running.speedDownSampleFactor;
        relTimeInMin = (0.5:h_AST2.running.currentIdx(i)-1) * downSampleFactor / h_AST2.init.inputRate / 60;
        yLabelText = 'Ang. vel. (d/s)';
        color = 'magenta';
    end
   
    if isempty(findobj(handles.(currentAxes), 'tag', currentPlotTag))
        h_AST2.acq.plotObj(i) = plot(handles.(currentAxes), relTimeInMin, dataToBePlot, 'b-');
        set(h_AST2.acq.plotObj(i), 'tag', currentPlotTag, 'color', color)
        ylabel(handles.(currentAxes), yLabelText); xlabel(handles.plotAxes1, 'Time (min)');
        h_AST2_setLimit(currentAxesID);
    else
        set(h_AST2.acq.plotObj(i), 'xdata', relTimeInMin, 'ydata', dataToBePlot, 'color', color);
%         ylabel(handles.(currentAxes), yLabelText); % set it everytime may slow things down...
        %         currentXLimStrName = ['xLimStr', num2str(i)];
        
%         this takes incredible amount of time... So is setting xlabel and ylabel
%         xLim = get(handles.(currentAxes), 'XLim'); 

% now we save the currentXLim and only update if it is 'movingFixedWidthWindow' 
% and is beyond the limit Now it is 10X faster(for smaller display data).
        xLim = h_AST2.disp.currentXLim{currentAxesID};
        if ~ischar(xLim) && ((relTimeInMin(end)>xLim(2)) || (relTimeInMin(end)<xLim(1))) && h_AST2.disp.movingFixedWidthWindow(currentAxesID)
            h_AST2_setLimit(currentAxesID);
        end
    end
end
% toc

    


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



function yLimStr1_Callback(hObject, eventdata, handles)
% hObject    handle to yLimStr1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yLimStr1 as text
%        str2double(get(hObject,'String')) returns contents of yLimStr1 as a double

h_AST2_genericSettingCallback(hObject, handles);
h_AST2_setLimit(1);


% --- Executes during object creation, after setting all properties.
function yLimStr1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yLimStr1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function xLimStr1_Callback(hObject, eventdata, handles)
% hObject    handle to xLimStr1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xLimStr1 as text
%        str2double(get(hObject,'String')) returns contents of xLimStr1 as a double

h_AST2_genericSettingCallback(hObject, handles);
h_AST2_setLimit(1);


% --- Executes during object creation, after setting all properties.
function xLimStr1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xLimStr1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in autoYScale1.
function autoYScale1_Callback(hObject, eventdata, handles)
% hObject    handle to autoYScale1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of autoYScale1


h_AST2_genericSettingCallback(hObject, handles);
h_AST2_setLimit(1);


function h_AST2_setLimit(flag)

global h_AST2;

if ~exist('flag', 'var') || isempty(flag)
    return
end

handles = h_AST2.currentHandles;

currentAutoYScaleName = ['autoYScale', num2str(flag)];
currentAxesName = ['plotAxes', num2str(flag)];
currentYLimStrName = ['yLimStr', num2str(flag)];
currentXLimStrName = ['xLimStr', num2str(flag)];


if get(handles.(currentAutoYScaleName), 'value')
    set(handles.(currentAxesName), 'YLimMode', 'auto');
else
    range = str2num(get(handles.(currentYLimStrName), 'String'));
    set(handles.(currentAxesName), 'YLim', range);
end

xString = get(handles.(currentXLimStrName), 'String');
if strcmpi(xString, 'auto')
    set(handles.(currentAxesName), 'XLimMode', 'auto');
    h_AST2.disp.currentXLim{flag} = 'auto';
else
    range = str2num(xString);
    if numel(range) == 1
        xlim = [-range, range]/2 + (h_AST2.acq.currentIdx-1)/h_AST2.init.inputRate/60;
        h_AST2.disp.movingFixedWidthWindow(flag) = 1;
    else
        xlim = range(1:2);
        h_AST2.disp.movingFixedWidthWindow(flag) = 0;
    end
    set(handles.(currentAxesName), 'XLim', xlim);
    h_AST2.disp.currentXLim{flag} = xlim;
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


% Calibration is no longer needed in AST2 but keep the code here for now. 

function h_calibrateVoltageRange(flag)

global h_AST2

currentAxesID = find(h_AST2.init.acqChIDs == h_AST2.acq.activeChIDs(flag), 1);

data = h_AST2.acq.data(flag, 1:h_AST2.acq.currentIdx-1);

[peakValue, peakPos] = h_AST2_findPeaks(data);
[valleyValue, valleyPos] = h_AST2_findPeaks(-data);
valleyValue = -valleyValue;
%     if ~err && ~err2
h_AST2.init.voltageRange(currentAxesID, :) = [mean(valleyValue), mean(peakValue)];
if diff(h_AST2.init.voltageRange(currentAxesID, :))>3 % add error checking here. Other para checking can be add later.
    disp(['Ch# ',num2str(h_AST2.acq.activeChIDs(flag)),'''s calibrated voltage range is ',...
        num2str(h_AST2.init.voltageRange(currentAxesID, :))]);
else
    disp('Calibration error. Range too small')
end
%     else
%         disp('error in calibration')
%     end




function [peakValue, peakPos] = h_AST2_findPeaks(data)

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


% --- Executes on selection change in dispOpt1.
function dispOpt1_Callback(hObject, eventdata, handles)
% hObject    handle to dispOpt1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns dispOpt1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from dispOpt1

h_AST2_genericSettingCallback(hObject, handles);
h_setYLabel(hObject, handles);


    
    
% --- Executes during object creation, after setting all properties.
function dispOpt1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dispOpt1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yLimStr2_Callback(hObject, eventdata, handles)
% hObject    handle to yLimStr2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yLimStr2 as text
%        str2double(get(hObject,'String')) returns contents of yLimStr2 as a double

h_AST2_genericSettingCallback(hObject, handles);
h_AST2_setLimit(2);


% --- Executes during object creation, after setting all properties.
function yLimStr2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yLimStr2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function xLimStr2_Callback(hObject, eventdata, handles)
% hObject    handle to xLimStr2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xLimStr2 as text
%        str2double(get(hObject,'String')) returns contents of xLimStr2 as a double

h_AST2_genericSettingCallback(hObject, handles);
h_AST2_setLimit(2);


% --- Executes during object creation, after setting all properties.
function xLimStr2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xLimStr2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in autoYScale2.
function autoYScale2_Callback(hObject, eventdata, handles)
% hObject    handle to autoYScale2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of autoYScale2

h_AST2_genericSettingCallback(hObject, handles);
h_AST2_setLimit(2);


% --- Executes on selection change in dispOpt2.
function dispOpt2_Callback(hObject, eventdata, handles)
% hObject    handle to dispOpt2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns dispOpt2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from dispOpt2

h_AST2_genericSettingCallback(hObject, handles);
h_setYLabel(hObject, handles);


% --- Executes during object creation, after setting all properties.
function dispOpt2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dispOpt2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yLimStr3_Callback(hObject, eventdata, handles)
% hObject    handle to yLimStr3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yLimStr3 as text
%        str2double(get(hObject,'String')) returns contents of yLimStr3 as a double

h_AST2_genericSettingCallback(hObject, handles);
h_AST2_setLimit(3);


% --- Executes during object creation, after setting all properties.
function yLimStr3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yLimStr3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function xLimStr3_Callback(hObject, eventdata, handles)
% hObject    handle to xLimStr3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xLimStr3 as text
%        str2double(get(hObject,'String')) returns contents of xLimStr3 as a double

h_AST2_genericSettingCallback(hObject, handles);
h_AST2_setLimit(3);


% --- Executes during object creation, after setting all properties.
function xLimStr3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xLimStr3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in autoYScale3.
function autoYScale3_Callback(hObject, eventdata, handles)
% hObject    handle to autoYScale3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of autoYScale3

h_AST2_genericSettingCallback(hObject, handles);
h_AST2_setLimit(3);


% --- Executes on selection change in dispOpt3.
function dispOpt3_Callback(hObject, eventdata, handles)
% hObject    handle to dispOpt3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns dispOpt3 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from dispOpt3

h_AST2_genericSettingCallback(hObject, handles);
h_setYLabel(hObject, handles);


% --- Executes during object creation, after setting all properties.
function dispOpt3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dispOpt3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yLimStr4_Callback(hObject, eventdata, handles)
% hObject    handle to yLimStr4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yLimStr4 as text
%        str2double(get(hObject,'String')) returns contents of yLimStr4 as a double

h_AST2_genericSettingCallback(hObject, handles);
h_AST2_setLimit(4);


% --- Executes during object creation, after setting all properties.
function yLimStr4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yLimStr4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function xLimStr4_Callback(hObject, eventdata, handles)
% hObject    handle to xLimStr4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xLimStr4 as text
%        str2double(get(hObject,'String')) returns contents of xLimStr4 as a double

h_AST2_genericSettingCallback(hObject, handles);
h_AST2_setLimit(4);


% --- Executes during object creation, after setting all properties.
function xLimStr4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xLimStr4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in autoYScale4.
function autoYScale4_Callback(hObject, eventdata, handles)
% hObject    handle to autoYScale4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of autoYScale4

h_AST2_setLimit(4);
h_AST2_genericSettingCallback(hObject, handles);


% --- Executes on selection change in dispOpt4.
function dispOpt4_Callback(hObject, eventdata, handles)
% hObject    handle to dispOpt4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns dispOpt4 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from dispOpt4

h_setYLabel(hObject, handles);
h_AST2_genericSettingCallback(hObject, handles);


% --- Executes during object creation, after setting all properties.
function dispOpt4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dispOpt4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function h_writeAST2Header

global h_AST2

fieldsToBeRemoved = {'sessionObj'};
header = h_AST2.init;
header = rmfield(header, fieldsToBeRemoved);
header.activeChIDs = h_AST2.acq.activeChIDs;
header.startTime = datestr(now, 'dd-mmm-yyyy HH:MM:SS.FFF');
headerStr = h_makeHeaderStr(header, 'header');
headerStr = [headerStr, 'header_end', 10];%note: 10 is sprintf('%s\n', '')

fwrite(h_AST2.acq.FID, headerStr);



function h_setYLabel(hObject, handles)

currentDispOpt = get(hObject, 'value');
currentTag = get(hObject, 'tag');
currentAxesNumStr = currentTag(end);
if currentDispOpt == 1 % raw data
    yLabelText = 'Voltage (V)';
else
    yLabelText = 'Ang. vel. (d/s)';
end

currentPlotAxes = ['plotAxes', currentAxesNumStr];
ylabel(handles.(currentPlotAxes), yLabelText);



function h_computeRunningData(flag)

% flag is the position at the active channel list.

global h_AST2

currentAxesID = find(h_AST2.init.acqChIDs == h_AST2.acq.activeChIDs(flag), 1);
speedDownSampleFactor = h_AST2.running.speedDownSampleFactor;

% segmentLength = h_AST2.init.inputRate / h_AST2.init.recordRate; 
% it may not be always only the last segment. E.g., someone changes the display opt mid-way. 
rawDataEndIdx = h_AST2.acq.currentIdx-1;
runningDataIdx = h_AST2.running.currentIdx(flag);
rawDataStartIdx = (runningDataIdx-1)*speedDownSampleFactor + 1;
dataLength = rawDataEndIdx - rawDataStartIdx + 1;
segmentLength = h_AST2.running.speedDownSampleFactor;
numSegment = dataLength / segmentLength;

% add an error check:
if numSegment ~= round(numSegment)
    disp(['Error!, subSegmentLength = ', num2str(numSegment)]);
end

% initialize the storage space and add buffer as needed.
if runningDataIdx == 1
    h_AST2.running.speedData{flag} = zeros(1, 50000); % at a speed sampling rate of 20Hz, this is ~40 min of recording
elseif runningDataIdx+numSegment > length(h_AST2.running.speedData{flag})
    h_AST2.running.speedData{flag} = horzcat(h_AST2.running.speedData{flag}, zeros(1, 50000));
end

time = (1:h_AST2.running.speedDownSampleFactor)/h_AST2.init.inputRate; % this is in seconds.
voltageRange = h_AST2.init.voltageRange(currentAxesID, :);

for i = 1:numSegment
    currentData = h_AST2.acq.data(flag, (rawDataStartIdx+(i-1)*segmentLength):(rawDataStartIdx+i*segmentLength-1));
    [speed, isTransition] = h_computeSpeed(time, currentData, voltageRange);
    h_AST2.running.speedData{flag}(runningDataIdx) = speed; % use cell here as probably not every channel is running
    runningDataIdx = runningDataIdx + 1;
    h_AST2.running.numOfTransition(flag) = h_AST2.running.numOfTransition(flag) + isTransition;
end
h_AST2.running.currentIdx(flag) = runningDataIdx;



function [speed, isTransition] = h_computeSpeed(time, data, voltageRange)

% this returns one single point for each stretch of data. So it
% downsamples. How much to put in can be controlled separately in the
% parental function.

isTransition = 0; % this is a flag for the number of transition events that will be used for calibration later.
deltaVoltage = diff(voltageRange); % it starts at 5V and will be calibrated along the way. 
thresh = 3/5*deltaVoltage;
% idx = h_AST2.acq.currentIdx;

diffData = diff(data);
ind = find(abs(diffData)>3); %the the sensor go pass 360 degree. under normal circumstances, there should be only one in one session.
for i = 1:length(ind) %Just in case it goes back and forth
    isTransition = 1;
    if diffData(ind(i))<thresh
        data(ind(i)+1:end) = data(ind(i)+1:end)+deltaVoltage;
    elseif diffData(ind(i))>thresh
        data(ind(i)+1:end) = data(ind(i)+1:end)-deltaVoltage;
    end
end

dataInDegree = (data / deltaVoltage) * 360;


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

duration = mean(time(end-10:end)) - mean(time(1:11));
speed = deltaDegree/duration;


% --- Executes on button press in enable1.
function enable1_Callback(hObject, eventdata, handles)
% hObject    handle to enable1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of enable1

h_AST2_genericSettingCallback(hObject, handles)


% --- Executes on button press in enable2.
function enable2_Callback(hObject, eventdata, handles)
% hObject    handle to enable2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of enable2

h_AST2_genericSettingCallback(hObject, handles)


% --- Executes on button press in enable3.
function enable3_Callback(hObject, eventdata, handles)
% hObject    handle to enable3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of enable3

h_AST2_genericSettingCallback(hObject, handles)


% --- Executes on button press in enable4.
function enable4_Callback(hObject, eventdata, handles)
% hObject    handle to enable4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of enable4

h_AST2_genericSettingCallback(hObject, handles)


function h_updateActiveChannels

global h_AST2

I = false(1, 4);
handles = h_AST2.currentHandles;

for i = 1:4
    if get(handles.(['enable', num2str(i)]), 'value')==1
        I(i) = true;
    end
end

h_AST2.acq.activeChIDs = h_AST2.init.acqChIDs(I);


% --------------------------------------------------------------------
function Untitled_1_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function saveSetting1_Callback(hObject, eventdata, handles)
% hObject    handle to saveSetting1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

h_AST2_saveSettings(1)

% --------------------------------------------------------------------
function saveSetting2_Callback(hObject, eventdata, handles)
% hObject    handle to saveSetting2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

h_AST2_saveSettings(2)


% --------------------------------------------------------------------
function saveSetting3_Callback(hObject, eventdata, handles)
% hObject    handle to saveSetting3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

h_AST2_saveSettings(3)

% --------------------------------------------------------------------
function loadSetting1_Callback(hObject, eventdata, handles)
% hObject    handle to loadSetting1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

h_AST2_loadSettings(1);

% --------------------------------------------------------------------
function loadSetting2_Callback(hObject, eventdata, handles)
% hObject    handle to loadSetting2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

h_AST2_loadSettings(2);

% --------------------------------------------------------------------
function loadSetting3_Callback(hObject, eventdata, handles)
% hObject    handle to loadSetting3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

h_AST2_loadSettings(3);

function h_AST2_genericSettingCallback(hObject, handles)

% this function try to setting the state variable for various fields so
% that coding is simplified. Copied and modified from FV_genericSettingCallback

global h_AST2;  

objName = get(hObject, 'Tag');
objType = get(hObject, 'Style');

switch objType
    case {'popupmenu', 'radiobutton', 'checkbox'}
        currentValue = get(hObject,'Value');
        h_AST2.setting.(objName).value = currentValue;

    case 'togglebutton'
        currentValue = get(hObject,'Value');
        if currentValue % toggle button color need to be changed.
            set(handles.(objName),'Value',currentValue,'BackgroundColor',[0.8 0.8 0.8]);
        else
            set(handles.(objName),'Value',currentValue,'BackgroundColor',[ 0.9255    0.9137    0.8471]);
        end
        
        h_AST2.setting.(objName).value = currentValue;
        h_AST2.setting.(objName).BackgroundColor = get(hObject,'BackgroundColor');%this is necessary for all toggle bottons.

    case 'edit'
        h_AST2.setting.(objName).string = get(hObject, 'String');
    otherwise
end

% if numel(handles.(objName))>1 % if there are multiple fields with same name
%     FV_setParaAccordingToState(handles);%this need to be add if there are multiple fields.
% end


function h_AST2_saveSettings(flag)

global h_AST2

% flag =    1 (default), 2 or 3

if ~exist('flag', 'var') || isempty(flag)
    flag = 1;
end

pname = h_findFilePath('h_animalStateTracker2.m');
fileName = fullfile(pname, 'h_AST2_settings.mat');

if exist(fileName, 'file')
    load(fileName); %this should load a variable called "settings".
end

settings(flag).setting = h_AST2.setting;

% exclusionList = getExclusionList; % there are certain state settings should not be saved. Such as filenames.
% settings(flag).state = rmfield(settings(flag).state, exclusionList);

save(fileName, 'settings');

% function exclusionList = getExclusionList
% exclusionList = {
%     'AST_fileName' % this is set for info but really should not be in state settings, so exclude it.
%     };

function err = h_AST2_loadSettings(flag)

global h_AST2

% flag =    1 (default), 2 or 3

if ~exist('flag', 'var') || isempty(flag)
    flag = 1;
end

handles = h_AST2.currentHandles;

pname = h_findFilePath('h_animalStateTracker2.m');
fileName = fullfile(pname, 'h_AST2_settings.mat');

if exist(fileName, 'file')
%     load(fileName); %this should load a variable called "settings".
    temp = load(fileName); % old way of handling is no longer allowed in new MATLAB
    settings = temp.settings;
    err = 0;
else
    disp(['cannot find setting file: ', fileName]);
    err = 1;
    return
end

h_AST2.setting = settings(flag).setting;

% exclusionList = getExclusionList; % when loading settings, there are certain variable should not be loaded.

h_AST2_setParaAccordingToState;
% FV_updateDispSettingVar(handles);


function h_AST2_setParaAccordingToState(inclusionList, exclusionList)

global h_AST2;

handles = h_AST2.currentHandles;
state = h_AST2.setting;

% inclusionList is a cell str containing the fields want to set. 
% this is better than exclusionList as there are always newly added fields.
if ~exist('inclusionList', 'var') || isempty(inclusionList)
    inclusionList = fieldnames(state); % if it is empty, include everything
elseif ~iscell(inclusionList)
        inclusionList = {inclusionList};
end

% probably should do both inclusionList and exclusionList
if ~exist('exclusionList', 'var') || isempty(exclusionList)
    exclusionList = {''}; % if it is empty, include everything
elseif ~iscell(exclusionList)
        exclusionList = {exclusionList};
end

names = fieldnames(state);
names = names(ismember(names, inclusionList));
names = names(~ismember(names, exclusionList));

for i = 1:length(names)
    try
        ptynames = fieldnames(eval(['state.',names{i}]));
        handlename = ['handles.',names{i}];
        for j = 1:length(ptynames)
            varname = ['state.',names{i},'.',ptynames{j}];
            set(eval(handlename),ptynames{j},eval(varname));
        end
    end
end



    

