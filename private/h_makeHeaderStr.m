
function headerstr = h_makeHeaderStr(inputVar, varName)

% this function is copied from a subfunction in spc_makeheaderstr.m
% edited and modified by HN

if ~exist('varName', 'var') || isempty(varName)
    varName = 'header';
end

evalc([varName, ' = inputVar']);

headerstr = '';
evalc(['fn = fieldnames(', varName, ')']);

for i=1:length(fn)
    a = cell2struct(fn(i), 'a', 1);
    fieldstr = [varName, '.', a.a];
    ntr = 0; ctr = 0; sttr = 0;
    try
        eval (['ntr = isnumeric(', fieldstr, ');']);
        eval (['ctr = ischar(', fieldstr, ');']);
        eval (['sttr = isstruct(', fieldstr, ');']); 
    catch
        display (['ntr = isnumeric(', fieldstr, ');']);
        display (['ctr = ischar(', fieldstr, ');']);
        display (['sttr = isstruct(', fieldstr, ');']); 
    end
    if ntr
        valstr = mat2str(eval (fieldstr));
        exestr = [fieldstr, ' = ', valstr, ';', 10];
    elseif ctr
        strA = eval(fieldstr);
        strA(strfind(strA, '''')) = [];        
        valstr = strcat('''', strA, '''');
        exestr = [fieldstr, ' = ', valstr, ';', 10];
    elseif sttr
        exestr = h_makeHeaderStr(eval(fieldstr), fieldstr); %decode_struct (fieldstr);
    else
        exestr = '';    
    end
%     eval(headerstr);
    headerstr = [headerstr, exestr];        
end
