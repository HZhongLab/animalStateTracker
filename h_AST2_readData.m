function [header, data] = h_AST2_readData(filename)

% first row is time (absolute date number), 
% others are data.

fid = fopen(filename, 'r');

headerStr = '';
while true
    tline = fgetl(fid);
    assert(ischar(tline), 'end of file reached prematurely');
    if strcmp(tline, 'header_end')
        break;
    end;
    headerStr = [headerStr, tline];
end;


data = fread(fid, inf, 'int16');
fclose(fid);

try
    evalc(headerStr);
    numOfCh = length(header.activeChIDs);
    data = reshape(data, [numOfCh, numel(data)/numOfCh])./ header.scale;
catch
    header = [];
    data = [];
end

