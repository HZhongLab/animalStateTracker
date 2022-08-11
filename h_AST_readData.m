function data = h_AST_readData(filename)

% if data is two rows, first is time (absolute date number), second is
% angular speed (degree/s)
% if data has 5 rows, 3 is beginning voltage, 4 is end voltage, 5 is
% duration between begining and end of the timepoint

fid = fopen(filename);
data = fread(fid, inf, 'double');
fclose(fid);

[pname, fname, fExt] = fileparts(filename);
if strcmpi(fExt, '.dat2')
    data = reshape(data, [5, numel(data)/5]);
else
    data = reshape(data, [2, numel(data)/2]);
end