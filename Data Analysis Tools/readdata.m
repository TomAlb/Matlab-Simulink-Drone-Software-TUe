%% Load the data into MATLAB from a binary log file
% Usage: >> [datapoints, numpoints] = readdata('datafile.log')
% Header information format:
%           String "MWLOGV##"
%           Time/Date 4 bytes (time())
%           Number of Signals per record Logged 1 bytes (256 max)
%           Data Type of Signals Logged  1 bytes (1-10)
%           Number of bytes per record 2 (65535 max)
% Plot Data Example: plot([1:numpoints], datapoints(1,:), [1:numpoints], datapoints(2,:))
% MathWorks Pilot Engineering 2015
% Steve Kuznicki
function [datapts, numpts] = readdata(dataFile)
%%
datapts = 0;
numpts = 0;

if nargin == 0
    dataFile = 'data.bin';
end

fid = fopen(dataFile, 'r');
% load the header information
hdrToken = fread(fid, 8, 'char');
if strncmp(char(hdrToken),'MWLOGV',6) == true
    logTime = uint32(fread(fid, 1, 'uint32'));
    numflds = double(fread(fid, 1, 'uint8'));
    typefld = uint8(fread(fid, 1, 'uint8'));
    recSize = uint16(fread(fid, 1, 'uint16'));
    fieldTypeStr = get_elem_type(typefld);
    datapts = fread(fid, double([numflds, Inf]), fieldTypeStr);
    fclose(fid);
    numpts = size(datapts,2);
end

end