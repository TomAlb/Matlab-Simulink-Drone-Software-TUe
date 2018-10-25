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
%% get the element type string
function [dtypeStr] = get_elem_type(dtype)
    switch(dtype)
        case 1
            dtypeStr = 'double';
        case 2
            dtypeStr = 'single';
        case 3
            dtypeStr = 'int32';
        case 4
            dtypeStr = 'uint32';
        case 5
            dtypeStr = 'int16';
        case 6
            dtypeStr = 'uint16';
        case 7
            dtypeStr = 'int8';
        case 8
            dtypeStr = 'uint8';
        case 9
            dtypeStr = 'logical';
        case 10
            dtypeStr = 'embedded.fi';
    end
end