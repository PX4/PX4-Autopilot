function ConvertCToC(nStates)

% This function converts the nextP and P covaraince matrix expressions from
% P[row index][col index] to P(row index,col index) syntax to enable use
% of the matrix library type used by PX4

%% Define file to read in
fileName = strcat('C_code',int2str(nStates),'.txt');
delimiter = '';

%% Format string for each line of text:
%   column1: text (%s)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%s%[^\n\r]';

%% Open the text file.
fileID = fopen(fileName,'r');

%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter,  'ReturnOnError', false);

%% Close the text file.
fclose(fileID);

%% Create output variable
SymbolicOutput = [dataArray{1:end-1}];

%% Clear temporary variables
clearvars filename delimiter formatSpec fileID dataArray ans;

%% Replace [row][col] brackets for nextP and P arrays with (row,col)

% replace 2-D left indexes
for rowIndex = 1:nStates
    for colIndex = 1:nStates
        strRowIndex = int2str(rowIndex-1);
        strColIndex = int2str(colIndex-1);
        strRep = sprintf('P(%d,%d)',(rowIndex-1),(colIndex-1));
        strPat = strcat('P\[',strRowIndex,'\]\[',strColIndex,'\]');
        for lineIndex = 1:length(SymbolicOutput)
            str = char(SymbolicOutput(lineIndex));
            SymbolicOutput(lineIndex) = {regexprep(str, strPat, strRep)};
        end
    end
end

%% Write to file
fileName = strcat('C_code_use_matrix_lib',int2str(nStates),'.txt');
fid = fopen(fileName,'wt');
for lineIndex = 1:length(SymbolicOutput)
    fprintf(fid,char(SymbolicOutput(lineIndex)));
    fprintf(fid,'\n');
end
fclose(fid);
clear all;