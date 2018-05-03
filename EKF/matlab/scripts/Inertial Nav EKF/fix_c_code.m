function fix_c_code(fileName)
%% Initialize variables
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

%% replace brackets and commas
for lineIndex = 1:length(SymbolicOutput)
    SymbolicOutput(lineIndex) = regexprep(SymbolicOutput(lineIndex), '_l_', '(');
    SymbolicOutput(lineIndex) = regexprep(SymbolicOutput(lineIndex), '_c_', ',');
    SymbolicOutput(lineIndex) = regexprep(SymbolicOutput(lineIndex), '_r_', ')');
end

%% Convert indexing and replace brackets

% replace 1-D indexes
for arrayIndex = 1:99
    strIndex = int2str(arrayIndex);
    strRep = sprintf('[%d]',(arrayIndex-1));
    strPat = strcat('\(',strIndex,'\)');
    for lineIndex = 1:length(SymbolicOutput)
        str = char(SymbolicOutput(lineIndex));
        SymbolicOutput(lineIndex) = {regexprep(str, strPat, strRep)};
    end
end

% replace 2-D left indexes
for arrayIndex = 1:99
    strIndex = int2str(arrayIndex);
    strRep = sprintf('[%d,',(arrayIndex-1));
    strPat = strcat('\(',strIndex,'\,');
    for lineIndex = 1:length(SymbolicOutput)
        str = char(SymbolicOutput(lineIndex));
        SymbolicOutput(lineIndex) = {regexprep(str, strPat, strRep)};
    end
end

% replace 2-D right indexes
for arrayIndex = 1:99
    strIndex = int2str(arrayIndex);
    strRep = sprintf(',%d]',(arrayIndex-1));
    strPat = strcat('\,',strIndex,'\)');
    for lineIndex = 1:length(SymbolicOutput)
        str = char(SymbolicOutput(lineIndex));
        SymbolicOutput(lineIndex) = {regexprep(str, strPat, strRep)};
    end
end

% replace commas
for lineIndex = 1:length(SymbolicOutput)
    str = char(SymbolicOutput(lineIndex));
    SymbolicOutput(lineIndex) = {regexprep(str, '\,', '][')};
end

%% Change covariance matrix variable name to P
for lineIndex = 1:length(SymbolicOutput)
    strIn = char(SymbolicOutput(lineIndex));
    strIn = regexprep(strIn,'OP\[','P[');
    SymbolicOutput(lineIndex) = cellstr(strIn);
end

%% Write to file
fid = fopen(fileName,'wt');
for lineIndex = 1:length(SymbolicOutput)
    fprintf(fid,char(SymbolicOutput(lineIndex)));
    fprintf(fid,'\n');
end
fclose(fid);
clear all;

end