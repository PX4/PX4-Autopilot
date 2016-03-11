function SaveScriptCode(nStates)
%% Load Data
fileName = strcat('SymbolicOutput',int2str(nStates),'.mat');
load(fileName);

%% Open output file
fileName = strcat('SymbolicOutput',int2str(nStates),'.txt');
fid = fopen(fileName,'wt');

%% Write equation for state transition matrix
if exist('SF','var')
    
    fprintf(fid,'SF = zeros(%d,1);\n',numel(SF));
    for rowIndex = 1:numel(SF)
        string = char(SF(rowIndex,1));
        fprintf(fid,'SF(%d) = %s;\n',rowIndex,string);
    end
    
    % fprintf(fid,'\n');
    % fprintf(fid,'F = zeros(%d,%d);\n',nStates,nStates);
    % for rowIndex = 1:nStates
    %     for colIndex = 1:nStates
    %         string = char(F(rowIndex,colIndex));
    %         % don't write out a zero-assignment
    %         if ~strcmpi(string,'0')
    %             fprintf(fid,'F(%d,%d) = %s;\n',rowIndex,colIndex,string);
    %         end
    %     end
    % end
    % fprintf(fid,'\n');
    
end
%% Write equations for control influence (disturbance) matrix
if exist('SG','var')
    
    fprintf(fid,'\n');
    fprintf(fid,'SG = zeros(%d,1);\n',numel(SG));
    for rowIndex = 1:numel(SG)
        string = char(SG(rowIndex,1));
        fprintf(fid,'SG(%d) = %s;\n',rowIndex,string);
    end
    fprintf(fid,'\n');
    
    % fprintf(fid,'\n');
    % fprintf(fid,'G = zeros(%d,%d);\n',nStates,numel([da;dv]));
    % for rowIndex = 1:nStates
    %     for colIndex = 1:numel([da;dv])
    %         string = char(G(rowIndex,colIndex));
    %         % don't write out a zero-assignment
    %         if ~strcmpi(string,'0')
    %             fprintf(fid,'G(%d,%d) = %s;\n',rowIndex,colIndex,string);
    %         end
    %     end
    % end
    % fprintf(fid,'\n');
    
end
%% Write equations for state error matrix
if exist('SQ','var')
    
    fprintf(fid,'\n');
    fprintf(fid,'SQ = zeros(%d,1);\n',numel(SQ));
    for rowIndex = 1:numel(SQ)
        string = char(SQ(rowIndex,1));
        fprintf(fid,'SQ(%d) = %s;\n',rowIndex,string);
    end
    fprintf(fid,'\n');
    
    % fprintf(fid,'\n');
    % fprintf(fid,'Q = zeros(%d,%d);\n',nStates,nStates);
    % for rowIndex = 1:nStates
    %     for colIndex = 1:nStates
    %         string = char(Q(rowIndex,colIndex));
    %         % don't write out a zero-assignment
    %         if ~strcmpi(string,'0')
    %             fprintf(fid,'Q(%d,%d) = %s;\n',rowIndex,colIndex,string);
    %         end
    %     end
    % end
    % fprintf(fid,'\n');
    
end
%% Write equations for covariance prediction
% Only write out upper diagonal (matrix is symmetric)
if exist('SPP','var')
    
    fprintf(fid,'\n');
    fprintf(fid,'SPP = zeros(%d,1);\n',numel(SPP));
    for rowIndex = 1:numel(SPP)
        string = char(SPP(rowIndex,1));
        fprintf(fid,'SPP(%d) = %s;\n',rowIndex,string);
    end
    fprintf(fid,'\n');
    
end

if exist('PP','var')
    
    fprintf(fid,'\n');
    fprintf(fid,'nextP = zeros(%d,%d);\n',nStates,nStates);
    for colIndex = 1:nStates
        for rowIndex = 1:colIndex
            string = char(PP(rowIndex,colIndex));
            % don't write out a zero-assignment
            if ~strcmpi(string,'0')
                fprintf(fid,'nextP(%d,%d) = %s;\n',rowIndex,colIndex,string);
            end
        end
    end
    fprintf(fid,'\n');
    
end

%% Write equations for velocity and position data fusion
if exist('H_VP','var')
    
    [nRow,nCol] = size(H_VP);
    fprintf(fid,'\n');
    fprintf(fid,'H_VP = zeros(%d,%d);\n',nRow,nCol);
    for rowIndex = 1:nRow
        for colIndex = 1:nCol
            string = char(H_VP(rowIndex,colIndex));
            % don't write out a zero-assignment
            if ~strcmpi(string,'0')
                fprintf(fid,'H_VP(%d,%d) = %s;\n',rowIndex,colIndex,string);
            end
        end
    end
    fprintf(fid,'\n');
    
    [nRow,nCol] = size(SK_VP);
    fprintf(fid,'\n');
    fprintf(fid,'SK_VP = zeros(%d,%d);\n',nRow,nCol);
    for rowIndex = 1:nRow
        for colIndex = 1:nCol
            string = char(SK_VP(rowIndex,colIndex));
            % don't write out a zero-assignment
            if ~strcmpi(string,'0')
                fprintf(fid,'SK_VP(%d,%d) = %s;\n',rowIndex,colIndex,string);
            end
        end
    end
    fprintf(fid,'\n');
    
    [nRow,nCol] = size(K_VP);
    fprintf(fid,'\n');
    fprintf(fid,'Kfusion = zeros(%d,%d);\n',nRow,nCol);
    for rowIndex = 1:nRow
        for colIndex = 1:nCol
            string = char(K_VP(rowIndex,colIndex));
            % don't write out a zero-assignment
            if ~strcmpi(string,'0')
                fprintf(fid,'Kfusion(%d,%d) = %s;\n',rowIndex,colIndex,string);
            end
        end
    end
    fprintf(fid,'\n');
    
end
%% Write equations for true airspeed data fusion
if exist('SH_TAS','var')
    
    fprintf(fid,'\n');
    fprintf(fid,'SH_TAS = zeros(%d,1);\n',numel(SH_TAS));
    for rowIndex = 1:numel(SH_TAS)
        string = char(SH_TAS(rowIndex,1));
        fprintf(fid,'SH_TAS(%d) = %s;\n',rowIndex,string);
    end
    
    [nRow,nCol] = size(H_TAS);
    fprintf(fid,'\n');
    fprintf(fid,'H_TAS = zeros(1,%d);\n',nCol);
    for rowIndex = 1:nRow
        for colIndex = 1:nCol
            string = char(H_TAS(rowIndex,colIndex));
            % don't write out a zero-assignment
            if ~strcmpi(string,'0')
                fprintf(fid,'H_TAS(1,%d) = %s;\n',colIndex,string);
            end
        end
    end
    fprintf(fid,'\n');
    
    fprintf(fid,'\n');
    fprintf(fid,'SK_TAS = zeros(%d,1);\n',numel(SK_TAS));
    for rowIndex = 1:numel(SK_TAS)
        string = char(SK_TAS(rowIndex,1));
        fprintf(fid,'SK_TAS(%d) = %s;\n',rowIndex,string);
    end
    fprintf(fid,'\n');
    
    [nRow,nCol] = size(K_TAS);
    fprintf(fid,'\n');
    fprintf(fid,'Kfusion = zeros(%d,1);\n',nRow,nCol);
    for rowIndex = 1:nRow
        string = char(K_TAS(rowIndex,1));
        % don't write out a zero-assignment
        if ~strcmpi(string,'0')
            fprintf(fid,'Kfusion(%d) = %s;\n',rowIndex,string);
        end
    end
    fprintf(fid,'\n');
    
end
%% Write equations for sideslip data fusion
if exist('SH_BETA','var')
    
    fprintf(fid,'\n');
    fprintf(fid,'SH_BETA = zeros(%d,1);\n',numel(SH_BETA));
    for rowIndex = 1:numel(SH_BETA)
        string = char(SH_BETA(rowIndex,1));
        fprintf(fid,'SH_BETA(%d) = %s;\n',rowIndex,string);
    end
    
    [nRow,nCol] = size(H_BETA);
    fprintf(fid,'\n');
    fprintf(fid,'H_BETA = zeros(1,%d);\n',nCol);
    for rowIndex = 1:nRow
        for colIndex = 1:nCol
            string = char(H_BETA(rowIndex,colIndex));
            % don't write out a zero-assignment
            if ~strcmpi(string,'0')
                fprintf(fid,'H_BETA(1,%d) = %s;\n',colIndex,string);
            end
        end
    end
    fprintf(fid,'\n');
    
    fprintf(fid,'\n');
    fprintf(fid,'SK_BETA = zeros(%d,1);\n',numel(SK_BETA));
    for rowIndex = 1:numel(SK_BETA)
        string = char(SK_BETA(rowIndex,1));
        fprintf(fid,'SK_BETA(%d) = %s;\n',rowIndex,string);
    end
    fprintf(fid,'\n');
    
    [nRow,nCol] = size(K_BETA);
    fprintf(fid,'\n');
    fprintf(fid,'Kfusion = zeros(%d,1);\n',nRow,nCol);
    for rowIndex = 1:nRow
        string = char(K_BETA(rowIndex,1));
        % don't write out a zero-assignment
        if ~strcmpi(string,'0')
            fprintf(fid,'Kfusion(%d) = %s;\n',rowIndex,string);
        end
    end
    fprintf(fid,'\n');
    
end
%% Write equations for magnetometer data fusion
if exist('SH_MAG','var')
    
    fprintf(fid,'\n');
    fprintf(fid,'SH_MAG = zeros(%d,1);\n',numel(SH_MAG));
    for rowIndex = 1:numel(SH_MAG)
        string = char(SH_MAG(rowIndex,1));
        fprintf(fid,'SH_MAG(%d) = %s;\n',rowIndex,string);
    end
    fprintf(fid,'\n');
    
    [nRow,nCol] = size(H_MAG);
    fprintf(fid,'\n');
    fprintf(fid,'H_MAG = zeros(1,%d);\n',nCol);
    for colIndex = 1:nCol
        string = char(H_MAG(1,colIndex));
        % don't write out a zero-assignment
        if ~strcmpi(string,'0')
            fprintf(fid,'H_MAG(%d) = %s;\n',colIndex,string);
        end
    end
    fprintf(fid,'\n');
    
    fprintf(fid,'\n');
    fprintf(fid,'SK_MX = zeros(%d,1);\n',numel(SK_MX));
    for rowIndex = 1:numel(SK_MX)
        string = char(SK_MX(rowIndex,1));
        fprintf(fid,'SK_MX(%d) = %s;\n',rowIndex,string);
    end
    fprintf(fid,'\n');
    
    [nRow,nCol] = size(K_MX);
    fprintf(fid,'\n');
    fprintf(fid,'Kfusion = zeros(%d,1);\n',nRow,nCol);
    for rowIndex = 1:nRow
        string = char(K_MX(rowIndex,1));
        % don't write out a zero-assignment
        if ~strcmpi(string,'0')
            fprintf(fid,'Kfusion(%d) = %s;\n',rowIndex,string);
        end
    end
    fprintf(fid,'\n');
    
    [nRow,nCol] = size(H_MAG);
    fprintf(fid,'\n');
    fprintf(fid,'H_MAG = zeros(1,%d);\n',nCol);
    for colIndex = 1:nCol
        string = char(H_MAG(2,colIndex));
        % don't write out a zero-assignment
        if ~strcmpi(string,'0')
            fprintf(fid,'H_MAG(%d) = %s;\n',colIndex,string);
        end
    end
    fprintf(fid,'\n');
    
    fprintf(fid,'\n');
    fprintf(fid,'SK_MY = zeros(%d,1);\n',numel(SK_MY));
    for rowIndex = 1:numel(SK_MY)
        string = char(SK_MY(rowIndex,1));
        fprintf(fid,'SK_MY(%d) = %s;\n',rowIndex,string);
    end
    fprintf(fid,'\n');
    
    [nRow,nCol] = size(K_MY);
    fprintf(fid,'\n');
    fprintf(fid,'Kfusion = zeros(%d,1);\n',nRow,nCol);
    for rowIndex = 1:nRow
        string = char(K_MY(rowIndex,1));
        % don't write out a zero-assignment
        if ~strcmpi(string,'0')
            fprintf(fid,'Kfusion(%d) = %s;\n',rowIndex,string);
        end
    end
    fprintf(fid,'\n');
    
    [nRow,nCol] = size(H_MAG);
    fprintf(fid,'\n');
    fprintf(fid,'H_MAG = zeros(1,%d);\n',nCol);
    for colIndex = 1:nCol
        string = char(H_MAG(3,colIndex));
        % don't write out a zero-assignment
        if ~strcmpi(string,'0')
            fprintf(fid,'H_MAG(%d) = %s;\n',colIndex,string);
        end
    end
    fprintf(fid,'\n');
    
    fprintf(fid,'\n');
    fprintf(fid,'SK_MZ = zeros(%d,1);\n',numel(SK_MZ));
    for rowIndex = 1:numel(SK_MZ)
        string = char(SK_MZ(rowIndex,1));
        fprintf(fid,'SK_MZ(%d) = %s;\n',rowIndex,string);
    end
    fprintf(fid,'\n');
    
    [nRow,nCol] = size(K_MZ);
    fprintf(fid,'\n');
    fprintf(fid,'Kfusion = zeros(%d,1);\n',nRow,nCol);
    for rowIndex = 1:nRow
        string = char(K_MZ(rowIndex,1));
        % don't write out a zero-assignment
        if ~strcmpi(string,'0')
            fprintf(fid,'Kfusion(%d) = %s;\n',rowIndex,string);
        end
    end
    fprintf(fid,'\n');
    
end
%% Write equations for optical flow sensor angular LOS data fusion
if exist('SH_LOS','var')
    
    fprintf(fid,'\n');
    fprintf(fid,'SH_LOS = zeros(%d,1);\n',numel(SH_LOS));
    for rowIndex = 1:numel(SH_LOS)
        string = char(SH_LOS(rowIndex,1));
        fprintf(fid,'SH_LOS(%d) = %s;\n',rowIndex,string);
    end
    fprintf(fid,'\n');
    
    fprintf(fid,'\n');
    [nRow,nCol] = size(H_LOS);
    fprintf(fid,'\n');
    fprintf(fid,'H_LOS = zeros(1,%d);\n',nCol);
    for colIndex = 1:nCol
        string = char(H_LOS(1,colIndex));
        % don't write out a zero-assignment
        if ~strcmpi(string,'0')
            fprintf(fid,'H_LOS(%d) = %s;\n',colIndex,string);
        end
    end
    fprintf(fid,'\n');
    
    fprintf(fid,'\n');
    [nRow,nCol] = size(H_LOS);
    fprintf(fid,'\n');
    fprintf(fid,'H_LOS = zeros(1,%d);\n',nCol);
    for colIndex = 1:nCol
        string = char(H_LOS(2,colIndex));
        % don't write out a zero-assignment
        if ~strcmpi(string,'0')
            fprintf(fid,'H_LOS(%d) = %s;\n',colIndex,string);
        end
    end
    
%     fprintf(fid,'\n');
%     fprintf(fid,'SKK_LOS = zeros(%d,1);\n',numel(SKK_LOS));
%     for rowIndex = 1:numel(SKK_LOS)
%         string = char(SKK_LOS(rowIndex,1));
%         fprintf(fid,'SKK_LOS(%d) = %s;\n',rowIndex,string);
%     end
    
    fprintf(fid,'\n');
    fprintf(fid,'SK_LOS = zeros(%d,1);\n',numel(SK_LOS));
    for rowIndex = 1:numel(SK_LOS)
        string = char(SK_LOS(rowIndex,1));
        fprintf(fid,'SK_LOS(%d) = %s;\n',rowIndex,string);
    end
    
    [nRow,nCol] = size(K_LOSX);
    fprintf(fid,'\n');
    fprintf(fid,'Kfusion = zeros(%d,1);\n',nRow,nCol);
    for rowIndex = 1:nRow
        string = char(K_LOSX(rowIndex));
        % don't write out a zero-assignment
        if ~strcmpi(string,'0')
            fprintf(fid,'Kfusion(%d) = %s;\n',rowIndex,string);
        end
    end
    fprintf(fid,'\n');
    
    [nRow,nCol] = size(K_LOSY);
    fprintf(fid,'\n');
    fprintf(fid,'Kfusion = zeros(%d,1);\n',nRow,nCol);
    for rowIndex = 1:nRow
        string = char(K_LOSY(rowIndex));
        % don't write out a zero-assignment
        if ~strcmpi(string,'0')
            fprintf(fid,'Kfusion(%d) = %s;\n',rowIndex,string);
        end
    end
    
end
%% Write observation fusion equations for optical flow sensor scale factor error estimation
if exist('SH_OPT','var')
    
    fprintf(fid,'\n');
    for rowIndex = 1:numel(SH_OPT)
        string = char(SH_OPT(rowIndex,1));
        fprintf(fid,'SH_OPT(%d) = %s;\n',rowIndex,string);
    end
    fprintf(fid,'\n');
    
    fprintf(fid,'\n');
        string = char(H_OPT(1));
        % don't write out a zero-assignment
        if ~strcmpi(string,'0')
            fprintf(fid,'H_OPT(1) = %s;\n',1,string);
        end
    fprintf(fid,'\n');
    
    fprintf(fid,'\n');
        string = char(H_OPT(2));
        % don't write out a zero-assignment
        if ~strcmpi(string,'0')
            fprintf(fid,'H_OPT(2) = %s;\n',1,string);
        end
    fprintf(fid,'\n');
    
    fprintf(fid,'\n');
    for rowIndex = 1:numel(SK_OPT)
        string = char(SK_OPT(rowIndex,1));
        fprintf(fid,'SK_OPT(%d) = %s;\n',rowIndex,string);
    end
    
    fprintf(fid,'\n');
        string = char(K_OPT(1));
        % don't write out a zero-assignment
        if ~strcmpi(string,'0')
            fprintf(fid,'K_OPT(1) = %s;\n',1,string);
        end
    fprintf(fid,'\n');
    
    fprintf(fid,'\n');
        string = char(K_OPT(2));
        % don't write out a zero-assignment
        if ~strcmpi(string,'0')
            fprintf(fid,'K_OPT(2) = %s;\n',1,string);
        end
    fprintf(fid,'\n');
    
end
%% Write equations for laser range finder data fusion
if exist('SH_RNG','var')
    
    fprintf(fid,'\n');
    fprintf(fid,'SH_RNG = zeros(%d,1);\n',numel(SH_RNG));
    for rowIndex = 1:numel(SH_RNG)
        string = char(SH_RNG(rowIndex,1));
        fprintf(fid,'SH_RNG(%d) = %s;\n',rowIndex,string);
    end
    
    [nRow,nCol] = size(H_RNG);
    fprintf(fid,'\n');
    fprintf(fid,'H_RNG = zeros(1,%d);\n',nCol);
    for rowIndex = 1:nRow
        for colIndex = 1:nCol
            string = char(H_RNG(rowIndex,colIndex));
            % don't write out a zero-assignment
            if ~strcmpi(string,'0')
                fprintf(fid,'H_RNG(1,%d) = %s;\n',colIndex,string);
            end
        end
    end
    fprintf(fid,'\n');
    
    fprintf(fid,'\n');
    fprintf(fid,'SK_RNG = zeros(%d,1);\n',numel(SK_RNG));
    for rowIndex = 1:numel(SK_RNG)
        string = char(SK_RNG(rowIndex,1));
        fprintf(fid,'SK_RNG(%d) = %s;\n',rowIndex,string);
    end
    fprintf(fid,'\n');
    
    [nRow,nCol] = size(K_RNG);
    fprintf(fid,'\n');
    fprintf(fid,'Kfusion = zeros(%d,1);\n',nRow,nCol);
    for rowIndex = 1:nRow
        string = char(K_RNG(rowIndex,1));
        % don't write out a zero-assignment
        if ~strcmpi(string,'0')
            fprintf(fid,'Kfusion(%d) = %s;\n',rowIndex,string);
        end
    end
    fprintf(fid,'\n');
    
end

%% Write equations for simple magnetomter data fusion
if exist('SH_MAGS','var')
    
    fprintf(fid,'\n');
    fprintf(fid,'SH_MAGS = zeros(%d,1);\n',numel(SH_MAGS));
    for rowIndex = 1:numel(SH_MAGS)
        string = char(SH_MAGS(rowIndex,1));
        fprintf(fid,'SH_MAGS(%d) = %s;\n',rowIndex,string);
    end
    fprintf(fid,'\n');
    
    [nRow,nCol] = size(H_MAGS);
    fprintf(fid,'\n');
    fprintf(fid,'H_MAGS = zeros(1,%d);\n',nCol);
    for colIndex = 1:nCol
        string = char(H_MAGS(1,colIndex));
        % don't write out a zero-assignment
        if ~strcmpi(string,'0')
            fprintf(fid,'H_MAGS(%d) = %s;\n',colIndex,string);
        end
    end
    fprintf(fid,'\n');
        
    fprintf(fid,'\n');
    fprintf(fid,'SK_MAGS = zeros(%d,1);\n',numel(SK_MAGS));
    for rowIndex = 1:numel(SK_MAGS)
        string = char(SK_MAGS(rowIndex,1));
        fprintf(fid,'SK_MAGS(%d) = %s;\n',rowIndex,string);
    end
    fprintf(fid,'\n');
    
    [nRow,nCol] = size(K_MAGS);
    fprintf(fid,'\n');
    fprintf(fid,'Kfusion = zeros(%d,1);\n',nRow,nCol);
    for rowIndex = 1:nRow
        string = char(K_MAGS(rowIndex,1));
        % don't write out a zero-assignment
        if ~strcmpi(string,'0')
            fprintf(fid,'Kfusion(%d) = %s;\n',rowIndex,string);
        end
    end
    fprintf(fid,'\n');
end

%% Write equations for X accel fusion
if exist('SH_ACCX','var')
    
    fprintf(fid,'\n');
    fprintf(fid,'SH_ACCX = zeros(%d,1);\n',numel(SH_ACCX));
    for rowIndex = 1:numel(SH_ACCX)
        string = char(SH_ACCX(rowIndex,1));
        fprintf(fid,'SH_ACCX(%d) = %s;\n',rowIndex,string);
    end
    
    [nRow,nCol] = size(H_ACCX);
    fprintf(fid,'\n');
    fprintf(fid,'H_ACCX = zeros(1,%d);\n',nCol);
    for rowIndex = 1:nRow
        for colIndex = 1:nCol
            string = char(H_ACCX(rowIndex,colIndex));
            % don't write out a zero-assignment
            if ~strcmpi(string,'0')
                fprintf(fid,'H_ACCX(1,%d) = %s;\n',colIndex,string);
            end
        end
    end
    fprintf(fid,'\n');
    
    fprintf(fid,'\n');
    fprintf(fid,'SK_ACCX = zeros(%d,1);\n',numel(SK_ACCX));
    for rowIndex = 1:numel(SK_ACCX)
        string = char(SK_ACCX(rowIndex,1));
        fprintf(fid,'SK_ACCX(%d) = %s;\n',rowIndex,string);
    end
    fprintf(fid,'\n');
    
    [nRow,nCol] = size(K_ACCX);
    fprintf(fid,'\n');
    fprintf(fid,'Kfusion = zeros(%d,1);\n',nRow,nCol);
    for rowIndex = 1:nRow
        string = char(K_ACCX(rowIndex,1));
        % don't write out a zero-assignment
        if ~strcmpi(string,'0')
            fprintf(fid,'Kfusion(%d) = %s;\n',rowIndex,string);
        end
    end
    fprintf(fid,'\n');
    
end

%% Write equations for Y accel fusion
if exist('SH_ACCY','var')
    
    fprintf(fid,'\n');
    fprintf(fid,'SH_ACCY = zeros(%d,1);\n',numel(SH_ACCY));
    for rowIndex = 1:numel(SH_ACCY)
        string = char(SH_ACCY(rowIndex,1));
        fprintf(fid,'SH_ACCY(%d) = %s;\n',rowIndex,string);
    end
    
    [nRow,nCol] = size(H_ACCY);
    fprintf(fid,'\n');
    fprintf(fid,'H_ACCY = zeros(1,%d);\n',nCol);
    for rowIndex = 1:nRow
        for colIndex = 1:nCol
            string = char(H_ACCY(rowIndex,colIndex));
            % don't write out a zero-assignment
            if ~strcmpi(string,'0')
                fprintf(fid,'H_ACCY(1,%d) = %s;\n',colIndex,string);
            end
        end
    end
    fprintf(fid,'\n');
    
    fprintf(fid,'\n');
    fprintf(fid,'SK_ACCY = zeros(%d,1);\n',numel(SK_ACCY));
    for rowIndex = 1:numel(SK_ACCY)
        string = char(SK_ACCY(rowIndex,1));
        fprintf(fid,'SK_ACCY(%d) = %s;\n',rowIndex,string);
    end
    fprintf(fid,'\n');
    
    [nRow,nCol] = size(K_ACCY);
    fprintf(fid,'\n');
    fprintf(fid,'Kfusion = zeros(%d,1);\n',nRow,nCol);
    for rowIndex = 1:nRow
        string = char(K_ACCY(rowIndex,1));
        % don't write out a zero-assignment
        if ~strcmpi(string,'0')
            fprintf(fid,'Kfusion(%d) = %s;\n',rowIndex,string);
        end
    end
    fprintf(fid,'\n');
    
end

%% Close output file
fclose(fid);

end
