function estimatorLogViewer(fname)
close all;
load(fname);

stateFieldNames = [fieldnames(estimatorData.EST0);fieldnames(estimatorData.EST1)];
stateFieldNames(strcmp('T',stateFieldNames) | strcmp('index',stateFieldNames) | ...
    strcmp('nStat',stateFieldNames) | strcmp('fNaN',stateFieldNames) | ...
    strcmp('fHealth',stateFieldNames) | strcmp('fTOut',stateFieldNames)) = [];

innovFieldNamesT = [fieldnames(estimatorData.EST4);fieldnames(estimatorData.EST5)];
innovFieldNamesT(strcmp('T',innovFieldNamesT) | strcmp('index',innovFieldNamesT)) = [];
innovFieldInds = cellfun(@(x)x(end)=='V',innovFieldNamesT);
innovFieldNames = innovFieldNamesT(~innovFieldInds);


% lay out static objects
f = figure('Units','normalized');
set(f,'Position',[.25,.25,.5,.5]);

tabs = uitabgroup(f);
statesTab = uitab(tabs,'Title','States');
innovsTab = uitab(tabs,'Title','Innovations');

axesConfig = {'Units','Normalized','Position',[.05,.1,.65,.85],'NextPlot','add'};
BGconfig = {'Units','Normalized','Position',[.75,.1,.2,.85]};
TBconfig = {'Style','Checkbox','Units','Normalized','Position',[.75,0,.2,.1],'String','Show variance'};

statesAxes = axes('Parent',statesTab,axesConfig{:},'XLabel',text('string','Time (sec)'));
innovsAxes = axes('Parent',innovsTab,axesConfig{:},'XLabel',text('string','Time (sec)'));

statesBG = uibuttongroup(statesTab,BGconfig{:});
innovsBG = uibuttongroup(innovsTab,BGconfig{:});

statesVarToggle = uicontrol(statesTab,TBconfig{:});
innovsVarToggle = uicontrol(innovsTab,TBconfig{:});

% lay out dynamic objects
stateButtons = zeros(numel(stateFieldNames),1);
for k = 1:numel(stateFieldNames)
    stateButtons(k) = uicontrol(statesBG,'Style','Checkbox','Units','normalized',...
        'String',stateFieldNames{k},'Position',[0,1 - k/numel(stateFieldNames),1, 1/numel(stateFieldNames)],...
        'Callback',@(es,ed)toggleLine(es,ed,statesAxes,statesVarToggle,estimatorData.EST0,estimatorData.EST1,estimatorData.EST2,estimatorData.EST3));
end
innovButtons = zeros(numel(innovFieldNames),1);
for k = 1:numel(innovFieldNames)
    innovButtons(k) = uicontrol(innovsBG,'Style','Checkbox','Units','normalized',...
        'String',innovFieldNames{k},'Position',[0,1 - k/numel(innovFieldNames),1, 1/numel(innovFieldNames)],...
        'Callback',@(es,ed)toggleLine(es,ed,innovsAxes,innovsVarToggle,estimatorData.EST4,estimatorData.EST5,[],[]));
end

set(statesVarToggle,'Callback',@(es,ed)toggleVariance(es,ed,statesAxes,estimatorData.EST2,estimatorData.EST3));
set(innovsVarToggle,'Callback',@(es,ed)toggleVariance(es,ed,innovsAxes,estimatorData.EST4,estimatorData.EST5));
end

function toggleLine(es,~,axes,varToggle,struct1,struct2,vstruct1,vstruct2)
if es.Value == 0
    delete(findobj(axes,'UserData',es.String));
else
    if any(strcmp(es.String,fieldnames(struct1)))
        dataSrc = struct1;
    else
        dataSrc = struct2;
    end
    p = plot(dataSrc.Tsec,dataSrc.(es.String));
    set(p,'Parent',axes,'UserData',es.String);
end
if varToggle.Value == 1
    if ~isempty(vstruct1) && ~isempty(vstruct2)
        toggleVariance(varToggle,[],axes,vstruct1,vstruct2);
    else
        toggleVariance(varToggle,[],axes,struct1,struct2);
    end
end
updateLegend(axes);
end

function toggleVariance(es,~,axes,struct1,struct2)
if es.Value == 0
    delete(findobj(axes,'Type','Patch'));
else
    lines = findobj(axes,'Type','Line');
    if isempty(lines)
        return;
    end
    stateNames = {lines.UserData};
    if any(strncmp('s',stateNames,1))
        varNames = strrep(stateNames,'s','P');
    else
        varNames = strrep(stateNames,'I','IV');
    end
    for k = 1:numel(varNames)
        if any(strcmp(varNames{k},fieldnames(struct1)))
            dataSrc = struct1;
        else
            dataSrc = struct2;
        end
        centerLineTimeFull = get(lines(k),'XData');
        centerLineDataFull = get(lines(k),'YData');

        startTime = max(centerLineTimeFull(1),dataSrc.Tsec(1));
        endTime = min(centerLineTimeFull(end),dataSrc.Tsec(end));
        plotTimeFull = dataSrc.Tsec(dataSrc.Tsec >= startTime & dataSrc.Tsec <= endTime);
        plotDataFull = dataSrc.(varNames{k})(dataSrc.Tsec >= startTime & dataSrc.Tsec <= endTime);

        centerLineTime = centerLineTimeFull(centerLineTimeFull >= startTime & centerLineTimeFull <= endTime);
        centerLineData = centerLineDataFull(centerLineTimeFull >= startTime & centerLineTimeFull <= endTime);

        plotTime = linspace(startTime,endTime,250);
        plotData = sqrt(interp1(plotTimeFull,plotDataFull,plotTime));
        centerLineData = interp1(centerLineTime,centerLineData,plotTime);

        % plotTime = downsample(centerLineTime,round(numel(plotDataT)/350),0);
        if strcmp('IV',varNames{k}(end-1:end))
            plotDataL = -plotData;
            plotDataU = plotData;
        else
            plotDataL = centerLineData-plotData;
            plotDataU = centerLineData+plotData;
        end
        p = patch([plotTime,fliplr(plotTime)],[plotDataL,fliplr(plotDataU)],lines(k).Color);
        set(p,'Parent',axes,'EdgeColor','none','FaceAlpha',.3,'UserData',stateNames{k});
    end
end
end

function updateLegend(axes)
lines = findobj(axes,'Type','Line');
if isempty(lines)
    legend(axes,'off');
else
    legend(axes,lines,{lines.UserData});
end
end