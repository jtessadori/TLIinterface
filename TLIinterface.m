classdef TLIinterface < handle
    %% December, 2018 Jacopo Tessadori
    %% Based on: https://www.frontiersin.org/articles/10.3389/fnins.2017.00251/full
    properties
        portToInterface;
        portFromInterface;
        portFromET;
        UDPtoInterface;
        UDPfromInterface;
        UDPfromET;
        intercomTimer;
        trialEventTimer;
        EPstretchTimer;
        EPdataLogger;
        screenRes;
        fileName;
        fs;
        outputLog;
        listener;
        currTime;
        motorSerialPort;
        EPparams;
        figureParams;
        targetCharacter;
        targetPos;
        finalEst;
        finalLbls;
        selectionTree;
        selLevel;
        phrase;
        stimType;
        EPsequence;
        EPdata;
        currStim;
        classifier;
        stretchScore;
    end
    
    properties (Hidden)
        modelName='TLIinterface_SimulinkSupport';
        instanceName;
        lastDataBlock;
        timingParams;
        isCalibrating=0;
        isFreeWriting=0;
        retry=0;
        RS;
        colorScheme;
        isDebugging;
        targetCharacterIdx;
        selHistory=[];
        stimNext=1;
    end
    
    methods
        %% Constructor
        function obj=TLIinterface(varargin)
            % Ports from and toward interface
            obj.portFromInterface=8051;
            obj.portToInterface=9000;
            
            % Port from eye tracker
            obj.portFromET=11000;
        end
        
        %% Other methods
        function initialize(obj)
            % I need to make sure that no other instances of this class are
            % present in base workspace, otherwise I will not be able to
            % make Simulink create a listener in the correct instance
            varList=evalin('base','whos');
            nCorrectClass=0;
            for currVar=1:length(varList)
                if strcmp(varList(currVar).class,mfilename)
                    nCorrectClass=nCorrectClass+1;
                    obj.instanceName=varList(currVar).name;
                    if nCorrectClass>1
                        error('Please remove from workspace other variables of this class before starting experiment. Thanks!')
                    end
                end
            end
            
            % If there are leftover COM ports in use, close them
            S=instrfind;
            if ~isempty(S)
                fclose(S);
                delete(S);
            end
            clear S
            
            obj.isDebugging=0;
            if obj.isDebugging
                fprintf('Warning: running in debug mode\n');
                obj.timingParams.restLength=.5;
                obj.timingParams.interTrialLength=.25;
            else
                % Define timing parameters common to experiment and calibration
                obj.timingParams.restLength=.5;
                obj.timingParams.interTrialLength=.75;
                obj.EPparams.stimsPerStretch=120;
                obj.EPparams.stimsPerType=30;
                obj.EPparams.interStimLength=0.2;
                obj.EPparams.stimLength=.1;
                obj.EPparams.stimTypes=3;
            end
                        
            % Determine name to be used to save file upon closing
            obj.fileName=datestr(now,30);

            % Prepares serial port for vibrating motor control
            obj.prepareSerialPort;
            
            % Start Simulink model
            obj.prepareSimulinkModel;
            
            % Perform a countdown to allow amplifier to settle
            obj.startCountdown(30);
            
            % Define interface object colors
            obj.colorScheme.bg=[.05,.05,.05];
            obj.colorScheme.textColor=[0,.4,0];
            obj.colorScheme.edgeColor=[.4,.4,.4];
            obj.colorScheme.targetColor=[.4,0,.1];
            obj.colorScheme.cursorColorMI=[0,.4,0];
            obj.colorScheme.cursorColorRest=[.6,.6,0];
            
            % Define cursor square
            obj.figureParams.squareSide=obj.screenRes(1)/15;
            obj.figureParams.squareVertexX=[-1,1,1,-1]*obj.figureParams.squareSide/2;
            obj.figureParams.squareVertexY=[-1,-1,1,1]*obj.figureParams.squareSide/2;
                        
            % Create figure
            obj.figureParams.handle=gcf;
            set(obj.figureParams.handle,'Tag',mfilename,...
                'Toolbar','none',...
                'MenuBar','none',...
                'Units','pixels',...
                'Resize','off',...
                'NumberTitle','off',...
                'Name','',...
                'Color',obj.colorScheme.bg,...
                'RendererMode','Manual',...
                'Renderer','OpenGL',...
                'WindowKeyPressFcn',@obj.KeyPressed,...
                'WindowButtonMotionFcn',@TLIinterface.onMouseMove);

            % Resize figure, then remove figure axis
            Pix_SS = get(0,'screensize');
            set(gcf,'position',Pix_SS);
            axis([0 Pix_SS(3) 0 Pix_SS(4)])
            set(gca,'YDir','reverse');
            axis('off')
            
            % Generate standard selection tree
            obj.selectionTree=TLIinterface.generateSelectionTree;
            
            % Generate text objects
            obj.figureParams.currentPhrase=text(obj.screenRes(1)*.5,obj.screenRes(2)*.3,'','Color',obj.colorScheme.textColor,'FontSize',48,'HorizontalAlignment','Center');
            obj.figureParams.letters(1)=text(obj.screenRes(1)*.3,obj.screenRes(2)*.5,'','Color',obj.colorScheme.textColor,'FontSize',48,'HorizontalAlignment','Center');
            obj.figureParams.letters(2)=text(obj.screenRes(1)*.7,obj.screenRes(2)*.5,'','Color',obj.colorScheme.textColor,'FontSize',48,'HorizontalAlignment','Center');
            
            % Add cursor square
            obj.figureParams.cursorHandle=patch(obj.screenRes(1)/2+obj.figureParams.squareVertexX,obj.screenRes(2)/2+obj.figureParams.squareVertexY,obj.colorScheme.bg,'EdgeColor',obj.colorScheme.edgeColor);
        end
                
        function closeActivity(obj)
            % Stop running timer
            stop(obj.trialEventTimer);
            pause(1);
            
            % Stop band vibration
            if isvalid(obj.motorSerialPort)
                fprintf(obj.motorSerialPort,'e8\n');
                pause(0.003)
                fprintf(obj.motorSerialPort,'p\n');
                pause(0.003)
                fprintf(obj.motorSerialPort,'r0\n');
                pause(0.003)
                fprintf(obj.motorSerialPort,'e4\n');
                pause(0.003)
                fprintf(obj.motorSerialPort,'p\n');
                pause(0.003)
                fprintf(obj.motorSerialPort,'r0\n');
                fclose(obj.motorSerialPort);
                delete(obj.motorSerialPort);
            end
            
            % Close figure
            delete(gcf);
            
            % Stop Simulink model
            obj.listener=[];
            set_param(obj.modelName,'SimulationCommand','Stop');
            set_param(obj.modelName,'StartFcn','');
        end
                
        function startCalibration(obj)
            % Initialize devices
            obj.isCalibrating=1;
            obj.initialize;
            
            % Begin actual calibration
            obj.beginSelection;
        end
         
        function stopCalibration(obj)
            obj.closeActivity;
        end
        
        function startExperiment(obj)
            % Assuming here that experiment is started on a training
            % session recording
            obj.isFreeWriting=1;
            obj.startTesting;
        end
        
        function startTesting(obj)
            % Assuming here that experiment is started on a training
            % session recording
            obj.isCalibrating=0;
            obj.trainOfflineClassifier;
            obj.outputLog=[];
            obj.phrase=[];
            obj.initialize;
            
            % Begin actual experiment
            obj.beginSelection;
        end
        
        function beginSelection(obj)
            try                
                if ~obj.isFreeWriting
                    % Decide new target character
                    obj.targetCharacterIdx=ceil(rand*length(obj.selectionTree.entries));
%                     obj.targetCharacterIdx=1;
                    obj.targetCharacter=obj.selectionTree.entries(obj.targetCharacterIdx);
                    
                    % Log data
                    obj.logEntry({'targetCharacter','targetCharacterIdx'})
                end
                
                % Set current selection level to 1, clear selection history
                obj.selHistory=[];
                obj.selLevel=1;
                
                % Move on to perform first selection
                obj.beginTrial;
            catch ME
                fprintf('%s Line: %d\n',ME.message,ME.stack(1).line)
                keyboard;
            end
        end
                
        function beginTrial(obj,~,~)
            try 
                if ~obj.isFreeWriting
                    % Highlight background of correct choice
                    obj.targetPos=obj.selectionTree.mat(obj.selLevel,obj.targetCharacterIdx);
                    set(obj.figureParams.letters(:),'EdgeColor',obj.colorScheme.bg);
                    set(obj.figureParams.letters(obj.targetPos),'EdgeColor',obj.colorScheme.targetColor);
                end
                
                % Set content of left and right text boxes
                tempMat=obj.selectionTree.mat;
                if obj.selLevel>1
                    tempMat(:,prod(obj.selectionTree.mat(1:obj.selLevel-1,:)==repmat(obj.selHistory,1,obj.selectionTree.nEntries),1)==0)=0;
                    % If one text box is empty and the other one contains
                    % only one element, immediately select it
                    if numel(obj.selectionTree.entries(tempMat(obj.selLevel,:)==1))==0||numel(obj.selectionTree.entries(tempMat(obj.selLevel,:)==2))==0
                        stop(obj.trialEventTimer);
                        obj.endSelection;
                        return;
                    end
                end
                s1=obj.selectionTree.entries(tempMat(obj.selLevel,:)==1);
                s2=obj.selectionTree.entries(tempMat(obj.selLevel,:)==2);
                if max(length(s1),length(s2))<=5
                    set(obj.figureParams.letters(1),'String',s1,'BackgroundColor',obj.colorScheme.bg);
                    set(obj.figureParams.letters(2),'String',s2,'BackgroundColor',obj.colorScheme.bg);
                else
                    set(obj.figureParams.letters(1),'String',sprintf('%s...%s',s1{1},s1{end}),'BackgroundColor',obj.colorScheme.bg);
                    set(obj.figureParams.letters(2),'String',sprintf('%s...%s',s2{1},s2{end}),'BackgroundColor',obj.colorScheme.bg);
                end
                
                % Start timer for next event
                obj.trialEventTimer=timer;
                obj.trialEventTimer.StartDelay=obj.timingParams.restLength;
                obj.trialEventTimer.TimerFcn=@obj.performIteration;
                start(obj.trialEventTimer);
                
                % Log trial data
                obj.logTime('trialStartTimes');
            catch ME
                fprintf('%s Line: %d\n',ME.message,ME.stack(1).line)
                keyboard;
            end
        end
        
        function performIteration(obj,~,~)
            try
                % Update iteration counter, change cursor color to signal
                % beginning of EP stretch, generate sequence of stimuli
                obj.logEntry({'selLevel','targetPos'});
                set(obj.figureParams.cursorHandle,'FaceColor',obj.colorScheme.cursorColorMI);
                obj.EPsequence=zeros(obj.EPparams.stimsPerStretch,1);
                stimIdx=randperm(obj.EPparams.stimsPerStretch);
                obj.EPsequence(stimIdx(1:obj.EPparams.stimsPerType))=1;
                switch obj.EPparams.stimTypes
                    case 2
                        obj.EPsequence=obj.EPsequence+1;
                    case 3
                        obj.EPsequence(stimIdx(obj.EPparams.stimsPerType+1:2*obj.EPparams.stimsPerType))=2;
                end
                obj.currStim=1;
                
                % Start timer for EP stretch
                obj.trialEventTimer=timer;
                obj.trialEventTimer.TimerFcn=@obj.updateEPstretch;
                obj.trialEventTimer.StartDelay=obj.EPparams.interStimLength;
                start(obj.trialEventTimer);
                
                % Log trial data
                obj.logTime('iterationTimes');
            catch ME
                fprintf('%s Line: %d\n',ME.message,ME.stack(1).line)
                keyboard;
            end
        end
        
        function updateEPstretch(obj,~,~)
            try
                % Generate new timer event, then either deliver stim or stop
                % it. If all stims have been delivered, move on to next phase
                obj.trialEventTimer=timer;
                obj.trialEventTimer.TimerFcn=@obj.updateEPstretch;
                if obj.stimNext
                    if obj.currStim<=length(obj.EPsequence)
                        obj.stimType=obj.EPsequence(obj.currStim);
                    else
                        r=rand;
                        obj.stimType=(r<0.125)+(r<0.25);
                    end
                    stimStrength=120;
                    switch obj.stimType
                        case 0
                            fprintf(obj.motorSerialPort,'e2\n');
                        case 1
                            fprintf(obj.motorSerialPort,sprintf('e%d\n',4+(obj.targetPos~=1)*4));
                        case 2
                            fprintf(obj.motorSerialPort,sprintf('e%d\n',4+(obj.targetPos==1)*4));
                    end
                    pause(0.05);
                    fprintf(obj.motorSerialPort,sprintf('r%d\n',stimStrength));
                    obj.logEntry({'stimType'});
                    obj.logTime('stimStart');
                    obj.stimNext=0;
                    obj.currStim=obj.currStim+1;
                    obj.trialEventTimer.StartDelay=obj.EPparams.stimLength;
                    
%                     % Start timer to log data
%                     obj.EPdataLogger=timer;
%                     obj.EPdataLogger.TimerFcn=@obj.logEPdata;
%                     obj.EPdataLogger.StartDelay=0.8;
%                     start(obj.EPdataLogger);
                else
                    % Stop tactile stimulation
                    obj.stimNext=1;
                    fprintf(obj.motorSerialPort,'e14\n');
                    pause(0.05);
                    fprintf(obj.motorSerialPort,'r0\n');
                    if obj.currStim>length(obj.EPsequence)
                        if obj.isCalibrating
                            obj.trialEventTimer.TimerFcn=@obj.endTrial;
                        elseif obj.stimType>0
                            % Recover relevant stimuli delivered since last
                            % trial start, average them, then provide a
                            % prediction
                            stimSinceTrialStartIdx=obj.outputLog.stimStart>obj.outputLog.iterationTimes(end);
                            cl1median=median(obj.outputLog.EPdata(logical(stimSinceTrialStartIdx.*(obj.outputLog.stimType==1)),:,:));
                            cl2median=median(obj.outputLog.EPdata(logical(stimSinceTrialStartIdx.*(obj.outputLog.stimType==2)),:,:));
                            [trialEst,trialScore]=TSclassifier.timePredict(obj.classifier,cat(1,cl1median,cl2median));
                            obj.stretchScore=diff(real(trialScore));
                            obj.logEntry({'stretchScore'});
                            obj.logTime('scoreEstTimes');
                            fprintf('%d - %d\n',sum(stimSinceTrialStartIdx.*(obj.outputLog.stimType==1)),sum(stimSinceTrialStartIdx.*(obj.outputLog.stimType==2)));
                            fprintf('%0.2f\n',diff(real(trialScore)));
                            fprintf('%0.2f\n',obj.classifier.scoreProbModel(sum(obj.outputLog.stretchScore(obj.outputLog.scoreEstTimes>obj.outputLog.iterationTimes(end)))));
                            if abs(.5-obj.classifier.scoreProbModel(sum(obj.outputLog.stretchScore(obj.outputLog.scoreEstTimes>obj.outputLog.iterationTimes(end)))))>.4
                                obj.finalEst=trialEst(1);
                                obj.trialEventTimer.TimerFcn=@obj.endTrial;
                            end
                        end
                    end
                    obj.trialEventTimer.StartDelay=obj.EPparams.interStimLength;
                end
                start(obj.trialEventTimer);
            catch ME
                fprintf('%s Line: %d\n',ME.message,ME.stack(1).line)
                keyboard;
            end
        end
        
        function logEPdata(obj,~,~)
            % Log time data around last stim
            obj.EPdata=obj.lastDataBlock(1,end-obj.fs+1:end,:);
            obj.logEntry({'EPdata'});
        end
                                        
        function endTrial(obj,~,~)
            try
                % If not free writing, log labels
                if ~obj.isFreeWriting
                    obj.finalLbls=obj.targetPos;
                    obj.logEntry({'finalLbls'});
                end
                if obj.isCalibrating
                    % If calibrating, actual label does not count
                    obj.finalEst=obj.targetPos;
                    obj.logEntry({'finalEst'});
                elseif ~obj.isFreeWriting
                    % If in testing mode, log estimation, then overwrite it
                    % with correct choice
                    obj.logEntry({'finalEst'});
                    obj.finalEst=obj.targetPos;
                else
                    % In free writing mode, log estimation and deal with it
                    obj.logEntry({'finalEst'});
                end
                
                % Update selection history so far
                obj.selHistory=cat(1,obj.selHistory,obj.finalEst);
                
                % Provide feedback on final trial estimation
                set(obj.figureParams.cursorHandle,'FaceColor',obj.colorScheme.bg,'EdgeColor',obj.colorScheme.edgeColor);
                set(obj.figureParams.letters(obj.finalEst),'BackgroundColor',obj.colorScheme.targetColor,'EdgeColor',obj.colorScheme.bg);
                
                % Remove edge from target letters
                set(obj.figureParams.letters(obj.targetPos),'EdgeColor',obj.colorScheme.bg);
                drawnow;
                
%                 % If large enough number of iterations have occurred
%                 if obj.isCalibrating&&length(obj.outputLog.movCorrect)>=400
%                     fprintf('Target accuracy reached, shutting down.\n');
%                     obj.stopCalibration;
%                     return;
%                 else
%                     fprintf('%d/400\n',length(obj.outputLog.movCorrect));
%                 end
                
                % Update selection counter. If last level is reached, end
                % current selection
                obj.trialEventTimer=timer;
                obj.trialEventTimer.StartDelay=obj.timingParams.interTrialLength;
                if obj.selLevel==obj.selectionTree.nSplits
                    obj.trialEventTimer.TimerFcn=@obj.endSelection;
                else
                    obj.selLevel=obj.selLevel+1;
                    obj.trialEventTimer.TimerFcn=@obj.beginTrial;
                end
                
                % Wait before starting new trial
                start(obj.trialEventTimer);
                
                % Log trial data
                obj.logTime('trialEndTimes');
            catch ME
                fprintf('%s Line: %d\n',ME.message,ME.stack(1).line)
                keyboard;
            end
        end
        
        function endSelection(obj,~,~)
            try
                % Add selected character to phrase and update display
                obj.phrase=cat(2,obj.phrase,obj.selectionTree.entries{prod(obj.selectionTree.mat(1:length(obj.selHistory),:)==repmat(obj.selHistory,1,obj.selectionTree.nEntries))==1});
                set(obj.figureParams.currentPhrase,'String',obj.phrase);
                
                % Begin new selection
                obj.beginSelection;
            catch ME
                fprintf('%s Line: %d\n',ME.message,ME.stack(1).line)
                keyboard;
            end
        end
                
        function intercomHandling(obj,~,~)
            try
                % Recover incoming data. Stop execution if no data is present
                inString=obj.UDPfromInterface.step;
                if isempty(inString)&&~obj.retry
                    return;
                end
                
                % When input from interface is received, start a new trial
                fprintf('instring: %s\n',inString);
                obj.beginTrial;
            catch ME
                fprintf('%s Line: %d\n',ME.message,ME.stack(1).line)
                keyboard;
            end
        end
        
        function prepareSimulinkModel(obj)
            % Check whether simulink model file can be found
            if ~exist(obj.modelName,'file')
                warning('Cannot find model %s.\nPress Enter to continue.\n',obj.modelName);
                input('');
                [fName,pathName]=uigetfile('*.slx','Select Simulink model to load:');
                obj.modelName=sprintf('%s\\%s',pathName,fName);
            end
            % Load model
            load_system(obj.modelName);
            
            % Check whether simulation was already running, and, in case,
            % stop it
            if bdIsLoaded(obj.modelName)&&strcmp(get_param(obj.modelName,'SimulationStatus'),'running')
                set_param(obj.modelName,'SimulationCommand','Stop');
            end
            
            % Add event listener to triggered buffer event.
            set_param(obj.modelName,'StartFcn',sprintf('%s.simulinkModelStartFcn(''%s'',''%s'')',mfilename,obj.modelName,obj.instanceName))
            set_param(obj.modelName,'StopFcn',sprintf('%s.simulinkModelStopFcn(''%s'',''%s'')',mfilename,obj.instanceName,obj.fileName))
            set_param(obj.modelName,'StopTime','inf');
            set_param(obj.modelName,'FixedStep',['1/',num2str(obj.fs)]);
            set_param(obj.modelName,'SimulationCommand','Start');
        end
        
        function newDataAvailable(obj,block,~)
            obj.lastDataBlock(1,:,:)=block.OutputPort(1).Data;
        end
        
        function prepareSerialPort(obj)
            obj.motorSerialPort=serial('COM9','BaudRate',230400,'Parity','even');
            try
                fopen(obj.motorSerialPort);
                pause(1);
                fprintf(obj.motorSerialPort,'e4\n');
                pause(0.3);
                fprintf(obj.motorSerialPort,'e8\n');
                pause(0.3);
                fprintf(obj.motorSerialPort,'e2\n');
                pause(0.3);
            catch
                warning('Unable to open serial port communication with band motors');
            end
        end
        
        function KeyPressed(obj,~,eventdata,~)
            % This is called each time a keyboard key is pressed while the mouse cursor
            % is within the window figure area
            if strcmp(eventdata.Key,'escape')
                obj.stopCalibration;
            end
        end
        
        function startCountdown(obj,nSecs)
            % countdown to experiment start
            figure;
            obj.figureParams.handle=gcf;
            set(obj.figureParams.handle,'Tag',mfilename,...
                'Toolbar','none',...
                'MenuBar','none',...
                'Units','pixels',...
                'Resize','off',...
                'NumberTitle','off',...
                'Name','',...
                'Color',[.7,.7,.7],...
                'RendererMode','Manual',...
                'Renderer','OpenGL',...
                'WindowKeyPressFcn',@obj.KeyPressed,...
                'WindowButtonMotionFcn',@TLIinterface.onMouseMove);
            
            % Resize figure, then remove figure axis
            Pix_SS = get(0,'screensize');
            set(gcf,'position',Pix_SS);
            axis([0 Pix_SS(3) 0 Pix_SS(4)])
            set(gca,'YDir','reverse');
            axis('off')
            
            for cntDown=nSecs:-1:1
                if ~exist('textHandle','var')
                    textHandle=text(obj.screenRes(1)*.5,obj.screenRes(2)*.3,num2str(cntDown),'FontSize',64,'HorizontalAlignment','center');
                else
                    if ~ishghandle(textHandle) %i.e. figure was closed during countdown
                        return
                    else
                        set(textHandle,'String',num2str(cntDown));
                    end
                end
                pause(1);
            end
            delete(textHandle);
            close(obj.figureParams.handle);
        end
        
        function logEntry(obj,fieldNames)
            try
                for currField=1:length(fieldNames)
                    if isempty(obj.outputLog)||~isfield(obj.outputLog,fieldNames{currField})
                        if ndims(obj.(fieldNames{currField}))==2 %#ok<ISMAT>
                            obj.outputLog.(fieldNames{currField})=reshape(obj.(fieldNames{currField}),1,size(obj.(fieldNames{currField}),1),size(obj.(fieldNames{currField}),2));
                        else
                            obj.outputLog.(fieldNames{currField})=obj.(fieldNames{currField});
                        end
                    else
                        if ndims(obj.(fieldNames{currField}))==2 %#ok<ISMAT>
                            obj.outputLog.(fieldNames{currField})=cat(1,obj.outputLog.(fieldNames{currField}),reshape(obj.(fieldNames{currField}),1,size(obj.(fieldNames{currField}),1),size(obj.(fieldNames{currField}),2)));
                        else
                            obj.outputLog.(fieldNames{currField})=cat(1,obj.outputLog.(fieldNames{currField}),obj.(fieldNames{currField}));
                        end
                    end
                end
            catch ME
                fprintf('%s Line: %d\n',ME.message,ME.stack(1).line)
                keyboard;
            end
        end
        
        function logTime(obj,currEventName)
            if isfield(obj.outputLog,currEventName)
                obj.outputLog.(currEventName)=cat(1,obj.outputLog.(currEventName),obj.currTime);
            else
                obj.outputLog.(currEventName)=obj.currTime;
            end
        end
        
        %% Plotting and analysis functions
        
        function [lbls,data]=averageEPs(obj)
            % Frequency filters data
            normalize=@(x)(x-repmat(mean(x),size(x,1),1))./repmat(1.4826*mad(x,1),size(x,1),1);
            [B,A]=cheby1(2,6,[.1,10]/(obj.fs/2));
%             fltrdData=filter(B,A,obj.outputLog.rawData);
            fltrdData=normalize(filter(B,A,obj.outputLog.rawData));
            fltrdData=resample(fltrdData,20,obj.fs);
            newFs=20;
%             newFs=512;
            fltrdData=TLIinterface.applyLapFilter(fltrdData);
            
            % Recover relevant windows
            winLims=[-.2,.8];
%             winLims=[.3,.6];
            winIdxs=round(winLims*newFs);
            lbls=obj.outputLog.stimType;
            data=zeros(length(lbls),winIdxs(2)-winIdxs(1),size(obj.outputLog.rawData,2));
            for currTrial=1:length(obj.outputLog.stimStart)
                data(currTrial,:,:)=reshape(fltrdData(round(obj.outputLog.stimStart(currTrial)*newFs)+winIdxs(1)+1:round(obj.outputLog.stimStart(currTrial)*newFs)+winIdxs(2),:),1,floor((winLims(2)-winLims(1))*newFs),[]);
%                 data(currTrial,:,:)=data(currTrial,:,:)-repmat(mean(data(currTrial,1:round(-winLims(1)*newFs),:),2),1,size(data,2),1);
                data(currTrial,:,:)=data(currTrial,:,:)-repmat(mean(data(currTrial,:,:),2),1,size(data,2),1);
            end
        end
        
        function [ceLbls,feats]=recoverTrialMoments(obj)
            % Recover moments in each trial, for each time point and each
            % class
            lbls=obj.outputLog.stimType(1:size(obj.outputLog.EPdata,1));
            classId=unique(lbls);
            nClasses=length(classId);
            trialMeans=[];
            trialSD=[];
            trialSkew=[];
            trialLbls=[];
            trialKurt=[];
            for currTrial=1:floor(length(obj.outputLog.stimType)/obj.EPparams.stimsPerStretch)
                relData=obj.outputLog.EPdata((currTrial-1)*obj.EPparams.stimsPerStretch+1:currTrial*obj.EPparams.stimsPerStretch,:,:);
                relDataShort=zeros(size(relData,1),ceil(size(relData,2)/(obj.fs/20)),size(relData,3));
                for currCh=1:16
                    relDataShort(:,:,currCh)=resample(squeeze(relData(:,:,currCh)'),20,obj.fs)';
                end
                relData=relDataShort;
                relLbls=lbls((currTrial-1)*obj.EPparams.stimsPerStretch+1:currTrial*obj.EPparams.stimsPerStretch);
                for currClass=1:nClasses
                    classData=relData(relLbls==classId(currClass),:,:);
                    trialMeans=cat(1,trialMeans,mean(classData,1));
                    trialSD=cat(1,trialSD,std(classData,[],1));
                    trialSkew=cat(1,trialSkew,skewness(classData,[],1));
                    trialKurt=cat(1,trialKurt,kurtosis(classData,[],1));
                    trialLbls=cat(1,trialLbls,classId(currClass));
                end
            end
            ceLbls=trialLbls(trialLbls>0);
            ceMean=trialMeans(trialLbls>0,:,:);
            ceSD=trialSD(trialLbls>0,:,:);
            ceSkew=trialSkew(trialLbls>0,:,:);
            ceKurt=trialKurt(trialLbls>0,:,:);
            
            % First four trial still are affected by startup
            ceLbls=ceLbls(120:end);
            ceMean=ceMean(120:end,:,:);
            ceSD=ceSD(120:end,:,:);
            ceSkew=ceSkew(120:end,:,:);
            ceKurt=ceKurt(120:end,:,:);
            
            % Join moments together and treat them as features for
            % classifier
            feats=cat(3,ceMean,ceSD,ceSkew,ceKurt);
            feats=reshape(feats,size(feats,1),[]);
        end
        
        function [stBAcc,classAcc]=estimateOfflineAcc(obj)
            [lbls,data]=averageEPs(obj);
            % Remove first selection, affected by ampl noise, and confounds
            lbls(1:120)=0;
            data2=data(lbls>0,:,:);
            lbls2=lbls(lbls>0);
            feats=reshape(data2,size(data2,1),[]);
            
            p=zeros(size(feats,2),1);
            for currFeat=1:size(feats,2)
                p(currFeat)=ranksum(feats(lbls2==1,currFeat),feats(lbls2==2,currFeat));
            end
            feats2=feats(:,p<.05);
            
            for currSel=1:floor(length(lbls2)/60)
                relIdxs=(currSel-1)*60+1:currSel*60;
                lblsV1=lbls2;
                lblsV2=lbls2;
                lblsV2(relIdxs)=3-lblsV2(relIdxs);
                discr=fitcsvm(feats2,lblsV1,'ClassNames',[1;2],'Standardize',true,'KernelScale',30,'KernelFunction','rbf');
                stEst=discr.predict(feats2);
                stBAcc1=testAcc(lblsV1,stEst);
                discr=fitcsvm(feats2,lblsV2,'ClassNames',[1;2],'Standardize',true,'KernelScale',30,'KernelFunction','rbf');
                stEst=discr.predict(feats2);
                stBAcc2=testAcc(lblsV2,stEst);
                classEst(currSel)=stBAcc1>stBAcc2;
                [stBAcc1,stBAcc2]
            end
            classAcc=mean(classEst);
            stBAcc=max(stBAcc1,stBAcc2);
            
%             discr=fitcsvm(feats2,lbls2,'crossVal','on','KFold',10,'ClassNames',[1;2],'Standardize',true,'KernelScale',30,'KernelFunction','rbf');
% %             discr=TreeBagger(400,feats2,lbls2,'OOBVarImp','On');
%             stEst=discr.kfoldPredict;
%             stBAcc=testAcc(lbls2,stEst);
%             
%             classEst=zeros(floor(length(lbls2)/60),1);
%             for currTrial=1:floor(length(lbls2)/60)
%                 relIdx=(currTrial-1)*60+1:currTrial*60;
%                 classEst(currTrial)=mode(double(stEst(relIdx)==lbls2(relIdx)));
%             end
%             classAcc=mean(classEst);
%             fprintf('Single trial BAcc: %0.2f; class acc: %0.2f\n',stBAcc,classAcc);
        end
        
        function trainOfflineClassifier(obj)
            % Compute correct/erroneous lbls and means over repetitions
            % within the same trial
            [ceLbls,ceMean]=averageEPs(obj);
            
            % Train classifier
            obj.classifier=TSclassifier.timeTrain(ceMean,ceLbls);
            
            % Determine score/prob model
            [~,score]=TSclassifier.timePredict(obj.classifier,ceMean);
            scoreProbModel=fittype('1/(1+exp(-(a*x+b)))');
            obj.classifier.scoreProbModel=fit(real(score),ceLbls-1,scoreProbModel,'Robust','on');
        end
               
        function plotGAs(obj)
            % Recover data
            [lbls,data]=averageEPs(obj);
            
            % Average and plot them
            winLims=[-.2,.8];
            t=linspace(winLims(1),winLims(2),size(data,2));
            tStep=median(diff(t));
            for currCh=1:16
                subplot(4,4,currCh);
                hold on
                clrs='rgk';
                for currClass=1:3
                    plot(t,median(data(lbls==currClass-1,:,currCh)),clrs(currClass));
                    set(gca,'XLim',[winLims(1),winLims(2)]);
                    for currSample=1:size(data,2)
%                         p=anova1(squeeze(data(:,currSample,currCh)),lbls,'off');
                        p=ranksum(squeeze(data(lbls==1,currSample,currCh)),data(lbls==2,currSample,currCh));
                        if p<.05
                            patch(t(currSample)+[-.5,.5,.5,-.5]*tStep,reshape([get(gca,'YLim');get(gca,'YLim')],[],1),'yellow','FaceAlpha',.3,'EdgeAlpha',0);
                        end
                    end
                end
            end
%             % Normalize data, apply CARs
%             normalize=@(x)(x-repmat(mean(x),size(x,1),1))./repmat(1.4826*mad(x,1),size(x,1),1);
%             CARdata=obj.outputLog.EPdata;
%             for currTrial=1:size(CARdata,1)
%                 normData=normalize(squeeze(CARdata(currTrial,:,:)));
%                 CARdata(currTrial,:,:)=normData-repmat(median(normData,2),1,size(normData,2));
%             end
%             
%             % Plot results
%             t=linspace(0,size(obj.outputLog.EPdata,2)/obj.fs,size(obj.outputLog.EPdata,2));
%             lbls=obj.outputLog.stimType(1:size(obj.outputLog.EPdata,1));
%             classId=unique(lbls);
%             nClasses=length(classId);
%             lbls(1:10)=NaN;
%             clrs='bkgrcmy';
%             tStep=median(diff(t));
%             for currCh=1:16
%                 for currClass=1:nClasses
%                     sig=squeeze(mean(CARdata(lbls==classId(currClass),:,currCh)));
%                     subplot(4,4,currCh);
%                     plot(t,sig,clrs(currClass));
%                     hold on;
%                     set(gca,'XLim',[t(1),t(end)]);
%                     for currSample=1:size(CARdata,2)
%                         p=anova1(squeeze(CARdata(:,currSample,currCh)),lbls,'off');
%                         if p<.05
%                             patch(t(currSample)+[-.5,.5,.5,-.5]*tStep,reshape([get(gca,'YLim');get(gca,'YLim')],[],1),'yellow','FaceAlpha',.3,'EdgeAlpha',0);
%                         end
%                     end
%                 end
%             end
        end
                
        %% Dependent properties
        function res=get.screenRes(~)
            res=get(0,'screensize');
            res=res(3:end);
        end
        
        function samplingRate=get.fs(~)
            samplingRate=512;
        end
        
        function cTime=get.currTime(obj)
            cTime=get_param(obj.modelName,'SimulationTime');
        end
    end
    
    methods (Static)
        function simulinkModelStopFcn(instanceName,fileName)
            commandString=sprintf('%s.outputLog.rawData=rawData;\n%s.outputLog.timeLog=%s.outputLog.rawData.Time;\n%s.outputLog.rawData=%s.outputLog.rawData.Data;\nsave(''%s'',''%s'');\n',instanceName,instanceName,instanceName,instanceName,instanceName,fileName,instanceName);
            evalin('base',commandString);
        end
        
        function simulinkModelStartFcn(modelName,instanceName)
            % Start function for Simulink model
            blockName=sprintf('%s/Buffer',modelName);
            commandString=sprintf('%s.listener=add_exec_event_listener(''%s'',''PostOutputs'',@%s.newDataAvailable);\n',instanceName,blockName,instanceName);
            evalin('base',commandString);
        end
        
        function onMouseMove(~,~)
            % Makes mouse pointer invisible
            if ~strcmp(get(gcf,'Pointer'),'custom')
                set(gcf,'PointerShapeCData',NaN(16));
                set(gcf,'Pointer','custom');
            end
        end
                        
        function st=generateSelectionTree(varargin)
            % If not argument is passed a selection tree is generated with
            % standard alphabet plus digits and basic extra characters
            if nargin==0
                entriesList={'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z'};
            else
                entriesList=varargin{1};
            end
            nEntries=length(entriesList);
            nSplits=ceil(log2(nEntries));
            st.mat=zeros(nSplits,nEntries);
            for currLine=1:nSplits
                st.mat(currLine,:)=(square(linspace(0,2*pi*(1-1/nEntries),nEntries)*2^(currLine-1))<0)+1;
            end
            st.entries=entriesList;
            st.nEntries=nEntries;
            st.nSplits=nSplits;
        end
        
        function [outData,fltrWeights]=applyLapFilter(inData)
            % Remove pegged channels from recording
            toBeRemoved=find(median(inData)==max(inData));
            load('C:\Code\Common\elMap16_new.mat');
            tempMat=elMap16.elMat;
            for currCh=1:length(toBeRemoved)
                tempMat(tempMat==toBeRemoved(currCh))=0;
            end
            
            fltrWeights=zeros(size(inData,2));
            for currEl=1:size(inData,2)
                neighborsMap=zeros(size(tempMat));
                neighborsMap(tempMat==currEl)=1;
                neighborsMap=imdilate(neighborsMap,strel('diamond',1));
                neighborsMap(tempMat==currEl)=0;
                validNeighbors=logical(neighborsMap.*tempMat);
                fltrWeights(currEl,tempMat(validNeighbors))=-1/sum(sum(validNeighbors));
                fltrWeights(currEl,currEl)=1;
            end
            outData=inData*fltrWeights';
            outData(:,toBeRemoved)=[];
        end
    end
end