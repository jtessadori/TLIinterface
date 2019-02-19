classdef TLIinterface < handle
    %% December, 2018 Jacopo Tessadori
    properties
        portToInterface;
        portFromInterface;
        portFromET;
        UDPtoInterface;
        UDPfromInterface;
        UDPfromET;
        intercomTimer;
        trialEventTimer;
        MIanimationTimer;
        screenRes;
        fileName;
        fs;
        outputLog;
        listener;
        currTime;
        motorSerialPort;
        figureParams;
        targetCharacter;
        targetPos;
        maxItsPerTrial;
        currIts;
        MIdata;
        MIest;
        MIscore;
        MIclassifier;
        errPdata;
        errPest;
        errPscore;
        errPclassifier;
        trialData;
        trialEst;
        trialClassifier;
        trialScore;
        movDirection;
        movCorrect;
        finalEst;
        finalLbls;
        selectionTree;
        selLevel;
        phrase;
        calErrChance;
        maxTrials;
    end
    
    properties (Hidden)
        modelName='TLIinterface_SimulinkSupport';
        instanceName;
        lastDataBlock;
        timingParams;
        isCalibrating=0;
        retry=0;
        timeSinceTrialStart=Inf;
        estimateTimes;
        isTrialOngoing=0;
        RS;
        colorScheme;
        isDebugging;
        phantomData=cat(1,reshape(repmat(hann(512),1,16)*randn(16),1,512,16),reshape(repmat(blackman(512),1,16)*randn(16),1,512,16))
        isTrainingOngoing=0;
        nTrials=[0,0];
        targetCharacterIdx;
        selHistory=[];
    end
    
    methods
        %% Constructor
        function obj=TLIinterface(varargin)
            % Another TLIinterface object may be passed. In this case,
            % classifiers will be imported from the previous one
            if nargin==1
                if isa(varargin{1},'TLIinterface')
                    obj.errPclassifier=varargin{1}.errPclassifier;
                    obj.MIclassifier=varargin{1}.MIclassifier;
                    obj.trialClassifier=varargin{1}.trialClassifier;
                else
                    error('First (and only) input must be an instance of the TLIinterface class.');
                end
            end
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
                obj.timingParams.MIlength=2;
                obj.timingParams.winLims=[0.5,1.5]; % Beginning and end, in seconds after MI start, of time time window used for training
                obj.timingParams.winLength=obj.timingParams.winLims(2)-obj.timingParams.winLims(1); % Window length, in seconds, used to perform intention estimation
                obj.timingParams.FBlength=.25;
                obj.timingParams.interTrialLength=.25;
            else
                % Define timing parameters common to experiment and calibration
                obj.timingParams.restLength=.5;
                obj.timingParams.MIlength=3;
                obj.timingParams.winLims=[1.5,3]; % Beginning and end, in seconds after MI start, of time time window used for training
                obj.timingParams.winLength=obj.timingParams.winLims(2)-obj.timingParams.winLims(1); % Window length, in seconds, used to perform intention estimation
                obj.timingParams.FBlength=1;
                obj.timingParams.interTrialLength=.75;
            end
            
            % Chance of error during calibration
            obj.calErrChance=0.25;
                        
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
            obj.figureParams.cursorHandle2=patch(obj.screenRes(1)/2+obj.figureParams.squareVertexX,obj.screenRes(2)/2+obj.figureParams.squareVertexY,obj.colorScheme.cursorColorMI,'EdgeColor',obj.colorScheme.edgeColor,'Visible','off');
            
            % Add large feedback patches on sides of screen
            obj.figureParams.fbPatch(1)=patch(obj.screenRes(1)*[0,.3,.3,0],obj.screenRes(2)*[0,0,1,1],obj.colorScheme.cursorColorRest,'Visible','off');
            obj.figureParams.fbPatch(2)=patch(obj.screenRes(1)*[.7,1,1,.7],obj.screenRes(2)*[0,0,1,1],obj.colorScheme.cursorColorRest,'Visible','off');
            
            % Begin actual calibration
            obj.beginSelection;
        end
        
        function startExperiment(obj)
            % Clear logs
            obj.outputLog=[];
            obj.phrase=[];
            
            % Initialize devices
            obj.isCalibrating=0;
            obj.maxItsPerTrial=Inf;
            obj.initialize;
        end
        
        function closeActivity(obj)
            % Stop running timer
            stop(obj.trialEventTimer);
            stop(obj.MIanimationTimer);
            
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
            
            % Stop Simulink model
            obj.listener=[];
            set_param(obj.modelName,'SimulationCommand','Stop');
            set_param(obj.modelName,'StartFcn','');
        end
        
        function stopExperiment(obj)
            % Close required executables, in case they are still open
            %             !taskkill -f -im GazeTrackEyeXGazeStream.exe
            !taskkill -f -im TLIinterface.exe
            
            % Release occupied ports
            obj.UDPfromInterface.release;
            obj.UDPfromET.release;
            
            % Stop and delete timer objects
            obj.intercomTimer.stop;
            delete(obj.intercomTimer);
            obj.intercomTimer=[];
            
            obj.closeActivity
        end
        
        function startCalibration(obj)
            % Initialize devices
            obj.isCalibrating=1;
            obj.maxItsPerTrial=3;
            obj.maxTrials=300;
            obj.initialize;
        end
        
        function startTesting(obj)
            % Clear logs
            obj.outputLog=[];
            obj.phrase=[];
            
            % Initialize devices
            obj.isCalibrating=1;
            obj.maxItsPerTrial=7;
            obj.maxTrials=150;
            obj.initialize;
        end
         
        function stopCalibration(obj)
            % Close figure
            delete(gcf);
            
            obj.closeActivity;
        end
        
        function beginSelection(obj)
            try                
                if obj.isCalibrating
                    % Decide new target caharacter
                    obj.targetCharacterIdx=ceil(rand*length(obj.selectionTree.entries));
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
                if obj.isCalibrating
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
                                                
                % Reset number of iterations
                obj.currIts=0;
                
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
                % beginning of MI task and start timer for new event
                obj.currIts=obj.currIts+1;
                obj.logEntry({'selLevel','currIts','targetPos'});
                set(obj.figureParams.cursorHandle,'FaceColor',obj.colorScheme.cursorColorMI);
                obj.trialEventTimer=timer;
                obj.trialEventTimer.StartDelay=obj.timingParams.MIlength;
                obj.trialEventTimer.TimerFcn=@obj.provideFeedback;
                start(obj.trialEventTimer);
                
                % Start timer for MI animation
                obj.MIanimationTimer=timer;
                obj.MIanimationTimer.TimerFcn=@obj.updateMIanimation;
                obj.MIanimationTimer.ExecutionMode='fixedRate';
                obj.MIanimationTimer.Period=.05;
                start(obj.MIanimationTimer);
                
                % Log trial data
                obj.logTime('iterationTimes');
            catch ME
                fprintf('%s Line: %d\n',ME.message,ME.stack(1).line)
                keyboard;
            end
        end
        
        function provideFeedback(obj,~,~)
            try
                % Recover relevant data to perform MI classification and
                % log it
                obj.MIdata=squeeze(obj.lastDataBlock(:,obj.timingParams.winLims(1)*obj.fs+1:obj.timingParams.winLims(2)*obj.fs,:));
                % If debugging, substitute actual data with phantom
                % data
                if obj.isDebugging
                    errChance=0.2;
                    dataSel=xor(rand<errChance,obj.targetPos>1);
                    obj.MIdata=squeeze(obj.phantomData(dataSel+1,:,:))+randn(512,16)/100;
                end
                obj.logEntry({'MIdata'});
                
                % Perform MI classification
                obj.classifyData('MI');
                
                % Provide feedback by changing expected destination target
                % to selection result
                set(obj.figureParams.cursorHandle,'FaceColor',obj.colorScheme.bg);
                set(obj.figureParams.letters(obj.movDirection),'BackgroundColor',obj.colorScheme.cursorColorRest);
                set(obj.figureParams.fbPatch(obj.movDirection),'Visible','on');
                set(obj.figureParams.cursorHandle2,'Visible','off');
                obj.logEntry({'movDirection','movCorrect'});
                
                % Start timer for new event
                obj.trialEventTimer=timer;
                obj.trialEventTimer.StartDelay=obj.timingParams.FBlength;
                obj.trialEventTimer.TimerFcn=@obj.evaluateFeedback;
                start(obj.trialEventTimer);
                
                % Log trial data
                obj.logTime('feedbackStartTimes');
                                
                % Provide tactile feedback
                stop(obj.MIanimationTimer);
                switch obj.movDirection
                    case 1
                        fprintf(obj.motorSerialPort,'e8\n');
                        pause(0.05);
                        fprintf(obj.motorSerialPort,'r120\n');
                        pause(0.05);
                        fprintf(obj.motorSerialPort,'e4\n');
                        pause(0.05);
                        fprintf(obj.motorSerialPort,'r0\n');
                    case 2
                        fprintf(obj.motorSerialPort,'e4\n');
                        pause(0.05);
                        fprintf(obj.motorSerialPort,'r120\n');
                        pause(0.05);
                        fprintf(obj.motorSerialPort,'e8\n');
                        pause(0.05);
                        fprintf(obj.motorSerialPort,'r0\n');
                end
            catch ME
                fprintf('%s Line: %d\n',ME.message,ME.stack(1).line)
                keyboard;
            end
        end
                        
        function evaluateFeedback(obj,~,~)
            try
                % Recover relevant data to perform errP classification and
                % log it
                obj.errPdata=squeeze(obj.lastDataBlock(1,end-(obj.timingParams.FBlength*obj.fs)+1:end,:));
                % If debugging, substitute actual data with phantom
                % data
                if obj.isDebugging
                    obj.errPdata=squeeze(obj.phantomData((obj.targetPos==(obj.movDirection+2))+1,1:floor(obj.timingParams.FBlength*obj.fs),:))+randn(floor(obj.timingParams.FBlength*obj.fs),16)/100;
                end
                obj.logEntry({'errPdata'});
                
                % Stop VT feedback
                fprintf(obj.motorSerialPort,'e12\n');
                fprintf(obj.motorSerialPort,'r0\n');
                
                % If calibrating, log number of correct and erroneous
                % movements
                if obj.isCalibrating
                    obj.nTrials(obj.movCorrect+1)=sum(obj.outputLog.movCorrect==obj.movCorrect);
                end
                
                % Perform errP classification
                obj.classifyData('errP');
                
                % Trial data is given by scores of estimation according to
                % both classifiers. Perform trial prediction and log it
                obj.trialData=[obj.MIscore;obj.errPscore*sign(obj.movDirection-1.5)];
                obj.classifyData('trial');
                obj.logEntry({'trialData'});
                
                % Remove feedback
                set(obj.figureParams.letters(:),'BackgroundColor',obj.colorScheme.bg);
                set(obj.figureParams.fbPatch(:),'Visible','off');
                
                % Decide whether to start new trial or new iteration, then
                % set timer for new event
                obj.trialEventTimer=timer;
                obj.trialEventTimer.StartDelay=obj.timingParams.restLength;
                if isempty(obj.trialClassifier)||~isfield(obj.trialClassifier,'scoreTransform')
                    trialEndCondition=0;
                else
                    relIdx=max(length(obj.outputLog.currIts)-obj.currIts+1,1):length(obj.outputLog.currIts);
                    trialEndCondition=obj.trialClassifier.scoreTransform(sum(obj.outputLog.trialScore(relIdx)))>.95||obj.trialClassifier.scoreTransform(sum(obj.outputLog.trialScore(relIdx)))<.05;
                end
                if ~trialEndCondition&&obj.currIts<obj.maxItsPerTrial
                    set(obj.figureParams.cursorHandle,'FaceColor',obj.colorScheme.cursorColorMI);
                    obj.trialEventTimer.TimerFcn=@obj.performIteration;
                else
                    set(obj.figureParams.cursorHandle,'FaceColor',obj.colorScheme.cursorColorRest);
                    obj.trialEventTimer.TimerFcn=@obj.endTrial;
                end
                start(obj.trialEventTimer);
                
                % Log trial data
                obj.logTime('feedbackEvalTimes');
            catch ME
                fprintf('%s Line: %d\n',ME.message,ME.stack(1).line)
                keyboard;
            end
        end
        
        function endTrial(obj,~,~)
            try
                % Make decision on last trial estimations
%                 obj.finalEst=mode(obj.outputLog.trialEst(max(1,end-obj.currIts+1):end));
                if obj.isCalibrating
                    obj.finalLbls=obj.targetPos;
                    obj.logEntry({'finalLbls'});
                    % Log result of actual prediction, then overwrite it so
                    % that cursor always move in correct direction
                    obj.finalEst=mode(obj.outputLog.MIest(max(1,end-obj.currIts+1):end));
                    obj.logEntry({'finalEst'});
                    obj.finalEst=obj.targetPos;
                else
                    obj.finalEst=(median(obj.outputLog.trialScore(max(1,end-obj.currIts+1):end))<0)+1;
                    obj.logEntry({'finalEst'});
                end
                
                % Update selection history so far
                obj.selHistory=cat(1,obj.selHistory,obj.finalEst);
                
                % Provide feedback on final trial estimation
                set(obj.figureParams.cursorHandle,'FaceColor',obj.colorScheme.bg);
                set(obj.figureParams.letters(obj.finalEst),'BackgroundColor',obj.colorScheme.targetColor,'EdgeColor',obj.colorScheme.bg);
                
                % Remove edge from target letters
                set(obj.figureParams.letters(obj.targetPos),'EdgeColor',obj.colorScheme.bg);
                drawnow;
                
                % If calibration lasted long enough
                if obj.isCalibrating&&length(obj.outputLog.movCorrect)>=obj.maxTrials
                    fprintf('Calibration complete, shutting down.\n');
                    obj.stopCalibration;
                    return;
                else
                    fprintf('%d/%d\n',length(obj.outputLog.movCorrect),obj.maxTrials);
                end
                
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
        
        function classifyData(obj,dataType)
            switch dataType
                case 'MI'
                    % Perform prediction, if a classifier is available
                    if isempty(obj.MIclassifier)
                        obj.MIest=xor(obj.targetPos-1,rand<obj.calErrChance)+1;
                        obj.MIscore=0;
                    else
                        [obj.MIest,obj.MIscore]=TSclassifier.predict(obj.MIclassifier,obj.outputLog.MIdata(end,:,:));
                    end
                    obj.logEntry({'MIest','MIscore'});
                case 'errP'
                    % Perform prediction
                    if isempty(obj.errPclassifier)
                        obj.errPest=round(rand);
                        obj.errPscore=0;
                    else
                        % Compute and store super-trials covariance matrices
                        [obj.errPest,obj.errPscore]=TSclassifier.timePredict(obj.errPclassifier,obj.outputLog.errPdata(end,:,:));
                    end
                    obj.logEntry({'errPest','errPscore'});
                case 'trial'
                    % Perform prediction
                    if isempty(obj.trialClassifier)
                        obj.trialEst=mode(obj.outputLog.MIest(max(end-obj.maxItsPerTrial+1,1):end));
                        obj.trialScore=0;
                    else
                        [obj.trialEst,obj.trialScore]=obj.trialClassifier.clsfr.predict(obj.trialData');
                        obj.trialScore=obj.trialScore(1);
                    end
                    obj.logEntry({'trialEst','trialScore'});
            end
        end
        
        function updateMIanimation(obj,~,~)
            try
                % Fill counter, provide tactile feedback
                timeSinceMIstart=obj.currTime-obj.outputLog.iterationTimes(end);
                pTimeElapsed=timeSinceMIstart/obj.timingParams.MIlength;
                if pTimeElapsed<=1
                    fprintf(obj.motorSerialPort,'e12\n');
                    fprintf(obj.motorSerialPort,sprintf('r%d\n',round(pTimeElapsed*60)));
                end
                set(obj.figureParams.cursorHandle2,'Visible','on','YData',obj.screenRes(2)/2+obj.figureParams.squareVertexY*(1-pTimeElapsed));
                set(obj.figureParams.cursorHandle,'FaceColor',obj.colorScheme.bg);
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
                fprintf(obj.motorSerialPort,'p\n');
                pause(0.3);
                fprintf(obj.motorSerialPort,'e8\n');
                pause(0.3);
                fprintf(obj.motorSerialPort,'p\n');
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
        
        % Plotting and analysis functions
        
        function trainClassifiers(obj)
            %% MI classifier
            obj.MIclassifier=TSclassifier.train(obj.outputLog.MIdata,obj.outputLog.targetPos(1:size(obj.outputLog.MIdata,1)));
            
            %% errP classifier
            obj.errPclassifier=TSclassifier.timeTrain(obj.outputLog.errPdata,obj.outputLog.movCorrect(1:size(obj.outputLog.errPdata,1)));
            
            %% Trial classifier
            actualMove=obj.outputLog.movDirection;
            [~,~,MIscore1]=TSclassifier.crossVal(obj.outputLog.MIdata,obj.outputLog.targetPos(1:size(obj.outputLog.MIdata,1)),2);
            [~,~,MIscore2]=TSclassifier.crossValTime(obj.outputLog.errPdata,obj.outputLog.movCorrect(1:size(obj.outputLog.errPdata,1)),2);
            MIscore2(actualMove==1)=-MIscore2(actualMove==1);
            relIdx=(MIscore2~=0)&(MIscore1~=0);
            if obj.isDebugging
                obj.trialClassifier.clsfr=fitcsvm([MIscore1(relIdx)+randn(size(MIscore1(relIdx)))/100,MIscore2(relIdx)+randn(size(MIscore2(relIdx)))/100],obj.outputLog.targetPos(relIdx),'Standardize',true,'KernelScale','auto','KernelFunction','polynomial','PolynomialOrder',2);
            else
                obj.trialClassifier.clsfr=fitcsvm([MIscore1(relIdx),MIscore2(relIdx)],obj.outputLog.targetPos(relIdx),'Standardize',true,'KernelScale','auto','KernelFunction','polynomial','PolynomialOrder',2);
            end
            [obj.trialClassifier.clsfr,scoreTransform]=obj.trialClassifier.clsfr.fitPosterior;
            if strcmp(scoreTransform.Type,'sigmoid')
                A=scoreTransform.Slope;
                B=scoreTransform.Intercept;
                obj.trialClassifier.scoreTransform=@(x)1./(1+exp(A*x+B));
                obj.trialClassifier.clsfr.ScoreTransform='none';
            end
        end
        
        function plotMI_GAs(obj)
            wndwdData=obj.outputLog.relData.*repmat(blackman(size(obj.outputLog.relData,2))',size(obj.outputLog.relData,1),1,size(obj.outputLog.relData,3));
            fftData=abs(fft(wndwdData,[],2));
            f=linspace(0,obj.fs,size(fftData,2));
            lbls=obj.outputLog.targetPos;
            lbls=lbls(1:size(fftData,1));
            for currCh=1:16
                subplot(4,4,currCh);
                loglog(f,squeeze(fftData(:,:,currCh))','b');
                hold on;
                loglog(f,squeeze(mean(fftData(lbls==0,:,currCh))),'r');
                loglog(f,squeeze(mean(fftData(lbls==1,:,currCh))),'g');
                set(gca,'XLim',[8,30]);
            end
        end
        
        function plotGAs(obj)
            t=linspace(0,size(obj.outputLog.errPdata,2)/obj.fs,size(obj.outputLog.errPdata,2));
            lbls=obj.outputLog.movCorrect;
            lbls=lbls(1:size(obj.outputLog.errPdata,1));
            for currCh=1:16
                errSig=squeeze(median(obj.outputLog.errPdata(lbls==0,:,currCh)));
                corrSig=squeeze(median(obj.outputLog.errPdata(lbls==1,:,currCh)));
                subplot(4,4,currCh);
                plot(t,errSig,'r');
                hold on;
                plot(t,corrSig,'k');
                plot(t,errSig-corrSig,'g','LineWidth',2);
            end
        end
        
        function plotTrialData(obj)
            actualMove=obj.outputLog.movDirection;
            MIscore2=obj.outputLog.errPscore;
            MIscore2(actualMove==1)=-MIscore2(actualMove==1);
            MIscore1=obj.outputLog.MIscore;
            MIlbls=obj.outputLog.targetPos(1:length(MIscore2));
            outlrsBlnk=@(x)x.*(abs(x)<7*mad(x,1));
            MIscore1=outlrsBlnk(MIscore1);
            MIscore2=outlrsBlnk(MIscore2);
            relIdx=MIscore1~=0.*MIscore2~=0;
            MIscore1=MIscore1(relIdx);
            MIscore2=MIscore2(relIdx);
            MIlbls=MIlbls(relIdx);
            scatter(MIscore1(MIlbls==1),MIscore2(MIlbls==1),'k')
            hold on
            scatter(MIscore1(MIlbls==2),MIscore2(MIlbls==2),'g')
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
        
        function tsts=get.timeSinceTrialStart(obj)
            if ~isempty(obj.outputLog)&&isfield(obj.outputLog,'trialStartTimes')
                tsts=obj.currTime-obj.outputLog.trialStartTimes(end);
            else
                tsts=NaN;
            end
        end
        
        function md=get.movDirection(obj)
            if ~obj.isCalibrating&&~isempty(obj.trialEst)&&obj.trialEst~=0&&obj.currIts>1
                md=obj.trialEst;
            else
                md=obj.MIest;
            end
        end
        
        function mc=get.movCorrect(obj)
            mc=obj.targetPos==obj.movDirection;
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
        
        function [est,score]=predict(classifier,currCov)
            %Project current entry to TS
            proj=classifier.RS.project(currCov);
            
            % Apply PCA
            proj=proj'*classifier.coeff;
            
            % Feature selection
            proj=proj(:,classifier.featsIdx);
            
            % Provide estimation
            est=classifier.clsfr.predict(proj);
            
            % Compute score
            score=real(proj*classifier.clsfr.Coeffs(2,1).Linear+classifier.clsfr.Coeffs(2,1).Const);
        end
        
        function clsfr=trainClassifier(clsfr,covData,lbls)
            % Consider only the most recent covariance matrices to
            % determine centroid
            clsfr.RS=riemannSpace(covData(max(1,end-20):end,:,:),'cov');
            
            % Project data before classification
            projs=zeros(size(covData,1),size(covData,3)*(size(covData,3)+1)/2);
            for currTrial=1:size(covData,1)
                projs(currTrial,:,:)=clsfr.RS.project(squeeze(covData(currTrial,:,:)));
            end
            
            % Perform PCA
            [clsfr.coeff,projs,latent]=pca(projs);
            
            % Perform feature selection
            clsfr.featsIdx=TSclassifier.selectFeatures(projs,lbls,latent);
            
            % Leave only desired features
            projs=projs(:,clsfr.featsIdx);
            
            % Perform training
            clsfr.clsfr=fitcdiscr(projs,lbls);
            
            % Define 'predict' function
            clsfr.predict=@(clsfr,x)TLIinterface.predict(clsfr,x);
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
    end
end