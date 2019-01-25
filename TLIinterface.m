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
        screenRes;
        fileName;
        fs;
        outputLog;
        listener;
        currTime;
        motorSerialPort;
        figureParams;
        targetPos;
        maxItsPerTrial;
        currIts;
        MIdata;
        MIcov;
        MIest;
        MIscore;
        MIclassifier;
        errPdata;
        errPsuperTrialCov;
        errPest;
        errPscore;
        errPclassifier;
        trialData;
        trialEst;
        trialClassifier;
        trialScore;
        cursorPos;
        movDirection;
        movCorrect;
        finalEst;
        finalLbls;
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
            
            obj.isDebugging=1;
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
                obj.timingParams.restLength=2;
                obj.timingParams.MIlength=4;
                obj.timingParams.winLims=[2.5,3.5]; % Beginning and end, in seconds after MI start, of time time window used for training
                obj.timingParams.winLength=obj.timingParams.winLims(2)-obj.timingParams.winLims(1); % Window length, in seconds, used to perform intention estimation
                obj.timingParams.FBlength=2;
                obj.timingParams.interTrialLength=2;
            end
            
            % Determing number of iterations before implementing a choice
            obj.maxItsPerTrial=7;
            
            % Determine name to be used to save file upon closing
            obj.fileName=datestr(now,30);

            % Prepares serial port for vibrating motor control
            obj.prepareSerialPort;
            
            % Start Simulink model
            obj.prepareSimulinkModel;
            
            % Perform a countdown to allow amplifier to settle
            obj.startCountdown(3);
        end
        
        function startExperiment(obj)
            % Initialize devices
            obj.isCalibrating=0;
            obj.initialize;
            
            % Define interface object colors
            obj.colorScheme.bg=[.05,.05,.05];
            obj.colorScheme.edgeColor=[.4,.4,.4];
            obj.colorScheme.targetColor=[.4,0,.1];
            obj.colorScheme.cursorColorMI=[0,.4,0];
            obj.colorScheme.cursorColorRest=[.6,.6,0];
            
            % Define figure properties
            % Squares
            obj.figureParams.squareSide=obj.screenRes(1)/15;
            obj.figureParams.squareCenterX=obj.screenRes(1)/2+[-obj.screenRes(1)/10,0,obj.screenRes(1)/10];
            obj.figureParams.squareCenterY=repmat(obj.screenRes(2)/2,3,1);
            obj.figureParams.squareVertexX=[-1,1,1,-1]*obj.figureParams.squareSide/2;
            obj.figureParams.squareVertexY=[-1,-1,1,1]*obj.figureParams.squareSide/2;
            % Arrows
            obj.figureParams.leftArrow.X=[-1,-.4,-.4,1,1,-.4,-.4]*obj.figureParams.squareSide;
            obj.figureParams.leftArrow.Y=[0,.5,.2,.2,-.2,-.2,-.5]*obj.figureParams.squareSide;
            obj.figureParams.rightArrow.X=[-1,.4,.4,1,.4,.4,-1]*obj.figureParams.squareSide;
            obj.figureParams.rightArrow.Y=[.2,.2,.5,0,-.5,-.2,-.2]*obj.figureParams.squareSide;
            obj.figureParams.headlessArrow.X=[-1,1,1,-1]*obj.figureParams.squareSide;
            obj.figureParams.headlessArrow.Y=[.2,.2,-.2,-.2]*obj.figureParams.squareSide;
            
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
            
            % Plot squares
            for currSquare=1:length(obj.figureParams.squareCenterX)
                obj.figureParams.squareHandles(currSquare)=patch(obj.figureParams.squareCenterX(currSquare)+obj.figureParams.squareVertexX,obj.figureParams.squareCenterY(currSquare)+obj.figureParams.squareVertexY,obj.colorScheme.bg,'EdgeColor',obj.colorScheme.edgeColor);
            end
            
            % Begin actual calibration
            obj.beginTrial;
%             obj.initialize;
%             
%             % Launch gaze tracker and Unity interface
%             %             !C:\Code\Sources\GazeTrackEyeXGazeStream\GazeTrackEyeXGazeStream.exe &
%             !C:\Code\2018_12_TLI\TLIinterface\build\TLIinterface.exe &
%             
%             % Open incoming and outgoing udp ports
%             obj.UDPtoInterface=dsp.UDPSender('RemoteIPPort',obj.portToInterface);
%             obj.UDPfromInterface=dsp.UDPReceiver('LocalIPPort',obj.portFromInterface,'ReceiveBufferSize',26000,'MaximumMessageLength',26000);
%             obj.UDPfromET=dsp.UDPReceiver('LocalIPPort',obj.portFromET,'ReceiveBufferSize',26000,'MaximumMessageLength',26000);
%             
%             % Start a timer to handle communication with interface
%             obj.intercomTimer=timer;
%             obj.intercomTimer.ExecutionMode='FixedRate';
%             obj.intercomTimer.Period=0.1;
%             obj.intercomTimer.TimerFcn=@obj.intercomHandling;
%             start(obj.intercomTimer);
        end
        
        function closeActivity(obj)
            % Stop running timer
            stop(obj.trialEventTimer);
            
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
            obj.initialize;
            
            % Define interface object colors
            obj.colorScheme.bg=[.05,.05,.05];
            obj.colorScheme.edgeColor=[.4,.4,.4];
            obj.colorScheme.targetColor=[.4,0,.1];
            obj.colorScheme.cursorColorMI=[0,.4,0];
            obj.colorScheme.cursorColorRest=[.6,.6,0];
            
            % Define figure properties
            % Squares
            obj.figureParams.squareSide=obj.screenRes(1)/15;
            obj.figureParams.squareCenterX=obj.screenRes(1)/2+[-obj.screenRes(1)/10,0,obj.screenRes(1)/10];
            obj.figureParams.squareCenterY=repmat(obj.screenRes(2)/2,3,1);
            obj.figureParams.squareVertexX=[-1,1,1,-1]*obj.figureParams.squareSide/2;
            obj.figureParams.squareVertexY=[-1,-1,1,1]*obj.figureParams.squareSide/2;
            % Arrows
            obj.figureParams.leftArrow.X=[-1,-.4,-.4,1,1,-.4,-.4]*obj.figureParams.squareSide;
            obj.figureParams.leftArrow.Y=[0,.5,.2,.2,-.2,-.2,-.5]*obj.figureParams.squareSide;
            obj.figureParams.rightArrow.X=[-1,.4,.4,1,.4,.4,-1]*obj.figureParams.squareSide;
            obj.figureParams.rightArrow.Y=[.2,.2,.5,0,-.5,-.2,-.2]*obj.figureParams.squareSide;
            obj.figureParams.headlessArrow.X=[-1,1,1,-1]*obj.figureParams.squareSide;
            obj.figureParams.headlessArrow.Y=[.2,.2,-.2,-.2]*obj.figureParams.squareSide;
            
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
            
            % Plot squares
            for currSquare=1:length(obj.figureParams.squareCenterX)
                obj.figureParams.squareHandles(currSquare)=patch(obj.figureParams.squareCenterX(currSquare)+obj.figureParams.squareVertexX,obj.figureParams.squareCenterY(currSquare)+obj.figureParams.squareVertexY,obj.colorScheme.bg,'EdgeColor',obj.colorScheme.edgeColor);
            end
            
            % Begin actual calibration
            obj.beginTrial;
        end
         
        function stopCalibration(obj)
            % Close figure
            delete(gcf);
            
            obj.closeActivity;
        end
                
        function beginTrial(obj,~,~)
            try
%                 if obj.isCalibrating
                    % Decide new target
                    obj.targetPos=(rand>.5)*(length(obj.figureParams.squareCenterX)-1)+1;
                    
                    % Reset cursor pos
                    obj.cursorPos=ceil(length(obj.figureParams.squareHandles)/2);
                    
                    % Reset square colors
                    set(obj.figureParams.squareHandles,'EdgeColor',obj.colorScheme.edgeColor,'FaceColor',obj.colorScheme.bg);
                    
                    % Highlight selected target
                    set(obj.figureParams.squareHandles(obj.targetPos),'EdgeColor',obj.colorScheme.targetColor);
                    
                    % Highlight cursor pos
                    set(obj.figureParams.squareHandles(round(length(obj.figureParams.squareCenterX)/2)),'EdgeColor',obj.colorScheme.bg,'FaceColor',obj.colorScheme.cursorColorRest);
                    
                    % Reset number of iterations so far
                    obj.currIts=0;
                    
                    % Start timer for next event
                    obj.trialEventTimer=timer;
                    obj.trialEventTimer.StartDelay=obj.timingParams.restLength;
                    obj.trialEventTimer.TimerFcn=@obj.performIteration;
                    start(obj.trialEventTimer);
%                 else
%                     obj.isTrialOngoing=1;
%                 end
                
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
                obj.logEntry({'targetPos','currIts','cursorPos'});
                set(obj.figureParams.squareHandles(obj.cursorPos),'FaceColor',obj.colorScheme.cursorColorMI);
                obj.trialEventTimer=timer;
                obj.trialEventTimer.StartDelay=obj.timingParams.MIlength;
                obj.trialEventTimer.TimerFcn=@obj.provideFeedback;
                start(obj.trialEventTimer);
                
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
                set(obj.figureParams.squareHandles(obj.cursorPos),'FaceColor',obj.colorScheme.bg,'EdgeColor',obj.colorScheme.edgeColor);
                if ~isempty(obj.trialEst)&&obj.trialEst~=0&&obj.currIts>1
                    set(obj.figureParams.squareHandles(obj.cursorPos+obj.trialEst),'FaceColor',obj.colorScheme.cursorColorRest);
                else
                    set(obj.figureParams.squareHandles(obj.cursorPos+obj.MIest),'FaceColor',obj.colorScheme.cursorColorRest);
                end
                obj.logEntry({'movDirection','movCorrect'});
                
                % Start timer for new event
                obj.trialEventTimer=timer;
                obj.trialEventTimer.StartDelay=obj.timingParams.FBlength;
                obj.trialEventTimer.TimerFcn=@obj.evaluateFeedback;
                start(obj.trialEventTimer);
                
                % Log trial data
                obj.logTime('feedbackStartTimes');
            catch ME
                fprintf('%s Line: %d\n',ME.message,ME.stack(1).line)
                keyboard;
            end
        end
        
        function classifyData(obj,dataType)
            switch dataType
                case 'MI'
                    % Compute and store covariance matrix of current data
                    obj.MIcov=cov(obj.MIdata);
                    
                    % Perform prediction, if a classifier is available
                    if isempty(obj.MIclassifier)
                        obj.MIest=sign(randn);
                        obj.MIscore=0;
                    else
                        [obj.MIest,obj.MIscore]=obj.MIclassifier.predict(obj.MIclassifier,obj.MIcov);
                    end
                    obj.logEntry({'MIcov','MIest','MIscore'});
                case 'errP'                    
                    % Compute class means
                    obj.nTrials(obj.movCorrect+1)=sum(obj.outputLog.movCorrect==obj.movCorrect);
                    if ~isfield(obj.errPclassifier,'classMeans')%%||size(obj.errPclassifier.classMeans,1)<obj.targetPos
                        obj.errPclassifier.classMeans=zeros(2,size(obj.errPdata,1),size(obj.errPdata,2));
                    end
                    obj.errPclassifier.classMeans(obj.movCorrect+1,:,:)=(squeeze(obj.errPclassifier.classMeans(obj.movCorrect+1,:,:))*obj.nTrials(obj.movCorrect+1)+obj.errPdata)/(obj.nTrials(obj.movCorrect+1)+1);
                    
                    % Compute and store super-trials covariance matrices
                    superTrial=cat(2,reshape(permute(obj.errPclassifier.classMeans,[1,3,2]),[],size(obj.errPdata,1))',obj.errPdata);
                    obj.errPsuperTrialCov=cov(superTrial);
                    
                    % Perform prediction
                    if ~isfield(obj.errPclassifier,'predict')
                        obj.errPest=round(rand);
                        obj.errPscore=0;
                    else
                        [obj.errPest,obj.errPscore]=obj.errPclassifier.predict(obj.errPclassifier,obj.errPsuperTrialCov);
                    end
                    obj.logEntry({'errPsuperTrialCov','errPest','errPscore'});
                case 'trial'
                    % Perform prediction
                    if isempty(obj.trialClassifier)
                        obj.trialEst=mode(obj.outputLog.MIest(max(end-obj.maxItsPerTrial+1,1):end));
                        obj.trialScore=0;
                    else
                        [obj.trialEst,obj.trialScore]=obj.trialClassifier.clsfr.predict(obj.trialData');
                        obj.trialScore=obj.trialScore(1);
                        fprintf('%d\n',obj.trialEst);
                    end
                    obj.logEntry({'trialEst','trialScore'});
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
                
                % Perform errP classification
                obj.classifyData('errP');
                
                % Trial data is given by scores of estimation according to
                % both classifiers. Perform trial prediction and log it
                obj.trialData=[obj.MIscore;obj.errPscore*obj.movDirection];
                obj.classifyData('trial');
                obj.logEntry({'trialData'});
                
                % Remove feedback and restore target current position
                set(obj.figureParams.squareHandles(obj.cursorPos+obj.movDirection),'FaceColor',obj.colorScheme.bg);
                
                % Decider whether to start new trial or new iteration, then
                % set timer for new event
                obj.trialEventTimer=timer;
                obj.trialEventTimer.StartDelay=obj.timingParams.restLength;
                trialEndCondition=prod(obj.outputLog.trialEst(max(end-obj.currIts+1,1):end))>0&&(1-prod(min(obj.outputLog.trialScore(max(1,end-obj.currIts+1):end),1-obj.outputLog.trialScore(max(1,end-obj.currIts+1):end))))>0.99;
                if ~trialEndCondition&&obj.currIts<obj.maxItsPerTrial
                    set(obj.figureParams.squareHandles(obj.cursorPos),'FaceColor',obj.colorScheme.cursorColorMI,'EdgeColor',obj.colorScheme.bg);
                    obj.trialEventTimer.TimerFcn=@obj.performIteration;
                else
                    set(obj.figureParams.squareHandles(obj.cursorPos),'FaceColor',obj.colorScheme.cursorColorRest,'EdgeColor',obj.colorScheme.bg);
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
                obj.finalEst=mode(obj.outputLog.trialEst(max(1,end-obj.currIts+1):end));
                obj.finalLbls=sign(obj.targetPos-obj.cursorPos);
                obj.logEntry({'finalEst','finalLbls'});
                
                % Provide feedback on final trial estimation
                set(obj.figureParams.squareHandles(obj.cursorPos),'FaceColor',obj.colorScheme.bg,'EdgeColor',obj.colorScheme.edgeColor);
                set(obj.figureParams.squareHandles(obj.cursorPos+obj.finalEst),'FaceColor',obj.colorScheme.targetColor,'EdgeColor',obj.colorScheme.bg);
                
                % If accuracy in the last 15 iterations for trial
                % classifier is >90%, close calibration
                if obj.isCalibrating&&sum(obj.outputLog.trialScore~=0)>15&&mean(obj.outputLog.trialEst(end-15+1:end)==sign(obj.outputLog.targetPos(end-15+1:end)-obj.outputLog.cursorPos(end-15+1:end)))>0.9
                    fprintf('Target accuracy reached, shutting down.\n');
                    obj.stopCalibration;
                end
                
                % Wait before starting new trial
                obj.trialEventTimer=timer;
                obj.trialEventTimer.StartDelay=obj.timingParams.interTrialLength;
                obj.trialEventTimer.TimerFcn=@obj.beginTrial;
                start(obj.trialEventTimer);
                
                % Update classifiers with new data, if calibration is
                % ongoing
                if obj.isCalibrating
                    obj.updateClassifiers;
                end
                
                % Log trial data
                obj.logTime('trialEndTimes');
            catch ME
                fprintf('%s Line: %d\n',ME.message,ME.stack(1).line)
                keyboard;
            end
        end
                
        function updateClassifiers(obj)
            try
                % Cannot train classifiers until at least one trial per
                % class has been observed.
                if min(obj.nTrials)>1&&length(unique(obj.outputLog.targetPos))>1
                    % If this is first time training classifiers, I need to
                    % update previous superTrials
                    if ~obj.isTrainingOngoing
                        % Update past super-trials covariance matrices with
                        % avaiable class means
                        for currTrial=1:length(obj.outputLog.targetPos)-1
                            superTrial=cat(2,reshape(permute(obj.errPclassifier.classMeans,[1,3,2]),[],size(obj.errPdata,1))',squeeze(obj.outputLog.errPdata(currTrial,:,:)));
                            obj.outputLog.errPsuperTrialCov(currTrial,:,:)=cov(superTrial);
                        end
                    
                        obj.isTrainingOngoing=1;    
                    end
                    
                    %% MI classifier
                    obj.MIclassifier=TLIinterface.trainClassifier(obj.MIclassifier,obj.outputLog.MIcov,obj.outputLog.targetPos-2);
                    
                    %% errP classifier
                    obj.errPclassifier=TLIinterface.trainClassifier(obj.errPclassifier,obj.outputLog.errPsuperTrialCov,obj.outputLog.movCorrect);
                    
                    % Cannot train trial classifier until at least two
                    % predictions per class have been performed
                    nMIests=zeros(2,1);
                    nErrPests=zeros(2,1);
                    for currClass=1:2
                        relIdx=obj.outputLog.targetPos==1+(currClass-1)*2;
                        nMIests(currClass)=sum(obj.outputLog.MIscore(relIdx)~=0);
                    end
                    for currClass=1:2
                        relIdx=obj.outputLog.movCorrect==currClass-1;
                        nErrPests(currClass)=sum(obj.outputLog.errPscore(relIdx)~=0);
                    end
                    
                    if all(nMIests)~=0&&all(nErrPests)~=0
                        %% Trial classifier
                        actualMove=obj.outputLog.movDirection;
                        MIscore2=obj.outputLog.errPscore;
                        MIscore2(actualMove==-1)=-MIscore2(actualMove==-1);
                        relIdx=(MIscore2~=0)&(obj.outputLog.MIscore~=0);
                        if obj.isDebugging
                            % MIscore2 can occasionally grow to very large
                            % numbers, providing numerical issues. Solve this
                            MIscore2(abs(MIscore2)>100)=100*sign(MIscore2(abs(MIscore2)>100));
                            obj.trialClassifier.clsfr=fitcsvm([obj.outputLog.MIscore(relIdx)+randn(size(obj.outputLog.MIscore(relIdx)))/100,MIscore2(relIdx)+randn(size(MIscore2(relIdx)))/100],obj.outputLog.targetPos(relIdx)-2);
                        else
%                             obj.trialClassifier.clsfr=fitNaiveBayes([obj.outputLog.MIscore(relIdx),MIscore2(relIdx)],obj.outputLog.targetPos(relIdx)-2);
                            obj.trialClassifier.clsfr=fitcsvm([obj.outputLog.MIscore(relIdx),MIscore2(relIdx)],obj.outputLog.targetPos(relIdx)-2,'Standardize',true,'KernelScale','auto','KernelFunction','polynomial','PolynomialOrder',2);
                        end
                        obj.trialClassifier.clsfr=obj.trialClassifier.clsfr.fitPosterior;
                    end
                end
            catch ME
                fprintf('%s Line: %d\n',ME.message,ME.stack(1).line)
                stop(obj.trialEventTimer);
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
                
        %% Analysis functions
%         function BAcc=testClassifier(obj)
%             % Compute Riemann-mean on data
%             localRS=riemannSpace(obj.outputLog.relData);
%             
%             % Project both train and test data to tangent space
%             s=TSclassifier.projectData(obj.outputLog.relData,localRS);
%             
%             % Perform PCA
%             proj=s*obj.classifier.coeff;
%             
%             % Leave only selected features
%             proj=proj(:,obj.classifier.featsIdx);
%             
%             % Classify test projections
%             localClassEst=obj.classifier.clsfr.predict(proj);
%             
%             % Evaluate results for current partition
%             BAcc=testAcc(obj.outputLog.targetPos(1:length(localClassEst)),localClassEst);
%             fprintf('BAcc: %0.2f\n',BAcc);
%         end
%         
%         function crossBAcc=crossValData(obj)
%             crossBAcc=TSclassifier.crossVal(obj.outputLog.relData,obj.outputLog.targetPos(1:size(obj.outputLog.relData,1)),30);
%         end
%         
%         function onlineBAcc=testOnlineClassifier(obj)
%             onlineBAcc=testAcc(obj.outputLog.targetPos(~isnan(obj.outputLog.classEst)),obj.outputLog.classEst(~isnan(obj.outputLog.classEst)));
%         end
        
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
            t=linspace(0,size(obj.outputLog.relData,2)/obj.fs,size(obj.outputLog.relData,2));
            lbls=obj.outputLog.targetPos;
            lbls=lbls(1:size(obj.outputLog.relData,1));
            for currCh=1:16
                subplot(4,4,currCh);
                plot(t,squeeze(obj.outputLog.relData(:,:,currCh))','b');
                hold on;
                plot(t,squeeze(mean(obj.outputLog.relData(lbls==0,:,currCh))),'r');
                plot(t,squeeze(mean(obj.outputLog.relData(lbls==1,:,currCh))),'g');
            end
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
            if ~isempty(obj.trialEst)&&obj.trialEst~=0&&obj.currIts>1
                md=sign(obj.trialEst);
            else
                md=sign(obj.MIest);
            end
        end
        
        function mc=get.movCorrect(obj)
            mc=sign(obj.targetPos-obj.cursorPos)==obj.movDirection;
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
    end
end