classdef TLIinterface < handle
    %% December, 2018 Jacopo Tessadori
    properties
        portToInterface;
        portFromInterface;
        portFromET;
        UDPtoInterface;
        UDPfromInterface;
        UDPfromET;
        estimateTimer;
        intercomTimer;
        figureUpdateTimer;
        bufferHistory;
        gazePos;
        screenRes;
        fileName;
        fs;
        outputLog;
        listener;
        classifiersLibrary;
        classEst;
        STIGclassEst;
        calClassEst;
        classOut;
        STIGclassOut;
        calClassOut;
        classOutLP=0;
        libraryClassification;
        estCov;
        dataCov;
        currTime;
        motorSerialPort;
        figureParams;
        currTarget;
        calibrationClassifier;
        confidenceThreshold=0.05;
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
    end
    
    methods
        %% Constructor
        function obj=TLIinterface
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
                        warning('Please remove from workspace other variables of this class before starting experiment. Thanks!')
                        return;
                    end
                end
            end
            
            % Define timing parameters common to experiment and calibration
            obj.timingParams.winLength=1; % Window length, in seconds, used to perform intention estimation
            obj.timingParams.CDlength=3;
            obj.timingParams.MIlength=3; % This will be exact time for calibration, minimum time during experiment
            
            % Determine name to be used to save file upon closing
            obj.fileName=datestr(now,30);

            % Prepares serial port for vibrating motor control
            obj.prepareSerialPort;
            
            % Start Simulink model
            obj.prepareSimulinkModel;
            
            % Perform a countdown to allow amplifier to settle
            obj.startCountdown(20);
            
            % Load classifier library
            load('clsfrLib.mat')
            obj.classifiersLibrary=clsfrLibrary;
            for currClsfr=1:length(obj.classifiersLibrary)
                obj.classifiersLibrary(currClsfr).predict=@(x)Rdist(cov(squeeze(x)),squeeze(obj.classifiersLibrary(currClsfr).karchMeans(1,:,:)))>Rdist(cov(squeeze(x)),squeeze(obj.classifiersLibrary(currClsfr).karchMeans(2,:,:)));
            end
            
            % Start a timer to periodically provide an estimation of
            % current intention
            obj.estimateTimer=timer;
            obj.estimateTimer.ExecutionMode='FixedSpacing';
            obj.figureUpdateTimer.StartDelay=2;
            obj.estimateTimer.Period=0.1;
            obj.estimateTimer.TimerFcn=@obj.estimateIntention;
            start(obj.estimateTimer);
        end
        
        function startExperiment(obj)
            obj.initialize;
            
            % Launch gaze tracker and Unity interface
            %             !C:\Code\Sources\GazeTrackEyeXGazeStream\GazeTrackEyeXGazeStream.exe &
            !C:\Code\2018_12_TLI\TLIinterface\build\TLIinterface.exe &
            
            % Open incoming and outgoing udp ports
            obj.UDPtoInterface=dsp.UDPSender('RemoteIPPort',obj.portToInterface);
            obj.UDPfromInterface=dsp.UDPReceiver('LocalIPPort',obj.portFromInterface,'ReceiveBufferSize',26000,'MaximumMessageLength',26000);
            obj.UDPfromET=dsp.UDPReceiver('LocalIPPort',obj.portFromET,'ReceiveBufferSize',26000,'MaximumMessageLength',26000);
            
            % Start a timer to handle communication with interface
            obj.intercomTimer=timer;
            obj.intercomTimer.ExecutionMode='FixedRate';
            obj.intercomTimer.Period=0.1;
            obj.intercomTimer.TimerFcn=@obj.intercomHandling;
            start(obj.intercomTimer);
        end
        
        function closeActivity(obj)
            % Stop and delete timer objects
            obj.estimateTimer.stop;
            delete(obj.estimateTimer);
            obj.estimateTimer=[];
                        
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
            
            % Set figure properties
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
            
            % Define and add two possible targets
            targetHalfWidth=obj.screenRes(1)/100;
            targetCenter=0.2;
            xPos=[-targetHalfWidth,targetHalfWidth,targetHalfWidth,-targetHalfWidth];
            yPos=[0,0,obj.screenRes(2),obj.screenRes(2)];
            obj.figureParams.target(1)=patch(targetCenter*obj.screenRes(1)+xPos,yPos,'black');
            obj.figureParams.target(2)=patch((1-targetCenter)*obj.screenRes(1)+xPos,yPos,'black');
            
            % Start timer to update figure
            obj.figureUpdateTimer=timer;
            obj.figureUpdateTimer.ExecutionMode='FixedSpacing';
            obj.figureUpdateTimer.StartDelay=2;
            obj.figureUpdateTimer.Period=0.01;
            obj.figureUpdateTimer.TimerFcn=@obj.updateTrial;
            start(obj.figureUpdateTimer);
            
            % Begin actual calibration
            obj.beginTrial;
        end
         
        function stopCalibration(obj)
            % Stop and delete timer objects
            obj.figureUpdateTimer.stop;
            delete(obj.figureUpdateTimer);
            obj.figureUpdateTimer=[];
            
            % Close figure
            delete(gcf);
            
            obj.closeActivity;
        end
        
        function updateTrial(obj,~,~)
            try
                if isnan(obj.timeSinceTrialStart)
                    return;
                end
                textString=sprintf('%d',ceil(obj.timingParams.CDlength-obj.timeSinceTrialStart));
                if obj.timeSinceTrialStart>=obj.timingParams.CDlength+obj.timingParams.MIlength
                    obj.endTrial;
                elseif obj.timeSinceTrialStart>=obj.timingParams.CDlength
                    set(obj.figureParams.CDtext,'visible','off');
                else 
                    set(obj.figureParams.CDtext,'string',textString,'visible','on');
                end
            catch ME
                fprintf('%s Line: %d\n',ME.message,ME.stack(1).line)
                keyboard;
            end
        end
        
        function beginTrial(obj)
            try
                if obj.isCalibrating
                    % Decide new target
                    obj.currTarget=rand>.5;
                    
                    % Generate text string, if it does not exist already
                    if ~isfield(obj.figureParams,'CDtext')
                        obj.figureParams.CDtext=text(obj.screenRes(1)*0.5,obj.screenRes(2)*0.4,'75','HorizontalAlignment','Center','VerticalAlignment','Middle','FontSize',64,'visible','off');
                    end
                    
                    % Color selected target
                    set(obj.figureParams.target(obj.currTarget+1),'FaceColor',[.1,.6,.1]);
                    set(obj.figureParams.target(2-obj.currTarget),'FaceColor',[.7,.7,.7]);
                else
                    obj.isTrialOngoing=1;
                end
                
                % Log trial data
                if isfield(obj.outputLog,'trialStartTimes')
                    obj.outputLog.trialStartTimes=cat(1,obj.outputLog.trialStartTimes,obj.currTime);
                else
                    obj.outputLog.trialStartTimes=obj.currTime;
                end
            catch ME
                fprintf('%s Line: %d\n',ME.message,ME.stack(1).line)
                keyboard;
            end
        end
        
        function endTrial(obj)
            % Train classifier and start new trial, if this is calibration session.
            % Otherwise, provide response to Unity interface
            if obj.isCalibrating
                obj.trainClassifier;
                obj.beginTrial;
            else
                obj.UDPtoInterface.step(uint8(num2str(obj.classEst)));
                obj.isTrialOngoing=0;
            end
            
            % Interrupt tactile feedback and clear low-pass classEst
            obj.classOutLP=0;
            fprintf(obj.motorSerialPort,'e12\n');
            fprintf(obj.motorSerialPort,'r0\n');
        end
        
        function updateSTIGdata(obj)
            try
                % Compute an estimation with one each of the available
                % classifiers
                obj.libraryClassification=zeros(1,length(obj.classifiersLibrary));
                for currClsfr=1:length(obj.classifiersLibrary)
                    obj.libraryClassification(currClsfr)=(obj.classifiersLibrary(currClsfr).predict(obj.lastDataBlock)-.5)*2;
                end
                
                % Update log of all classifications
                logEntry(obj,{'libraryClassification'});
                
                % If possible, compute covariance of estimations
                if size(obj.outputLog.libraryClassification,1)<length(obj.classifiersLibrary)
                    obj.estCov(1,:,:)=(obj.outputLog.libraryClassification(end,:)-mean(obj.outputLog.libraryClassification))'*(obj.outputLog.libraryClassification(end,:)-mean(obj.outputLog.libraryClassification));
                end
                
                % Log estimation covariance matrix
                logEntry(obj,{'estCov'});
            catch ME
                fprintf('%s Line: %d\n',ME.message,ME.stack(1).line)
                keyboard;
            end
        end
        
        function trainClassifier(obj)
            try
                % If at least one data window for each direction has been
                % recorded, perform classification
                if isfield(obj.outputLog,'currTarget')&&length(unique(obj.outputLog.currTarget))>1
                    % Compute Karcher means
                    classTags=unique(obj.outputLog.currTarget);
                    obj.calibrationClassifier.karchMeans=zeros(length(classTags),size(obj.outputLog.dataCov,3),size(obj.outputLog.dataCov,3));
                    cellCovHist=cell(size(obj.outputLog.dataCov,1),1);
                    for currTrial=1:size(obj.outputLog.dataCov,1)
                        cellCovHist{currTrial}=squeeze(obj.outputLog.dataCov(currTrial,:,:));
                    end
                    for currClass=1:length(classTags)
                        relIdx=find(obj.outputLog.currTarget==classTags(currClass));
                        obj.calibrationClassifier.karchMeans(currClass,:,:)=karcher(cellCovHist{relIdx}); %#ok<FNDSB>
                    end
                    if ~isfield(obj.calibrationClassifier,'Rdist1')
                        obj.calibrationClassifier.Rdist1=[];
                        obj.calibrationClassifier.Rdist2=[];
                    end
                    obj.calibrationClassifier.Rdist1=cat(1,obj.calibrationClassifier.Rdist1,Rdist(cov(squeeze(obj.lastDataBlock)),squeeze(obj.calibrationClassifier.karchMeans(1,:,:))));
                    obj.calibrationClassifier.Rdist2=cat(1,obj.calibrationClassifier.Rdist2,Rdist(cov(squeeze(obj.lastDataBlock)),squeeze(obj.calibrationClassifier.karchMeans(2,:,:))));
                    obj.calibrationClassifier.predict=@(x)Rdist(cov(squeeze(x)),squeeze(obj.calibrationClassifier.karchMeans(1,:,:)))>Rdist(cov(squeeze(x)),squeeze(obj.calibrationClassifier.karchMeans(2,:,:)));
                    obj.calibrationClassifier.out=@(x)Rdist(cov(squeeze(x)),squeeze(obj.calibrationClassifier.karchMeans(1,:,:)))-Rdist(cov(squeeze(x)),squeeze(obj.calibrationClassifier.karchMeans(2,:,:)));
                end
            catch ME
                fprintf('%s Line: %d\n',ME.message,ME.stack(1).line)
                keyboard;
            end
        end
        
        function estimateIntention(obj,~,~)
            try
                % If last data block entirely occurred after end of count
                % down, start estimation
                if isnan(obj.timeSinceTrialStart)||(obj.isCalibrating&&obj.timeSinceTrialStart<obj.timingParams.winLength+obj.timingParams.CDlength)||(~obj.isCalibrating&&~obj.isTrialOngoing)
                    return;
                end
                
%                 % WARNING: The following will substitute actual data with
%                 % phantom ones. Comment this part when actually running
%                 % code.
%                 isTargetCorrect=rand<1;
%                 if ~isempty(obj.currTarget)
%                     tempTarget=obj.currTarget;
%                     if ~isTargetCorrect
%                         tempTarget=1-tempTarget;
%                     end
%                     obj.lastDataBlock(1,:,:)=[zeros(512*3-16,16);eye(16)]*chol(squeeze(obj.classifiersLibrary(1).karchMeans(tempTarget+1,:,:)));
%                 else
%                     return
%                 end
%                 fprintf('WARNING: phantom data is being processed!\n');
                
                % Compute current datablock covariance matrix
                obj.dataCov=[];
                obj.dataCov(1,:,:)=cov(squeeze(obj.lastDataBlock));
                             
                % Log data
                obj.estimateTimes=obj.currTime;
                obj.logEntry({'estimateTimes','lastDataBlock','dataCov','currTarget'});
                
                % Do not perform estimations unless at least one target per
                % class has been observed
                if obj.isCalibrating&&length(unique(obj.outputLog.currTarget))<2
                    return;
                end
                
                % Update STIG data, then perform STIG estimation, unless
                % experiment is being performed with previously trained
                % classifier
                if obj.isCalibrating||~isfield(obj.calibrationClassifier,'predict')
                    obj.updateSTIGdata;
                    if size(obj.outputLog.libraryClassification,1)<length(obj.classifiersLibrary)
                        % Use mode of existing classifiers for first trials
                        obj.STIGclassEst=mode(obj.outputLog.libraryClassification(end,:));
                        obj.STIGclassOut=mean(obj.outputLog.libraryClassification(end,:));
                    else
                        % Use STIG if number of trials is sufficiently high (i.e.
                        % larger than number of available classifiers)
                        Q=squeeze(sum(obj.outputLog.estCov,1)/(length(obj.outputLog.libraryClassification)-1));
                        [V,D]=eig(Q);
                        maxEigIdx=find(diag(D)==max(diag(D)),1);
                        v=V(:,maxEigIdx);
                        obj.STIGclassOut=sum(obj.outputLog.libraryClassification(end,:).*v');
                        obj.STIGclassEst=sign(obj.STIGclassOut);
                    end
                    obj.STIGclassEst=(obj.STIGclassEst+1)/2;
                    obj.logEntry({'STIGclassEst','STIGclassOut'});
                    
                    % Temporarily set classEst and classOut to STIG values
                    % (will be changed later if calibrated classifier is better
                    % performing)
                    obj.classEst=obj.STIGclassEst;
                    obj.classOut=obj.STIGclassOut;
                end
                
                % If a calibrated classifier exists, make a classification with
                % that, as well
                if isfield(obj.calibrationClassifier,'predict')
                    % Perform prediction with calibration classifier
                    obj.calClassEst=obj.calibrationClassifier.predict(obj.lastDataBlock);
                    obj.calClassOut=obj.calibrationClassifier.out(obj.lastDataBlock);
                    obj.logEntry({'calClassEst','calClassOut'});
                    
                    % If using a previosly calibrated classifier, ignore
                    % STIG results
                    if obj.isCalibrating
                        % Compare BAcc with that of STIG,
                        % on comparable windows
                        calBAcc=testAcc(obj.outputLog.currTarget(end-length(obj.outputLog.calClassEst)+1:end),obj.outputLog.calClassEst);
                        STIGBAcc=testAcc(obj.outputLog.currTarget(end-length(obj.outputLog.calClassEst)+1:end),obj.outputLog.STIGclassEst(end-length(obj.outputLog.calClassEst)+1:end));
                        
                        % Set current classEst as prediction of classifier with
                        % higher BAcc
                        if calBAcc>=STIGBAcc
                            obj.classEst=obj.calClassEst;
                            obj.classOut=obj.calClassOut;
                        else
                            obj.classEst=obj.STIGclassEst;
                            obj.classOut=obj.STIGclassOut;
                        end
                    else
                        obj.classEst=obj.calClassEst;
                        obj.classOut=obj.calClassOut;
                    end
                end
                obj.logEntry({'classEst','classOut'});
                
                % Compute number of relevant classifications (i.e. those
                % belonging to current trial)
                if obj.isCalibrating
                    nRelEst=sum(obj.outputLog.estimateTimes>obj.outputLog.trialStartTimes(end)+obj.timingParams.CDlength+obj.timingParams.winLength);
                else
                    nRelEst=sum(obj.outputLog.estimateTimes>obj.outputLog.trialStartTimes(end)+obj.timingParams.winLength);
                end
                
                % Compute confidence level
                P1=signtest(atan(obj.outputLog.classOut(end-nRelEst+1:end)),0,'tail','left'); % i.e. median is less than zero, intended direction is left
                P2=signtest(atan(obj.outputLog.classOut(end-nRelEst+1:end)),0,'tail','right'); % i.e. median is less greater zero, intended direction is right
                
                % Set serial band vibration level
                if P1~=P2 % They should only be equal on first sample of new trial
                    if P1<P2
                        fprintf(obj.motorSerialPort,'e8\n');
                        fprintf(obj.motorSerialPort,'r0\n');
                        fprintf(obj.motorSerialPort,'e4\n');
                    else
                        fprintf(obj.motorSerialPort,'e4\n');
                        fprintf(obj.motorSerialPort,'r0\n');
                        fprintf(obj.motorSerialPort,'e8\n');
                    end
                    currVibrationValue=round((1-min(P1,P2))*80);
                    fprintf(obj.motorSerialPort,sprintf('r%d\n',currVibrationValue));
                end
                fprintf('P1: %0.2f; P2: %0.2f; nRelEst: %d\n',P1,P2,nRelEst);
                
                % During experiment, conclude trial if either P1 or P2 is
                % low enough and enough time has elapsed since trial start
                if ~obj.isCalibrating&&(P1<obj.confidenceThreshold||P2<obj.confidenceThreshold)&&obj.timeSinceTrialStart>obj.timingParams.CDlength+obj.timingParams.MIlength
                    obj.endTrial;
                end
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
                        obj.outputLog.(fieldNames{currField})=obj.(fieldNames{currField});
                    else
                        obj.outputLog.(fieldNames{currField})=cat(1,obj.outputLog.(fieldNames{currField}),obj.(fieldNames{currField}));
                    end
                end
            catch ME
                fprintf('%s Line: %d\n',ME.message,ME.stack(1).line)
                keyboard;
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
    end
end