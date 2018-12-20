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
        gazePos;
        screenRes;
        fileName;
        fs;
        outputLog;
        listener;
        classifiersLibrary;
        bufferHistory;
        classEst;
        STIGclassEst;
        calClassEst;
        classOut;
        STIGclassOut;
        calClassOut;
        classOutLP=0;
        classHist;
        estCovHist;
        dataCovHist;
        currTime;
        motorSerialPort;
        figureParams;
        currTarget;
        calibrationClassifier;
        deadZone=0.2;
    end
    
    properties (Hidden)
        modelName='TLIinterface_SimulinkSupport';
        instanceName;
        lastDataBlock;
        timingParams;
        estCov;
        isCalibrating=0;
        retry=0;
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
            
            % Determine name to be used to save file upon closing
            obj.fileName=datestr(now,30);

            % Prepares serial port for vibrating motor control
            obj.prepareSerialPort;
            
            % Start Simulink model
            obj.prepareSimulinkModel;
            
            % Perform a countdown to allow amplifier to settle
            obj.startCountdown(20);
            
            % Start logging trial times (first one is going to be
            % inccorect, but there's no way of actually knowing when
            % Fanny's interface launches)
            obj.outputLog.trialStartTimes=obj.currTime;
            
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
            obj.estimateTimer.Period=0.2;
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
            % Define time paramters for calibration
            obj.timingParams.preCDlength=1;
            obj.timingParams.CDlength=5;
            obj.timingParams.postCDlength=.5;
            
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
                if isempty(obj.outputLog)||~isfield(obj.outputLog,'trialStartTimes')
                    return;
                end
                timeSinceTrialStart=obj.currTime-obj.outputLog.trialStartTimes(end);
                textString=sprintf('%d',ceil(obj.timingParams.preCDlength+obj.timingParams.CDlength-(obj.currTime-obj.outputLog.trialStartTimes(end))));
                if timeSinceTrialStart>=obj.timingParams.preCDlength+obj.timingParams.CDlength+obj.timingParams.postCDlength
                    obj.beginTrial;
                elseif timeSinceTrialStart>=obj.timingParams.preCDlength+obj.timingParams.CDlength
                    set(obj.figureParams.CDtext,'visible','off');
                elseif timeSinceTrialStart>=obj.timingParams.preCDlength
                    set(obj.figureParams.CDtext,'string',textString,'visible','on');
                end
%                 fprintf('Pos: %0.4f, speed: %0.4f\n',obj.cursorPos,obj.cursorSpeed);
            catch ME
                fprintf('%s Line: %d\n',ME.message,ME.stack(1).line)
                keyboard;
            end
        end
        
        function beginTrial(obj)
            try
                % Log incoming data and covariance matrix, if one is available
                % and labelled
                if ~isempty(obj.lastDataBlock)&&~(length(unique(obj.lastDataBlock))==1)&&~isempty(obj.currTarget)
%                     %                 isTargetCorrect=rand<0.55;
%                     if ~isempty(obj.currTarget)
%                         isTargetCorrect=1;
%                         tempTarget=obj.currTarget;
%                         if ~isTargetCorrect
%                             tempTarget=1-tempTarget;
%                         end
%                         obj.lastDataBlock=[zeros(size(obj.lastDataBlock,1)-16,16);eye(16)]*chol(squeeze(obj.classifiersLibrary(1).karchMeans(tempTarget+1,:,:)))*sqrt(size(obj.lastDataBlock,1));
%                     end
                    reshapedData=reshape(obj.lastDataBlock,1,size(obj.lastDataBlock,1),size(obj.lastDataBlock,2));
                    if isempty(obj.bufferHistory)
                        obj.bufferHistory=reshapedData;
                        obj.dataCovHist=reshape(cov(squeeze(reshapedData(:,end-obj.timingParams.winLength*obj.fs+1:end,:))),1,size(reshapedData,3),size(reshapedData,3));
                        obj.outputLog.targetLog=obj.currTarget;
                    else
                        obj.bufferHistory=cat(1,obj.bufferHistory,reshapedData);
                        obj.dataCovHist=cat(1,obj.dataCovHist,reshape(cov(squeeze(reshapedData(:,end-obj.timingParams.winLength*obj.fs+1:end,:))),1,size(reshapedData,3),size(reshapedData,3)));
                        obj.outputLog.targetLog=cat(1,obj.outputLog.targetLog,obj.currTarget);
                    end
                    
                    % Log current classification
                    if ~isempty(obj.STIGclassEst)
                        if ~isfield(obj.outputLog,'STIGclassEst')
                            obj.outputLog.STIGclassEst=obj.STIGclassEst;
                        else
                            obj.outputLog.STIGclassEst=cat(1,obj.outputLog.STIGclassEst,obj.STIGclassEst);
                        end
                    end
                    if ~isempty(obj.calClassEst)
                        if ~isfield(obj.outputLog,'calClassEst')
                            obj.outputLog.calClassEst=obj.calClassEst;
                        else
                            obj.outputLog.calClassEst=cat(1,obj.outputLog.calClassEst,obj.calClassEst);
                        end
                    end
                    if ~isempty(obj.calClassEst)
                        if isfield(obj.outputLog,'classEst')
                            obj.outputLog.classEst=cat(1,obj.outputLog.classEst,obj.classEst);
                        else
                            obj.outputLog.classEst=obj.classEst;
                        end
                    end
                    
                    % Update STIG data
                    obj.updateSTIGdata;
                end
                
                % Decide new target
                obj.currTarget=rand>.5;
                
                % Generate text string, if it does not exist already
                if ~isfield(obj.figureParams,'CDtext')
                    obj.figureParams.CDtext=text(obj.screenRes(1)*0.5,obj.screenRes(2)*0.4,'75','HorizontalAlignment','Center','VerticalAlignment','Middle','FontSize',64,'visible','off');
                end
                
                % Color selected target
                set(obj.figureParams.target(obj.currTarget+1),'FaceColor',[.1,.6,.1]);
                set(obj.figureParams.target(2-obj.currTarget),'FaceColor',[.7,.7,.7]);
                
                % Interrupt tactile feedback and clear low-pass classEst
                obj.classOutLP=0;
                fprintf(obj.motorSerialPort,'e12\n');
                fprintf(obj.motorSerialPort,'r0\n');
                
                % Train classifier
                obj.trainClassifier;
                
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
        
        function updateSTIGdata(obj)
            % Compute an estimation with one each of the available
            % classifiers
            tempEst=zeros(1,length(obj.classifiersLibrary));
            for currClsfr=1:length(obj.classifiersLibrary)
                tempEst(currClsfr)=(obj.classifiersLibrary(currClsfr).predict(obj.lastDataBlock)-.5)*2;
            end
            if isempty(obj.classHist)
                obj.classHist=tempEst;
            else
                obj.classHist=cat(1,obj.classHist,tempEst);
            end
            if size(obj.classHist,1)<length(obj.classifiersLibrary)%||length(unique(classEst(1:currTrial)))<3%||C<3
                obj.estCov=(obj.classHist(end,:)-mean(obj.classHist))'*(obj.classHist(end,:)-mean(obj.classHist));
            end
            
            % Load covariance matrix
            if ~isempty(obj.estCov)
                if isempty(obj.estCovHist)
                    obj.estCovHist=reshape(obj.estCov,1,size(obj.estCov,1),size(obj.estCov,2));
                else
                    obj.estCovHist=cat(1,obj.estCovHist,reshape(obj.estCov,1,size(obj.estCov,1),size(obj.estCov,2)));
                end
            end
        end
        
        function trainClassifier(obj)
            try
                % If at least one data window for each direction has been
                % recorded, perform classification
                if isfield(obj.outputLog,'targetLog')&&length(unique(obj.outputLog.targetLog))>1
                    % Compute Karcher means
                    classTags=unique(obj.outputLog.targetLog);
                    obj.calibrationClassifier.karchMeans=zeros(length(classTags),size(obj.dataCovHist,3),size(obj.dataCovHist,3));
                    cellCovHist=cell(size(obj.dataCovHist,1),1);
                    for currTrial=1:size(obj.dataCovHist,1)
                        cellCovHist{currTrial}=squeeze(obj.dataCovHist(currTrial,:,:));
                    end
                    for currClass=1:length(classTags)
                        relIdx=find(obj.outputLog.targetLog==classTags(currClass));
%                         tempClassEst=[obj.outputLog.targetLog(1:length( obj.outputLog.targetLog)-length(obj.calClassEst));obj.calClassEst];
%                         relIdx=find(obj.outputLog.targetLog==classTags(currClass)&obj.outputLog.targetLog==tempClassEst);
%                         relIdx=relIdx(max(1,end-200):end);
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
                % If no data blocks are availble, abort estimation.
                % Otherwise, recover data belonging to this trial
                if isempty(obj.lastDataBlock)||length(unique(obj.lastDataBlock))==1||(isempty(obj.outputLog)&&obj.isCalibrating)
                    return;
                end
                timeSinceTrialStart=obj.currTime-obj.outputLog.trialStartTimes(end);
                if obj.isCalibrating
                    if timeSinceTrialStart<obj.timingParams.preCDlength||timeSinceTrialStart>=obj.timingParams.preCDlength+obj.timingParams.CDlength
                        return;
                    end
                end
%                     %                 isTargetCorrect=rand<0.55;
%                     if ~isempty(obj.currTarget)
%                         isTargetCorrect=1;
%                         tempTarget=obj.currTarget;
%                         if ~isTargetCorrect
%                             tempTarget=1-tempTarget;
%                         end
%                         obj.lastDataBlock=[zeros(size(obj.lastDataBlock,1)-16,16);eye(16)]*chol(squeeze(obj.classifiersLibrary(1).karchMeans(tempTarget+1,:,:)))*sqrt(size(obj.lastDataBlock,1));
%                     end
                if obj.isCalibrating
                    relSamples=(timeSinceTrialStart-obj.timingParams.preCDlength)*obj.fs;
                else
                    relSamples=timeSinceTrialStart*obj.fs;
                end
                relSamples=min(relSamples,obj.fs*obj.timingParams.winLength);
                relData=obj.lastDataBlock(max(end-relSamples+1,1):end,:)*sqrt(relSamples/size(obj.lastDataBlock,1));
                if size(relData,1)<3
                    return;
                end
                if ~isfield(obj.outputLog,'estimateTimes')
                    obj.outputLog.estimateTimes=obj.currTime;
                else
                    obj.outputLog.estimateTimes=cat(1,obj.outputLog.estimateTimes,obj.currTime);
                end

                if isempty(obj.classHist)
                    obj.STIGclassEst=obj.currTarget;
                    obj.STIGclassOut=0;
                else
                    if size(obj.classHist,1)<length(obj.classifiersLibrary)%||length(unique(classEst(1:currTrial)))<3%||C<3
                        % Use mode of existing classifiers for first trials
                        obj.STIGclassEst=mode(obj.classHist(end,:));
                    else
                        % Use STIG if number of trials is sufficiently high (i.e.
                        % larger than number of available classifiers)
                        Q=squeeze(sum(obj.estCovHist,1)/(length(obj.classHist)-1));
                        [V,D]=eig(Q);
                        maxEigIdx=find(diag(D)==max(diag(D)),1);
                        v=V(:,maxEigIdx);
                        obj.STIGclassOut=sum(obj.classHist(end,:).*v');
                        obj.STIGclassEst=sign(obj.STIGclassOut);
                    end
                    obj.STIGclassEst=(obj.STIGclassEst+1)/2;
                end
                obj.classEst=obj.STIGclassEst;
                obj.classOut=obj.STIGclassOut;
                
                % If calibrated classifier, make a classification with
                % that, as well
%                 calBAcc=0;
%                 STIGBAcc=0;
                if isfield(obj.calibrationClassifier,'predict')
                    % Perform prediction with calibration classifier
                    obj.calClassEst=obj.calibrationClassifier.predict(relData);
                    obj.calClassOut=obj.calibrationClassifier.out(relData);
                    
                    if isfield(obj.outputLog,'calClassEst')
                        % Compare BAcc with that of STIG,
                        % on comparable windows
                        calBAcc=testAcc(obj.outputLog.targetLog(end-length(obj.outputLog.calClassEst)+1:end),obj.outputLog.calClassEst);
                        STIGBAcc=testAcc(obj.outputLog.targetLog(end-length(obj.outputLog.calClassEst)+1:end),obj.outputLog.STIGclassEst(end-length(obj.outputLog.calClassEst)+1:end));
                        
                        % Set current classEst as prediction of classifier with
                        % higher BAcc
                        if calBAcc>=STIGBAcc
                            obj.classEst=obj.calClassEst;
                            obj.classOut=obj.calClassOut;
                        else
                            obj.classEst=obj.STIGclassEst;
                            obj.classOut=obj.STIGclassOut;
                        end
                    end
                end
                if ~obj.isCalibrating
                    obj.classOut=obj.calClassOut;
                end
%                 fprintf('currTarget: %d, calEst: %d, STIGest: %d, calBAcc: %0.2f, STIGBAcc: %0.2f\n',obj.currTarget,obj.calClassEst,obj.STIGclassEst,calBAcc,STIGBAcc);
                
                % Set serial band vibration level
                obj.classOutLP=obj.classOutLP+obj.classOut*.1;
                if obj.classOutLP<0
                    fprintf(obj.motorSerialPort,'e8\n');
                    fprintf(obj.motorSerialPort,'r0\n');
                    fprintf(obj.motorSerialPort,'e4\n');
                else
                    fprintf(obj.motorSerialPort,'e4\n');
                    fprintf(obj.motorSerialPort,'r0\n');
                    fprintf(obj.motorSerialPort,'e8\n');
                end
                currVibrationValue=round(abs(obj.classOutLP)*400);
                fprintf(obj.motorSerialPort,sprintf('r%d\n',currVibrationValue));
%                 fprintf('classOutLP: %0.2f\n',obj.classOutLP);
%                 fprintf('vibration value: %d, classOutLP: %0.2f\n',currVibrationValue,obj.classOutLP);
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
                fprintf('instring: %s\n',inString);
                
                % Determine value to send to interface, then send it
                outValue=obj.classOut;%sign(obj.classOutLP)*(max(0,abs(obj.classOutLP)-obj.deadZone));
                fprintf('outValue: %0.2f, classOutLP: %0.2f\n',outValue,obj.classOutLP);
                if isempty(outValue) % Should only occur on first trial
                    outValue=-1;
                end
                if abs(obj.classOutLP)<obj.deadZone
                    obj.retry=1;
                    return;
                end
                obj.retry=0;
                %             outString=uint8(num2str(double(obj.gazePos.Y>obj.screenRes(2)*.5)));
                outString=uint8(num2str(outValue>0));
                obj.UDPtoInterface.step(outString);
                obj.updateSTIGdata;
                
                % Interrupt tactile feedback and clear low-pass classEst
                obj.classOutLP=0;
                fprintf(obj.motorSerialPort,'e12\n');
                fprintf(obj.motorSerialPort,'r0\n');
                obj.outputLog.trialStartTimes=cat(1,obj.outputLog.trialStartTimes,obj.currTime);
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
            obj.lastDataBlock=block.OutputPort(1).Data;
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
    end
end