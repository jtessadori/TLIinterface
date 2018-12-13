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
        gazePos;
        screenRes;
        fileName;
        fs;
        outputLog;
        listener;
    end
    
    properties (Hidden)
        modelName='TLIinterface_SimulinkSupport';
        instanceName;
        lastDataBlock;
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
        function startExperiment(obj)
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
            
            % Determine name to be used to save file upon closing
            obj.fileName=datestr(now,30);
            
%             % Prompts user to select a condition
%             selectCondition(obj);
%             setConditionSpecificParams(obj);
            
            % Start Simulink model
            obj.prepareSimulinkModel;
            
            % Close required executables, in case they are still open
%             !taskkill -f -im GazeTrackEyeXGazeStream.exe
            !taskkill -f -im TLIinterface.exe

            % Launch gaze tracker and Unity interface
%             !C:\Code\Sources\GazeTrackEyeXGazeStream\GazeTrackEyeXGazeStream.exe &
            !C:\Code\2018_12_TLI\TLIinterface\build\TLIinterface.exe &
            
            % Open incoming and outgoing udp ports
            obj.UDPtoInterface=dsp.UDPSender('RemoteIPPort',obj.portToInterface);
            obj.UDPfromInterface=dsp.UDPReceiver('LocalIPPort',obj.portFromInterface,'ReceiveBufferSize',26000,'MaximumMessageLength',26000);
            obj.UDPfromET=dsp.UDPReceiver('LocalIPPort',obj.portFromET,'ReceiveBufferSize',26000,'MaximumMessageLength',26000);
            
            % Start a timer to periodically provide an estimation of
            % current intention
            obj.estimateTimer=timer;
            obj.estimateTimer.ExecutionMode='FixedRate';
            obj.estimateTimer.Period=0.2;
            obj.estimateTimer.TimerFcn=@obj.estimateIntention;
            start(obj.estimateTimer);
            
            % Start a timer to handle communication with interface
            obj.intercomTimer=timer;
            obj.intercomTimer.ExecutionMode='FixedRate';
            obj.intercomTimer.Period=0.1;
            obj.intercomTimer.TimerFcn=@obj.intercomHandling;
            start(obj.intercomTimer);
        end
        
        function stopExperiment(obj)
            % Release occupied ports
            obj.UDPfromInterface.release;
            obj.UDPfromET.release;

            % Stop and delete timer objects
            obj.estimateTimer.stop;
            delete(obj.estimateTimer);
            obj.intercomTimer.stop;
            delete(obj.intercomTimer);
            obj.estimateTimer=[];
            obj.intercomTimer=[];
            
            % Stop Simulink model
            obj.listener=[];
            set_param(obj.modelName,'SimulationCommand','Stop');
            set_param(obj.modelName,'StartFcn','');
        end
        
        function estimateIntention(obj,~,~)
%             try                
%                 % Recover latest gaze position and update relevant property
%                 newEntry='something';
%                 while ~isempty(newEntry)
%                     previousEntry=newEntry;
%                     newEntry=obj.UDPfromET.step;
%                 end
%                 gazeString=char(previousEntry)';
%                 if strcmp(gazeString','something') % Apparently, this can occur because of simultaneous executions
%                     return;
%                 end
%                 separatorPos=strfind(gazeString,';');
%                 if isempty(separatorPos)
%                     return;
%                 end
%                 obj.gazePos.X=str2double(gazeString(1:separatorPos-1));
%                 obj.gazePos.Y=str2double(gazeString(separatorPos+1:end));
%                 
%                 % Print gaze position in console
% %                 fprintf('%s\n',previousEntry);
%             catch ME
%                 keyboard;
%             end
            % Eye-tracker not working. Using mouse position as a surrogate
            coords=get(0, 'PointerLocation');
            obj.gazePos.X=coords(1);
            obj.gazePos.Y=coords(2);
        end
        
        function intercomHandling(obj,~,~)
            inString=obj.UDPfromInterface.step;
            if isempty(inString)
                return;
            end
            fprintf('instring: %s\n',inString);
            outString=uint8(num2str(double(obj.gazePos.Y>obj.screenRes(2)*.5)));
            obj.UDPtoInterface.step(outString);
            fprintf('outstring: %s\n',outString);
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
                
        %% Dependent properties
        function res=get.screenRes(~)
            res=get(0,'screensize');
            res=res(3:end);
        end
        
        function samplingRate=get.fs(~)
            samplingRate=512;
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
            commandString=sprintf('%s.listener=add_exec_event_listener(''%s'',''PostOutputs'',@%s.newDataAvailable);',instanceName,blockName,instanceName);
            evalin('base',commandString);
        end
    end
end