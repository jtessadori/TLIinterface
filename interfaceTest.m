classdef interfaceTest < handle
    %% December, 2018 Jacopo Tessadori
    properties
        portToInterface;
        portFromInterface;
        portFromET;
        UDPtoInterface;
        UDPfromInterface;
        UDPfromET;
        updateTimer;
        intercomTimer;
        gazePos;
        screenRes;
    end
    
    methods
        %% Constructor
        function obj=interfaceTest
            % Ports from and toward interface
            obj.portFromInterface=8051;
            obj.portToInterface=9000;
            
            % Port from eye tracker
            obj.portFromET=11000;
        end
        
        %% Other methods
        function startExperiment(obj)
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
            
            % Start a timer to periodically provide an update
            obj.updateTimer=timer;
            obj.updateTimer.ExecutionMode='FixedRate';
            obj.updateTimer.Period=0.05;
            obj.updateTimer.TimerFcn=@obj.update;
            obj.updateTimer.start;
            
            % Start a timer to handle communication with interface
            obj.intercomTimer=timer;
            obj.intercomTimer.ExecutionMode='FixedRate';
            obj.intercomTimer.Period=0.1;
            obj.intercomTimer.TimerFcn=@obj.intercomHandling;
            obj.intercomTimer.start;
        end
        
        function stopExperiment(obj)
            % Release occupied ports
            obj.UDPfromInterface.release;
            obj.UDPfromET.release;

            % Stop and delete timer objects
            obj.updateTimer.stop;
            delete(obj.updateTimer);
            obj.intercomTimer.stop;
            delete(obj.intercomTimer);
        end
        
        function update(obj,~,~)
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
            try
                inString=obj.UDPfromInterface.step;
                if isempty(inString)
                    return;
                end
                fprintf('instring: %s\n',inString);
                outString=uint8(num2str(double(obj.gazePos.Y>obj.screenRes(2)*.5)));
                obj.UDPtoInterface.step(outString);
                fprintf('outstring: %s\n',outString);
            catch ME
                keyboard;
            end
        end
        
        %% Dependent properties
        function res=get.screenRes(~)
            res=get(0,'screensize');
            res=res(3:end);
        end
    end
end