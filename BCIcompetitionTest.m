clear
% close all;
clc;

% Recover filenames of all experiments
pathName='C:\Data\BCIcompetition\BCICIV_2a_gdf\';
fileList=dir(pathName);
fileNames={fileList.name};

% For each experiment, perform classification
BACC=zeros(9,1);
for currSubj=1:9
    % Find relevant files for current subject
    relFiles=find(cellfun(@(x)numel(strfind(x,sprintf('0%d',currSubj))>0),fileNames));
    
    % Load each file and add either labels or data
    lbls=[];
    relWins=[];
    for currFile=1:length(relFiles)
        if numel(strfind(fileNames{relFiles(currFile)},'.mat'))>0
            load(sprintf('%s%s',pathName,fileNames{relFiles(currFile)}));
            lbls=cat(1,lbls,classlabel);
        else
            HDR=sopen(sprintf('%s%s',pathName,fileNames{relFiles(currFile)}));
            [S,HDR]=sread(HDR);
            
            % Fill NaNs with random data
            n=randn(size(S));
            S(isnan(S))=n(isnan(S));
            
            % Apply freq filters
            [B,A]=butter(2,[8,30]/(HDR.SampleRate/2));
            freqData=filter(B,A,S);
            
            % Convert formats
            relEvents=(HDR.EVENT.TYP==769)+(HDR.EVENT.TYP==770)+(HDR.EVENT.TYP==771)+(HDR.EVENT.TYP==772)+(HDR.EVENT.TYP==783);
            relIdx=find(relEvents);
            tempWins=zeros(sum(relEvents),2*HDR.SampleRate,22);
            for currTrial=1:length(relIdx)
                tempWins(currTrial,:,:)=freqData(HDR.EVENT.POS(relIdx(currTrial))+HDR.SampleRate*.5+1:HDR.EVENT.POS(relIdx(currTrial))+HDR.SampleRate*2.5,1:22);
            end
            relWins=cat(1,relWins,tempWins);
        end
    end
    
%     % Reduce classes to two
%     relWins(lbls>2,:,:)=[];
%     lbls(lbls>2)=[];

    % Perform classification
    BACC(currSubj)=TSclassifier.crossVal(relWins,lbls,5);
    
    % Close file
    HDR=sclose(HDR);
end