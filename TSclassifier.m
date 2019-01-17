classdef TSclassifier < handle
    %% January, 2019 Jacopo Tessadori
    methods (Static)
        function s=projectData(inData,RS)
            % Projects all entries in inData to tangent space defined by RS
            if iscell(inData)
                s=zeros(size(inData,1),size(inData{1},2)*(size(inData{1},2)+1)/2);
                for currTrial=1:size(inData,1)
                    s(currTrial,:)=RS.project(cov(inData{currTrial}));
                end
            else
                s=zeros(size(inData,1),size(inData,3)*(size(inData,3)+1)/2);
                for currTrial=1:size(inData,1)
                    s(currTrial,:)=RS.project(cov(squeeze(inData(currTrial,:,:))));
                end
            end
        end
        
        function featsIdx=selectFeatures(inData,lbls,weights)
            % Compute one-way ANOVA for each feature, then sort
            % p-values
            p=zeros(size(inData,2),1);
            for currFeat=1:size(inData,2)
                p(currFeat)=anova1(inData(:,currFeat),lbls,'off');
            end
            [pSorted,pIdx]=sort(p);
            
            % Compute number of features to keep
            th=0.05;
            nFeatures=2;
            while true
                if sum(weights(pIdx(1:nFeatures)))/size(inData,2)*th<=pSorted(nFeatures)||nFeatures==length(p)
                    break;
                else
                    nFeatures=nFeatures+1;
                end
            end
            featsIdx=pIdx(1:nFeatures);
        end
        
        function [BAcc,classEst]=crossVal(data,lbls,nPartitions)
            % Generate partitions
            C.NumTestSets=nPartitions;
            C.groups=ceil(linspace(1/length(lbls),C.NumTestSets,length(lbls)));
            C.training=@(currGroup)C.groups~=currGroup;
            C.test=@(currGroup)C.groups==currGroup;
            
            % Perform cross-validation
            classEst=zeros(size(lbls));
            for currP=1:C.NumTestSets
                % Recover training and testing sets
                trainData=data(C.training(currP),:,:);
                trainLbls=lbls(C.training(currP));
                testLbls=lbls(C.test(currP));
                
                % Compute Riemann-mean on training data only
                RS=riemannSpace(trainData);
                
                % Project both train and test data to tangent space
                s=TSclassifier.projectData(data,RS);
                
                % Split projections in training and testing set
                trainProj=s(C.training(currP),:);
                testProj=s(C.test(currP),:);
                
                % Perform PCA
                [coeff,trainProj,latent]=pca(trainProj);
                testProj=testProj*coeff;
                
                % Perform feature selection
                featsIdx=TSclassifier.selectFeatures(trainProj,trainLbls,latent);
                fprintf('nFeats: %d\n',length(featsIdx));
                
                % Leave only selected features
                trainProj=trainProj(:,featsIdx);
                testProj=testProj(:,featsIdx);
                
                % Perform training 
%                 clsfr=fitcsvm(trainProj,trainLbls,'Standardize',true,'KernelScale','auto','KernelFunction','polynomial','PolynomialOrder',2);
%                 clsfr=fitcsvm(trainProj,trainLbls,'KernelFunction', 'gaussian','KernelScale', 16,'BoxConstraint', 1,'Standardize', true,'ClassNames', [0; 1]);
                clsfr=fitcdiscr(trainProj,trainLbls);
%                 clsfr = fitcensemble(trainProj,trainLbls,'Method','RobustBoost','NumLearningCycles',300,'Learners','Tree','RobustErrorGoal',0.002,'RobustMaxMargin',1);
                
                % Classify test projections
                classEst(C.test(currP))=clsfr.predict(testProj);
                
                % Evaluate results for current partition
                partBAcc=testAcc(testLbls,classEst(C.test(currP)));
                fprintf('Fold %d/%d BAcc: %0.2f\n',currP,C.NumTestSets,partBAcc);
            end
            BAcc=testAcc(lbls,classEst);
            fprintf('\n Cross-val BAcc: %0.2f\n\n',BAcc);
        end
        
        function plotTSproj(data,lbls)
            % Compute Riemann-mean on training data only
            RS=riemannSpace(data);
            
            % Project both train and test data to tangent space
            data=TSclassifier.projectData(data,RS);
            
            % Perform PCA
            [~,data]=pca(data);
            
            % Compute one-way ANOVA for each feature, then sort
            % p-values
            p=zeros(size(data,2),1);
            for currFeat=1:size(data,2)
                p(currFeat)=anova1(data(:,currFeat),lbls,'off');
            end
            [~,pIdx]=sort(p);
            
            % Plot first three features
            clrs='rgkbcmy';
            lblValues=unique(lbls);
            hold on
            for currValue=1:length(lblValues)
                plot3(data(lbls==lblValues(currValue),pIdx(1)),data(lbls==lblValues(currValue),pIdx(2)),data(lbls==lblValues(currValue),pIdx(3)),['o',clrs(currValue)]);
            end
        end
        
        function [BAcc,classEst]=crossValTime(data,lbls,nPartitions)
            % Generate partitions
            C.NumTestSets=nPartitions;
            C.groups=ceil(linspace(1/length(lbls),C.NumTestSets,length(lbls)));
            C.training=@(currGroup)C.groups~=currGroup;
            C.test=@(currGroup)C.groups==currGroup;
            
            % Perform cross-validation
            classEst=zeros(size(lbls));
            for currP=1:C.NumTestSets
                % Recover training and testing sets
                trainData=data(C.training(currP),:,:);
                trainLbls=lbls(C.training(currP));
                testLbls=lbls(C.test(currP));
                
                % Compute class means
                classTags=unique(lbls);
                classMeans=zeros(length(classTags),size(data,2),size(data,3));
                for currClass=1:length(classTags)
                    classMeans(currClass,:,:)=median(trainData(trainLbls==classTags(currClass),:,:));
                end
                clsfr.classMeans=classMeans;
                
                % Construct super-trials
                superTrainFeats=zeros(size(data,1),size(data,2),size(data,3)*(length(classTags)+1));
                for currTrial=1:size(data,1)
                    superTrainFeats(currTrial,:,:)=cat(2,reshape(permute(classMeans,[1,3,2]),[],size(data,2))',squeeze(data(currTrial,:,:)));
                end
                
                % Compute Riemann-mean on training data only
                RS=riemannSpace(superTrainFeats);
                
                % Project both train and test data to tangent space
                s=TSclassifier.projectData(superTrainFeats,RS);
                
                % Split projections in training and testing set
                trainProj=s(C.training(currP),:);
                testProj=s(C.test(currP),:);
                
                % Perform PCA
                [coeff,trainProj,latent]=pca(trainProj);
                testProj=testProj*coeff;
                
                % Perform feature selection
                featsIdx=TSclassifier.selectFeatures(trainProj,trainLbls,latent);
                fprintf('nFeats: %d\n',length(featsIdx));
                
                % Leave only selected features
                trainProj=trainProj(:,featsIdx);
                testProj=testProj(:,featsIdx);
                
                % Perform training
%                 clsfr=fitcsvm(trainProj,trainLbls,'Standardize',true,'KernelScale','auto','KernelFunction','polynomial','PolynomialOrder',2);
%                 clsfr=fitcsvm(trainProj,trainLbls,'KernelFunction', 'gaussian','KernelScale', 16,'BoxConstraint', 1,'Standardize', true,'ClassNames', [0; 1]);
                clsfr=fitcdiscr(trainProj,trainLbls);
%                 clsfr = fitcensemble(trainProj,trainLbls,'Method','RobustBoost','NumLearningCycles',300,'Learners','Tree','RobustErrorGoal',0.15,'RobustMaxMargin',1);
                
                % Classify test projections
                classEst(C.test(currP))=clsfr.predict(testProj);
                clear clsfr
                
                % Evaluate results for current partition
                partBAcc=testAcc(testLbls,classEst(C.test(currP)));
                fprintf('Fold %d/%d BAcc: %0.2f\n',currP,C.NumTestSets,partBAcc);
            end
            BAcc=testAcc(lbls,classEst);
            fprintf('\n Cross-val BAcc: %0.2f\n\n',BAcc);
        end
    end
end