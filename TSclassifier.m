classdef TSclassifier < handle
    %% January, 2019 Jacopo Tessadori
    methods (Static)
        function BAcc=crossVal(data,lbls,nPartitions)
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
                s=zeros(length(lbls),size(data,3)*(size(data,3)+1)/2);
                for currTrial=1:size(data,1)
                    s(currTrial,:)=RS.project(cov(squeeze(data(currTrial,:,:))));
                end
                
                % Split projections in training and testing set
                trainProj=s(C.training(currP),:);
                testProj=s(C.test(currP),:);
                
                % Perform PCA
                [coeff,trainProj,latent]=pca(trainProj);
                testProj=testProj*coeff;
                
                % Compute one-way ANOVA for each feature, then sort
                % p-values
                p=zeros(size(trainProj,2));
                for currFeat=1:size(trainProj,2)
                    p(currFeat)=anova1(trainProj(:,currFeat),trainLbls,'off');
                end
                [pSorted,pIdx]=sort(p);
                
                % Compute number of features to keep
                th=0.05;
                nFeatures=1;
                while true
                    if sum(latent(pIdx(1:nFeatures)))/size(trainProj,2)*th<=pSorted(nFeatures)
                        break;
                    else
                        nFeatures=nFeatures+1;
                    end
                end
                featsIdx=pIdx(1:nFeatures);
                
                % Leave only selected features
                trainProj=trainProj(:,featsIdx);
                testProj=testProj(:,featsIdx);
                
                % Perform training (second order polynomial svm, in this
                % case)
%                 clsfr=fitcsvm(trainProj,trainLbls,'Standardize',true,'KernelScale','auto','KernelFunction','polynomial','PolynomialOrder',2);
%                 clsfr=fitcsvm(trainProj,trainLbls,'KernelFunction', 'gaussian','KernelScale', 16,'BoxConstraint', 1,'Standardize', true,'ClassNames', [0; 1]);
                clsfr=fitcdiscr(trainProj,trainLbls);
                
                % Classify test projections
                classEst(C.test(currP))=clsfr.predict(testProj);
                
                % Evaluate results for current partition
                partBAcc=testAcc(testLbls,classEst(C.test(currP)));
                fprintf('Fold %d/%d BAcc: %0.2f\n',currP,C.NumTestSets,partBAcc);
            end
            BAcc=testAcc(lbls,classEst);
            fprintf('\n Cross-val BAcc: %0.2f\n\n',BAcc);
        end
    end
end