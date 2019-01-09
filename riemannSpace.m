classdef riemannSpace < handle
    %% January, 2019 Jacopo Tessadori
    %% From: https://ieeexplore.ieee.org/abstract/document/6046114
    properties
        Rmean;
        sqrtMean;
        invSqrtMean;
    end
    
    methods
        function obj=riemannSpace(relWins)
            % Compute covariance matrices
            covMats=cell(size(relWins,1),1);
            for currTrial=1:size(relWins,1)
                covMats{currTrial}=cov(squeeze(relWins(currTrial,:,:)));
            end
            
            % Compute Riemannian mean
            obj.Rmean=karcher(covMats{1:size(relWins,1)});
            
            % Compute sqrt of Rmean and inv(Rmean)
            obj.sqrtMean=obj.sqrt(obj.Rmean);
            obj.invSqrtMean=obj.sqrt(inv(obj.Rmean));
        end
        
        function Si=invMap(obj,Pi)
            Si=obj.sqrtMean*obj.log(obj.invSqrtMean*Pi*obj.invSqrtMean)*obj.sqrtMean;
        end
        
        function Pi=map(obj,Si)
            Pi=obj.sqrtMean*obj.exp(obj.invSqrtMean*Si*obj.invSqrtMean)*obj.sqrtMean;
        end
        
        function si=project(obj,Pi)
            si=triu(obj.invSqrtMean*obj.invMap(Pi)*obj.invSqrtMean);
            si=si(triu(true(size(si))));
        end
    end
    
    methods (Static)
        function expP=exp(inData)
            % Performs exp of symmetric positive-definite matrices
            [V,D]=eig(inData);
            expP=V*diag(exp(diag(D)))*V';
        end
        
        function logP=log(inData)
            % Performs exp of symmetric positive-definite matrices
            [V,D]=eig(inData);
            logP=V*diag(log(diag(D)))*V';
        end
        
        function sqrtP=sqrt(inData)
            % Performs (extended) square root of symmetric
            % positive-definite matrices
            [V,D]=eig(inData);
            sqrtP=V*diag(sqrt(diag(D)))*V';
        end
    end
end