function [idxTarget, vale, stcc] = function_stccRecursion(...
    nodo, G, targetNodes, flagLog, idxTarget, vale, stcc)
%%% INPUTS:
%%% Suppose a LTI framework: x(k+1) = Ax(k) + Bu(k), y(k) = Cx(k)
%%% - nodo: current driver, identifyed by B
%%% - G: network described by A
%%% - targetNodes: target nodes in G, described by C
%%% - flagLog: if > 0, apply a logarithmic transformation to the
%%% controllability matrix
%%% - idxTarget: returns the highest ranked configuration of controllable
%%% targets for the driver. Updated during the recursion.
%%% - vale: vale = -2 if no target can be reached by the driver, otherwise
%%% it corresponds to the maximum element of the controllability matrix.
%%% Updated during the recursion.
%%% - stcc: number of controllable targets. Updated during the recursion.


flagTrovato = 0;

subNodi = dfsearch(G,nodo);


for j = (max(idxTarget)+1):size(targetNodes,1)
    if flagTrovato == 0
        
        %%% define targets
        idxTargetTemp = [idxTarget, j];
        
        target = targetNodes(idxTargetTemp);
        
        % target
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Are the targets reachable from the driver?
        if ~isequal(ismember(target, subNodi), ones(size(target,1),1))
            %%% The targets cannot be simultaneously reached, thus they
            %%% cannot simultaneously be controlled.
        else
            subG = subgraph(G, subNodi);
            A = adjacency(subG);
            B = zeros( subG.numnodes, 1);
            [ ~, b] = ismember(nodo,table2cell(subG.Nodes));
            
            B(b) = 1;
            
            T = zeros(size(target,1),subG.numnodes);
            for jj = 1:size(target,1)
                [ ~, b] = ismember(target{jj},table2cell(subG.Nodes));
                T(jj,b) = 1;
            end
            
            
            if flagLog == 0
                CO = ctrb(A',B);
            else
                COlog = ctrb(A',B);
                COlog(COlog==0) = NaN;
                COlog = log10(COlog) + 1;
                COlog(isnan(COlog)) = 0;
                CO = COlog;
            end
            
            rango = rank( T*CO );
            
            
            if rango == length(target)
                %%% all the targets can be controlled simultaneously
                
                vale = max(max(CO));
                flagTrovato = 1;
                idxTarget = idxTargetTemp;
                stcc = length(idxTargetTemp);
                
                %%%% recursion
                [idxTarget, vale, stcc] = function_stccRecursion( ...
                    nodo, G, targetNodes, flagLog, idxTarget, vale, stcc);
            end
        end
        
    end
end
end

