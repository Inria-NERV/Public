function [idxTarget, vale, stcc] = function_stepwiseKalmanCriterion(...
    G, currentTargets, rangeH, flagLog, flagSubGtar)
%%% INPUTS:
%%% Suppose a LTI framework: x(k+1) = Ax(k) + Bu(k), y(k) = Cx(k)
%%% - G: network described by A, it must be connected
%%% - currentTargets: target nodes in G, described by C
%%% - rangeH: nodes that have to be tested as single drivers (described by
%%% B)
%%% - flagLog: if > 0, apply a logarithmic transformation to the
%%% controllability matrix
%%% - flagSubGtar: if > 0, remove nodes that are not on a path from the
%%% driver to the targets in the computation of the controllability matrix
%%% OUTPUTS:
%%% - idxTarget: returns the highest ranked configuration of controllable
%%% targets, for each driver in rangeH
%%% - vale: for each driver in rangeH. vale = -2 if no target can be
%%% reached, otherwise it corresponds to the maximum element of the
%%% controllability matrix
%%% - stcc: number of controllable targets, for each driver in rangeH


nomiGeni = table2cell(G.Nodes);

idxTarget = cell(  length(rangeH),1 );
stcc       = zeros( length(rangeH),1 );
vale      = zeros( length(rangeH),1 );


targetNodes = nomiGeni(currentTargets);
dr = 0;
for h = rangeH %%% e.g. rangeH = 1:N
    dr  = dr + 1;
    nodo = G.Nodes{h,1}{1};
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% def driver
    
    subNodi = dfsearch(G, nodo);
    raggiungibili = intersect(subNodi, targetNodes);
    
    if ~isempty(raggiungibili)
        
        idxIni    = 1;
        flagWhile = 1;
        
        while flagWhile
            
            flagWhile = 0;
            
            %%% Find the first target that can be controlled.
            %%% It has to be accessible
            continua = 0;
            for idx = idxIni:length(targetNodes)
                if continua == 0
                    
                    if ismember(targetNodes{idx}, subNodi)
                        continua = 1;
                        idxIni = idx;
                    end
                end
            end
            
            
            
            if idxIni == 0
                %%% no target can be reached (controlled by itself)
                vale(dr) = -2;
                stcc(dr)  = 0;
            elseif idxIni > 0 && idxIni < length(targetNodes)
                %%% start recursion from this point
                idxTarget{dr} = idxIni;
                subG = subgraph(G,subNodi);
                
                if flagSubGtar == 1
                    d = distances(subG);
                    %%% d(i,j) is the length of the shortest path between
                    %%% node i and node j
                    [~,subGa] = ismember(nodo,table2cell(subG.Nodes));
                    [~,subGb] = ismember(...
                        targetNodes,table2cell(subG.Nodes));
                    subGb(subGb==0) = [];
                    [~, rimuovi] = ismember(...
                        d(:,subGb), Inf(1,length(subGb)), 'rows' );
                    rimuovi([subGa;subGb]) = 0;
                    
                    
                    subGtar = subG.rmnode(subG.Nodes{rimuovi==1,1});
                    
                    
                    [idxTarget{dr}, vale(dr), stcc(dr)] = ...
                        function_stccRecursion(...
                        nodo, subGtar, targetNodes, flagLog, ...
                        idxTarget{dr}, vale(dr), stcc(dr));
                    
                else
                    [idxTarget{dr}, vale(dr), stcc(dr)] = ...
                        function_stccRecursion(...
                        nodo, subG, targetNodes, flagLog, ...
                        idxTarget{dr}, vale(dr), stcc(dr));
                    
                end
                
                %%% Here if the first cannot be controlled together with
                %%% any other one???
                if vale(dr) == 0 && stcc(dr) == 0
                    
                    if idxIni + 1 == length(targetNodes)
                        
                        if ismember(targetNodes{idxIni+1}, subNodi)
                            vale(dr) = 1;
                            stcc(dr)  = 1;
                            idxTarget{dr} = idxIni + 1;
                        else
                            % the targets cannot be accessed by the driver
                            vale(dr) = -2;
                            stcc(dr)  = 0;
                        end
                        
                    else
                        flagWhile = 1;
                        idxIni = idxIni + 1;
                    end
                    
                end
                
                
            elseif idxIni == length(targetNodes)
                %%% only the last target can be accessed
                vale(dr) = 1;
                stcc(dr)  = 1;
                idxTarget{dr} = idxIni;
            else
                disp('ERRORE!!! idxIni');
            end
            
        end
        
        if vale(dr) <= 1
            trovato = 0;
            for i = 1:length(targetNodes)
                if trovato == 0
                    if ismember(targetNodes{i}, subNodi)
                        trovato = i;
                    end
                end
            end
            
            vale(dr)      = 1;
            stcc( dr)      = 1;
            idxTarget{dr} = trovato;
        end
        
    else
        %%% the targets cannot be accessed by the driver
        vale(dr) = -2;
        stcc( dr) = 0;
        idxTarget{dr} = NaN;
    end
    
    
end

