clear all
close all hidden
clc

%%% 08/09/2020
%%%%%% Toy model step-wise target controllability
%%%%%% Binary tree with a single cycle.

flagPlot = 1;
% lv = 5;
lv = 10;
N = 2^(lv+1)-1;
E = N - 1;

row = zeros(E,1);
col = zeros(E,1);
val = ones(E,1);
k = 1;
for i = 1:2^lv-1
    row(k) = i;
    col(k) = 2*i;
    k = k + 1;
    row(k) = i;
    col(k) = 2*i + 1;
    k = k + 1;
end

A = sparse(row,col,val); %%% full binary tree

adj = [full(A); zeros(N-(2^lv-1),N)];

%%%%%% Declare names
nomiG = cell(N,1);
for i = 1:N
    nomiG{i} = num2str(i);
end

Gtree = digraph(adj, nomiG);
if flagPlot == 1
    figure;
    h = plot(Gtree,'Layout','layered');
end

% %%% Include a cycle
adj(2,1) = 1; adj(3,2) = 1;

Gtree = digraph(adj, nomiG);


B = zeros(N,1); B(1,1) = 1;

tg = zeros(lv,1);
for i = 1:lv
    tg(i) = 2^(i+1)-(i+1);
end

if flagPlot == 1
    fs = 20;
    fn = 'helvetica';
    cTarget = [0.6667    0.8980    0.4392];
    cDriver = [0.4 0 0.6];
    %%% FIG.
    posizionex = h.XData;
    posizioney = h.YData;
    
    h = plot(Gtree, 'XData', posizionex, 'YData', posizioney);
    
    
    h.NodeLabel = {};
    highlight(h,1:N,'NodeColor','k','MarkerSize',8,'Marker','o');
    highlight(h,row,col,'EdgeColor','k');
    highlight(h,[2,3],[1,2],'EdgeColor','k');
    highlight(h,tg,'NodeColor',cTarget,'Marker','s','MarkerSize',20);
    highlight(h,1,'NodeColor',cDriver,'Marker','v','MarkerSize',20);
    shg;
    
    hold on
    plot(NaN,NaN,'v','MarkerFaceColor',cDriver,...
        'Color',cDriver,'MarkerSize',8,'DisplayName','Driver');
    plot(NaN,NaN,'s','MarkerFaceColor',cTarget,'MarkerSize',8,...
        'Color',cTarget,'DisplayName','Target');
    hold off
    
    h.HandleVisibility = 'off';
%     [qq,icons] = legend('Driver','Target',...
%         'FontSize',fs,'FontName',fn,'Location','best');
%     icons = findobj(icons, '-property',...
%         'Marker', '-and', '-not', 'Marker', 'none');
%     set(icons,'MarkerSize',8);
    
    axis off;
    pbaspect([2 1 1]);
    legend boxoff;
end

%%
%%% Compare naive application of Kalman criterion and step-wise target
%%% controllability
T = zeros(length(tg),N);
for i = 1:length(tg)
    T(i,tg(i)) = 1;
end


CO = ctrb(adj',B);

if rank(T*CO) ~= length(tg)
    disp(['Standard procedure computation rank = ',...
        num2str(rank(T*CO)),'.']);
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

rangeH         = 1;
currentTargets = tg;
flagLog        = 1;
flagSubGtar    = 1;

[idxTarget, vale, stcc] = ...
    function_stepwiseKalmanCriterion(...
    Gtree, currentTargets, rangeH, flagLog, flagSubGtar);

if rank(T*CO) == stcc
    disp(['Naive application of Kalman criterion and step-wise target',...
        ' controllability give the same result.']);
end


if length(tg) == stcc
    disp(['Step-wise target control centrality = ',num2str(stcc),...
        '.']);
else
    disp(['Step-wise target control centrality = ',num2str(stcc),...
        '.']);
end





