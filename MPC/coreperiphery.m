function isCore = coreperiphery(A, L, N, f, c)
%COREPERIPHERY SHORT_DESCRIPTION.
%   ISCORE = COREPERIPHERY(A, L, N, C, F) CASE_DESCRIPTION.
%
%   Inputs:
%
 %  A  Supra-adjacency matrix.
%       L  Number of layers.
%       N  Number of nodes per layer.
%       F  Single-layer richness function. If not defined, strength is
%           used.
%       C  `c` coefficients vector such as defined in [1].
%
%   Output:
%
%       ISCORE  ISCORE_DESCRIPTION.
%
%   Examples:
%
%       % EXAMPLE1_DESCRIPTION:
%       EXAMPLE1_CODE
%
%       % EXAMPLE2_DESCRIPTION:
%       EXAMPLE2_CODE
%
%   See also JG.CONN.INDEX.MULTI.RICHNESS, JG.CONN.INDEX.MONO.RICHNESS.
%
%   Copyright 2018 <a href="http://guillonjeremy.co">GUILLON Jeremy</a>.


%% Parsing inputs
% 

if nargin < 5 || isempty(c)
    c = ones(L,1) / L; % Richness coefficients default values
end
if nargin < 4 || isempty(f)
    f = @bnt.richness;
end

%% Compute Multiplex Core-Periphery
%

[mu, muMinus, muPlus] = bnt.multirichness(A, L, N, f, c);

[rankedMu, rankingInd] = sort(mu, 'descend');

[maxMuPlus, rankOfMaxMuPlus] = max(muPlus(rankingInd));

isCore = zeros(N,1);
isCore(rankingInd(1:rankOfMaxMuPlus)) = 1;
isCore = logical(isCore);

end








