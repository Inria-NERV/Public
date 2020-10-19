Matlab code associated with the example in Fig. S1 of the paper 
Bassignana G., Fransson J., Henry V., Colliot O., Zujovic V., De Vico Fallani F. Step-wise target controllability identifies dysregulations of macrophage networks in multiple sclerosis.

Compute the step-wise target control centrality of the root node of a modified full binary tree, in which an edge is added to include a cycle between the first three nodes.

Run the main_example.m script to compute the centrality.

function_stepwiseKalmanCriterion.m performs the step-wise search of the highest-ranked controllable configuration of targets.

function_stccRecursion.m is an ausiliary function for the computation. 

The above functions take in input a connected graph, that is a graph with a single weakly connected component.