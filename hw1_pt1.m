
close all
clear all
clc


addpath(genpath('/home/rdml/git/rob534_hw/hw1_code/provided_code'))
map = read_map('maze1.pgm');

[start, n_nodes] = get_start(map);
state = start;
goal = get_goal(map);

[nbrs, n_nbrs] = get_neighbors(map, state);

cost = heuristic(map, state, goal);


oSet = zeros(n_nodes,1);
cSet = zeros(n_nodes,1);
bSet = zeros(n_nodes,1);
gScore = ones(n_nodes,1)*inf; % known cost from initial node to n
fScore = ones(n_nodes,1)*inf; % heuristic cost from node n to goal

oList(1) = state;
oSet( state ) = 1;
gScore( state ) = 0;
fScore( state ) = heuristic(map, state, goal);
fScore( goal ) = 0;

fu = true;



while fu == true
    
    figure(2)
    oMap = zeros(map.R, map.C);

    for i=1:length(oSet)
        if oSet(i) == 1
            [sx, sy] = state_from_index(map, i);
            oMap(sx, sy) = 0.1;
        end
        if cSet(i) == 1
            [sx, sy] = state_from_index(map, i);
            oMap(sx, sy) = 0.1;
        end
    end
    
    % this finds node with lowest fScore and makes current
    min = inf;
    mindex = -1;
    for i=1:length(oList)
        oList(i)
        fScore(oList(i))
        if fScore( oList(i) ) < min
            min = fScore( oList(i) );
            mindex = i;
        end
    end
    
    contour(oMap)
    waitforbuttonpress()
    pause(0.01)

    state = oList(mindex);
    oList(mindex) = [];
    
    oSet(state) = 0;
    cSet(state) = 1;
    
    % end finding current node
    if state == goal % if the current node equals goal, construct path
        totalPath(1) = goal;
        state = bSet(goal);
        while state ~= start % work backwards to start
            state = bSet(state);
            totalPath(length(totalPath)+1) = state; % append path
        end
        flipud(totalPath);
        fu = false;
        continue;
    end % end construct path

    % for nbrs
    [nbrs, n_nbrs] = get_neighbors(map, state);

    for n=1:n_nbrs
        if cSet(nbrs(n)) == 1 % has it already been eval? in cSet
            continue;
        end
        ngScore = gScore(state) + heuristic(map,nbrs(n),goal); % calc temporary gscore, estimate of total cost
        if oSet(nbrs(n)) == 0
            oSet(nbrs(n)) = 1;  % add nbr to open set
            oList(length(oList)+1) = nbrs(n);
        else
            if ngScore >= gScore(nbrs(n)) % is temp gscore worse than stored g score of nbr
                continue;
            end
        end
        
        bSet(nbrs(n)) = state;
        gScore(nbrs(n)) = ngScore;
        fScore(nbrs(n)) = gScore(nbrs(n)) + heuristic(map, nbrs(n), goal);
    end
    %  end condition for while loop, check if oSet is empty
    if length(oList) == 0
        fu = false;
    end
end



        


