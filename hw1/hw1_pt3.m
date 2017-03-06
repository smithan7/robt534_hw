
close all
clear all
clc


%addpath(genpath('/home/rdml/git/rob534_hw/hw1_code/provided_code'))
addpath(genpath('C:\Users\sgtas\OneDrive\Documents\GitHub\robt534_hw'))
map = read_map_for_dynamics('maze1.pgm');

[start, n_nodes] = get_start_dynamic(map);

[x,y,dx,dy] = dynamic_state_from_index(map, start);
goal = get_goal_dynamic(map);
pause(1)

epsilon = 10.01;
iter=0;
time = cputime;
while epsilon > 1.001 && cputime-time < 1

    state = start;

    oSet = zeros(n_nodes,1);
    cSet = zeros(n_nodes,1);
    bSet = zeros(n_nodes,1);
    gScore = ones(n_nodes,1)*inf; % known cost from initial node to n
    fScore = ones(n_nodes,1)*inf; % heuristic cost from node n to goal

    oList = [];
    oList(1) = state;
    oSet( state ) = 1;
    gScore( state ) = 0;
    fScore( state ) = dynamic_heuristic(map, state, goal);
    fScore( goal ) = 0;

    fWeight = epsilon;
    fu = true;
    nodes_searched = 0;

    while fu == true
        nodes_searched = nodes_searched+1;

        figure(2)
        oMap = zeros(map.C, map.R);

        for i=1:length(oSet)
            if oSet(i) == 1
                [sx, sy] = dynamic_state_from_index(map, i);
                oMap(sy, sx) = gScore(i);
            end
            if cSet(i) == 1
                [sx, sy] = dynamic_state_from_index(map, i);
                oMap(sy, sx) = gScore(i);
            end
        end

        % this finds node with lowest fScore and makes current
        min = inf;
        mindex = -1;
        for i=1:length(oList)
            if fScore( oList(i) ) < min
                min = fScore( oList(i) );
                mindex = i;
            end
        end

        imagesc(oMap)
        %waitforbuttonpress()
        pause(0.01)

        state = oList(mindex);
        oList(mindex) = [];

        oSet(state) = 0;
        cSet(state) = 1;

        % end finding current node
        if state == goal % if the current node equals goal, construct path
            totalPath = [];
            totalPath(1) = goal;
            state = bSet(goal);
            while state ~= start % work backwards to start
                [nx, ny] = dynamic_state_from_index(map, state);
                oMap(ny, nx) = 100;
                imagesc(oMap)

                state = bSet(state);
                totalPath(length(totalPath)+1) = state; % append path
            end
            iter=iter+1;
            l(iter)=length(totalPath);
            e(iter)=epsilon;
            nc(iter)=nodes_searched;
            t(iter) = cputime - time;
            
            flipud(totalPath);
            fu = false;
            continue;
        end % end construct path

        % for nbrs
        [nbrs, n_nbrs] = get_neighbors_dynamic(map, state);

        for n=1:n_nbrs
            if cSet(nbrs(n)) == 1 % has it already been eval? in cSet
                continue;
            end
            ngScore = gScore(state) + dynamic_heuristic(map,state,nbrs(n)); % calc temporary gscore, estimate of total cost
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
            fScore(nbrs(n)) = gScore(nbrs(n)) + fWeight*dynamic_heuristic(map, nbrs(n), goal);
        end
        %  end condition for while loop, check if oSet is empty
        if length(oList) == 0
            fu = false;
        end
    end
    epsilon = epsilon-0.5*(epsilon-1);
end

        


