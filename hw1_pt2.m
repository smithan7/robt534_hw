
close all
clear all
clc


%addpath(genpath('/home/rdml/git/rob534_hw/hw1_code/provided_code'))
addpath(genpath('C:\Users\sgtas\OneDrive\Documents\GitHub\robt534_hw'))

t = cputime;

map = read_map('maze1.pgm');

[start, n_nodes] = get_start(map);
state = start;
[g.x g.y] = state_from_index(map, get_goal(map));

greedy = 0.05;
extend = 3;


fu = true;


n = struct('x',1,'y',1,'p',-1);
nodes(1) = n;

while fu
    N = length(nodes);
    
    if rand() < 1-greedy
        xr = rand() * map.C;
        yr = rand() * map.R;
    else
       xr = g.x;
       yr = g.y;
    end
    
    
    min = inf;
    minI = 1;
    for i = 1:N
        if map.cells(round(nodes(i).x), round(nodes(i).y)) == 0
            d = sqrt( (xr-nodes(i).x)^2 + (yr-nodes(i).y)^2 );
            if d < min
                min = d;
                minI = i;
            end
        end
    end
    
    dx = extend*rand()*(xr - nodes(minI).x)/min;
    dy = extend*rand()*(yr - nodes(minI).y)/min;
    
    if ~check_hit(map, nodes(minI).x, nodes(minI).y, dx, dy)
        n = struct('x', nodes(minI).x + dx,'y', nodes(minI).y + dy, 'p',minI);
        nodes(N+1) = n;
        % check end condition
        
        if abs(g.x-nodes(N+1).x) < 1 && abs(g.y-nodes(N+1).y) < 1
            fu = false;
        end
        
        
        figure(1)
        hold all
        plot(nodes(N+1).x, nodes(N+1).y, 'r.')
        parent = nodes(N+1).p;
        plot([nodes(parent).x, nodes(N+1).x],[nodes(parent).y, nodes(N+1).y],'r','linewidth',1)
        axis([1 map.R 1 map.C])
        pause(0.01)
    end
    
end

time = cputime - t


figure(1)
hold all
axis([1 map.R 1 map.C])
%         pause(0.01)

parent = length(nodes)
length = 0;
while parent >= 0
    
    child = parent;
    parent = nodes(parent).p;
    length = length + sqrt( (nodes(child).x-nodes(parent).x)^2 + (nodes(child).y-nodes(parent).y)^2 );
    
    plot([nodes(child).x, nodes(parent).x],[nodes(child).y, nodes(parent).y], 'g', 'linewidth',2)
    
end

l = length

