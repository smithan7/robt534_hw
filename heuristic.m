function [ cost ] = heuristic( map, stateid, goalid )

    [sx, sy] = state_from_index(map, stateid);
    [gx, gy] = state_from_index(map, goalid);

    cost = sqrt((sx-gx)^2 + (sy-gy)^2);
end

