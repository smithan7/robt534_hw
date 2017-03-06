function [ cost ] = dynamic_heuristic( map, stateid, goalid )

    [sx, sy, sdx, sdy] = dynamic_state_from_index(map, stateid);
    [gx, gy, gdx, gdy] = dynamic_state_from_index(map, goalid);

    cost = sqrt((sx-gx)^2 + (sy-gy)^2 + (sdx-gdx)^2 + (sdy-gdy)^2);
end

