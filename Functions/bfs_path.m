function path = bfs_path(maze, startPos, goalPos)
    nrows=size(maze,1);
    ncols=size(maze,2);

    visited=false(nrows,ncols);
    parent = zeros(nrows,ncols,2);
    queue = startPos;

    visited(startPos(1), startPos(2))=true;
    dirs=[-1 0;1 0;0 -1;0 1];
    found=false;

    while ~isempty(queue)
        cur=queue(1,:); queue(1,:)=[];
        if isequal(cur, goalPos)
            found=true; break;
        end

        for k=1:4
            nr=cur(1)+dirs(k,1);
            nc=cur(2)+dirs(k,2);

            if nr>=1 && nr<=nrows && nc>=1 && nc<=ncols ...
                    && maze(nr,nc)==0 && ~visited(nr,nc)
                queue(end+1,:)=[nr nc];
                visited(nr,nc)=true;
                parent(nr,nc,:) = cur;
            end
        end
    end

    if ~found
        path=[];
        return;
    end

    % Reconstruct path
    path=goalPos;
    while ~isequal(path(1,:), startPos)
        p = squeeze(parent(path(1,1), path(1,2),:))';
        path=[p; path];
    end
end
