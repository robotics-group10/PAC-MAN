function maze = generateMaze(nrows,ncols)
    if mod(nrows,2)==0 || mod(ncols,2)==0
        error('nrows and ncols must be odd.');
    end

    maze = ones(nrows,ncols);
    visited = false((nrows+1)/2, (ncols+1)/2);
    stack = [1,1];

    visited(1,1)=true;
    maze(1,1)=0;

    dirs=[-1 0;1 0;0 -1;0 1];

    while ~isempty(stack)
        cur=stack(end,:); r=cur(1); c=cur(2);
        mazeR=2*r-1; mazeC=2*c-1;
        nbrs=[];

        for k=1:4
            nr=r+dirs(k,1); nc=c+dirs(k,2);
            if nr>=1 && nr<=size(visited,1) && nc>=1 && nc<=size(visited,2) ...
                    && ~visited(nr,nc)
                nbrs(end+1,:)=[nr nc];
            end
        end

        if isempty(nbrs)
            stack(end,:)=[];
        else
            next=nbrs(randi(size(nbrs,1)),:);
            midR=mazeR+(next(1)-r);
            midC=mazeC+(next(2)-c);

            maze(mazeR,mazeC)=0;
            maze(midR,midC)=0;
            maze(2*next(1)-1,2*next(2)-1)=0;

            visited(next(1),next(2))=true;
            stack=[stack; next];
        end
    end
end