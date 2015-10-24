function arrange()
%ARRANGE
global BOUNDARY
global DIAMETER
global START
global END
SORT=sortrows(BOUNDARY); % sort boundary with x
START(1,1)=SORT(1,1);  % first point of START
START(1,2)=SORT(1,1);
l=length(SORT); % number of points
count=floor((SORT(l,1)-SORT(1,1))/DIAMETER); %number of grids
flag=0; % upper bound or lower bound
for i=1:count+1 % calculate points coordinates
    if flag==0
        temp=START(i,2);
        for j=1:l
            if SORT(j,1)==START(i,1) && SORT(j,2)>temp
                temp=SORT(j,2);
            end
        end
        END(i,1)=START(i,1);
        END(i,2)=temp;
        START(i+1,1)=START(i,1)+DIAMETER;
        if i<count+1
            temp=-9999;
            for j=1:l
                if SORT(j,1)==START(i+1,1) && SORT(j,2)>temp
                    temp=SORT(j,2);
                end
            end
            START(i+1,2)=temp
        end
        flag=1;
    else
        temp=START(i,2);
        for j=1:l
            if SORT(j,1)==START(i,1) && SORT(j,2)<temp
                temp=SORT(j,2);
            end
        end
        END(i,1)=START(i,1);
        END(i,2)=temp;
        START(i+1,1)=START(i,1)+DIAMETER;
        if i<count+1
            temp=9999;
            for j=1:l
                if SORT(j,1)==START(i+1,1) && SORT(j,2)<temp
                    temp=SORT(j,2);
                end
            end
            START(i+1,2)=temp;
        end
        flag=0;
    end   
end
end
