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
tolerence=0.1;

for i=1:count+1 % calculate points coordinates
    if flag==0
        temp=START(i,2);        
        for j=1:l
            if abs(SORT(j,1)-START(i,1))<=tolerence %max
                temp(end+1)=SORT(j,2);
            end
        end
        [max_value,max_position]=max(temp);
        temp(max_position)=min(temp);
        END(i,1)=START(i,1);
        END(i,2)=max(temp);
        if i<count+1
            START(i+1,1)=START(i,1)+DIAMETER;
            temp=-9999;
            for j=1:l
                if abs(SORT(j,1)-START(i+1,1))<=tolerence
                    temp(end+1)=SORT(j,2);
                end
            end
            [max_value,max_position]=max(temp);
            temp(max_position)=min(temp);
            START(i+1,2)=max(temp);
        end
        flag=1;
    else
        temp=START(i,2);
        for j=1:l
            if abs(SORT(j,1)==START(i,1))<=toterence %min
                temp(end+1)=SORT(j,2);
            end
        end
        [min_value,min_position]=max(temp);
        temp(min_position)=max(temp);
        END(i,1)=START(i,1);
        END(i,2)=min(temp);
        if i<count+1
            START(i+1,1)=START(i,1)+DIAMETER;
            temp=9999;
            for j=1:l
                if abs(SORT(j,1)-START(i+1,1))<=tolerence
                    temp(end+1)=SORT(j,2);
                end
            end
            [min_value,min_position]=min(temp);
            temp(min_position)=max(temp);
            START(i+1,2)=min(temp);
        end
        flag=0;
    end   
end
end
