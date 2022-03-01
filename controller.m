function [ temp_outFinal ] = controller( in )
% Takes flight parameters of an aircraft and temp_outputs the direction control

% in: Data Structure that stores input information for the aircraft
% controller. 
%       (in.x, in.y): Current Location of the aircraft
%       (in.xd, in.yd): Destination of aircraft
%       in.theta: Current direction of motion
% temp_out : Data Structure that stores the output information from the aircraft
%       temp_out.val: +1, 0, -1 ( +1 - turn left, 0 - go straight, -1 - turn right)

    x_destination = in.xd;
    y_destination = in.yd;
    x_current = in.x;
    y_current = in.y;
    theta = in.theta;
    Points_Distance = [x_destination,y_destination;x_current,y_current];
    current_Distance = pdist(Points_Distance,'euclidean');
    temp_out = zeros(3,1);
    points = zeros(3,2);
    % Next movement calculation
    if( ~(current_Distance == 0))
        if( theta == 0 || theta == 360)
            x_forward = 1;y_forward = 0;
            x_right = 0;y_right = -1;
            x_left = 0;y_left = 1;
            
        elseif(theta == 90)
            x_forward = 0;y_forward = 1;
            x_right = 1;y_right = 0;
            x_left = -1;y_left = 0;
        
        elseif(theta == 180)
            x_forward = -1;y_forward = 0;
            x_right = 0;y_right = 1;
            x_left = 0;y_left = -1;
       
        elseif(theta == 270)
            x_forward = 0;y_forward = -1;
            x_right = -1;y_right = 0;
            x_left = 1;y_left = 0;
        
        end
        
        %eucliden distance calculation from current location to destination
        euc_DistanceForward = pdist([x_destination,y_destination;(x_current+x_forward),(y_current+y_forward)],'euclidean');
        euc_DistanceLeft = pdist([x_destination,y_destination;(x_current+x_left),(y_current+y_left)],'euclidean');
        euc_DistanceRight = pdist([x_destination,y_destination;(x_current+x_right),(y_current+y_right)],'euclidean');
       
        %step distance calculation from current location to destination
        current_Distance_Forward = abs(x_destination-(x_current+x_forward))+abs(y_destination-(y_current+y_forward));
        current_Distance_Left = abs(x_destination-(x_current+x_left))+abs(y_destination-(y_current+y_left));
        current_Distance_Right = abs(x_destination-(x_current+x_right))+abs(y_destination-(y_current+y_right));
      
       %Considering distances in a matrix and sorting them from minimum
       %distance senario to maximum distance
        Dis_Matrix = [current_Distance_Left,current_Distance_Forward,current_Distance_Right;euc_DistanceLeft,euc_DistanceForward,euc_DistanceRight; 1,0,-1];
        Sorted_Dis_Matrix = Dis_Matrix;
        for i=1:2
            for j=(i+1):3
                if ( Sorted_Dis_Matrix(1,i) > Sorted_Dis_Matrix(1,j)||( Sorted_Dis_Matrix(1,i) == Sorted_Dis_Matrix(1,j) && Sorted_Dis_Matrix(2,i) > Sorted_Dis_Matrix(2,j)))
                    local_tmp=Sorted_Dis_Matrix(1,j);
                    Sorted_Dis_Matrix(1,j)=Sorted_Dis_Matrix(1,i);
                    Sorted_Dis_Matrix(1,i) = local_tmp;
                    local_tmp=Sorted_Dis_Matrix(2,j);
                    Sorted_Dis_Matrix(2,j)=Sorted_Dis_Matrix(2,i);
                    Sorted_Dis_Matrix(2,i) = local_tmp;
                    local_tmp=Sorted_Dis_Matrix(3,j);
                    Sorted_Dis_Matrix(3,j)=Sorted_Dis_Matrix(3,i);
                    Sorted_Dis_Matrix(3,i) = local_tmp;
                end
            end
        end
        %Assigning output value based on the chosen senario
        for i=1:3
            if ( Sorted_Dis_Matrix(3,i) == -1)
                temp_out(i) = -1;
                points(i,1) = x_current+x_right;
                points(i,2) = y_current+y_right;
            elseif ( Sorted_Dis_Matrix(3,i) == 1)
                temp_out(i) = +1;
                points(i,1) = x_current+x_left;
                points(i,2) = y_current+y_left;
            elseif ( Sorted_Dis_Matrix(3,i) == 0)
                temp_out(i) = 0;
                points(i,1) = x_current+x_forward;
                points(i,2) = y_current+y_forward;
      
            end
        end
        temp_outFinal = horzcat(temp_out,points);
  
    else
        temp_out = [2;2;2];
        points = [x_destination,y_destination;x_destination,y_destination;x_destination,y_destination];
        temp_outFinal = horzcat(temp_out,points);
    end
