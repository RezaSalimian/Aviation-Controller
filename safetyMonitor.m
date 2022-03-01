function [ flag ] = safetyMonitor( in1, in2 )
% in1, in2: Data Structure that stores information about the aircraft
%       m: Message from neighbouring aircraft 
%           - empty if aircraft not in neighbourhood
%           - (x, y, xd, yd, theta) of other aircraft if non-empty
%
% flag: true if the safety is voilated and false otherwise.
    temp_out_1 = in1.m(1,:);
    temp_out_2 = in2.m(1,:);
    flag = false;
    if (temp_out_1(2) == temp_out_2(2) && temp_out_1(3) == temp_out_2(3)) 
        flag = true;
    end


end

