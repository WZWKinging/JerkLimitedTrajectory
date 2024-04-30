function [ T1, T2, T3, T4, T5, T6, T7, direction_acc, direction_dec, v_m ] = binarySearchUpdateDurationsGivenTotalTime( T1234567, j_max, a0, a_max, v0, v_max, p0, pt )

    delt_p = pt - p0;
    if delt_p == 0
        v_now = - sign(v0) * v_max;
    else
        v_now = sign(delt_p) * v_max;
    end
    v_s = 0;
    v_e = v_now;
    
    MeetTheSetPoint = 0;
    delt_p_thr = 0.000001;
    delt_t_thr = 0.01;
    delt_vel_dec = 0.01;
    
    iterCounter = 1;
    maxIter = 100;
    
    while(MeetTheSetPoint ~= 1)

        [ T1, T2, T3, T4, T5, T6, T7, dir_acc, dir_dec, deltP ] = updateDurations_Position_Setpoint( v_now, a0, v0, j_max, a_max, v_max, p0, pt );
        direction_acc = dir_acc;
        direction_dec = dir_dec;
        total_t1234567 = T1 + T2 + T3 + T4 + T5 + T6 + T7;
        if (abs(deltP - delt_p) <= delt_p_thr) && (iterCounter < maxIter)

            if abs(total_t1234567 - T1234567) < delt_t_thr
                MeetTheSetPoint = 1;
                break;
            else
                if (T1 < 0.0001 && T2 < 0.0001 && T3 < 0.0001)
                    MeetTheSetPoint = 1;
                    break;
                end
                MeetTheSetPoint = 0;
                v_now = v_now - sign(v_now) * delt_vel_dec;   
                iterCounter = iterCounter + 1;
                continue;
            end
        else
            if iterCounter >= maxIter
                MeetTheSetPoint = 1;
                break;
            else
                MeetTheSetPoint = 0;
            end          
        end

        if abs(deltP) > abs(delt_p)
            v_s = 0;
            v_e = v_now;
            v_now = (v_s + v_e) / 2;
        else
            if abs(deltP) < abs(delt_p)
                v_s = v_now;
                v_e = sign(delt_p) * v_max;
                v_now = (v_s + v_e) / 2;
            else

            end
        end
        iterCounter = iterCounter + 1;
    end
    v_m = v_now;
end

