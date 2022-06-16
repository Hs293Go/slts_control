%% Save simulink data
%%%%%%%%%% time %%%%%%%%%%%%%%%%%%%%
FlightData.time = time_sq;
N = length(FlightData.time);
FlightData.N = N;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FlightData.e_p = e_p;
FlightData.e_v = e_v;
FlightData.v_p_inertial = v_p_inertial;
FlightData.x_p_inertial = x_p_inertial;
FlightData.r_tilde = r_tilde;
FlightData.delta_p_bot_error = delta_p_bot_error;
FlightData.delta_T_error = delta_T_error;
FlightData.delta_T = delta_T;
FlightData.delta_q_bot = delta_q_bot;
save('SingleDroneTest','FlightData')