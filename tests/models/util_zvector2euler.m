% compute euler angles from z vector
function [theta,phi] = util_zvector2euler(n_z)
theta = -asin(n_z(1));
cos_theta = cos(theta);
sin_phi = n_z(2)/cos_theta;
cos_phi = n_z(3)/cos_theta;
phi = atan2(sin_phi,cos_phi);