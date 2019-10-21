function dth = compute_dth(robotpos, eetarget, TT) %compute delta theta basically
    eeposnow = robotpos(end,:);
    J = compute_Jacobian(robotpos, TT);
    pseudo_inv = pinv(J);
    vel = eetarget - eeposnow;
    dth = pseudo_inv * vel; %just computing the change in angles
end
