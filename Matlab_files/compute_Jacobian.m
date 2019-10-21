function jack = compute_Jacobian(robpos, TT)
    z0 = [0, 0, 1]; 
    trans = TT(:,:,1);
    z1 = trans(1:3,1:3)*[0, 0, 1]'; % find the z axis representation in terms of base 0 frame
    trans = TT(:,:,2);
    z2 = trans(1:3,1:3)*[0, 0, 1]';
    trans = TT(:,:,3);
    z3 = trans(1:3,1:3)*[0, 0, 1]';
    trans = TT(:,:,4);
    z4 = trans(1:3,1:3)*[0, 0, 1]';
    trans = TT(:,:,5);
    z5 = trans(1:3,1:3)*[0, 0, 1]';
    
    c0 = cross(z0, (robpos(end,:) - robpos(1,:))); %check that robpos has 7 rows
    c1 = cross(z1, (robpos(end,:) - robpos(2,:))); %these are the z's axes for each joint with respect to the base frame
    c2 = cross(z2, (robpos(end,:) - robpos(3,:)));
    c3 = cross(z3, (robpos(end,:) - robpos(4,:)));
    c4 = cross(z4, (robpos(end,:) - robpos(5,:)));
    c5 = cross(z5, (robpos(end,:) - robpos(6,:)));
    
    jack = [c0', c1', c2', c3', c4', c5'];
    add = [z0', z1, z2, z3, z4, z5];
    jack = [jack; add];
end