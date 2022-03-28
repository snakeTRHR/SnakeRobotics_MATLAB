function curvature = calCurvature(x0, x1, x2, y0, y1, y2)
    dxn = x1-x0;
    dxp = x2-x1;
    dyn = y1-y0;
    dyp = y2-y1;
    dn = sqrt(dxn*dxn+dyn*dyn);
    dp = sqrt(dxp*dxp+dyp*dyp);
    dx = 1.0/(dn+dp)*(dp/dn*dxn+dn/dp*dxp);
    ddx = 2.0/(dn+dp)*(dxp/dp-dxn/dn);
    dy = 1.0/(dn+dp)*(dp/dn*dyn+dn/dp*dyp);
    ddy = 2.0/(dn+dp)*(dyp/dp-dyn/dn);
    curvature = (ddy*dx-ddx*dy)/(power((dx*dx+dy*dy), 1.5));
end

