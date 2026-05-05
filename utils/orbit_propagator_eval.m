function [pos_eci, vel_eci] = orbit_propagator_eval(t, mu, Re, J2, a, ecc, inc_deg, raan_deg, aop_deg, ta_deg)
%ORBIT_PROPAGATOR_EVAL J2-aware orbital state reconstruction from mean elements.

    inc = inc_deg * pi/180;
    raan0 = raan_deg * pi/180;
    aop0 = aop_deg * pi/180;
    ta0 = ta_deg * pi/180;

    n = sqrt(mu / a^3);
    p = a * (1 - ecc^2);
    j2_fac = J2 * (Re / p)^2;
    d_raan = -1.5 * j2_fac * n * cos(inc);
    d_aop  =  0.75 * j2_fac * n * (5*cos(inc)^2 - 1);
    d_ma   =  n + 0.75 * j2_fac * n * sqrt(1 - ecc^2) * (3*cos(inc)^2 - 1);

    E0 = 2 * atan2(sqrt(1-ecc) * sin(ta0/2), sqrt(1+ecc) * cos(ta0/2));
    ma0 = E0 - ecc * sin(E0);

    ma = mod(ma0 + d_ma * t, 2*pi);
    raan = raan0 + d_raan * t;
    aop = aop0 + d_aop * t;

    E = ma;
    for iter = 1:10
        dE = (ma - E + ecc*sin(E)) / (1 - ecc*cos(E));
        E = E + dE;
        if abs(dE) < 1e-12
            break;
        end
    end

    nu = 2 * atan2(sqrt(1+ecc)*sin(E/2), sqrt(1-ecc)*cos(E/2));
    r_mag = a * (1 - ecc*cos(E));
    p_pqw = r_mag * [cos(nu); sin(nu); 0];
    v_pqw = sqrt(mu*a)/r_mag * [-sin(E); sqrt(1-ecc^2)*cos(E); 0];

    cO = cos(raan); sO = sin(raan);
    cw = cos(aop);  sw = sin(aop);
    ci = cos(inc);  si = sin(inc);

    R_pqw_to_eci = [ cO*cw - sO*sw*ci,  -cO*sw - sO*cw*ci,   sO*si;
                     sO*cw + cO*sw*ci,  -sO*sw + cO*cw*ci,  -cO*si;
                     sw*si,              cw*si,               ci   ];

    pos_eci = R_pqw_to_eci * p_pqw;
    vel_eci = R_pqw_to_eci * v_pqw;
end
