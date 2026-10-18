function build_environment(sys)
%BUILD_ENVIRONMENT  Build the ENVIRONMENT subsystem inside the ADCS model.
%
%   Blocks: Ephemeris_Truth, Orbit_Propagator, Magnetic_Field_Model, Eclipse_Model
%   Outports: Sun_vec_ECI, Moon_vec_ECI, B_ECI, pos_ECI, vel_ECI, eclipse_flag

    ar = {'autorouting','on'};

    % ── Outports (parent-level output interface) ──
    %  1: Sun_vec_ECI   2: Moon_vec_ECI   3: B_ECI
    %  4: pos_ECI       5: vel_ECI        6: eclipse_flag
    add_outports(sys, ...
        {'Sun_vec_ECI','Moon_vec_ECI','B_ECI','pos_ECI','vel_ECI','eclipse_flag'}, ...
        700, 40, 70);

    % ── Time sources ──
    add_block('simulink/Sources/Clock', [sys '/Clock'], ...
        'Position', [30  120  60  140]);

    % ── Ephemeris data arrays (loaded from CSV in init_adcs_params) ──
    add_block('simulink/Sources/Constant', [sys '/Ephem_t'], ...
        'Position', [30  30  110  50], 'Value', 'ephem_t_sec');
    add_block('simulink/Sources/Constant', [sys '/Ephem_sun'], ...
        'Position', [30  70  110  90], 'Value', 'ephem_sun');
    add_block('simulink/Sources/Constant', [sys '/Ephem_moon'], ...
        'Position', [30  110  110  130], 'Value', 'ephem_moon');
    add_block('simulink/Sources/Constant', [sys '/Epoch_JD'], ...
        'Position', [30  150  110  170], 'Value', 'epoch_jd');

    add_block('simulink/Sources/Constant', [sys '/Mu_Earth'], ...
        'Position', [30  190  110  210], 'Value', 'mu_earth');
    add_block('simulink/Sources/Constant', [sys '/R_Earth'], ...
        'Position', [30  220  110  240], 'Value', 'R_earth');
    add_block('simulink/Sources/Constant', [sys '/J2_Earth'], ...
        'Position', [30  250  110  270], 'Value', 'J2_earth');
    add_block('simulink/Sources/Constant', [sys '/Orbit_A'], ...
        'Position', [30  280  110  300], 'Value', 'orbit_a');
    add_block('simulink/Sources/Constant', [sys '/Orbit_Ecc'], ...
        'Position', [30  310  110  330], 'Value', 'orbit_ecc');
    add_block('simulink/Sources/Constant', [sys '/Orbit_Inc'], ...
        'Position', [30  340  110  360], 'Value', 'orbit_inc');
    add_block('simulink/Sources/Constant', [sys '/Orbit_RAAN'], ...
        'Position', [30  370  110  390], 'Value', 'orbit_RAAN');
    add_block('simulink/Sources/Constant', [sys '/Orbit_AOP'], ...
        'Position', [30  400  110  420], 'Value', 'orbit_AOP');
    add_block('simulink/Sources/Constant', [sys '/Orbit_TA'], ...
        'Position', [30  430  110  450], 'Value', 'orbit_TA');
    add_block('simulink/Sources/Constant', [sys '/B0_Earth'], ...
        'Position', [30  460  110  480], 'Value', 'B0_earth');

    % ── Ephemeris_Truth  (CSV-based interpolation) ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Ephemeris_Truth'], 'Position', [200  30  400  140]);

    % ── Orbit_Propagator  (Keplerian 2-body from orbital elements) ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Orbit_Propagator'], 'Position', [200  180  400  250]);

    % ── Magnetic_Field_Model  (tilted-dipole IGRF approximation) ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Magnetic_Field_Model'], 'Position', [200  290  400  350]);

    % ── Eclipse_Model  (cylindrical Earth shadow) ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Eclipse_Model'], 'Position', [200  390  400  460]);

    % ── Set MATLAB Function block scripts ──
    set_mfb_script([sys '/Ephemeris_Truth'],     ephemeris_truth_code());
    set_mfb_script([sys '/Orbit_Propagator'],    orbit_propagator_code());
    set_mfb_script([sys '/Magnetic_Field_Model'],magnetic_field_code());
    set_mfb_script([sys '/Eclipse_Model'],       eclipse_model_code());
    set_mfb_data_sizes([sys '/Ephemeris_Truth'], ...
        {'t','1'; 'ephem_t','[53 1]'; 'ephem_sun','[53 3]'; ...
         'ephem_moon','[53 3]'; 'epoch_jd','1'; ...
         'sun_eci','[3 1]'; 'moon_eci','[3 1]'});
    set_mfb_data_sizes([sys '/Orbit_Propagator'], ...
        {'t','1'; 'mu','1'; 'Re','1'; 'J2','1'; 'a','1'; 'ecc','1'; ...
         'inc_deg','1'; 'raan_deg','1'; 'aop_deg','1'; 'ta_deg','1'; ...
         'pos_eci','[3 1]'; 'vel_eci','[3 1]'});
    set_mfb_data_sizes([sys '/Magnetic_Field_Model'], ...
        {'t','1'; 'pos_eci','[3 1]'; 'epoch_jd','1'; 'B0','1'; 'Re','1'; 'B_eci','[3 1]'});
    set_mfb_data_sizes([sys '/Eclipse_Model'], ...
        {'pos_eci','[3 1]'; 'sun_eci','[3 1]'; 'Re','1'; 'eclipse_flag','1'});

    % ── Wiring ──
    % Clock + ephemeris data → Ephemeris_Truth
    add_line(sys, 'Clock/1',      'Ephemeris_Truth/1', ar{:});
    add_line(sys, 'Ephem_t/1',    'Ephemeris_Truth/2', ar{:});
    add_line(sys, 'Ephem_sun/1',  'Ephemeris_Truth/3', ar{:});
    add_line(sys, 'Ephem_moon/1', 'Ephemeris_Truth/4', ar{:});
    add_line(sys, 'Epoch_JD/1',   'Ephemeris_Truth/5', ar{:});
    add_line(sys, 'Clock/1',      'Orbit_Propagator/1', ar{:});
    add_line(sys, 'Mu_Earth/1',   'Orbit_Propagator/2', ar{:});
    add_line(sys, 'R_Earth/1',    'Orbit_Propagator/3', ar{:});
    add_line(sys, 'J2_Earth/1',   'Orbit_Propagator/4', ar{:});
    add_line(sys, 'Orbit_A/1',    'Orbit_Propagator/5', ar{:});
    add_line(sys, 'Orbit_Ecc/1',  'Orbit_Propagator/6', ar{:});
    add_line(sys, 'Orbit_Inc/1',  'Orbit_Propagator/7', ar{:});
    add_line(sys, 'Orbit_RAAN/1', 'Orbit_Propagator/8', ar{:});
    add_line(sys, 'Orbit_AOP/1',  'Orbit_Propagator/9', ar{:});
    add_line(sys, 'Orbit_TA/1',   'Orbit_Propagator/10', ar{:});

    % Ephemeris → outports
    add_line(sys, 'Ephemeris_Truth/1', 'Sun_vec_ECI/1',  ar{:});
    add_line(sys, 'Ephemeris_Truth/2', 'Moon_vec_ECI/1', ar{:});

    % Orbit Propagator → outports
    add_line(sys, 'Orbit_Propagator/1', 'pos_ECI/1', ar{:});
    add_line(sys, 'Orbit_Propagator/2', 'vel_ECI/1', ar{:});

    % pos_ECI branch → Magnetic_Field_Model
    add_line(sys, 'Clock/1',            'Magnetic_Field_Model/1', ar{:});
    add_line(sys, 'Orbit_Propagator/1', 'Magnetic_Field_Model/2', ar{:});
    add_line(sys, 'Epoch_JD/1',         'Magnetic_Field_Model/3', ar{:});
    add_line(sys, 'B0_Earth/1',         'Magnetic_Field_Model/4', ar{:});
    add_line(sys, 'R_Earth/1',          'Magnetic_Field_Model/5', ar{:});
    add_line(sys, 'Magnetic_Field_Model/1', 'B_ECI/1',            ar{:});

    % pos_ECI + Sun_vec → Eclipse_Model
    add_line(sys, 'Orbit_Propagator/1', 'Eclipse_Model/1', ar{:});
    add_line(sys, 'Ephemeris_Truth/1',  'Eclipse_Model/2', ar{:});
    add_line(sys, 'R_Earth/1',          'Eclipse_Model/3', ar{:});
    add_line(sys, 'Eclipse_Model/1',    'eclipse_flag/1',  ar{:});
end


% ═══════════════════════════════════════════════════════════════════════════
%          E M B E D D E D   C O D E   G E N E R A T O R S
% ═══════════════════════════════════════════════════════════════════════════

% ─────────────────────────────────────────────────────────────────────────
%  Ephemeris Truth  (CSV-based lookup with interpolation)
% ─────────────────────────────────────────────────────────────────────────
function s = ephemeris_truth_code()
    L = {
    'function [sun_eci, moon_eci] = fcn(t, ephem_t, ephem_sun, ephem_moon, epoch_jd)'
    '%#codegen'
    '% Ground-truth Sun & Moon direction vectors interpolated from'
    '% pre-loaded ephemeris CSV data (ephemeris_2026_weekly.csv).'
    '%'
    '% Inputs:  t          – simulation time [s] from epoch_jd'
    '%          ephem_t    – [Nx1] time breakpoints [s] from epoch'
    '%          ephem_sun  – [Nx3] Sun unit direction vectors (ECI)'
    '%          ephem_moon – [Nx3] Moon unit direction vectors (ECI)'
    '% Outputs: sun_eci    – Sun direction unit vector ECI [3x1]'
    '%          moon_eci   – Moon direction unit vector ECI [3x1]'
    '% Caller should validate epoch coverage during initialization; this'
    '% block clamps outside the CSV span only to keep simulation finite.'
    ''
    '  N = size(ephem_t, 1);'
    ''
    '  % The CSV time axis starts at 2026-01-01T00:00:00Z, while the'
    '  % simulator epoch is configured by epoch_jd.'
    '  csv_epoch_jd = 2461041.5;'
    '  tc = t + (epoch_jd - csv_epoch_jd) * 86400.0;'
    ''
    '  % ── Clamp to data range ──'
    '  if tc < ephem_t(1)'
    '      tc = ephem_t(1);'
    '  elseif tc > ephem_t(N)'
    '      tc = ephem_t(N);'
    '  end'
    ''
    '  % ── Find bracketing interval  (linear search, N is small ~53) ──'
    '  idx = 1;'
    '  for k = 1:N-1'
    '      if ephem_t(k+1) >= tc'
    '          idx = k;'
    '          break;'
    '      end'
    '  end'
    ''
    '  % ── Linear interpolation fraction ──'
    '  dt_seg = ephem_t(idx+1) - ephem_t(idx);'
    '  if dt_seg > 0'
    '      frac = (tc - ephem_t(idx)) / dt_seg;'
    '  else'
    '      frac = 0;'
    '  end'
    ''
    '  % ── Interpolate Sun ──'
    '  s1 = ephem_sun(idx, :)'';'
    '  s2 = ephem_sun(idx+1, :)'';'
    '  sun_raw = s1 + frac * (s2 - s1);'
    '  sn = sqrt(sun_raw(1)^2 + sun_raw(2)^2 + sun_raw(3)^2);'
    '  if sn > 0'
    '      sun_eci = sun_raw / sn;'
    '  else'
    '      sun_eci = [1; 0; 0];'
    '  end'
    ''
    '  % ── Interpolate Moon ──'
    '  m1 = ephem_moon(idx, :)'';'
    '  m2 = ephem_moon(idx+1, :)'';'
    '  moon_raw = m1 + frac * (m2 - m1);'
    '  mn = sqrt(moon_raw(1)^2 + moon_raw(2)^2 + moon_raw(3)^2);'
    '  if mn > 0'
    '      moon_eci = moon_raw / mn;'
    '  else'
    '      moon_eci = [0; 1; 0];'
    '  end'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  Orbit Propagator  (Keplerian 2-body with J2 secular drift)
% ─────────────────────────────────────────────────────────────────────────
function s = orbit_propagator_code()
    L = {
    'function [pos_eci, vel_eci] = fcn(t, mu, Re, J2, a, ecc, inc_deg, raan_deg, aop_deg, ta_deg)'
    '%#codegen'
    '% J2-aware Keplerian propagation from the configured mean elements.'
    '  inc = inc_deg * pi/180.0;'
    '  raan0 = raan_deg * pi/180.0;'
    '  aop0 = aop_deg * pi/180.0;'
    '  ta0 = ta_deg * pi/180.0;'
    ''
    '  n = sqrt(mu / (a*a*a));'
    '  p = a * (1.0 - ecc*ecc);'
    '  if p <= 0.0'
    '      pos_eci = [a; 0; 0];'
    '      vel_eci = [0; sqrt(mu/a); 0];'
    '      return;'
    '  end'
    ''
    '  j2_fac = J2 * (Re / p)^2;'
    '  ci = cos(inc);'
    '  d_raan = -1.5 * j2_fac * n * ci;'
    '  d_aop  =  0.75 * j2_fac * n * (5.0*ci*ci - 1.0);'
    '  d_ma   =  n + 0.75 * j2_fac * n * sqrt(max(1.0 - ecc*ecc, 0.0)) * (3.0*ci*ci - 1.0);'
    ''
    '  E0 = 2.0 * atan2(sqrt(max(1.0-ecc, 0.0)) * sin(ta0/2.0), ...'
    '                   sqrt(1.0+ecc) * cos(ta0/2.0));'
    '  ma0 = E0 - ecc * sin(E0);'
    '  ma = mod(ma0 + d_ma * t, 2.0*pi);'
    '  raan = raan0 + d_raan * t;'
    '  aop = aop0 + d_aop * t;'
    ''
    '  E = ma;'
    '  for iter = 1:12'
    '      dE = (ma - E + ecc*sin(E)) / max(1.0 - ecc*cos(E), 1e-12);'
    '      E = E + dE;'
    '      if abs(dE) < 1e-12'
    '          break;'
    '      end'
    '  end'
    ''
    '  nu = 2.0 * atan2(sqrt(1.0+ecc)*sin(E/2.0), sqrt(max(1.0-ecc, 0.0))*cos(E/2.0));'
    '  r_mag = a * (1.0 - ecc*cos(E));'
    '  p_pqw = r_mag * [cos(nu); sin(nu); 0];'
    '  vel_scale = (a*a * d_ma) / max(r_mag, 1.0);'
    '  v_pqw = vel_scale * [-sin(E); sqrt(max(1.0-ecc*ecc, 0.0))*cos(E); 0];'
    ''
    '  cO = cos(raan); sO = sin(raan);'
    '  cw = cos(aop);  sw = sin(aop);'
    '  si = sin(inc);'
    '  R_pqw_to_eci = [ cO*cw - sO*sw*ci,  -cO*sw - sO*cw*ci,   sO*si;'
    '                   sO*cw + cO*sw*ci,  -sO*sw + cO*cw*ci,  -cO*si;'
    '                   sw*si,              cw*si,               ci   ];'
    '  dR_draan = [ -sO*cw - cO*sw*ci,   sO*sw - cO*cw*ci,   cO*si;'
    '                cO*cw - sO*sw*ci,  -cO*sw - sO*cw*ci,   sO*si;'
    '                0,                   0,                  0     ];'
    '  dR_daop = [ -cO*sw - sO*cw*ci,  -cO*cw + sO*sw*ci,   0;'
    '              -sO*sw + cO*cw*ci,  -sO*cw - cO*sw*ci,   0;'
    '               cw*si,              -sw*si,              0 ];'
    ''
    '  pos_eci = R_pqw_to_eci * p_pqw;'
    '  vel_eci = R_pqw_to_eci * v_pqw + (d_raan * dR_draan + d_aop * dR_daop) * p_pqw;'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  Magnetic Field (tilted dipole)
% ─────────────────────────────────────────────────────────────────────────
function s = magnetic_field_code()
    L = {
    'function B_eci = fcn(t, pos_eci, epoch_jd, B0, Re)'
    '%#codegen'
    '% Earth-fixed tilted dipole rotated into ECI at the current epoch.'
    '  r = sqrt(pos_eci(1)^2 + pos_eci(2)^2 + pos_eci(3)^2);'
    '  if r < 1.0'
    '      B_eci = [0; 0; 0];'
    '      return;'
    '  end'
    ''
    '  theta_m = 11.5 * pi/180.0;'
    '  phi_m = -69.0 * pi/180.0;'
    '  m_ecef = [sin(theta_m)*cos(phi_m); sin(theta_m)*sin(phi_m); cos(theta_m)];'
    ''
    '  jd = epoch_jd + t / 86400.0;'
    '  gmst = gmst_rad(jd);'
    '  cg = cos(gmst); sg = sin(gmst);'
    '  R_ecef_to_eci = [cg, -sg, 0; sg, cg, 0; 0, 0, 1];'
    '  m_eci = R_ecef_to_eci * m_ecef;'
    ''
    '  r_hat = pos_eci / r;'
    '  mdotr = m_eci(1)*r_hat(1) + m_eci(2)*r_hat(2) + m_eci(3)*r_hat(3);'
    '  B_eci = B0 * (Re / r)^3 * (3.0 * mdotr * r_hat - m_eci);'
    'end'
    ''
    'function th = gmst_rad(jd)'
    '% Low-order GMST expression in degrees converted to radians.'
    '  T = (jd - 2451545.0) / 36525.0;'
    '  th_deg = 280.46061837 + 360.98564736629 * (jd - 2451545.0) + ...'
    '           0.000387933*T*T - (T*T*T)/38710000.0;'
    '  th = mod(th_deg, 360.0) * pi/180.0;'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  Eclipse Model (cylindrical shadow)
% ─────────────────────────────────────────────────────────────────────────
function s = eclipse_model_code()
    L = {
    'function eclipse_flag = fcn(pos_eci, sun_eci, Re)'
    '%#codegen'
    '% Cylindrical Earth shadow. sun_eci points from Earth toward the Sun.'
    '  sn = sqrt(sun_eci(1)^2 + sun_eci(2)^2 + sun_eci(3)^2);'
    '  rn = sqrt(pos_eci(1)^2 + pos_eci(2)^2 + pos_eci(3)^2);'
    '  if sn < 1e-12 || rn < 1.0'
    '      eclipse_flag = 0.0;'
    '      return;'
    '  end'
    '  s_hat = sun_eci / sn;'
    '  proj = pos_eci(1)*s_hat(1) + pos_eci(2)*s_hat(2) + pos_eci(3)*s_hat(3);'
    '  perp_vec = pos_eci - proj * s_hat;'
    '  perp = sqrt(perp_vec(1)^2 + perp_vec(2)^2 + perp_vec(3)^2);'
    '  eclipse_flag = double((proj < 0.0) && (perp < Re));'
    'end'
    };
    s = strjoin(L, newline);
end
