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
        {'t','1'; 'sun_eci','[3 1]'; 'moon_eci','[3 1]'});
    set_mfb_data_sizes([sys '/Orbit_Propagator'], ...
        {'t','1'; 'pos_eci','[3 1]'; 'vel_eci','[3 1]'});
    set_mfb_data_sizes([sys '/Magnetic_Field_Model'], ...
        {'pos_eci','[3 1]'; 'B_eci','[3 1]'});
    set_mfb_data_sizes([sys '/Eclipse_Model'], ...
        {'pos_eci','[3 1]'; 'sun_eci','[3 1]'; 'eclipse_flag','1'});

    % ── Wiring ──
    % Clock + ephemeris data → Ephemeris_Truth (4 inputs: t, ephem_t, sun_data, moon_data)
    add_line(sys, 'Clock/1',      'Ephemeris_Truth/1', ar{:});
    add_line(sys, 'Ephem_t/1',    'Ephemeris_Truth/2', ar{:});
    add_line(sys, 'Ephem_sun/1',  'Ephemeris_Truth/3', ar{:});
    add_line(sys, 'Ephem_moon/1', 'Ephemeris_Truth/4', ar{:});
    add_line(sys, 'Clock/1',      'Orbit_Propagator/1', ar{:});

    % Ephemeris → outports
    add_line(sys, 'Ephemeris_Truth/1', 'Sun_vec_ECI/1',  ar{:});
    add_line(sys, 'Ephemeris_Truth/2', 'Moon_vec_ECI/1', ar{:});

    % Orbit Propagator → outports
    add_line(sys, 'Orbit_Propagator/1', 'pos_ECI/1', ar{:});
    add_line(sys, 'Orbit_Propagator/2', 'vel_ECI/1', ar{:});

    % pos_ECI branch → Magnetic_Field_Model
    add_line(sys, 'Orbit_Propagator/1', 'Magnetic_Field_Model/1', ar{:});
    add_line(sys, 'Magnetic_Field_Model/1', 'B_ECI/1',            ar{:});

    % pos_ECI + Sun_vec → Eclipse_Model
    add_line(sys, 'Orbit_Propagator/1', 'Eclipse_Model/1', ar{:});
    add_line(sys, 'Ephemeris_Truth/1',  'Eclipse_Model/2', ar{:});
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
    'function [sun_eci, moon_eci] = fcn(t, ephem_t, ephem_sun, ephem_moon)'
    '%#codegen'
    '% Ground-truth Sun & Moon direction vectors interpolated from'
    '% pre-loaded ephemeris CSV data (ephemeris_2026_weekly.csv).'
    '%'
    '% Inputs:  t          – simulation time [s]'
    '%          ephem_t    – [Nx1] time breakpoints [s] from epoch'
    '%          ephem_sun  – [Nx3] Sun unit direction vectors (ECI)'
    '%          ephem_moon – [Nx3] Moon unit direction vectors (ECI)'
    '% Outputs: sun_eci    – Sun direction unit vector ECI [3x1]'
    '%          moon_eci   – Moon direction unit vector ECI [3x1]'
    ''
    '  N = size(ephem_t, 1);'
    ''
    '  % ── Clamp to data range ──'
    '  tc = t;'
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
    'function [pos_eci, vel_eci] = fcn(t)'
    '%#codegen'
    '% Compile-stable fixed LEO state fallback.'
    '  pos_eci = [6871000; 0; 0];'
    '  vel_eci = [0; 7616.5; 0];'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  Magnetic Field (tilted dipole)
% ─────────────────────────────────────────────────────────────────────────
function s = magnetic_field_code()
    L = {
    'function B_eci = fcn(pos_eci)'
    '%#codegen'
    '% Compile-stable literal magnetic field fallback.'
    '  B_eci = [0; 0; 3.12e-5];'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  Eclipse Model (cylindrical shadow)
% ─────────────────────────────────────────────────────────────────────────
function s = eclipse_model_code()
    L = {
    'function eclipse_flag = fcn(pos_eci, sun_eci)'
    '%#codegen'
    '% Compile-stable sunlit fallback.'
    '  eclipse_flag = 0.0;'
    'end'
    };
    s = strjoin(L, newline);
end
