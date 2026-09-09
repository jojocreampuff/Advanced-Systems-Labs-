function out = u_excite_sq(t, amp_mult)
% u_excite_sq  Noisy square-wave excitation for quadcopter torques
%   out = u_excite_sq(t, amp_mult)
%
% Produces a 3x1 excitation. Each channel is an independent noisy square
% wave with random amplitude (within +/- A_range) and random hold time
% (drawn from a distribution). The generator is time-based and uses
% persistent state so it is safe when called by variable-step ODE solvers.
%
% Key options (tweak inside code):
%   base_amp_range  - max absolute amplitude for channel (vector 3x1 or scalar)
%   dwell_dist      - 'uniform' or 'exponential'
%   dwell_params    - parameters for dwell distribution
%   smooth_ms       - transition smoothing in seconds (0 => abrupt)
%   rng_seed        - [] for random seed, numeric for reproducible runs

    %% --- envelope from user's original code (keeps decaying amplitude) ---
    eps_floor = 0.2;
    env = amp_mult*(eps_floor + (1 - eps_floor)*exp(-0.0001*t));

    %% --- parameters you can tune ---
    persistent P initialized
    if isempty(P)
        P.base_amp_range = [0.8; 0.8; 0.4];    % nominal max amplitude per channel (N*m)
        P.min_dwell = 0.05;                    % minimum hold time (s)
        P.max_dwell = 0.5;                     % maximum hold time (s) for uniform dwell
        P.dwell_dist = 'uniform';              % 'uniform' or 'exponential'
        P.exp_mean = 0.2;                      % mean for exponential dwell (s)
        P.smooth_ms = 0.015;                   % smoothing time constant in seconds (transition width)
        P.rng_seed = [];                       % set numeric for reproducible runs, [] = random
        P.last_t = -inf;                       % last time called
        P.channels = 3;                        % number of independent channels
        rng('shuffle');                        % default RNG
        if ~isempty(P.rng_seed)
            rng(P.rng_seed);
        end
        % Initialize channel state vectors
        P.current_amp = (2*rand(P.channels,1)-1).*P.base_amp_range; % initial signed amplitude
        P.next_amp = P.current_amp;              % placeholder
        P.next_switch_t = zeros(P.channels,1);   % next switch times
        P.trans_start = -inf(P.channels,1);      % transition start times
        P.trans_end = -inf(P.channels,1);        % transition end times
        % schedule initial switches (so holds start immediately)
        for i=1:P.channels
            [dwell] = sample_dwell(P);
            P.next_switch_t(i) = t + dwell;
        end
        initialized = true;
        P.initialized = initialized;
        initialized = []; %#ok<NASGU>
    end

    %% Helper to sample dwell time (nested function)
    function [d] = sample_dwell(Pstruct)
        switch lower(Pstruct.dwell_dist)
            case 'uniform'
                d = Pstruct.min_dwell + (Pstruct.max_dwell - Pstruct.min_dwell)*rand();
            case 'exponential'
                d = exprnd(Pstruct.exp_mean);
                % clamp to reasonable bounds:
                d = max(Pstruct.min_dwell, min(2.0, d));
            otherwise
                d = Pstruct.min_dwell + (Pstruct.max_dwell - Pstruct.min_dwell)*rand();
        end
    end

    %% --- Check for switching events based on absolute time t ---
    % If t moves backwards (unlikely) or first call, re-init switch schedule
    if t < P.last_t
        % re-sync next switches relative to new t
        for i=1:P.channels
            P.next_switch_t(i) = t + sample_dwell(P);
        end
    end
    P.last_t = t;

    % For each channel, if it's time to switch, draw new amplitude and schedule next
    for ch = 1:P.channels
        if t >= P.next_switch_t(ch)
            % choose new amplitude uniformly within +/- base range, could be other dist
            new_amp = (2*rand()-1) * P.base_amp_range(min(ch,end));
            % schedule smoothing window
            tau = P.smooth_ms; % seconds
            if tau > 0
                P.trans_start(ch) = t;
                P.trans_end(ch) = t + tau;
                P.next_amp(ch) = new_amp;
            else
                % abrupt change, set current immediately
                P.current_amp(ch) = new_amp;
                P.next_amp(ch) = new_amp;
                P.trans_start(ch) = -inf;
                P.trans_end(ch) = -inf;
            end
            % schedule next dwell
            P.next_switch_t(ch) = t + sample_dwell(P);
        end
    end

    %% --- compute raw (per-channel) square-ish value with smoothing ---
    raw = zeros(P.channels,1);
    for ch = 1:P.channels
        if t < P.trans_start(ch) || isinf(P.trans_start(ch))
            % still in previous hold
            raw(ch) = P.current_amp(ch);
        elseif t >= P.trans_start(ch) && t <= P.trans_end(ch)
            % in transition: smooth interpolation (use tanh logistic)
            tt = (t - P.trans_start(ch))/(P.trans_end(ch)-P.trans_start(ch)); % 0..1
            % smoothstep using tanh-based ease: s(tt) = 0.5*(1 + tanh( 3*(tt-0.5) ))
            s = 0.5*(1 + tanh(6*(tt-0.5))); % steeper than linear but smooth
            raw(ch) = (1-s)*P.current_amp(ch) + s*P.next_amp(ch);
            % if transition finished, commit
            if t >= P.trans_end(ch)
                P.current_amp(ch) = P.next_amp(ch);
                P.trans_start(ch) = -inf;
                P.trans_end(ch) = -inf;
            end
        else
            % transition already finished in previous call -> commit
            P.current_amp(ch) = P.next_amp(ch);
            raw(ch) = P.current_amp(ch);
            P.trans_start(ch) = -inf;
            P.trans_end(ch) = -inf;
        end
    end

    %% --- Optionally pass through a first-order low-pass to remove high-frequency content
    % If you prefer, uncomment the following to have a continuous filtered output.
    % (This uses an internal filter state and tau_filter seconds.)
    %
    % persistent filt_state filt_tau
    % if isempty(filt_state)
    %     filt_tau = 0.010; % filter time-constant (s) - tweak as required
    %     filt_state = raw;
    % end
    % % integrate filter assuming call frequency unknown - approximate using last_t
    % dt = max(1e-6, t - P.last_t);  % avoid zero
    % alpha = dt/(filt_tau + dt);
    % filt_state = (1-alpha)*filt_state + alpha*raw;
    % raw = filt_state;

    %% --- combine with envelope and optional other components ---
    % Here we return a 3x1 vector. You can add your previous multi-sine u_prob
    % or an orthonormal mixing matrix if you want cross-channel correlation.
    % For now produce independent channels and mix with envelope.
    out = env .* raw;

end