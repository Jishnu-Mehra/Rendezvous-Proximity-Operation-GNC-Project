xlog = out.get('X_true_log');
mlog = out.get('mode_log');

t = xlog.Time;
x = squeeze(xlog.Data);

tm = mlog.Time;
m = squeeze(mlog.Data);

% Ensure each row corresponds to one time sample
if size(x,1) ~= numel(t)
    x = x.';
end

% Find HOLD entry
idx_hold = find(m(:) == 7, 1, 'first');

if isempty(idx_hold)
    error('Mode 7 was not found in mode_log.');
end

t_hold = tm(idx_hold);

% Find closest state sample
[~, idx] = min(abs(t - t_hold));

x_hold = reshape(x(idx,:), 6, 1);

e = x_hold - [0; -1000; 0; 0; 0; 0];

fprintf('HOLD entry time: %.1f s\n', t_hold);
fprintf('Position error: %.3f m\n', norm(e(1:3)));
fprintf('Velocity error: %.6f m/s\n', norm(e(4:6)));
fprintf('State at HOLD entry:\n');
disp(x_hold);

ulog = out.get('u_applied_log');

t = ulog.Time;
u = squeeze(ulog.Data);

if size(u,1) ~= numel(t)
    u = u.';
end

dv_delivered = trapz(t, u, 1);

fprintf('Delivered delta-V components (m/s):\n');
disp(dv_delivered.');

fprintf('Maximum applied acceleration (m/s^2):\n');
disp(max(abs(u),[],1).');

ulog = out.get('u_applied_log');

t = ulog.Time;
u = squeeze(ulog.Data);

if size(u,1) ~= numel(t)
    u = u.';
end

% Integrate acceleration over each planned burn
windows = [
    t_burn1_start, t_burn1_end;
    t_mid_start,   t_mid_end;
    t_burn2_start, t_burn2_end
];

names = {'Burn 1', 'Midcourse', 'Burn 2'};

for k = 1:3
    idx = t >= windows(k,1) & t <= windows(k,2);

    dv = trapz(t(idx),u(idx,:)).';
    umax = max(abs(u(idx,:)),[],1).';

    fprintf('\n%s\n',names{k});
    fprintf('Delivered DV: [%.4f %.4f %.4f] m/s\n',dv);
    fprintf('DV magnitude: %.4f m/s\n',norm(dv));
    fprintf('Max acceleration: [%.5f %.5f %.5f] m/s^2\n',umax);
end

glog = out.get('u_guid_log');

tg = glog.Time;
ug = squeeze(glog.Data);

if size(ug,1) ~= numel(tg)
    ug = ug.';
end

idx = tg >= t_burn2_start & tg < t_burn2_end;

fprintf('Burn 2 commanded acceleration:\n');
disp(mean(ug(idx,:),1).');

fprintf('Burn 2 commanded DV:\n');
disp((mean(ug(idx,:),1)*t_burn).');