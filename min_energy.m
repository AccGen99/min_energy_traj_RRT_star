function val = min_energy(ini_state, final_state)

ini_vec = [ini_state(1) ini_state(2)];
f_vec = [final_state(1) final_state(2)];

shifted_x  = sqrt(sum((ini_vec - f_vec).^ 2));

tf = 1.0;

phi_ini_state = atan2(ini_state(2), ini_state(1));
phi_f_state = atan2(final_state(2), final_state(1));

phi_f = phi_f_state - phi_ini_state;

diff = run(tf, shifted_x, phi_f, 0, 0); % True = 1, False = 0
tf_max = tf*(1+diff(3));
t_array = linspace(tf, tf_max, 100);
energy_vals = 0*t_array;
for i = 1:length(t_array)
%    fprintf('Time = %f\n', t_array(i));
    if t_array(i) ~= 0
        returned_val = run(t_array(i), shifted_x, phi_f, 0, 0);
        if returned_val(2)
            energy_vals(i) = returned_val(1);
        else
            energy_vals(i) = 1e10;
        end
    else
        energy_vals(i) = 1e10;
    end
end

% figure
% plot(t_array, energy_vals)
% title('Difference Values vs Time')
% xlabel('Time')
% ylabel('Difference Values')

energy_vals;
min_val = min(energy_vals);
val_idx = find(energy_vals == min_val);
tf_ = t_array(val_idx);
run(tf_, shifted_x, phi_f, 0, 0);
val = min_val;
end