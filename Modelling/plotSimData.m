function [] = plotSimData(simul_data, varargin)
    % Capture varargin
    if ~isempty(varargin)
        args = struct(varargin{:});
    else
        args = struct();
    end

    if isfield(args, 'Omega_ref')
        plot_omega_ref = true;
    else
        plot_omega_ref = false;
    end
    t = simul_data.tout;

    plot_motor = isfield(simul_data, 'back_emf');
    plot_hover = isfield(simul_data, 'cart_position');

    %% Motor Data Plotting
    if plot_motor
        omega = simul_data.theta_p;
        tau_e = simul_data.tau_data;
        V = simul_data.V_data;
        I = simul_data.I_data;
        back_emf = simul_data.back_emf;
        theta = simul_data.theta;
        hall = simul_data.hall_data;

        %% Velocidade angular e torque
        n_fig = 1;
        figure(n_fig)
        subplot(2, 1, 1)
        plot(t, omega, 'lineWidth', 2); hold on;
        if plot_omega_ref
            omega_ref = args.Omega_ref;
            plot(omega_ref(:, 1), omega_ref(:, 2), 'lineWidth', 2);
        end
        xlabel('Tempo (s)'); ylabel('Velocidade angular (rad/s)')
        grid on; grid minor;
        subplot(2, 1, 2)
        plot(t, tau_e, 'lineWidth', 2);
        xlabel('Tempo (s)'); ylabel('Torque elétrico (Nm)')
        hold on; grid on; grid minor;
        n_fig = n_fig + 1;

        %% Correntes
        figure(n_fig)
        subplot(3, 1, 1)
        plot(t, I(:, 1), 'lineWidth', 2);
        xlabel('Tempo (s)'); ylabel('Corrente I_a (A)')
        hold on; grid on; grid minor;
        subplot(3, 1, 2)
        plot(t, I(:, 2), 'lineWidth', 2);
        xlabel('Tempo (s)'); ylabel('Corrente I_b (A)')
        hold on; grid on; grid minor;
        subplot(3, 1, 3)
        plot(t, I(:, 3), 'lineWidth', 2);
        xlabel('Tempo (s)'); ylabel('Corrente I_c (A)')
        hold on; grid on; grid minor;
        n_fig = n_fig + 1;

        %% Tensões
        figure(n_fig)
        subplot(3, 1, 1)
        plot(t, V(:, 1), 'lineWidth', 2);
        xlabel('Tempo (s)'); ylabel('Tensão V_{a} (A)')
        hold on; grid on; grid minor;
        subplot(3, 1, 2)
        plot(t, V(:, 2), 'lineWidth', 2);
        xlabel('Tempo (s)'); ylabel('Tensão V_{b} (A)')
        hold on; grid on; grid minor;
        subplot(3, 1, 3)
        plot(t, V(:, 3), 'lineWidth', 2);
        xlabel('Tempo (s)'); ylabel('Tensão V_{c} (A)')
        hold on; grid on; grid minor;
        n_fig = n_fig + 1;

        %% Back-EMF
        figure(n_fig)
        subplot(3, 1, 1)
        plot(t, back_emf(:, 1), 'lineWidth', 2);
        xlabel('Tempo (s)'); ylabel('Back EMF e_{a} (V)')
        hold on; grid on; grid minor;
        subplot(3, 1, 2)
        plot(t, back_emf(:, 2), 'lineWidth', 2);
        xlabel('Tempo (s)'); ylabel('Back EMF e_{b} (V)')
        hold on; grid on; grid minor;
        subplot(3, 1, 3)
        plot(t, back_emf(:, 3), 'lineWidth', 2);
        xlabel('Tempo (s)'); ylabel('Back EMF e_{c} (V)')
        hold on; grid on; grid minor;
        n_fig = n_fig + 1;

        %% Torque x Velocidade angular
        figure(n_fig)
        plot(omega, tau_e, 'lineWidth', 2);
        xlabel('Velocidade Angular (rad/s)'); ylabel('Torque Elétrico (Nm)')
        hold on; grid on; grid minor;
        n_fig = n_fig + 1;

        %% Hall State
        figure(n_fig)
        subplot(2, 1, 1)
        plot(t, rem(theta, 2*pi), 'lineWidth', 2);
        xlabel('Tempo (s)'); ylabel('\theta (rad)')
        hold on; grid on; grid minor;
        subplot(2, 1, 2)
        plot(t, 4*hall(:, 1) + 2*hall(:, 2) + hall(:, 3), 'lineWidth', 2);
        xlabel('Tempo (s)'); ylabel('Hall State')
        hold on; grid on; grid minor;
    end

    %% Hover Data Plotting
    if plot_hover
        x = simul_data.cart_position(:, 1);
        y = simul_data.cart_position(:, 2);
        psi = simul_data.cart_position(:, 3);
        theta = simul_data.theta(:, 3);
        w_r0 = simul_data.theta_p(:, 1);
        w_r1 = simul_data.theta_p(:, 2);
        tau = simul_data.tau_data;
        ws = [w_r0 w_r1];

        %% Plot da Trajetória
        figure
        subplot(2, 2, [1 3]);
        standardPlot(x, y, 'x (m)', 'y (m)', 2, true)
        subplot(2, 2, 2);
        standardPlot(t, x, 't (s)', 'x (m)', 2)
        subplot(2, 2, 4);
        standardPlot(t, y, 't (s)', 'y (m)', 2)

        %% Plot da Orientação
        figure
        subplot(1, 2, 1)
        standardPlot(t, rad2deg(rem(theta, 2*pi)), 't (s)', '\gamma (°)', 2)
        subplot(1, 2, 2)
        standardPlot(t, rad2deg(rem(psi, 2*pi)), 't (s)', '\psi (°)', 2)

        %% Plot das velocidades angulares das rodas
        figure
        standardPlot(t, ws, 't (s)', '\omega (°/s)', 2)
        legend('\omega_{w0}', '\omega_{w1}');

        %% Plot torque
        figure
        plot(t, tau, 'lineWidth', 2);
        xlabel('Tempo (s)'); ylabel('Torque elétrico (Nm)')
        legend('\tau_{w0}', '\tau_{w1}');
        hold on; grid on; grid minor;
    end
end
