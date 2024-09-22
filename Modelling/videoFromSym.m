function [] = videoFromSym(SimOut, varargin)
    % Define default values
    defaults = struct( ...
        'n_fig', 10, ...
        'Tend', 0, ...
        'duracao', 5, ...
        'frameRate', 24 ...
    );
    
    % Create a struct from varargin, ensuring it is a scalar struct
    if ~isempty(varargin)
        args = struct(varargin{:});
    else
        args = struct();
    end

    % Merge default values with input arguments
    params = defaults;
    fieldNames = fieldnames(args);
    for i = 1:length(fieldNames)
        params.(fieldNames{i}) = args.(fieldNames{i});
    end

    % Assign the final values to variables
    n_fig = params.n_fig;
    duracao = params.duracao;
    frameRate = params.frameRate;
    Tend = params.Tend;
    if Tend == 0
        Tend = SimOut.tout(end);
    end

    % Calcula janela para os plots
    folga = 0.2;
    x = SimOut.cart_position(:, 1);
    y = SimOut.cart_position(:, 2);
    min_X = min(x) - folga; max_X = max(x) + folga;
    min_Y = min(y) - folga; max_Y = max(y) + folga;
    min_Z = 0; max_Z = 0.3;

    n_frames = ceil(duracao*frameRate);

    ts = linspace(0, Tend, n_frames)';
    SimOut = interpEstados(ts, SimOut);
    cart_pos = SimOut.cart_position;
    for i = 1:n_frames
        states = get_states(SimOut, i);
        plot_hover(states, 'n_fig', n_fig);
        hold on;
        plot3(cart_pos(1:i, 1), cart_pos(1:i, 2), repmat(0.15, i, 1));
        xlim([min_X, max_X]); ylim([min_Y, max_Y]); zlim([min_Z, max_Z]);
        F(i) = getframe(gcf);
        hold off;
    end
    % create the video writer with 1 fps
    writerObj = VideoWriter('simulacao.avi');
    writerObj.FrameRate = frameRate;
    % set the seconds per image
    % open the video writer
    open(writerObj);
    % write the frames to the video
    for i=1:length(F)   
        writeVideo(writerObj, F(i));
    end
    % close the writer object
    close(writerObj);
end

function [sim_plot] = interpEstados(ts, sim)
    cart_pos = sim.cart_position;
    theta = sim.theta;
    t_sim = sim.tout;
    x = interp1(t_sim, cart_pos(:, 1), ts, 'spline', 'extrap');
    y = interp1(t_sim, cart_pos(:, 2), ts, 'spline', 'extrap');
    psi = interp1(t_sim, cart_pos(:, 3), ts, 'spline', 'extrap');
    th_w0 = interp1(t_sim, theta(:, 1), ts, 'spline', 'extrap');
    th_w1 = interp1(t_sim, theta(:, 2), ts, 'spline', 'extrap');
    th = interp1(t_sim, theta(:, 3), ts, 'spline', 'extrap');

    sim_plot = struct(...
        'cart_position', zeros(length(ts), 3), ...
        'theta', zeros(length(ts), 3) ...
        );
    sim_plot.cart_position = [x y psi];
    sim_plot.theta = [th_w0 th_w1 th];
end