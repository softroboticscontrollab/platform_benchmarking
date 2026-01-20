function Soft_CBF_plotting(seq_q, L, H, h, F_max, k_env, dt)
    % Input arguments: 
    % seq_q size: seq_length by n_limb matrix
    % L     size: n_limb by 1 vector [nl1, nl2,...] nl is the length of each actual
    % soft robot limb
    % Description:
    % Plot the simulated soft robot kinematics given the sequence of joint angle motion 
    % SOFTDYNAMICSSIMULATION_KINEMATICS_PLOT Summary of this function goes here
    %   Detailed explanation goes here
    
    syms q1 q2 L1 L2 m1 m2 real;
    DHparams = [];
    DHparams = extend_DH_from_q(DHparams, q1, L1);
    DHparams = extend_DH_from_q(DHparams, q2, L2);
    disp(DHparams)
    
    nl1 = L(1);
    nl2 = L(2);
    nl = max([nl1, nl2]);
    
    % Destination file name
    fname = 'withoutCBFconstraint';

    % adjust the video depending on operating system
    video_profile = 'MPEG-4';
    % TO DO fix this so it runs on all OSes
    if isunix
        video_profile = 'Motion JPEG AVI';
    end

    % Create a VideoWriter object with the specified compression profile
    videoWriterObj = VideoWriter(fname, video_profile);
    videoWriterObj.FrameRate = 30; % Set the frame rate (optional, default is 30)

    % Open the video file
    open(videoWriterObj);

    hold on;
    title('Robot arm double pendulum dynamics simulation')
    
    P1 = Polyhedron(H, h);

    seq_length = size(seq_q, 2);
    f = waitbar(0, 'Running for the cc robot arm trajectory...');
    % for i = 1:int64(1/dt):seq_length
    for i = 1:100000:seq_length
        waitbar(double(i)/seq_length, f, strcat('Running for timestep i in cc robot arm trajectory... i = ', sprintf('%d', i), '/', string(length(seq_q))));
        cla;
        plot(P1, 'color', 'w', 'EdgeColor', 'w');
        hold on;
        axis([-1.15*2*nl 1.15*2*nl -1.15*2*nl 1.15*2*nl]);
        axis square;
        plotRPPRarm_multiple_limbs(DHparams, [nl1, nl2], seq_q(:,i), [1, 1]);
        title(fname, 'Interpreter', 'none');
        subtitle('seq-q1(i)=' + string(seq_q(1, i)) + 'rads ' + 'seq-q2(i)=' + string(seq_q(2, i)) + 'rads ' + 'i:' + string(i) + '/' + string(length(seq_q)));
        xlabel('X (m)');
        ylabel('Y (m)');
        grid on;

        % Capture the current frame
        frame = getframe(gcf);
        
        % Write the frame to the video file
        writeVideo(videoWriterObj, frame);
    end

    % Close the video file
    close(videoWriterObj);
    close(f);

    disp('Video writing complete');
end
