function plot_rrr_arm(r)
%PLOT_ARM Summary of this function goes here
%   Detailed explanation goes here

    for i = 0:20:360
        for j = 0:20:360
            for k = 0:20:360
                fm = r.fkine([i j k] / 360 * pi);
                scatter3(fm(1, 4), fm(2, 4), fm(3, 4));
                hold on
            end
        end
    end
end

