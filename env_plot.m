function env_plot(points, fig, color)
  global DIAMETER
  diam = DIAMETER;

  vst = points;

  min_y = min(vst(:, 2));
  min_x = min(vst(:, 1));

  nv = [];
  for i = vst'
    nv(end + 1, :) = i - [min_x min_y]' + [diam / 2 diam / 2]';
    display(nv(end, :))
  end % translate all the points in vst to the first quadrant

%   figure
%   set(gca, 'xtick', [0:diam:(max(nv(:, 1)) + diam)]);
%   set(gca, 'ytick', [0:diam:(max(nv(:, 2)) + diam)]);
%   grid on

  for i = nv'
    x = floor(i(1) / diam);
    y = floor(i(2) / diam);

%     if abs(i(1) - x * diam) < 0.1
%         rectangle('Position', [(x - 1)*diam y*diam diam diam],...
%                   'FaceColor', color);
%     end
    if abs(i(2) - y * diam) < 0.1
        rectangle('Position', [x*diam (y - 1)*diam diam diam],...
                  'FaceColor', color);
    end
    if abs(i(2) - (y + 1) * diam) < 0.1
        rectangle('Position', [x*diam (y + 1)*diam diam diam],...
                  'FaceColor', color);
    end

%    set(0, 'CurrentFigure', fig);
    rectangle('Position', [x*diam y*diam diam diam],...
              'FaceColor', color);
    hold on
  end
end
