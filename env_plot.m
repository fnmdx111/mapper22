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

  nv = sortrows(nv, [1, 2]);

%   figure
%   set(gca, 'xtick', [0:diam:(max(nv(:, 1)) + diam)]);
%   set(gca, 'ytick', [0:diam:(max(nv(:, 2)) + diam)]);
%   grid on

  for i = nv'
    x = floor(i(1) / diam);
    y = floor(i(2) / diam);

%    set(0, 'CurrentFigure', fig);
    rectangle('Position', [x*diam y*diam diam diam],...
              'FaceColor', color);
    hold on
  end
end

function my_rect(l, b, w, h, color, fig)
    plot([l, l, l + w, l + w, l],...
         [b, b + h, b + h, b, b], color,...
         'LineWidth', 2,...
         'CurrentFigure', fig);
end
