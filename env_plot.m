function env_plot(color)

  global VISITED
  global DIAMETER

  diam = DIAMETER;
  vst = VISITED;

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

    rectangle('Position', [x*diam y*diam diam diam], 'FaceColor', color);
    hold on
  end
end
