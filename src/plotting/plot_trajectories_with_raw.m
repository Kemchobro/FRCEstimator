function plot_trajectories_with_raw(gt_path, gtsam_path, ekf_path, raw_path, vision_path, tag_path, output_path)
  toolkits = available_graphics_toolkits();
  if (isempty(toolkits))
    fprintf('No graphics toolkits available.\n');
    error('no_graphics_toolkit');
  endif
  if (any(strcmp(toolkits, 'gnuplot')))
    graphics_toolkit('gnuplot');
  endif
  gt = dlmread(gt_path, ',', 1, 0);
  gtsam = dlmread(gtsam_path, ',', 1, 0);
  ekf = dlmread(ekf_path, ',', 1, 0);
  raw = dlmread(raw_path, ',', 1, 0);
  vision = dlmread(vision_path, ',', 1, 0);
  tag = dlmread(tag_path, ',', 1, 0);

  figure('visible', 'off');
  h = [];
  labels = {};
  h(end + 1) = plot(gt(:,2), gt(:,3), 'k-', 'linewidth', 2);
  labels{end + 1} = 'Ground truth';
  hold on;
  h(end + 1) = plot(gtsam(:,2), gtsam(:,3), 'r-', 'linewidth', 2);
  labels{end + 1} = 'GTSAM';
  h(end + 1) = plot(ekf(:,2), ekf(:,3), 'm-', 'linewidth', 2);
  labels{end + 1} = 'EKF';
  h(end + 1) = plot(raw(:,2), raw(:,3), 'b-', 'linewidth', 1);
  labels{end + 1} = 'Wheel-only';
  if (!isempty(vision))
    h(end + 1) = plot(vision(:,2), vision(:,3), 'g.', 'markersize', 6);
    labels{end + 1} = 'Vision';
  endif
  if (!isempty(tag))
    h(end + 1) = plot(tag(:,2), tag(:,3), 'p', 'markersize', 12, 'markerfacecolor', [1.0, 0.65, 0.0]);
    labels{end + 1} = 'Tags';
  endif
  grid on;
  axis equal;
  xlabel('X (m)');
  ylabel('Y (m)');
  title('Ground Truth vs GTSAM vs EKF vs Wheel-only vs Vision vs Tags');
  legend(h, labels, 'location', 'best');

  print(output_path, '-dpng', '-r150');
end
