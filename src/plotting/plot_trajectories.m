function plot_trajectories(gt_path, gtsam_path, ekf_path, tag_path, output_path)
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
  tag = dlmread(tag_path, ',', 1, 0);

  figure('visible', 'off');
  plot(gt(:,2), gt(:,3), 'k-', 'linewidth', 2);
  hold on;
  plot(gtsam(:,2), gtsam(:,3), 'r-', 'linewidth', 2);
  plot(ekf(:,2), ekf(:,3), 'm-', 'linewidth', 2);
  plot(tag(:,2), tag(:,3), 'p', 'markersize', 12, 'markerfacecolor', [1.0, 0.65, 0.0]);
  grid on;
  axis equal;
  xlabel('X (m)');
  ylabel('Y (m)');
  title('Ground Truth vs GTSAM vs EKF vs Tags');
  legend('Ground truth', 'GTSAM', 'EKF', 'Tags', 'location', 'best');

  print(output_path, '-dpng', '-r150');
end
