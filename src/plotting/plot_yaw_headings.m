function plot_yaw_headings(gt_path, gtsam_path, ekf_path, output_path)
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

  gt_theta = unwrap(gt(:,4));
  gtsam_theta = unwrap(gtsam(:,4));
  ekf_theta = unwrap(ekf(:,4));

  figure('visible', 'off');
  plot(gt(:,1), gt_theta, 'k-', 'linewidth', 2);
  hold on;
  plot(gtsam(:,1), gtsam_theta, 'r-', 'linewidth', 2);
  plot(ekf(:,1), ekf_theta, 'm-', 'linewidth', 2);
  grid on;
  xlabel('Time (s)');
  ylabel('Yaw (rad)');
  title('Yaw Heading vs Time');
  legend('Ground truth', 'GTSAM', 'EKF', 'location', 'best');

  print(output_path, '-dpng', '-r150');
end
