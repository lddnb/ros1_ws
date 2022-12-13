options = {
    local_frame_num = 20,
    key_frame_distance = 2.0,
    -- 两个滤波参数非常重要！！！
    frame_filter_resolution = 1.3,
    loal_map_filter_resolution = 0.6,
    display_filter_resolution = 0.5,
    --NDT
    res = 1.0,
    step_size = 0.1,
    trans_eps = 0.01,
    max_iter = 30,
    back_end_options = {
      graph_optimizer_type = "g2o",
      use_gnss = true,
      use_loop_close = true,
      optimize_step_with_key_frame = 10000,
      optimize_step_with_gnss = 950,  -- 950
      optimize_step_with_loop = 100,
      key_frame_distance = 2.0,
      odom_edge_noise = "0.5 0.5 0.5 0.001 0.001 0.001",
      loop_edge_noise = "0.3 0.3 0.3 0.001 0.001 0.001",
      gnss_edge_noise = "2.0 2.0 2.0",
    }
}
-- 最顶层用于读入的配置文件必须带return
return options
