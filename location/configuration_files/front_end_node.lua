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
}
-- 最顶层用于读入的配置文件必须带return
return options
