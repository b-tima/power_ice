nothing_data = readmatrix("../data/nothing_5.csv");
water_data = readmatrix("../data/water_5.csv");
ice_data = readmatrix("../data/ice_5.csv");

nothing_dc = mean(nothing_data(1, :));
water_dc = mean(water_data(1, :));
ice_dc = mean(ice_data(1, :));

nothing_data_comp = nothing_data - nothing_dc;
water_data_comp = water_data - water_dc;
ice_data_comp = ice_data - ice_dc;

nothing_data_comp_1 = nothing_data_comp(2, ~isnan(nothing_data_comp(2, :)));
nothing_data_comp_2 = nothing_data_comp(3, ~isnan(nothing_data_comp(3, :)));
nothing_data_comp_3 = nothing_data_comp(4, ~isnan(nothing_data_comp(4, :)));

water_data_comp_1 = water_data_comp(2, ~isnan(water_data_comp(2, :)));
water_data_comp_2 = water_data_comp(3, ~isnan(water_data_comp(3, :)));
water_data_comp_3 = water_data_comp(4, ~isnan(water_data_comp(4, :)));

ice_data_comp_1 = ice_data_comp(2, ~isnan(ice_data_comp(2, :)));
ice_data_comp_2 = ice_data_comp(3, ~isnan(ice_data_comp(3, :)));
ice_data_comp_3 = ice_data_comp(4, ~isnan(ice_data_comp(4, :)));

%% Raw data

subplot(3, 3, 1);
plot(nothing_data_comp_1(10:end))
title("Nothing led 1");
subplot(3, 3, 2);
plot(nothing_data_comp_2(10:end))
title("Nothing led 2");
subplot(3, 3, 3);
plot(nothing_data_comp_3(10:end))
title("Nothing led 3");

subplot(3, 3, 4);
plot(water_data_comp_1(10:end))
title("Water led 1");
subplot(3, 3, 5);
plot(water_data_comp_2(10:end))
title("Water led 2");
subplot(3, 3, 6);
plot(water_data_comp_3(10:end))
title("Water led 3");

subplot(3, 3, 7);
plot(ice_data_comp_1(10:end))
title("Ice led 1");
subplot(3, 3, 8);
plot(ice_data_comp_1(10:end))
title("Ice led 2");
subplot(3, 3, 9);
plot(ice_data_comp_1(10:end))
title("Ice led 3");

pause;

%% Normalized data

nothing_mean_1 = mean(nothing_data_comp_1(10:end))
nothing_mean_2 = mean(nothing_data_comp_2(10:end))
nothing_mean_3 = mean(nothing_data_comp_3(10:end))

water_mean_1 = mean(water_data_comp_1(10:end))
water_mean_2 = mean(water_data_comp_2(10:end))
water_mean_3 = mean(water_data_comp_3(10:end))

ice_mean_1 = mean(ice_data_comp_1(10:end))
ice_mean_2 = mean(ice_data_comp_2(10:end))
ice_mean_3 = mean(ice_data_comp_3(10:end))

water_normalized_1 = water_mean_1 / nothing_mean_1;
water_normalized_2 = water_mean_2 / nothing_mean_2;
water_normalized_3 = water_mean_3 / nothing_mean_3;

ice_normalized_1 = ice_mean_1 / nothing_mean_1;
ice_normalized_2 = ice_mean_2 / nothing_mean_2;
ice_normalized_3 = ice_mean_3 / nothing_mean_3;

data_for_plot = [0, 1, 1, 1, 0; 0, water_normalized_1, water_normalized_2, water_normalized_3, 0; 0, ice_normalized_1, ice_normalized_2, ice_normalized_3, 0];
subplot(1, 1, 1);
stem(data_for_plot(1, :), "filled");
hold on;
stem(data_for_plot(2, :), "filled");
stem(data_for_plot(3, :), "filled");
hold off;

pause;

clf;

% nothing

subplot(1, 3, 1);

box_data_x_1 = [nothing_data_comp_1(10:end)/nothing_mean_1;water_data_comp_1(10:end)/nothing_mean_1;ice_data_comp_1(10:end)/nothing_mean_1]';

boxplot(box_data_x_1, "labels", {'nothing', 'water', 'ice'});
title("Led 1");
hold on;

subplot(1, 3, 2);

box_data_x_2 = [nothing_data_comp_2(10:end)/nothing_mean_2 ; water_data_comp_2(10:end)/nothing_mean_2 ; ice_data_comp_2(10:end)/nothing_mean_2]';

boxplot(box_data_x_2, "labels", {'nothing', 'water', 'ice'});
title("Led 2");
hold on;

subplot(1, 3, 3);

box_data_x_3 = [nothing_data_comp_3(10:end)/nothing_mean_3;water_data_comp_3(10:end)/nothing_mean_3;ice_data_comp_3(10:end)/nothing_mean_3]';

boxplot(box_data_x_3, "labels", {'nothing', 'water', 'ice'});
title("Led 3");
hold on;

